// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "ReUseX/Solidifier.hpp"
#include <spdlog/spdlog.h>

namespace ReUseX {

// Overload operator- for std::set
template <typename T>
std::set<T> operator-(const std::set<T> &a, const std::set<T> &b) {
  std::set<T> result;
  std::set_difference(a.begin(), a.end(), b.begin(), b.end(),
                      std::inserter(result, result.begin()));
  return result;
}

template <typename T>
std::set<T> operator+(const std::set<T> &a, const std::set<T> &b) {
  std::set<T> result;
  std::set_union(a.begin(), a.end(), b.begin(), b.end(),
                 std::inserter(result, result.begin()));
  return result;
}

/** Setup the MIP variables
 *
 * $$x_{c,l} \in {0, 1}, c \in C, l \in R_0 U \cup W$$
 */
void Solidifier::_setupVariables() {
  spdlog::trace("Create variables");
  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    _room_variables[*cit] = std::vector<Variable *>(_cc->n_rooms);
    _wall_variables[*cit] = std::vector<Variable *>(_cc->n_walls);

    const int c_id = std::get<CellData>((*_cc)[*cit].data).id;

    _outside_variables[*cit] = solver.create_variable(
        Variable::BINARY, 0, 1, fmt::format("x_{},o", c_id));

    for (size_t r = 0; r < _cc->n_rooms; ++r)
      _room_variables[*cit][r] = solver.create_variable(
          Variable::BINARY, 0, 1, fmt::format("x_{},r{}", c_id, r));
    for (size_t w = 0; w < _cc->n_walls; ++w)
      _wall_variables[*cit][w] = solver.create_variable(
          Variable::BINARY, 0, 1, fmt::format("x_{},w{}", c_id, w));
  }
}

/** Setup the MIP objective
 *
 * Minimize the cost function
 * $$F_c := -R_c + a(W_{F_b} + W_{F_i} )$$
 * where
 * $$R_c \coloneq \sum_{c \in C} \sum_{r \in R_o} x_{c,r} P_C(c,r)
 * volume(c)$$
 * $$W_{F_b} \coloneq \sum_{f_{c_a,c_b} \in F} \sum_{w \in W_{\bar{c_a},
 * c_b}} c_{c_b,w} (1-P_F(f_{c_a,c_b})) area(f_{c_a,c_b})$$ and
 * $$W_{F_i} \coloneq \sum_{f_{c_a,c_b} \in F} \sum_{w \in W_{c_a, c_b}}
 * (x_{c_a,w} - x_{c_a,w})(1-P_F(f_{c_a,c_b})) area(f_{c_a,c_b})$$
 */
void Solidifier::_setupObjective() {
  spdlog::trace("Create objective");

  auto area = _cc->property_map<Fd, double>("f:area");
  auto volume = _cc->property_map<Cd, double>("c:volume");
  auto f_sp = _cc->property_map<Fd, double>("f:support_probability");
  auto c_rp =
      _cc->property_map<Cd, std::vector<double>>("c:room_probabilities");

  Linear_objective *obj = solver.create_objective(Linear_objective::MINIMIZE);
  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    for (size_t r = 0; r < _cc->n_rooms; ++r) {
      // R_c
      const double weight = c_rp[*cit][r] * volume[*cit];
      Variable *x_cr = _room_variables[*cit][r];
      obj->add_coefficient(x_cr, -weight);
    }
  }

  for (auto fit = _cc->faces_begin(); fit != _cc->faces_end(); ++fit) {
    // For a particular cell pair ca, bc, we define the set W_{\bar{c_a},
    // c_b} of walls that are contained in cell cb but not in ca
    auto cit_a = _cc->get_a(*fit);
    auto cit_b = _cc->get_b(*fit);

    auto Wa = std::get<CellData>((*_cc)[cit_a].data).wall_ids;
    auto Wb = std::get<CellData>((*_cc)[cit_b].data).wall_ids;

    const double weight = (1 - f_sp[*fit]) * area[*fit];

    // W_{F_b}
    for (auto id : Wb - Wa) {
      Variable *Xcbw = _wall_variables[cit_b][id];
      obj->add_coefficient(Xcbw, alpha * weight);
    }

    // Analogously, we define the set W_{c_a, c_b} of walls that are
    // contained in both ca and cb
    // W_{F_i}
    for (auto id : Wa + Wb) {
      Variable *Xcbw = _wall_variables[cit_b][id];
      obj->add_coefficient(Xcbw, alpha * weight);
      Variable *Xcaw = _wall_variables[cit_a][id];
      obj->add_coefficient(Xcaw, -alpha * weight);
    }
  }
}

/** Constraint 1: Each cell must be assigned exactly one label from Ro
 *
 * $$\forall c \in C: \sum_{r \in R_0} x_{c,r} = 1$$
 */
void Solidifier::_c_1() {
  spdlog::trace("Create constraint 1");
  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    Linear_constraint *c1 = solver.create_constraint(1, 1, "c1");
    for (size_t r = 1; r < _cc->n_rooms + 1; ++r) {
      Variable *Xcr = _room_variables[*cit][r];
      c1->add_coefficient(Xcr, 1);
    }
  }
}

/** Constraint 2: At boundary faces of room interiors, the room label
 * may only occur on the positive side of the face
 *
 * $$\forall f_{c_a,c_b} \in F \forall r \in R: x_{c_a,r} - x_{c_b,r} \geqslant
 * 0$$
 */
void Solidifier::_c_2() {
  spdlog::trace("Create constraint 2");
  for (auto fit = _cc->faces_begin(); fit != _cc->faces_end(); ++fit) {
    for (size_t r = 1; r < _cc->n_rooms + 1; ++r) {
      Variable *x_car = _room_variables[_cc->get_a(*fit)][r];
      Variable *x_cbr = _room_variables[_cc->get_b(*fit)][r];
      Linear_constraint *c2 =
          solver.create_constraint(0, Linear_constraint::infinity(), "c2");
      c2->add_coefficient(x_car, 1);
      c2->add_coefficient(x_cbr, -1);
    }
  }
}

/** Constraint 3: Wall labels may only occur in cells which are assigned
 * the outside label
 *
 * $$\forall c \in C \forall \forall w \in W_c: x_{c,w} \leqslant x_{c,o}$$
 */
void Solidifier::_c_3() {
  spdlog::trace("Create constraint 3");
  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    for (size_t w = 0; w < _cc->n_walls; ++w) {
      Variable *Xcw = _wall_variables[*cit][w];
      Variable *Xco = _outside_variables[*cit];
      Linear_constraint *c3 =
          solver.create_constraint(-Linear_constraint::infinity(), 0, "c3");
      c3->add_coefficient(Xcw, 1);
      c3->add_coefficient(Xco, -1);
    }
  }
}

/** Constraint 4: The boundary faces of room interiors must also be the boundary
 * faces of an active wall
 * $$\forall f_{c_a,c_b} \in F : \sum_{w \in W_{\bar{c_a}, c_b} x_{c_b,w}
 * \geqslant x_{c_b,o} - x_{c_a,o}$$
 */
void Solidifier::_c_4() {
  spdlog::trace("Create constraint 4");
  for (auto fit = _cc->faces_begin(); fit != _cc->faces_end(); ++fit) {
    auto cit_a = _cc->get_a(*fit);
    auto cit_b = _cc->get_b(*fit);

    auto W_a = std::get<CellData>((*_cc)[cit_a].data).wall_ids;
    auto W_b = std::get<CellData>((*_cc)[cit_b].data).wall_ids;

    // TODO: Check signs of coefficients
    Linear_constraint *c4 =
        solver.create_constraint(0, Linear_constraint::infinity(), "c4");

    for (auto id : W_b - W_a) {
      Variable *Xcbw = _wall_variables[cit_b][id];
      c4->add_coefficient(Xcbw, 1);
    }

    Variable *Xcao = _outside_variables[cit_a];
    Variable *Xcbo = _outside_variables[cit_b];

    c4->add_coefficient(Xcbo, 1);
    c4->add_coefficient(Xcao, -1);
  }
}

/** Constraint 5: At wall boundaries that are inner faces, the wall label
 * must be on the negative side of the respective faces
 *
 * $$\forall f_{c_a,c_b} \in F \forall w \in W_{c_a, c_b}: x_{sb,w} -
 * x_{c_a,w} \geqslant 0$$
 */
void Solidifier::_c_5() {
  spdlog::trace("Create constraint 5");
  for (auto fit = _cc->faces_begin(); fit != _cc->faces_end(); ++fit) {
    auto cit_a = _cc->get_a(*fit);
    auto cit_b = _cc->get_b(*fit);

    auto W_a = std::get<CellData>((*_cc)[cit_a].data).wall_ids;
    auto W_b = std::get<CellData>((*_cc)[cit_b].data).wall_ids;

    for (auto id : W_a + W_b) {
      Variable *Xcaw = _wall_variables[cit_a][id];
      Variable *Xcbw = _wall_variables[cit_b][id];
      Linear_constraint *c5 =
          solver.create_constraint(0, Linear_constraint::infinity(), "c5");
      c5->add_coefficient(Xcbw, 1);
      c5->add_coefficient(Xcaw, -1);
    }
  }
}

/** Constraint 6: A wall may end at an inner face only if this face is a
 * boundary face of at least one other active wall
 *
 * $$\forall f_{c_a,c_b} \in F \forall w \in W_{c_a, c_b}: \sum_{w' \in
 * W_{\bar{c_a},c_b}} x_{c_b,w'} \geqslant x_{c_b,w}-x_{c_a,w}$$
 */
void Solidifier::_c_6() {
  spdlog::trace("Create constraint 6");
  for (auto fit = _cc->faces_begin(); fit != _cc->faces_end(); ++fit) {
    auto cit_a = _cc->get_a(*fit);
    auto cit_b = _cc->get_b(*fit);

    auto W_a = std::get<CellData>((*_cc)[cit_a].data).wall_ids;
    auto W_b = std::get<CellData>((*_cc)[cit_b].data).wall_ids;
    // TODO: Check Constraint
    // I assume there might be a mistake with the signs of the coefficients
    for (auto id : W_a + W_b) {
      Variable *Xcaw = _wall_variables[cit_a][id];
      Variable *Xcbw = _wall_variables[cit_b][id];
      Linear_constraint *c6 =
          solver.create_constraint(0, Linear_constraint::infinity(), "c6");
      c6->add_coefficient(Xcbw, -1);
      c6->add_coefficient(Xcaw, 1);

      for (auto id2 : W_b - W_a) {
        Variable *Xcbw2 = _wall_variables[cit_b][id2];
        c6->add_coefficient(Xcbw2, 1);
      }
    }
  }
}

/** Constraint 7: At boundary faces of outside area, the outside label
 * may only occur on the negative side of the separating face
 *
 * $$\forall f_{c_a,c_b} \in F : x_{c_a,o} - x_{c_b,o} \leqslant 0$$
 */
void Solidifier::_c_7() {
  spdlog::trace("Create constraint 7");
  for (auto fit = _cc->faces_begin(); fit != _cc->faces_end(); ++fit) {
    auto cit_a = _cc->get_a(*fit);
    auto cit_b = _cc->get_b(*fit);

    Variable *Xcao = _outside_variables[cit_a];
    Variable *Xcbo = _outside_variables[cit_b];

    Linear_constraint *c7 =
        solver.create_constraint(-Linear_constraint::infinity(), 0, "c7");
    c7->add_coefficient(Xcao, 1);
    c7->add_coefficient(Xcbo, -1);
  }
}

} // namespace ReUseX
