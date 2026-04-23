// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/Solidifier.hpp"
#include "core/logging.hpp"
#include <tbb/concurrent_unordered_map.h>

#ifdef USE_CUOPT
#include <CGAL/cuOpt_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::cuOpt_mixed_integer_program_traits<double>;
#elif defined(USE_HIGHS)
#include <CGAL/HiGHS_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::HiGHS_mixed_integer_program_traits<double>;
#elif defined(CGAL_USE_SCIP)
#include <CGAL/SCIP_mixed_integer_program_traits.h>
#include <scip/scip.h>
using MIP_Solver = CGAL::SCIP_mixed_integer_program_traits<double>;
#elif defined(CGAL_USE_GLPK)
#include <CGAL/GLPK_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::GLPK_mixed_integer_program_traits<double>;
#endif

#define SolverDebug 0

namespace reusex::geometry {

// PIMPL Implementation class
class Solidifier::Impl {
    public:
  using Variable = typename MIP_Solver::Variable;
  using Linear_objective = typename MIP_Solver::Linear_objective;
  using Linear_constraint = typename MIP_Solver::Linear_constraint;

  using Fd = CellComplex::Vertex;
  using Cd = CellComplex::Vertex;

  MIP_Solver solver;
  std::shared_ptr<const CellComplex> _cc;

  std::unordered_map<Cd, std::vector<Variable *>> _room_variables{};
  std::unordered_map<Cd, std::vector<Variable *>> _wall_variables{};

  static constexpr double alpha = 0.04;

  explicit Impl(std::shared_ptr<const CellComplex> cc) : _cc(cc) {
    reusex::trace("Solidifier created");
  }

  void _configureSolver();
  void _setupVariables();
  void _setupObjective();
  void _setupConstraints();
  void _c_1();
  void _c_2();
  void _c_3();
  void _c_4();
  void _c_5();
  void _c_6();
  void _c_7();

  std::pair<std::set<int>, std::set<int>> getWallIds(Cd cit_a, Cd cit_b) const {
    auto W_a = std::get<CellData>((*_cc)[cit_a].data).wall_ids;
    auto W_b = std::get<CellData>((*_cc)[cit_b].data).wall_ids;
    return {W_a, W_b};
  }
};

// Solidifier public interface implementation
Solidifier::Solidifier(std::shared_ptr<const CellComplex> cc)
    : pimpl_(std::make_unique<Impl>(cc)) {}

Solidifier::~Solidifier() = default;

std::optional<std::pair<std::unordered_map<Solidifier::Cd, int>,
                        std::unordered_map<Solidifier::Cd, std::set<int>>>>
Solidifier::solve() {

  reusex::info("Using MIP solver: {}", typeid(pimpl_->solver).name());
  reusex::trace("Start solving MIP");

  pimpl_->_configureSolver();
  pimpl_->_setupVariables();
  pimpl_->_setupObjective();
  pimpl_->_setupConstraints();

  // Log problem statistics
  reusex::debug(
      "Problem size: {} cells, {} rooms, {} walls",
      std::distance(pimpl_->_cc->cells_begin(), pimpl_->_cc->cells_end()),
      pimpl_->_cc->n_rooms, pimpl_->_cc->n_walls);
  reusex::debug("Total variables: {}, Total constraints: {}",
                      pimpl_->solver.number_of_variables(),
                      pimpl_->solver.number_of_constraints());

  reusex::trace("Start solving");
  auto sw = reusex::core::stopwatch{};

  // INFO: Solve
  if (!pimpl_->solver.solve()) {
    reusex::error("MIP solver failed to find solution");
    return {};
  }

  reusex::info("MIP solver succeeded");

  auto room_label = std::unordered_map<Cd, int>{};
  auto wall_label = std::unordered_map<Cd, std::set<int>>{};
  for (auto cit = pimpl_->_cc->cells_begin(); cit != pimpl_->_cc->cells_end();
       ++cit) {
    for (size_t i = 0; i < pimpl_->_room_variables[*cit].size(); ++i) {
      if (pimpl_->_room_variables[*cit][i]->solution_value() > 0.5)
        room_label[*cit] = static_cast<int>(i);
    }
    wall_label[*cit] = std::set<int>{};
    for (size_t i = 0; i < pimpl_->_wall_variables[*cit].size(); ++i) {
      if (pimpl_->_wall_variables[*cit][i]->solution_value() > 0.5)
        wall_label[*cit].insert(static_cast<int>(i));
    }
  }
  reusex::info("Solved MIP in {:>.3f} seconds", sw);

  return std::make_pair(room_label, wall_label);
}

std::shared_ptr<const CellComplex> Solidifier::get_cell_complex() const {
  return pimpl_->_cc;
}

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

/** Configure the MIP solver with parameters to improve performance and reduce
 * memory usage. The specific parameters depend on the solver being used (e.g.,
 * SCIP, Gurobi, CPLEX). Below are example configurations for SCIP, which is a
 * popular open-source MIP solver. Adjust the parameters as needed for other
 * solvers.
 */
void Solidifier::Impl::_configureSolver() {

#ifdef CGAL_USE_SCIP
  /*
SCIP *scip = solver.scip();

// --- Overall solver emphasis: minimize memory usage ---
// SCIPsetEmphasis(scip, SCIP_PARAMEMPHASIS_MEMORY, TRUE);

// --- Hard memory limit (MB) ---
SCIPsetRealParam(scip, "limits/memory", 50000.0); // 50 GB

// --- Limit search tree growth ---
SCIPsetLongintParam(scip, "limits/nodes", 500000);

// --- Store fewer feasible solutions ---
SCIPsetIntParam(scip, "limits/maxsol", 5);

// --- Reduce presolve rounds ---
SCIPsetIntParam(scip, "presolving/maxrounds", 3);

// --- Reduce cutting plane generation ---
SCIPsetIntParam(scip, "separating/maxrounds", 5);

// --- Disable conflict analysis (large memory consumer) ---
SCIPsetBoolParam(scip, "conflict/enable", FALSE);

// --- Reduce heuristic memory usage ---
SCIPsetHeuristics(scip, SCIP_PARAMSETTING_FAST, TRUE);

// --- Depth-first node exploration (stores fewer nodes) ---
SCIPsetCharParam(scip, "nodeselection/childsel", 'd');

// --- Encourage node compression / disk usage ---
SCIPsetRealParam(scip, "memory/savefac", 0.1);

// If your model has tons of binary variables, also add:
SCIPsetIntParam(scip, "separating/maxcuts", 50);
*/
#endif
}

/** Setup the MIP variables
 *
 * $$x_{c,l} \in {0, 1}, c \in C, l \in R_0 U \cup W$$
 */
void Solidifier::Impl::_setupVariables() {
  reusex::trace("Create variables");
  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    _room_variables[*cit] = std::vector<Variable *>(_cc->n_rooms + 1);
    _wall_variables[*cit] = std::vector<Variable *>(_cc->n_walls);

    const int c_id = (*_cc)[*cit].id;

    for (size_t l = 0; l < _cc->n_rooms + 1; ++l)
      _room_variables[*cit][l] = solver.create_variable(
          Variable::BINARY, 0, 1, fmt::format("x_{},r{}", c_id, l));
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
void Solidifier::Impl::_setupObjective() {
  reusex::trace("Create objective");

  auto area = _cc->property_map<Fd, double>("f:area");
  auto volume = _cc->property_map<Cd, double>("c:volume");
  auto f_sp = _cc->property_map<Fd, double>("f:support_probability");
  auto c_rp =
      _cc->property_map<Cd, std::vector<double>>("c:room_probabilities");

  Linear_objective *obj = solver.create_objective(Linear_objective::MINIMIZE);
  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    // R_c
    for (size_t l = 0; l < _cc->n_rooms + 1; ++l) {
      const double weight = c_rp[*cit][l] * volume[*cit];
#if SolverDebug
      reusex::trace(
          "Cell: {}, Label: {}, P_C: {:.3f}, volume: {:.3f} => weight: {:.3f}",
          std::get<CellData>((*_cc)[*cit].data).id, l, c_rp[*cit][l],
          volume[*cit], weight);
#endif
      Variable *x_cr = _room_variables[*cit][l];
      obj->add_coefficient(x_cr, -weight);
    }
  }

  for (auto fit = _cc->faces_between_cells_begin();
       fit != _cc->faces_between_cells_end(); ++fit) {
    auto cit_a = _cc->get_a(*fit);
    auto cit_b = _cc->get_b(*fit);

    auto [Wa, Wb] = getWallIds(cit_a, cit_b);

#if SolverDebug
    reusex::trace("Face {} A:{} B:{} Wa: [{}], Wb [{}]",
                        std::get<FaceData>((*_cc)[*fit].data).id,
                        std::get<CellData>((*_cc)[cit_a].data).id,
                        std::get<CellData>((*_cc)[cit_b].data).id,
                        fmt::join(Wa, ","), fmt::join(Wb, ","));
#endif

    const double weight = (1 - f_sp[*fit]) * area[*fit];

    // W_{F_b}: walls in cb but not in ca
#if SolverDebug
    reusex::trace("W_Fb");
#endif
    for (auto id : Wb - Wa) {
      Variable *Xcbw = _wall_variables[cit_b][id];
      obj->add_coefficient(Xcbw, alpha * weight);
#if SolverDebug
      reusex::trace("Var {:<8}, alpha {:.3f}, weight: {:.3f}",
                          Xcbw->name(), alpha, weight);
#endif
    }

    // W_{F_i}: walls in both ca and cb
#if SolverDebug
    reusex::trace("W_Fi");
#endif
    for (auto id : Wa + Wb) {
      Variable *Xcbw = _wall_variables[cit_b][id];
      obj->add_coefficient(Xcbw, alpha * weight);
#if SolverDebug
      reusex::trace("Var {:<8}, alpha {:.3f}, weight: {:.3f}",
                          Xcbw->name(), alpha, weight);
#endif
      Variable *Xcaw = _wall_variables[cit_a][id];
      obj->add_coefficient(Xcaw, -alpha * weight);
#if SolverDebug
      reusex::trace("Var {:<8}, alpha {:.3f}, weight: {:.3f}",
                          Xcaw->name(), alpha, weight);
#endif
    }
  }
}

/** Setup all the MIP constraints
 */
void Solidifier::Impl::_setupConstraints() {
  reusex::trace("Create constraints");
  _c_1();
  _c_2();
  _c_3();
  _c_4();
  _c_5();
  _c_6();
  _c_7();
}

/** Constraint 1: Each cell must be assigned exactly one label from Ro
 *
 * $$\forall c \in C: \sum_{r \in R_0} x_{c,r} = 1$$
 */
void Solidifier::Impl::_c_1() {
  reusex::trace("Create constraint 1");
  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    Linear_constraint *c1 = solver.create_constraint(1, 1, "c1");
    for (size_t l = 0; l < _cc->n_rooms + 1; ++l) {
      Variable *Xcr = _room_variables[*cit][l];
      c1->add_coefficient(Xcr, 1);
#if SolverDebug
      reusex::trace("Cell: {}, Label: {}, 1",
                          std::get<CellData>((*_cc)[*cit].data).id, l);
#endif
    }
  }
}

/** Constraint 2: At boundary faces of room interiors, the room label
 * may only occur on the positive side of the face
 *
 * $$\forall f_{c_a,c_b} \in F \forall r \in R: x_{c_a,r} - x_{c_b,r} \geqslant
 * 0$$
 */
void Solidifier::Impl::_c_2() {
  reusex::trace("Create constraint 2");
  for (auto fit = _cc->faces_between_cells_begin();
       fit != _cc->faces_between_cells_end(); ++fit) {
    for (size_t r = 1; r < _cc->n_rooms + 1; ++r) {
      Variable *x_car = _room_variables[_cc->get_a(*fit)][r];
      Variable *x_cbr = _room_variables[_cc->get_b(*fit)][r];
      Linear_constraint *c2 =
          solver.create_constraint(0, Linear_constraint::infinity(), "c2");
      c2->add_coefficient(x_car, 1);
      c2->add_coefficient(x_cbr, -1);
#if SolverDebug
      reusex::trace("Face: {}, Room: {}, A:{} - B:{}",
                          std::get<FaceData>((*_cc)[*fit].data).id, r,
                          x_car->name(), x_cbr->name());
#endif
    }
  }
}

/** Constraint 3: Wall labels may only occur in cells which are assigned
 * the outside label
 *
 * $$\forall c \in C \forall \forall w \in W_c: x_{c,w} \leqslant x_{c,o}$$
 */
void Solidifier::Impl::_c_3() {
  reusex::trace("Create constraint 3");
  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    for (size_t w = 0; w < _cc->n_walls; ++w) {
      Variable *Xcw = _wall_variables[*cit][w];
      Variable *Xco = _room_variables[*cit][0];
      Linear_constraint *c3 =
          solver.create_constraint(-Linear_constraint::infinity(), 0, "c3");
      c3->add_coefficient(Xcw, 1);
      c3->add_coefficient(Xco, -1);
#if SolverDebug
      reusex::trace("Cell: {}, Wall: {} - {} <= 0",
                          std::get<CellData>((*_cc)[*cit].data).id, Xcw->name(),
                          Xco->name());
#endif
    }
  }
}

/** Constraint 4: The boundary faces of room interiors must also be the boundary
 * faces of an active wall
 * $$\forall f_{c_a,c_b} \in F : \sum_{w \in W_{\bar{c_a}, c_b} x_{c_b,w}
 * \geqslant x_{c_b,o} - x_{c_a,o}$$
 */
void Solidifier::Impl::_c_4() {
  reusex::trace("Create constraint 4");
  for (auto fit = _cc->faces_between_cells_begin();
       fit != _cc->faces_between_cells_end(); ++fit) {
    auto cit_a = _cc->get_a(*fit);
    auto cit_b = _cc->get_b(*fit);

    auto [W_a, W_b] = getWallIds(cit_a, cit_b);

    Linear_constraint *c4 =
        solver.create_constraint(0, Linear_constraint::infinity(), "c4");

    for (auto id : W_b - W_a) {
      Variable *Xcbw = _wall_variables[cit_b][id];
      c4->add_coefficient(Xcbw, 1);
    }

    Variable *Xcao = _room_variables[cit_a][0];
    Variable *Xcbo = _room_variables[cit_b][0];

    c4->add_coefficient(Xcbo, -1);
    c4->add_coefficient(Xcao, 1);

#if SolverDebug
    reusex::trace("Face: {}, (Wb - Wa) x_cb[{}] - {} + {} >= 0",
                        std::get<FaceData>((*_cc)[*fit].data).id,
                        fmt::join(W_b - W_a, ", "), Xcbo->name(), Xcao->name());
#endif
  }
}

/** Constraint 5: At wall boundaries that are inner faces, the wall label
 * must be on the negative side of the respective faces
 *
 * $$\forall f_{c_a,c_b} \in F \forall w \in W_{c_a, c_b}: x_{sb,w} -
 * x_{c_a,w} \geqslant 0$$
 */
void Solidifier::Impl::_c_5() {
  reusex::trace("Create constraint 5");
  for (auto fit = _cc->faces_between_cells_begin();
       fit != _cc->faces_between_cells_end(); ++fit) {
    auto cit_a = _cc->get_a(*fit);
    auto cit_b = _cc->get_b(*fit);

    auto [W_a, W_b] = getWallIds(cit_a, cit_b);

    for (auto id : W_a + W_b) {
      Variable *Xcaw = _wall_variables[cit_a][id];
      Variable *Xcbw = _wall_variables[cit_b][id];
      Linear_constraint *c5 =
          solver.create_constraint(0, Linear_constraint::infinity(), "c5");
      c5->add_coefficient(Xcbw, 1);
      c5->add_coefficient(Xcaw, -1);
#if SolverDebug
      reusex::trace("Face: {}, Wall: {}, {} - {} >= 0",
                          std::get<FaceData>((*_cc)[*fit].data).id, id,
                          Xcbw->name(), Xcaw->name());
#endif
    }
  }
}

/** Constraint 6: A wall may end at an inner face only if this face is a
 * boundary face of at least one other active wall
 *
 * $$\forall f_{c_a,c_b} \in F \forall w \in W_{c_a, c_b}: \sum_{w' \in
 * W_{\bar{c_a},c_b}} x_{c_b,w'} \geqslant x_{c_b,w}-x_{c_a,w}$$
 */
void Solidifier::Impl::_c_6() {
  reusex::trace("Create constraint 6");
  for (auto fit = _cc->faces_between_cells_begin();
       fit != _cc->faces_between_cells_end(); ++fit) {
    auto cit_a = _cc->get_a(*fit);
    auto cit_b = _cc->get_b(*fit);

    auto [W_a, W_b] = getWallIds(cit_a, cit_b);

    for (auto id : W_a + W_b) {
      Linear_constraint *c6 =
          solver.create_constraint(0, Linear_constraint::infinity(), "c6");

      Variable *Xcaw = _wall_variables[cit_a][id];
      Variable *Xcbw = _wall_variables[cit_b][id];

      c6->add_coefficient(Xcbw, -1);
      c6->add_coefficient(Xcaw, 1);

      for (auto id2 : W_b - W_a) {
        Variable *Xcbw_ = _wall_variables[cit_b][id2];
        c6->add_coefficient(Xcbw_, 1);
      }

#if SolverDebug
      reusex::trace(
          "Face: {}, Wall: {}, sum(Wb - Wa) x_cb[{}] - {} + {} >= 0 ",
          std::get<FaceData>((*_cc)[*fit].data).id, fmt::join(W_b - W_a, ", "),
          id, Xcbw->name(), Xcaw->name());
#endif
    }
  }
}

/** Constraint 7: At boundary faces of outside area, the outside label
 * may only occur on the negative side of the separating face
 *
 * $$\forall f_{c_a,c_b} \in F : x_{c_a,o} - x_{c_b,o} \leqslant 0$$
 */
void Solidifier::Impl::_c_7() {
  reusex::trace("Create constraint 7");
  for (auto fit = _cc->faces_between_cells_begin();
       fit != _cc->faces_between_cells_end(); ++fit) {
    auto cit_a = _cc->get_a(*fit);
    auto cit_b = _cc->get_b(*fit);

    Variable *Xcao = _room_variables[cit_a][0];
    Variable *Xcbo = _room_variables[cit_b][0];

    Linear_constraint *c7 =
        solver.create_constraint(-Linear_constraint::infinity(), 0, "c7");
    c7->add_coefficient(Xcao, 1);
    c7->add_coefficient(Xcbo, -1);
#if SolverDebug
    reusex::trace("Face: {}, {} - {} <= 0",
                        std::get<FaceData>((*_cc)[*fit].data).id, Xcao->name(),
                        Xcbo->name());
#endif
  }
}

} // namespace reusex::geometry
