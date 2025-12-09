// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <ReUseX/geometry/CellComplex.hpp>

#ifdef CGAL_USE_SCIP
#include <CGAL/SCIP_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::SCIP_mixed_integer_program_traits<double>;
#elif defined(CGAL_USE_GLPK)
#include <CGAL/GLPK_mixed_integer_program_traits.h>
using MIP_Solver = CGAL::GLPK_mixed_integer_program_traits<double>;
#endif

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <tbb/concurrent_unordered_map.h>

namespace ReUseX::geometry {

class Solidifier {
    private:
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

    public:
  Solidifier() = delete;
  Solidifier(std::shared_ptr<const CellComplex> cc) : _cc(cc) {
    spdlog::trace("Solidifier created");
  }

  auto variables() { return solver.variables(); }

  std::optional<std::pair<std::unordered_map<Cd, int>,
                          std::unordered_map<Cd, std::set<int>>>>
  solve() {
    spdlog::trace("Start solving MIP");
    spdlog::stopwatch sw;

    _setupVariables();
    // for (auto var : solver.variables()) {
    //   spdlog::trace("Variable: {:<8}", var->name());
    // }
    _setupObjective();
    // auto obj = solver.objective();
    // for (auto var : solver.variables()) {
    //   auto coeff = obj->get_coefficient(var);
    //   auto offset = obj->offset();
    //   spdlog::trace(
    //       "Objective variable: {:<8} coeff: {:>8.3f} offset: {:>8.3f}",
    //       var->name(), coeff, offset);
    // }
    _setupConstraints();
    // for (auto con : solver.constraints()) {
    //   //   spdlog::trace("Constraint: {}", con->to_string());
    // }

    spdlog::trace("Start solving");

    // INFO: Solve
    if (!solver.solve()) {
      spdlog::error("Solving problem failed");
      return {};
    }

    auto room_label = std::unordered_map<Cd, int>{};
    auto wall_label = std::unordered_map<Cd, std::set<int>>{};
    for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
      for (size_t i = 0; i < _room_variables[*cit].size(); ++i) {
        if (_room_variables[*cit][i]->solution_value() > 0.5)
          room_label[*cit] = static_cast<int>(i);
      }
      wall_label[*cit] = std::set<int>{};
      for (size_t i = 0; i < _wall_variables[*cit].size(); ++i) {
        if (_wall_variables[*cit][i]->solution_value() > 0.5)
          wall_label[*cit].insert(static_cast<int>(i));
      }
    }
    spdlog::info("Solved MIP in {:>.3f} seconds", sw);

    return std::make_pair(room_label, wall_label);
  }

  auto toMesh(std::function<bool(const Cd)> filter)
      -> std::pair<Eigen::MatrixXd, Eigen::MatrixXi>;

    private:
  // TODO: Consider if the pre-compute functions should be part of this
  // class or be the resposibility of the function construcitn the cell
  // CellComplex. Some things, like are and volume are easier to compute
  // there, while the reconstruction logic would be nicer if self-contained
  // here. void _pre_compute_data(); void _set_f_area(Fd fit); void
  // _set_c_volume(Cd cit); void _set_fs_probability(Fd fit); void
  // _set_cr_probabilities(Cd cit);

  void _setupVariables();
  void _setupObjective();
  void _setupConstraints() {
    spdlog::trace("Create constraints");
    _c_1();
    _c_2();
    _c_3();
    _c_4();
    _c_5();
    _c_6();
    _c_7();
  }

  void _c_1();
  void _c_2();
  void _c_3();
  void _c_4();
  void _c_5();
  void _c_6();
  void _c_7();
};

} // namespace ReUseX::geometry
