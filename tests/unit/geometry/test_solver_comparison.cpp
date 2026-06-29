// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Force-enable both traits headers in this comparison test, regardless of
// which solver the library was built with. HiGHS is always linked; cuOpt
// is only available when the build selected it as the Solidifier solver.
#ifdef USE_CUOPT

#ifndef USE_HIGHS
#define USE_HIGHS
#endif

#include "core/logging.hpp"
#include <CGAL/HiGHS_mixed_integer_program_traits.h>
#include <CGAL/cuOpt_mixed_integer_program_traits.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using namespace Catch::Matchers;

using HiGHS_Solver = CGAL::HiGHS_mixed_integer_program_traits<double>;
using cuOpt_Solver = CGAL::cuOpt_mixed_integer_program_traits<double>;

namespace {

template <typename Solver> struct Result {
  bool solved = false;
  std::vector<double> values;
  double objective = 0.0;
};

// Build and solve a small LP / MIP on the given solver and return the result.
// The builder lambda receives the solver instance and is responsible for
// adding variables, the objective, and constraints. The objective coefficients
// passed to the builder are captured here so we can compute the objective
// value from the returned solution (CGAL traits don't expose obj value).
template <typename Solver, typename Builder>
Result<Solver> run(Builder &&build) {
  Solver solver;
  std::vector<double> obj_coeffs; // index aligned with variable index
  bool minimize = true;

  build(solver, obj_coeffs, minimize);

  Result<Solver> r;
  r.solved = solver.solve();
  if (!r.solved) return r;

  for (std::size_t i = 0; i < obj_coeffs.size(); ++i) {
    double v = static_cast<double>(solver.variables()[i]->solution_value());
    r.values.push_back(v);
    r.objective += obj_coeffs[i] * v;
  }
  return r;
}

// LP 1: minimize x + y s.t. x + y >= 1, x,y in [0, +inf).
// Optimal: any point on x+y=1, objective = 1.
template <typename Solver>
Result<Solver> solve_lp_continuous() {
  return run<Solver>([](Solver &s, std::vector<double> &obj, bool &min) {
    auto *x = s.create_variable(Solver::Variable::CONTINUOUS, 0.0, 1e30, "x");
    auto *y = s.create_variable(Solver::Variable::CONTINUOUS, 0.0, 1e30, "y");
    obj = {1.0, 1.0};
    min = true;
    auto *o = s.create_objective(Solver::Linear_objective::MINIMIZE);
    o->add_coefficient(x, 1.0);
    o->add_coefficient(y, 1.0);
    auto *c = s.create_constraint(1.0, 1e30, "sum_ge_1");
    c->add_coefficient(x, 1.0);
    c->add_coefficient(y, 1.0);
  });
}

// LP 2: minimize -2*x1 - x2 s.t. x1+x2 <= 3, x1 <= 2, x1,x2 >= 0.
// Optimal: x1=2, x2=1, objective=-5.
template <typename Solver>
Result<Solver> solve_lp_multi_constraint() {
  return run<Solver>([](Solver &s, std::vector<double> &obj, bool &min) {
    auto *x1 = s.create_variable(Solver::Variable::CONTINUOUS, 0.0, 1e30, "x1");
    auto *x2 = s.create_variable(Solver::Variable::CONTINUOUS, 0.0, 1e30, "x2");
    obj = {-2.0, -1.0};
    min = true;
    auto *o = s.create_objective(Solver::Linear_objective::MINIMIZE);
    o->add_coefficient(x1, -2.0);
    o->add_coefficient(x2, -1.0);
    auto *c1 = s.create_constraint(-1e30, 3.0, "capacity");
    c1->add_coefficient(x1, 1.0);
    c1->add_coefficient(x2, 1.0);
    auto *c2 = s.create_constraint(-1e30, 2.0, "x1_bound");
    c2->add_coefficient(x1, 1.0);
  });
}

// MIP: maximize 3*x1 + 2*x2 + x3 s.t. x1+x2+x3 = 1, x_i in {0,1}.
// Optimal: x1=1, others=0, objective=3.
template <typename Solver> Result<Solver> solve_mip_binary() {
  return run<Solver>([](Solver &s, std::vector<double> &obj, bool &min) {
    auto *x1 = s.create_variable(Solver::Variable::BINARY, 0, 1, "x1");
    auto *x2 = s.create_variable(Solver::Variable::BINARY, 0, 1, "x2");
    auto *x3 = s.create_variable(Solver::Variable::BINARY, 0, 1, "x3");
    obj = {3.0, 2.0, 1.0};
    min = false;
    auto *o = s.create_objective(Solver::Linear_objective::MAXIMIZE);
    o->add_coefficient(x1, 3.0);
    o->add_coefficient(x2, 2.0);
    o->add_coefficient(x3, 1.0);
    auto *c = s.create_constraint(1.0, 1.0, "assign_one");
    c->add_coefficient(x1, 1.0);
    c->add_coefficient(x2, 1.0);
    c->add_coefficient(x3, 1.0);
  });
}

// LP with a true two-sided range constraint: minimize x s.t. 1 <= x <= 3,
// x in [0, 10]. Optimal: x = 1, objective = 1.
template <typename Solver> Result<Solver> solve_lp_range_constraint() {
  return run<Solver>([](Solver &s, std::vector<double> &obj, bool &min) {
    auto *x = s.create_variable(Solver::Variable::CONTINUOUS, 0.0, 10.0, "x");
    obj = {1.0};
    min = true;
    auto *o = s.create_objective(Solver::Linear_objective::MINIMIZE);
    o->add_coefficient(x, 1.0);
    auto *c = s.create_constraint(1.0, 3.0, "range");
    c->add_coefficient(x, 1.0);
  });
}

// Infeasible: minimize x s.t. x >= 2, x <= 1.
template <typename Solver> Result<Solver> solve_infeasible() {
  return run<Solver>([](Solver &s, std::vector<double> &obj, bool &min) {
    auto *x = s.create_variable(Solver::Variable::CONTINUOUS, 0.0, 1e30, "x");
    obj = {1.0};
    min = true;
    auto *o = s.create_objective(Solver::Linear_objective::MINIMIZE);
    o->add_coefficient(x, 1.0);
    auto *c1 = s.create_constraint(2.0, 1e30, "lb");
    c1->add_coefficient(x, 1.0);
    auto *c2 = s.create_constraint(-1e30, 1.0, "ub");
    c2->add_coefficient(x, 1.0);
  });
}

} // namespace

TEST_CASE("HiGHS vs cuOpt: continuous LP",
          "[mip][solver_compare][gpu]") {
  auto h = solve_lp_continuous<HiGHS_Solver>();
  auto c = solve_lp_continuous<cuOpt_Solver>();

  REQUIRE(h.solved);
  REQUIRE(c.solved);
  reusex::core::info(
      "continuous LP — HiGHS obj={} ({},{})  cuOpt obj={} ({},{})", h.objective,
      h.values[0], h.values[1], c.objective, c.values[0], c.values[1]);

  REQUIRE_THAT(h.objective, WithinAbs(1.0, 1e-4));
  REQUIRE_THAT(c.objective, WithinAbs(1.0, 1e-4));
  REQUIRE_THAT(h.objective, WithinAbs(c.objective, 1e-4));
}

TEST_CASE("HiGHS vs cuOpt: multi-constraint LP",
          "[mip][solver_compare][gpu]") {
  auto h = solve_lp_multi_constraint<HiGHS_Solver>();
  auto c = solve_lp_multi_constraint<cuOpt_Solver>();

  REQUIRE(h.solved);
  REQUIRE(c.solved);
  reusex::core::info("multi-constraint LP — HiGHS x1={} x2={} obj={}; "
                     "cuOpt x1={} x2={} obj={}",
                     h.values[0], h.values[1], h.objective, c.values[0],
                     c.values[1], c.objective);

  REQUIRE_THAT(h.objective, WithinAbs(-5.0, 1e-4));
  REQUIRE_THAT(c.objective, WithinAbs(-5.0, 1e-4));
  REQUIRE_THAT(h.values[0], WithinAbs(c.values[0], 1e-4));
  REQUIRE_THAT(h.values[1], WithinAbs(c.values[1], 1e-4));
}

TEST_CASE("HiGHS vs cuOpt: binary MIP", "[mip][solver_compare][gpu]") {
  auto h = solve_mip_binary<HiGHS_Solver>();
  auto c = solve_mip_binary<cuOpt_Solver>();

  REQUIRE(h.solved);
  REQUIRE(c.solved);
  reusex::core::info("binary MIP — HiGHS ({},{},{}) obj={}; "
                     "cuOpt ({},{},{}) obj={}",
                     h.values[0], h.values[1], h.values[2], h.objective,
                     c.values[0], c.values[1], c.values[2], c.objective);

  // Both solvers should pick x1 (highest coefficient).
  REQUIRE_THAT(h.objective, WithinAbs(3.0, 1e-6));
  REQUIRE_THAT(c.objective, WithinAbs(3.0, 1e-6));
  for (std::size_t i = 0; i < 3; ++i) {
    REQUIRE_THAT(h.values[i], WithinAbs(c.values[i], 1e-6));
  }
}

TEST_CASE("HiGHS vs cuOpt: two-sided range constraint",
          "[mip][solver_compare][gpu]") {
  // Exercises the cuOpt range-splitting path (1 <= x <= 3 → two cuOpt rows).
  auto h = solve_lp_range_constraint<HiGHS_Solver>();
  auto c = solve_lp_range_constraint<cuOpt_Solver>();

  REQUIRE(h.solved);
  REQUIRE(c.solved);
  reusex::core::info(
      "range LP — HiGHS x={} obj={}; cuOpt x={} obj={}", h.values[0],
      h.objective, c.values[0], c.objective);

  REQUIRE_THAT(h.objective, WithinAbs(1.0, 1e-4));
  REQUIRE_THAT(c.objective, WithinAbs(1.0, 1e-4));
  REQUIRE_THAT(h.values[0], WithinAbs(c.values[0], 1e-4));
}

TEST_CASE("HiGHS vs cuOpt: infeasible detection",
          "[mip][solver_compare][gpu]") {
  auto h = solve_infeasible<HiGHS_Solver>();
  auto c = solve_infeasible<cuOpt_Solver>();

  REQUIRE_FALSE(h.solved);
  REQUIRE_FALSE(c.solved);
}

#endif // USE_CUOPT
