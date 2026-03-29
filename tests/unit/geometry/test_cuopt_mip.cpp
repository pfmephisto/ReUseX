// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifdef USE_CUOPT

#include <CGAL/cuOpt_mixed_integer_program_traits.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <spdlog/spdlog.h>

#include <cmath>

using namespace Catch::Matchers;
using MIP_Solver = CGAL::cuOpt_mixed_integer_program_traits<double>;
using Variable = MIP_Solver::Variable;
using Linear_objective = MIP_Solver::Linear_objective;
using Linear_constraint = MIP_Solver::Linear_constraint;

/** Test cuOpt CGAL traits with a simple continuous LP
 *
 * Problem: minimize x + y
 *          subject to: x + y >= 1
 *                      x, y >= 0
 *
 * Optimal solution: objective value = 1.0
 */
TEST_CASE("cuOpt CGAL traits: continuous LP", "[cuopt][mip][gpu]") {
  MIP_Solver solver;

  Variable* x = solver.create_variable(Variable::CONTINUOUS, 0.0, 1e30, "x");
  Variable* y = solver.create_variable(Variable::CONTINUOUS, 0.0, 1e30, "y");

  Linear_objective* obj = solver.create_objective(Linear_objective::MINIMIZE);
  obj->add_coefficient(x, 1.0);
  obj->add_coefficient(y, 1.0);

  // x + y >= 1  (lb=1, ub=inf)
  Linear_constraint* c = solver.create_constraint(1.0, 1e30, "c_sum");
  c->add_coefficient(x, 1.0);
  c->add_coefficient(y, 1.0);

  bool solved = solver.solve();
  REQUIRE(solved);

  double obj_val = x->solution_value() + y->solution_value();
  spdlog::info("cuOpt LP: x={}, y={}, obj={}", x->solution_value(),
               y->solution_value(), obj_val);

  REQUIRE_THAT(obj_val, WithinAbs(1.0, 1e-4));
  REQUIRE(x->solution_value() >= -1e-6);
  REQUIRE(y->solution_value() >= -1e-6);
}

/** Test cuOpt CGAL traits with binary variables (MIP)
 *
 * This mirrors the Solidifier's usage: binary assignment variables
 * with equality constraints.
 *
 * Problem: maximize 3*x1 + 2*x2 + x3
 *          subject to: x1 + x2 + x3 = 1   (assign exactly one)
 *                      x1, x2, x3 in {0,1}
 *
 * Optimal: x1=1, x2=0, x3=0, objective=3
 */
TEST_CASE("cuOpt CGAL traits: binary MIP", "[cuopt][mip][gpu]") {
  MIP_Solver solver;

  Variable* x1 = solver.create_variable(Variable::BINARY, 0, 1, "x1");
  Variable* x2 = solver.create_variable(Variable::BINARY, 0, 1, "x2");
  Variable* x3 = solver.create_variable(Variable::BINARY, 0, 1, "x3");

  Linear_objective* obj = solver.create_objective(Linear_objective::MAXIMIZE);
  obj->add_coefficient(x1, 3.0);
  obj->add_coefficient(x2, 2.0);
  obj->add_coefficient(x3, 1.0);

  // x1 + x2 + x3 = 1  (exactly one label, matching Solidifier constraint 1)
  Linear_constraint* c = solver.create_constraint(1.0, 1.0, "assign_one");
  c->add_coefficient(x1, 1.0);
  c->add_coefficient(x2, 1.0);
  c->add_coefficient(x3, 1.0);

  bool solved = solver.solve();
  REQUIRE(solved);

  spdlog::info("cuOpt MIP: x1={}, x2={}, x3={}", x1->solution_value(),
               x2->solution_value(), x3->solution_value());

  // Exactly one variable should be 1
  double sum = x1->solution_value() + x2->solution_value() + x3->solution_value();
  REQUIRE_THAT(sum, WithinAbs(1.0, 1e-6));

  // Each variable should be binary (0 or 1)
  for (auto* v : {x1, x2, x3}) {
    bool is_zero = std::abs(v->solution_value()) < 1e-6;
    bool is_one = std::abs(v->solution_value() - 1.0) < 1e-6;
    REQUIRE((is_zero || is_one));
  }

  // Optimal assignment: x1=1 (coefficient 3 is largest)
  REQUIRE_THAT(x1->solution_value(), WithinAbs(1.0, 1e-6));
}

/** Test cuOpt with multiple constraints (closer to Solidifier pattern)
 *
 * Problem: minimize -2*x1 - x2
 *          subject to: x1 + x2 <= 3    (capacity)
 *                      x1 <= 2         (individual bound)
 *                      x1, x2 >= 0    (continuous)
 *
 * Optimal: x1=2, x2=1, objective=-5
 */
TEST_CASE("cuOpt CGAL traits: multi-constraint LP", "[cuopt][mip][gpu]") {
  MIP_Solver solver;

  Variable* x1 = solver.create_variable(Variable::CONTINUOUS, 0.0, 1e30, "x1");
  Variable* x2 = solver.create_variable(Variable::CONTINUOUS, 0.0, 1e30, "x2");

  Linear_objective* obj = solver.create_objective(Linear_objective::MINIMIZE);
  obj->add_coefficient(x1, -2.0);
  obj->add_coefficient(x2, -1.0);

  // x1 + x2 <= 3
  Linear_constraint* c1 = solver.create_constraint(-1e30, 3.0, "capacity");
  c1->add_coefficient(x1, 1.0);
  c1->add_coefficient(x2, 1.0);

  // x1 <= 2
  Linear_constraint* c2 = solver.create_constraint(-1e30, 2.0, "x1_bound");
  c2->add_coefficient(x1, 1.0);

  bool solved = solver.solve();
  REQUIRE(solved);

  spdlog::info("cuOpt multi-constraint LP: x1={}, x2={}", x1->solution_value(),
               x2->solution_value());

  REQUIRE_THAT(x1->solution_value(), WithinAbs(2.0, 1e-4));
  REQUIRE_THAT(x2->solution_value(), WithinAbs(1.0, 1e-4));

  double obj_val = -2.0 * x1->solution_value() - x2->solution_value();
  REQUIRE_THAT(obj_val, WithinAbs(-5.0, 1e-4));
}

/** Test cuOpt handles infeasible problems correctly
 *
 * Problem: minimize x
 *          subject to: x >= 2
 *                      x <= 1
 *          (infeasible: no x satisfies both)
 */
TEST_CASE("cuOpt CGAL traits: infeasible problem", "[cuopt][mip][gpu]") {
  MIP_Solver solver;

  Variable* x = solver.create_variable(Variable::CONTINUOUS, 0.0, 1e30, "x");

  Linear_objective* obj = solver.create_objective(Linear_objective::MINIMIZE);
  obj->add_coefficient(x, 1.0);

  // x >= 2
  Linear_constraint* c1 = solver.create_constraint(2.0, 1e30, "lb");
  c1->add_coefficient(x, 1.0);

  // x <= 1
  Linear_constraint* c2 = solver.create_constraint(-1e30, 1.0, "ub");
  c2->add_coefficient(x, 1.0);

  bool solved = solver.solve();
  REQUIRE_FALSE(solved);

  spdlog::info("cuOpt correctly detected infeasible problem");
}

#endif // USE_CUOPT
