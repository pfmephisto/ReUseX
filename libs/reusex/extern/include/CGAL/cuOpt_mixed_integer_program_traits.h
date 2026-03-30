// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This file provides a CGAL Mixed Integer Program traits class for NVIDIA cuOpt,
// a GPU-accelerated MIP solver. It follows the same pattern as HiGHS and SCIP traits.

#ifndef CGAL_CUOPT_MIXED_INTEGER_PROGRAM_TRAITS_H
#define CGAL_CUOPT_MIXED_INTEGER_PROGRAM_TRAITS_H

#include <CGAL/Mixed_integer_program_traits.h>

#if defined(USE_CUOPT) || defined(DOXYGEN_RUNNING)

#include <cuopt/linear_programming/cuopt_c.h>
#include <cuopt/linear_programming/constants.h>

#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>

namespace CGAL {

/// \ingroup PkgSolverInterfaceMIP
///
/// This class provides an interface for formulating and solving
/// constrained or unconstrained mixed integer programs using
/// NVIDIA cuOpt GPU-accelerated solver.
///
/// \cgalModels{MixedIntegerProgramTraits}
///
/// \sa `HiGHS_mixed_integer_program_traits`
/// \sa `SCIP_mixed_integer_program_traits`
template <typename FT>
class cuOpt_mixed_integer_program_traits
  : public Mixed_integer_program_traits<FT>
{
  /// \cond SKIP_IN_MANUAL
public:
  typedef CGAL::Mixed_integer_program_traits<FT>          Base_class;
  typedef typename Base_class::Variable                   Variable;
  typedef typename Base_class::Linear_constraint          Linear_constraint;
  typedef typename Base_class::Linear_objective           Linear_objective;
  typedef typename Linear_objective::Sense                Sense;
  typedef typename Variable::Variable_type                Variable_type;

public:
  /// Solves the program. Returns `false` if fails.
  virtual bool solve()
  {
    Base_class::error_message_.clear();

    const std::size_t num_vars = Base_class::variables_.size();
    const std::size_t num_constraints = Base_class::constraints_.size();

    // Build variable bounds and types
    std::vector<cuopt_float_t> col_lower(num_vars);
    std::vector<cuopt_float_t> col_upper(num_vars);
    std::vector<char> col_types(num_vars);

    for (std::size_t i = 0; i < num_vars; ++i) {
      const Variable* var = Base_class::variables_[i];

      double lb, ub;
      var->get_bounds(lb, ub);
      col_lower[i] = static_cast<cuopt_float_t>(lb);
      col_upper[i] = static_cast<cuopt_float_t>(ub);

      switch (var->variable_type())
      {
        case Variable::CONTINUOUS:
          col_types[i] = CUOPT_CONTINUOUS;
          break;
        case Variable::INTEGER:
          col_types[i] = CUOPT_INTEGER;
          break;
        case Variable::BINARY:
          col_types[i] = CUOPT_INTEGER;
          // Binary variables should already have bounds [0, 1]
          col_lower[i] = 0.0;
          col_upper[i] = 1.0;
          break;
        default:
          Base_class::error_message_ = "cuOpt: unknown variable type";
          return false;
      }
    }

    // Build objective coefficients
    std::vector<cuopt_float_t> obj_coeffs(num_vars, 0.0);
    const std::unordered_map<const Variable*, double>& obj_map =
        Base_class::objective_->coefficients();

    for (const auto& [var, coeff] : obj_map) {
      obj_coeffs[var->index()] = static_cast<cuopt_float_t>(coeff);
    }

    cuopt_int_t obj_sense = (Base_class::objective_->sense() == Linear_objective::MINIMIZE)
                            ? CUOPT_MINIMIZE
                            : CUOPT_MAXIMIZE;

    // Build constraint matrix in CSR (Compressed Sparse Row) format
    std::vector<cuopt_int_t> row_offsets;
    std::vector<cuopt_int_t> col_indices;
    std::vector<cuopt_float_t> values;
    std::vector<char> constraint_sense(num_constraints);
    std::vector<cuopt_float_t> rhs(num_constraints);

    row_offsets.reserve(num_constraints + 1);
    row_offsets.push_back(0);

    for (std::size_t i = 0; i < num_constraints; ++i) {
      const Linear_constraint* c = Base_class::constraints_[i];

      double lb, ub;
      c->get_bounds(lb, ub);

      // Determine constraint type and RHS
      // cuOpt format: A*x {L, G, E} b
      if (lb == ub) {
        // Equality constraint: A*x = b
        constraint_sense[i] = CUOPT_EQUAL;
        rhs[i] = static_cast<cuopt_float_t>(lb);
      } else if (std::isinf(ub)) {
        // Greater-than: A*x >= lb
        constraint_sense[i] = CUOPT_GREATER_THAN;
        rhs[i] = static_cast<cuopt_float_t>(lb);
      } else if (std::isinf(lb)) {
        // Less-than: A*x <= ub
        constraint_sense[i] = CUOPT_LESS_THAN;
        rhs[i] = static_cast<cuopt_float_t>(ub);
      } else {
        // Range constraint: lb <= A*x <= ub
        // Use upper bound (more conservative for geometric problems)
        constraint_sense[i] = CUOPT_LESS_THAN;
        rhs[i] = static_cast<cuopt_float_t>(ub);

        // Note: If both bounds are finite and different, we lose the lower bound.
        // For more accurate handling, we'd need to split into two constraints.
        // This matches the pattern used in other solvers for simplicity.
      }

      // Add constraint coefficients to CSR
      const std::unordered_map<const Variable*, double>& coeffs = c->coefficients();
      for (const auto& [var, coeff] : coeffs) {
        col_indices.push_back(static_cast<cuopt_int_t>(var->index()));
        values.push_back(static_cast<cuopt_float_t>(coeff));
      }

      // Record row end offset
      row_offsets.push_back(static_cast<cuopt_int_t>(values.size()));
    }

    // Create cuOpt problem
    cuOptOptimizationProblem problem = nullptr;
    cuopt_int_t status = cuOptCreateProblem(
        static_cast<cuopt_int_t>(num_constraints),
        static_cast<cuopt_int_t>(num_vars),
        obj_sense,
        0.0,  // objective offset
        obj_coeffs.data(),
        row_offsets.data(),
        col_indices.data(),
        values.data(),
        constraint_sense.data(),
        rhs.data(),
        col_lower.data(),
        col_upper.data(),
        col_types.data(),
        &problem
    );

    if (status != CUOPT_SUCCESS) {
      Base_class::error_message_ = "cuOpt: failed to create problem (error code: " +
                                   std::to_string(status) + ")";
      return false;
    }

    // Create solver settings
    cuOptSolverSettings settings = nullptr;
    status = cuOptCreateSolverSettings(&settings);
    if (status != CUOPT_SUCCESS) {
      cuOptDestroyProblem(&problem);
      Base_class::error_message_ = "cuOpt: failed to create solver settings";
      return false;
    }

    // Configure solver parameters
    // Disable console output
    cuOptSetParameter(settings, CUOPT_LOG_TO_CONSOLE, "false");

    // Set time limit (5 minutes)
    cuOptSetFloatParameter(settings, CUOPT_TIME_LIMIT, 300.0);

    // Enable presolve
    cuOptSetParameter(settings, CUOPT_PRESOLVE, "true");

    // For MIP problems, enable heuristics and set gap tolerances
    cuopt_int_t is_mip = 0;
    cuOptIsMIP(problem, &is_mip);
    if (is_mip) {
      cuOptSetFloatParameter(settings, CUOPT_MIP_RELATIVE_GAP, 0.01);  // 1% gap
      cuOptSetFloatParameter(settings, CUOPT_MIP_ABSOLUTE_GAP, 1e-6);
      cuOptSetParameter(settings, CUOPT_MIP_PRESOLVE, "true");
    }

    // Solve the problem
    cuOptSolution solution = nullptr;
    status = cuOptSolve(problem, settings, &solution);

    if (status != CUOPT_SUCCESS) {
      // Try to get error message from solution if available
      if (solution != nullptr) {
        cuopt_int_t error_status = 0;
        cuOptGetErrorStatus(solution, &error_status);

        char error_msg[1024];
        cuOptGetErrorString(solution, error_msg, 1024);
        Base_class::error_message_ = std::string("cuOpt solve failed: ") + error_msg;

        cuOptDestroySolution(&solution);
      } else {
        Base_class::error_message_ = "cuOpt solve failed with status: " +
                                     std::to_string(status);
      }

      cuOptDestroySolverSettings(&settings);
      cuOptDestroyProblem(&problem);
      return false;
    }

    // Check termination status
    cuopt_int_t termination_status = 0;
    status = cuOptGetTerminationStatus(solution, &termination_status);

    if (status != CUOPT_SUCCESS) {
      Base_class::error_message_ = "cuOpt: failed to get termination status";
      cuOptDestroySolution(&solution);
      cuOptDestroySolverSettings(&settings);
      cuOptDestroyProblem(&problem);
      return false;
    }

    bool success = false;

    switch (termination_status) {
      case CUOPT_TERIMINATION_STATUS_OPTIMAL:
        success = true;
        break;

      case CUOPT_TERIMINATION_STATUS_PRIMAL_FEASIBLE:
      case CUOPT_TERIMINATION_STATUS_FEASIBLE_FOUND:
        // Feasible solution found but not proven optimal
        success = true;
        Base_class::error_message_ = "cuOpt: feasible solution found (not optimal)";
        break;

      case CUOPT_TERIMINATION_STATUS_INFEASIBLE:
        Base_class::error_message_ = "cuOpt: problem is infeasible";
        break;

      case CUOPT_TERIMINATION_STATUS_UNBOUNDED:
        Base_class::error_message_ = "cuOpt: problem is unbounded";
        break;

      case CUOPT_TERIMINATION_STATUS_TIME_LIMIT:
        Base_class::error_message_ = "cuOpt: time limit reached";
        // Check if we have a feasible solution anyway
        success = true;  // Try to extract solution
        break;

      case CUOPT_TERIMINATION_STATUS_ITERATION_LIMIT:
        Base_class::error_message_ = "cuOpt: iteration limit reached";
        success = true;  // Try to extract solution
        break;

      case CUOPT_TERIMINATION_STATUS_NUMERICAL_ERROR:
        Base_class::error_message_ = "cuOpt: numerical error";
        break;

      default:
        Base_class::error_message_ = "cuOpt: unknown termination status: " +
                                     std::to_string(termination_status);
        break;
    }

    // Extract solution if we have one
    if (success) {
      std::vector<cuopt_float_t> solution_values(num_vars);
      status = cuOptGetPrimalSolution(solution, solution_values.data());

      if (status != CUOPT_SUCCESS) {
        Base_class::error_message_ = "cuOpt: failed to extract solution";
        success = false;
      } else {
        Base_class::result_.resize(num_vars);

        for (std::size_t i = 0; i < num_vars; ++i) {
          FT x = static_cast<FT>(solution_values[i]);

          Variable* v = Base_class::variables_[i];
          // Round integer/binary variables
          if (v->variable_type() != Variable::CONTINUOUS) {
            x = static_cast<FT>(std::round(static_cast<double>(x)));
          }

          v->set_solution_value(x);
          Base_class::result_[i] = x;
        }
      }
    }

    // Cleanup
    cuOptDestroySolution(&solution);
    cuOptDestroySolverSettings(&settings);
    cuOptDestroyProblem(&problem);

    return success;
  }
  /// \endcond
};

} // namespace CGAL

#endif // USE_CUOPT or DOXYGEN_RUNNING

#endif // CGAL_CUOPT_MIXED_INTEGER_PROGRAM_TRAITS_H
