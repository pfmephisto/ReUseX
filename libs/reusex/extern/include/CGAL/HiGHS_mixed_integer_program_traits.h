// Copyright (c) 2018  Liangliang Nan. All rights reserved.
// Copyright (c) 2026  HiGHS implementation. All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s) : Liangliang Nan (SCIP original)
//           : HiGHS port

#ifndef CGAL_HIGHS_MIXED_INTEGER_PROGRAM_TRAITS_H
#define CGAL_HIGHS_MIXED_INTEGER_PROGRAM_TRAITS_H

#include <CGAL/Mixed_integer_program_traits.h>

#if defined(CGAL_USE_HIGHS) || defined(DOXYGEN_RUNNING)

#include "Highs.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace CGAL {

/// \ingroup PkgSolverInterfaceMIP
///
/// This class provides an interface for formulating and solving
/// constrained or unconstrained mixed integer programs using
/// HiGHS (which must be available on the system).
///
/// \cgalModels{MixedIntegerProgramTraits}
///
/// \sa `GLPK_mixed_integer_program_traits`
/// \sa `SCIP_mixed_integer_program_traits`
template <typename FT>
class HiGHS_mixed_integer_program_traits
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

    Highs highs;

    // Disables HiGHS output to stdout
    highs.setOptionValue("log_to_console", false);
    highs.setOptionValue("output_flag", false);

    // Set the objective sense (we'll set it later, but initialize the model)
    bool minimize = (Base_class::objective_->sense() == Linear_objective::MINIMIZE);
    highs.setOptionValue("objective_sense",
                         minimize ? "minimize" : "maximize");

    // Create variables (columns)
    const std::size_t num_vars = Base_class::variables_.size();

    std::vector<double> col_lower(num_vars);
    std::vector<double> col_upper(num_vars);
    std::vector<double> col_cost(num_vars);
    std::vector<HighsVarType> col_integrality(num_vars);

    // Initialize objective coefficients to zero
    for (std::size_t i = 0; i < num_vars; ++i) {
      col_cost[i] = 0.0;
    }

    // Set variable bounds and types
    for (std::size_t i = 0; i < num_vars; ++i) {
      const Variable* var = Base_class::variables_[i];

      double lb, ub;
      var->get_bounds(lb, ub);
      col_lower[i] = lb;
      col_upper[i] = ub;

      switch (var->variable_type())
      {
        case Variable::CONTINUOUS:
          col_integrality[i] = HighsVarType::kContinuous;
          break;
        case Variable::INTEGER:
          col_integrality[i] = HighsVarType::kInteger;
          break;
        case Variable::BINARY:
          col_integrality[i] = HighsVarType::kInteger;
          col_lower[i] = 0.0;
          col_upper[i] = 1.0;
          break;
      }
    }

    // Set objective coefficients
    const std::unordered_map<const Variable*, double>& obj_coeffs =
        Base_class::objective_->coefficients();
    typename std::unordered_map<const Variable*, double>::const_iterator obj_it;
    for (obj_it = obj_coeffs.begin(); obj_it != obj_coeffs.end(); ++obj_it) {
      const Variable* var = obj_it->first;
      double coeff = obj_it->second;
      col_cost[var->index()] = coeff;
    }

    // Build constraint matrix in CSC (Compressed Sparse Column) format
    // First, count non-zeros per column to build the matrix structure
    const std::size_t num_constraints = Base_class::constraints_.size();

    std::vector<double> row_lower(num_constraints);
    std::vector<double> row_upper(num_constraints);

    // We'll build the constraint matrix in COO format first, then convert
    std::vector<int> a_row_indices;
    std::vector<int> a_col_indices;
    std::vector<double> a_values;

    for (std::size_t i = 0; i < num_constraints; ++i) {
      const Linear_constraint* c = Base_class::constraints_[i];

      double lb, ub;
      c->get_bounds(lb, ub);
      row_lower[i] = lb;
      row_upper[i] = ub;

      const std::unordered_map<const Variable*, double>& coeffs = c->coefficients();
      typename std::unordered_map<const Variable*, double>::const_iterator coeff_it;

      for (coeff_it = coeffs.begin(); coeff_it != coeffs.end(); ++coeff_it) {
        const Variable* var = coeff_it->first;
        double coeff = coeff_it->second;

        a_row_indices.push_back(static_cast<int>(i));
        a_col_indices.push_back(static_cast<int>(var->index()));
        a_values.push_back(coeff);
      }
    }

    // Add the model to HiGHS
    HighsStatus status = highs.addCols(
        static_cast<int>(num_vars),
        col_cost.data(),
        col_lower.data(),
        col_upper.data(),
        0,  // num_nz (we'll add constraints separately)
        nullptr,  // start
        nullptr,  // index
        nullptr   // value
    );

    if (status != HighsStatus::kOk) {
      Base_class::error_message_ = "failed to add variables to HiGHS model";
      return false;
    }

    // Set integrality
    for (std::size_t i = 0; i < num_vars; ++i) {
      status = highs.changeColIntegrality(static_cast<int>(i), col_integrality[i]);
      if (status != HighsStatus::kOk) {
        Base_class::error_message_ = "failed to set variable integrality";
        return false;
      }
    }

    // Add constraints (rows)
    if (num_constraints > 0) {
      status = highs.addRows(
          static_cast<int>(num_constraints),
          row_lower.data(),
          row_upper.data(),
          static_cast<int>(a_values.size()),
          a_row_indices.data(),
          a_col_indices.data(),
          a_values.data()
      );

      if (status != HighsStatus::kOk) {
        Base_class::error_message_ = "failed to add constraints to HiGHS model";
        return false;
      }
    }

    // Enable or disable presolve (enabled by default in HiGHS)
    bool presolve = true;
    highs.setOptionValue("presolve", presolve ? "on" : "off");

    // Solve the problem
    status = highs.run();

    if (status != HighsStatus::kOk) {
      Base_class::error_message_ = "HiGHS solver failed with status: " +
                                   std::to_string(static_cast<int>(status));
      return false;
    }

    // Get the model status
    HighsModelStatus model_status = highs.getModelStatus();

    bool success = false;

    switch (model_status) {
      case HighsModelStatus::kOptimal:
        // Solution is optimal
        success = true;
        break;

      case HighsModelStatus::kInfeasible:
        Base_class::error_message_ = "model was infeasible";
        break;

      case HighsModelStatus::kUnbounded:
      case HighsModelStatus::kUnboundedOrInfeasible:
        Base_class::error_message_ = "model was unbounded";
        break;

      case HighsModelStatus::kTimeLimit:
        Base_class::error_message_ = "aborted due to time limit";
        // Some solvers may still have a feasible solution
        if (highs.getInfo().primal_solution_status ==
            static_cast<int>(HighsSolutionStatus::kSolutionStatusFeasible)) {
          success = true;
        }
        break;

      case HighsModelStatus::kIterationLimit:
        Base_class::error_message_ = "aborted due to iteration limit";
        // Check if we have a feasible solution
        if (highs.getInfo().primal_solution_status ==
            static_cast<int>(HighsSolutionStatus::kSolutionStatusFeasible)) {
          success = true;
        }
        break;

      default:
        Base_class::error_message_ = "solver terminated with status: " +
                                     std::to_string(static_cast<int>(model_status));
        break;
    }

    // Extract solution if available
    if (success || model_status == HighsModelStatus::kOptimal) {
      const HighsSolution& solution = highs.getSolution();

      if (solution.col_value.size() == num_vars) {
        Base_class::result_.resize(num_vars);

        for (std::size_t i = 0; i < num_vars; ++i) {
          FT x = solution.col_value[i];

          Variable* v = Base_class::variables_[i];
          if (v->variable_type() != Variable::CONTINUOUS) {
            x = static_cast<int>(std::round(x));
          }

          v->set_solution_value(x);
          Base_class::result_[i] = x;
        }

        success = true;
      } else {
        Base_class::error_message_ = "solution vector has incorrect size";
        success = false;
      }
    }

    return success;
  }
  /// \endcond
};

} // namespace CGAL

#endif // CGAL_USE_HIGHS or DOXYGEN_RUNNING

#endif // CGAL_HIGHS_MIXED_INTEGER_PROGRAM_TRAITS_H
