// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <CGAL/Mixed_integer_program_traits.h>

#if defined(USE_HIGHS) || defined(DOXYGEN_RUNNING)

#include "Highs.h"

#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

// Add spdlog for debug logging
#include <spdlog/spdlog.h>

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
    : public Mixed_integer_program_traits<FT> {
  /// \cond SKIP_IN_MANUAL
    public:
  typedef CGAL::Mixed_integer_program_traits<FT> Base_class;
  typedef typename Base_class::Variable Variable;
  typedef typename Base_class::Linear_constraint Linear_constraint;
  typedef typename Base_class::Linear_objective Linear_objective;
  typedef typename Linear_objective::Sense Sense;
  typedef typename Variable::Variable_type Variable_type;

    public:
  /// Solves the program. Returns `false` if fails.
  virtual bool solve() {
    Base_class::error_message_.clear();

    Highs highs;

    // Enable detailed logging for debugging
    highs.setOptionValue("log_to_console", true);
    highs.setOptionValue("output_flag", true);
    highs.setOptionValue("log_dev_level",
                         2); // 0=none, 1=info, 2=detailed, 3=verbose

    // Optionally write logs to file for detailed analysis
    // highs.openLogFile("highs_solver.log");

    // EXPERIMENTAL: Try to force PDLP solver (will likely fail for MIP)
    // spdlog::warn("EXPERIMENTAL: Attempting to enable PDLP solver for MIP
    // (will "
    //              "likely fail)");
    // HighsStatus pdlp_status = highs.setOptionValue("solver", "pdlp");
    // if (pdlp_status == HighsStatus::kOk) {
    //   spdlog::info("PDLP solver enabled (may be ignored for MIP problems)");
    // } else {
    //   spdlog::warn("Failed to enable PDLP solver (status: {})",
    //                static_cast<int>(pdlp_status));
    // }

    // Set the objective sense using the proper API
    bool minimize =
        (Base_class::objective_->sense() == Linear_objective::MINIMIZE);
    ObjSense sense = minimize ? ObjSense::kMinimize : ObjSense::kMaximize;
    highs.changeObjectiveSense(sense);

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
      const Variable *var = Base_class::variables_[i];

      double lb, ub;
      var->get_bounds(lb, ub);
      col_lower[i] = lb;
      col_upper[i] = ub;

      switch (var->variable_type()) {
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
    const std::unordered_map<const Variable *, double> &obj_coeffs =
        Base_class::objective_->coefficients();
    typename std::unordered_map<const Variable *, double>::const_iterator
        obj_it;
    for (obj_it = obj_coeffs.begin(); obj_it != obj_coeffs.end(); ++obj_it) {
      const Variable *var = obj_it->first;
      double coeff = obj_it->second;
      col_cost[var->index()] = coeff;
    }

    // Build constraint matrix in COO (Coordinate) format
    const std::size_t num_constraints = Base_class::constraints_.size();

    std::vector<double> row_lower(num_constraints);
    std::vector<double> row_upper(num_constraints);

    // Use row-organized map for efficient deduplication and packed row format
    // building HiGHS requires no duplicate entries, unlike SCIP which handles
    // them automatically Structure: row -> (col -> accumulated_value)
    std::map<int, std::map<int, double>> row_map;
    std::size_t total_entries = 0;

    for (std::size_t i = 0; i < num_constraints; ++i) {
      const Linear_constraint *c = Base_class::constraints_[i];

      double lb, ub;
      c->get_bounds(lb, ub);
      row_lower[i] = lb;
      row_upper[i] = ub;

      const std::unordered_map<const Variable *, double> &coeffs =
          c->coefficients();
      typename std::unordered_map<const Variable *, double>::const_iterator
          coeff_it;

      int row = static_cast<int>(i);
      for (coeff_it = coeffs.begin(); coeff_it != coeffs.end(); ++coeff_it) {
        const Variable *var = coeff_it->first;
        double coeff = coeff_it->second;
        int col = static_cast<int>(var->index());

        // Accumulate coefficients for duplicate (row, col) pairs
        row_map[row][col] += coeff;
        total_entries++;
      }
    }

    // Calculate total unique entries
    std::size_t unique_entries = 0;
    for (const auto &row_entry : row_map) {
      unique_entries += row_entry.second.size();
    }

    // Log deduplication statistics
    std::size_t num_duplicates = total_entries - unique_entries;
    spdlog::debug("HiGHS matrix building: {} total entries, {} unique entries, "
                  "{} duplicates",
                  total_entries, unique_entries, num_duplicates);
    if (num_duplicates > 0) {
      spdlog::warn(
          "HiGHS: Found {} duplicate matrix entries, accumulated coefficients",
          num_duplicates);
    }

    // Build packed row format (CSR) directly from row-organized map - O(n)
    // complexity addRows expects: starts[i] = position where row i begins in
    // indices/values arrays
    std::vector<HighsInt> a_starts;
    std::vector<HighsInt> a_indices; // column indices
    std::vector<double> a_values;

    a_starts.reserve(num_constraints + 1);
    a_indices.reserve(unique_entries);
    a_values.reserve(unique_entries);

    HighsInt current_pos = 0;
    for (std::size_t row = 0; row < num_constraints; ++row) {
      a_starts.push_back(current_pos);

      // Get entries for this row (if any exist)
      auto row_it = row_map.find(static_cast<int>(row));
      if (row_it != row_map.end()) {
        // Add all column entries for this row
        for (const auto &col_entry : row_it->second) {
          a_indices.push_back(col_entry.first); // column index
          a_values.push_back(col_entry.second); // value
          current_pos++;
        }
      }
    }
    a_starts.push_back(current_pos); // Final entry = total non-zeros

    spdlog::debug("HiGHS: Built packed row format: {} rows, {} non-zeros",
                  num_constraints, a_values.size());

    // Add the model to HiGHS
    HighsStatus status =
        highs.addCols(static_cast<int>(num_vars), col_cost.data(),
                      col_lower.data(), col_upper.data(),
                      0,       // num_nz (we'll add constraints separately)
                      nullptr, // start
                      nullptr, // index
                      nullptr  // value
        );

    if (status != HighsStatus::kOk) {
      Base_class::error_message_ = "failed to add variables to HiGHS model";
      return false;
    }

    // Set integrality
    for (std::size_t i = 0; i < num_vars; ++i) {
      status =
          highs.changeColIntegrality(static_cast<int>(i), col_integrality[i]);
      if (status != HighsStatus::kOk) {
        Base_class::error_message_ = "failed to set variable integrality";
        return false;
      }
    }

    // Add constraints (rows) in packed row format
    if (num_constraints > 0 && !a_values.empty()) {
      // Validate packed row format
      if (a_starts.size() != num_constraints + 1) {
        spdlog::error("HiGHS: Invalid starts array size: {} (expected {})",
                      a_starts.size(), num_constraints + 1);
        Base_class::error_message_ =
            "invalid packed row format: wrong starts array size";
        return false;
      }
      if (a_starts[0] != 0) {
        spdlog::error("HiGHS: Invalid starts array: first entry must be 0");
        Base_class::error_message_ =
            "invalid packed row format: starts[0] != 0";
        return false;
      }
      if (a_starts[num_constraints] != static_cast<HighsInt>(a_values.size())) {
        spdlog::error("HiGHS: Invalid starts array: last entry {} != num_nz {}",
                      a_starts[num_constraints], a_values.size());
        Base_class::error_message_ =
            "invalid packed row format: wrong final starts value";
        return false;
      }

      spdlog::debug(
          "HiGHS: Calling addRows with packed format: {} rows, {} non-zeros",
          num_constraints, a_values.size());

      status =
          highs.addRows(static_cast<int>(num_constraints), row_lower.data(),
                        row_upper.data(), static_cast<int>(a_values.size()),
                        a_starts.data(), a_indices.data(), a_values.data());

      if (status != HighsStatus::kOk) {
        Base_class::error_message_ = "failed to add constraints to HiGHS model";
        return false;
      }
    }

    // Enable or disable presolve (enabled by default in HiGHS)
    bool presolve = true;
    highs.setOptionValue("presolve", presolve ? "on" : "off");

    // Log problem dimensions for debugging
    spdlog::debug(
        "HiGHS MIP problem: {} variables, {} constraints, {} non-zeros",
        num_vars, num_constraints, a_values.size());

    // Solve the problem
    status = highs.run();

    if (status != HighsStatus::kOk) {
      Base_class::error_message_ = "HiGHS solver failed with status: " +
                                   std::to_string(static_cast<int>(status));
      return false;
    }

    // Get detailed solver info
    const HighsInfo &info = highs.getInfo();

    spdlog::debug("HiGHS solve status: run_status={}, model_status={}",
                  static_cast<int>(status),
                  static_cast<int>(highs.getModelStatus()));
    spdlog::debug("HiGHS info: primal_status={}, dual_status={}, objective={}",
                  info.primal_solution_status, info.dual_solution_status,
                  info.objective_function_value);

    // Get the model status
    HighsModelStatus model_status = highs.getModelStatus();

    bool success = false;

    switch (model_status) {
    case HighsModelStatus::kOptimal:
      spdlog::debug("HiGHS found optimal solution");
      success = true;
      break;

    case HighsModelStatus::kInfeasible:
      spdlog::warn("HiGHS: model was infeasible");
      Base_class::error_message_ = "model was infeasible";
      break;

    case HighsModelStatus::kUnbounded:
    case HighsModelStatus::kUnboundedOrInfeasible:
      spdlog::warn("HiGHS: model was unbounded");
      Base_class::error_message_ = "model was unbounded";
      break;

    case HighsModelStatus::kTimeLimit:
      spdlog::warn("HiGHS: aborted due to time limit");
      Base_class::error_message_ = "aborted due to time limit";
      // Some solvers may still have a feasible solution
      {
        HighsInt primal_status = highs.getInfo().primal_solution_status;
        spdlog::debug("HiGHS time limit: primal_status={}", primal_status);

        // SolutionStatus enum values from HiGHS:
        // kSolutionStatusNone = 0
        // kSolutionStatusInfeasible = 1
        // kSolutionStatusFeasible = 2
        if (primal_status ==
            static_cast<HighsInt>(SolutionStatus::kSolutionStatusFeasible)) {
          spdlog::debug("HiGHS: feasible solution found despite time limit");
          success = true;
        } else {
          spdlog::debug(
              "HiGHS: no feasible solution at time limit (primal_status={})",
              primal_status);
        }
      }
      break;

    case HighsModelStatus::kIterationLimit:
      spdlog::warn("HiGHS: aborted due to iteration limit");
      Base_class::error_message_ = "aborted due to iteration limit";
      // Check if we have a feasible solution
      {
        HighsInt primal_status = highs.getInfo().primal_solution_status;
        spdlog::debug("HiGHS iteration limit: primal_status={}", primal_status);

        if (primal_status ==
            static_cast<HighsInt>(SolutionStatus::kSolutionStatusFeasible)) {
          spdlog::debug(
              "HiGHS: feasible solution found despite iteration limit");
          success = true;
        } else {
          spdlog::debug("HiGHS: no feasible solution at iteration limit "
                        "(primal_status={})",
                        primal_status);
        }
      }
      break;

    default:
      spdlog::warn("HiGHS: solver terminated with status: {}",
                   static_cast<int>(model_status));
      Base_class::error_message_ =
          "solver terminated with status: " +
          std::to_string(static_cast<int>(model_status));
      break;
    }

    // Extract solution if available
    spdlog::debug("HiGHS solution extraction: success={}, model_status={}, "
                  "checking extraction",
                  success, static_cast<int>(model_status));

    if (success) {
      const HighsSolution &solution = highs.getSolution();

      spdlog::debug(
          "HiGHS solution vector: size={} (expected {}), extracting values",
          solution.col_value.size(), num_vars);

      if (solution.col_value.size() == num_vars) {
        Base_class::result_.resize(num_vars);

        for (std::size_t i = 0; i < num_vars; ++i) {
          FT x = solution.col_value[i];

          Variable *v = Base_class::variables_[i];
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

#endif // USE_HIGHS or DOXYGEN_RUNNING
