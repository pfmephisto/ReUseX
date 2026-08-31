// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>

namespace CGAL {

/// Backend-agnostic classification of a MIP solve outcome, shared by the
/// HiGHS and cuOpt traits so callers can distinguish *why* a solve failed
/// (infeasible vs. time limit vs. numerical error) without depending on a
/// specific solver's status enum.
enum class MIP_solve_status {
  not_solved,      ///< solve() has not been called yet
  optimal,         ///< proven optimal solution found
  feasible,        ///< feasible (but not proven optimal) solution found
  infeasible,      ///< problem proven infeasible
  unbounded,       ///< problem unbounded (or unbounded-or-infeasible)
  time_limit,      ///< aborted at the configured time limit, no usable solution
  iteration_limit, ///< aborted at the iteration limit, no usable solution
  numerical_error, ///< solver reported a numerical failure
  error            ///< model construction or other solver error
};

inline const char *to_string(MIP_solve_status s) {
  switch (s) {
  case MIP_solve_status::not_solved:
    return "not_solved";
  case MIP_solve_status::optimal:
    return "optimal";
  case MIP_solve_status::feasible:
    return "feasible";
  case MIP_solve_status::infeasible:
    return "infeasible";
  case MIP_solve_status::unbounded:
    return "unbounded";
  case MIP_solve_status::time_limit:
    return "time_limit";
  case MIP_solve_status::iteration_limit:
    return "iteration_limit";
  case MIP_solve_status::numerical_error:
    return "numerical_error";
  case MIP_solve_status::error:
    return "error";
  }
  return "unknown";
}

} // namespace CGAL
