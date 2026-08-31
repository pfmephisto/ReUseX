// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/geometry/CellComplex.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <unordered_map>
#include <utility>

namespace reusex::geometry {

/**
 * @brief Tunable parameters for the Solidifier MIP solve.
 *
 * Every tunable lives here (docs/STANDARDS.md §4); CLI flags mirror these
 * defaults rather than defining their own.
 */
struct SolidifierOptions {
  /// Wall-clock time limit for the MIP solve, in seconds. Guards against the
  /// solver hanging indefinitely on complex buildings (issue #212).
  double time_limit_seconds = 120.0;

  /// Weight on the wall/complexity term of the objective (was hardcoded).
  double alpha = 0.04;

  /// Relative MIP gap tolerance. Negative => use the solver default.
  double mip_gap = -1.0;
};

/**
 * @brief Backend-agnostic classification of the last MIP solve outcome.
 *
 * Lets callers distinguish *why* a solve failed (infeasible vs. time-limit
 * vs. error) without depending on the underlying solver's status enum.
 */
enum class SolveStatus {
  not_solved,
  optimal,
  feasible,
  infeasible,
  unbounded,
  time_limit,
  iteration_limit,
  numerical_error,
  error
};

/// Human-readable name for a SolveStatus (for logging).
const char *to_string(SolveStatus status);

/**
 * @brief Solidifier solves room segmentation using Mixed Integer Programming
 *
 * Uses PIMPL idiom to hide CGAL MIP solver implementation details.
 * This significantly reduces compile times by not exposing CGAL headers.
 */
class Solidifier {
    public:
  using Fd = CellComplex::Vertex;
  using Cd = CellComplex::Vertex;

  Solidifier() = delete;
  explicit Solidifier(std::shared_ptr<const CellComplex> cc,
                      SolidifierOptions options = SolidifierOptions{});
  ~Solidifier();

  // Delete copy (contains unique_ptr to incomplete type)
  Solidifier(const Solidifier &) = delete;
  Solidifier &operator=(const Solidifier &) = delete;

  // Delete move (simpler, could be implemented if needed)
  Solidifier(Solidifier &&) = delete;
  Solidifier &operator=(Solidifier &&) = delete;

  /**
   * @brief Solve the MIP problem for room segmentation
   * @return Optional pair of room labels and wall labels, or nullopt if solving
   * fails
   */
  std::optional<std::pair<std::unordered_map<Cd, int>,
                          std::unordered_map<Cd, std::set<int>>>>
  solve();

  /**
   * @brief Status of the most recent solve() call.
   *
   * Distinguishes timeout from infeasibility from a modelling error so the
   * caller can report an actionable reason on failure (issue #212).
   */
  SolveStatus last_solve_status() const;

  /**
   * @brief Convert solved cell complex to mesh
   * @param filter Function to filter which cells to include in mesh
   * @return Pair of vertex matrix and face matrix (Eigen format)
   */
  std::pair<Eigen::MatrixXd, Eigen::MatrixXi>
  toMesh(std::function<bool(const Cd)> filter);

    protected:
  // Internal accessor for implementations in separate compilation units
  std::shared_ptr<const CellComplex> get_cell_complex() const;

    private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

} // namespace reusex::geometry
