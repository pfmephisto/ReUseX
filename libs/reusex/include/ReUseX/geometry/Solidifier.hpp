// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <ReUseX/geometry/CellComplex.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <unordered_map>
#include <utility>

namespace ReUseX::geometry {

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
  explicit Solidifier(std::shared_ptr<const CellComplex> cc);
  ~Solidifier();

  // Delete copy (contains unique_ptr to incomplete type)
  Solidifier(const Solidifier&) = delete;
  Solidifier& operator=(const Solidifier&) = delete;
  
  // Delete move (simpler, could be implemented if needed)
  Solidifier(Solidifier&&) = delete;
  Solidifier& operator=(Solidifier&&) = delete;

  /**
   * @brief Solve the MIP problem for room segmentation
   * @return Optional pair of room labels and wall labels, or nullopt if solving fails
   */
  std::optional<std::pair<std::unordered_map<Cd, int>,
                          std::unordered_map<Cd, std::set<int>>>>
  solve();

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

} // namespace ReUseX::geometry
