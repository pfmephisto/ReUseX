// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Eigen/StdVector>
#include <spdlog/spdlog.h>

template <typename Scalar, int Rows>
using EigenVectorContainer =
    std::vector<Eigen::Matrix<Scalar, Rows, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<Scalar, Rows, 1>>>;

namespace ReUseX {
template <typename NType = double>
auto force_orthogonal_planes(EigenVectorContainer<NType, 4> &planes,
                             const double threshold = 0.1,
                             const Eigen::Matrix<NType, 3, 1> &up =
                                 Eigen::Matrix<NType, 3, 1>(0, 0, 1)) {
  spdlog::trace("Force planes to be orthogonal");

  for (auto &plane : planes) {
    auto normal = plane.template head<3>(); // This is a block, no copy
    const NType scalar = normal.dot(up);

    if (std::abs(scalar) < threshold) {
      // Wall: make normal orthogonal to 'up' in-place
      normal -= scalar * up; // modify the block directly
      normal.normalize();    // in-place normalization
    } else if (std::abs(scalar - 1.0) < threshold) {
      // Floor
      normal = up;
    } else if (std::abs(scalar + 1.0) < threshold) {
      // Ceiling
      normal = -up;
    } else {
      spdlog::warn(
          "Plane with normal ({}, {}, {}) is not vertical or horizontal",
          normal.x(), normal.y(), normal.z());
    }
  }

  return planes;
}

} // namespace ReUseX
