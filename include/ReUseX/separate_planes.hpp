// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <Eigen/StdVector>
#include <spdlog/spdlog.h>

#include <vector>

template <typename Scalar, int Rows>
using EigenVectorContainer =
    std::vector<Eigen::Matrix<Scalar, Rows, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<Scalar, Rows, 1>>>;

namespace ReUseX {
template <typename NType = double>
auto separate_planes(
    const EigenVectorContainer<NType, 4> &planes,
    const Eigen::Matrix<NType, 3, 1> &up = Eigen::Matrix<NType, 3, 1>(0, 0, 1),
    const double epsilon = 0.1) {
  spdlog::trace("Separate planes into vertical and horizontal planes");

  std::vector<size_t> vertical{};
  std::vector<size_t> horizontal{};

  for (size_t i = 0; i < planes.size(); ++i) {
    auto &plane = planes[i];
    const double dot_prod = plane.template head<3>().dot(up);

    if (std::abs(dot_prod) < epsilon) // Wall
      vertical.push_back(i);
    else if (std::abs(dot_prod - 1.0) < epsilon) // Floor
      horizontal.push_back(i);
    else if (std::abs(dot_prod + 1.0) < epsilon) // Ceiling
      horizontal.push_back(i);
    else
      spdlog::warn("Plane {} is not vertical or horizontal", i);
  }
  return std::make_tuple(vertical, horizontal);
}
} // namespace ReUseX
