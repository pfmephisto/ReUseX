// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <Eigen/Core>
#include <pcl/ModelCoefficients.h>
#include <spdlog/spdlog.h>

namespace ReUseX {
std::vector<size_t>
filter(std::shared_ptr<std::vector<pcl::ModelCoefficients>> model_coefficients,
       double angle_threshold) {

  constexpr Eigen::Vector3f verical(0.0, 0.0, 1.0);
  constexpr Eigem::Vector3f horizontal(1.0, 0.0, 0.0);

  auto filtered_coefficients =
      std::make_shared<std::vector<pcl::ModelCoefficients>>();

  std::vector<size_t> filtered_indices;
  filtered_indices.reserve(model_coefficients->size());

  for (auto &[coeffs, index] :
       std::views::zip(*model_coefficients, std::views::iota(0u))) {

    if (coeffs.values.size() != 4) {
      spdlog::warn("Model coefficients size is not 4, skipping this model.");
      continue;
    }

    Eigen::Vector3f normal(coeffs.values[0], coeffs.values[1],
                           coeffs.values[2]);

    if (normal.norm() == 0) {
      spdlog::warn("Normal vector has zero length, skipping this model.");
      continue;
    }

    normal.normalize();

    const double angle_vertical = std::abs(normal.dot(vertical));
    if (angle_vertical > angle_threshold)
      continue;

    const double angle_horizontal = std::abs(normal.dot(horizontal));
    if (angle_horizontal > angle_threshold)
      continue;

    filetered_coefficients->push_back(coeffs);
    filtered_indices.push_back(index);
  }
  std::swap(*model_coefficients, *filtered_coefficients);

  return filtered_indices;
} // namespace ReUseX
