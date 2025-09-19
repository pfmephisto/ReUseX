// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <Eigen/Core>
#include <filesystem>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

namespace ReUseX {

bool save(
    fs::path const &output_path,
    std::vector<pcl::ModelCoefficients> const &model_coefficients,
    std::vector<Eigen::Vector4f,
                Eigen::aligned_allocator<Eigen::Vector4f>> const &centroids,
    std::vector<std::shared_ptr<pcl::Indices>> const &inlier_indices);

bool read(fs::path const &input_path,
          std::vector<pcl::ModelCoefficients> &model_coefficients,
          std::vector<Eigen::Vector4f,
                      Eigen::aligned_allocator<Eigen::Vector4f>> &centroids,
          std::vector<std::shared_ptr<pcl::Indices>> &inlier_indices);
} // namespace ReUseX
