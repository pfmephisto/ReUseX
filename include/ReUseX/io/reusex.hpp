// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <ReUseX/types.hpp>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <filesystem>

namespace ReUseX::io {

auto getPlanes(CloudLConstPtr planes, CloudNConstPtr normals,
               CloudLocConstPtr locations)
    -> std::tuple<EigenVectorContainer<double, 4>,
                  EigenVectorContainer<double, 3>, std::vector<IndicesPtr>>;

[[nodiscard]]
bool save(
    std::filesystem::path const &output_path,
    std::vector<pcl::ModelCoefficients> const &model_coefficients,
    std::vector<Eigen::Vector4f,
                Eigen::aligned_allocator<Eigen::Vector4f>> const &centroids,
    std::vector<std::shared_ptr<pcl::Indices>> const &inlier_indices);

[[nodiscard]]
bool read(std::filesystem::path const &input_path,
          std::vector<pcl::ModelCoefficients> &model_coefficients,
          std::vector<Eigen::Vector4f,
                      Eigen::aligned_allocator<Eigen::Vector4f>> &centroids,
          std::vector<std::shared_ptr<pcl::Indices>> &inlier_indices);
} // namespace ReUseX::io
