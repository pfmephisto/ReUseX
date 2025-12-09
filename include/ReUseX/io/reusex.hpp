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

/**
 * @brief Extract plane data from labeled point clouds.
 * 
 * @param planes Labeled point cloud where labels indicate plane IDs.
 * @param normals Point cloud containing plane normals.
 * @param locations Point cloud containing plane centroid locations.
 * @return Tuple of (plane coefficients, centroids, inlier indices).
 */
auto getPlanes(CloudLConstPtr planes, CloudNConstPtr normals,
               CloudLocConstPtr locations)
    -> std::tuple<EigenVectorContainer<double, 4>,
                  EigenVectorContainer<double, 3>, std::vector<IndicesPtr>>;

/**
 * @brief Save plane data to file.
 * 
 * @param output_path Path to output file.
 * @param model_coefficients Plane model coefficients.
 * @param centroids Plane centroids.
 * @param inlier_indices Indices of points belonging to each plane.
 * @return True if save was successful, false otherwise.
 */
[[nodiscard]]
bool save(
    std::filesystem::path const &output_path,
    std::vector<pcl::ModelCoefficients> const &model_coefficients,
    std::vector<Eigen::Vector4f,
                Eigen::aligned_allocator<Eigen::Vector4f>> const &centroids,
    std::vector<std::shared_ptr<pcl::Indices>> const &inlier_indices);

/**
 * @brief Read plane data from file.
 * 
 * @param input_path Path to input file.
 * @param model_coefficients Output plane model coefficients.
 * @param centroids Output plane centroids.
 * @param inlier_indices Output indices of points belonging to each plane.
 * @return True if read was successful, false otherwise.
 */
[[nodiscard]]
bool read(std::filesystem::path const &input_path,
          std::vector<pcl::ModelCoefficients> &model_coefficients,
          std::vector<Eigen::Vector4f,
                      Eigen::aligned_allocator<Eigen::Vector4f>> &centroids,
          std::vector<std::shared_ptr<pcl::Indices>> &inlier_indices);
} // namespace ReUseX::io
