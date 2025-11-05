// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "ReUseX/types.hpp"

#include <Eigen/Core>
#include <boost/functional/hash.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/pca.h>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/zip.hpp>
#include <spdlog/spdlog.h>

namespace ReUseX::geometry {

auto dist_plane_point(const Eigen::Vector4d &plane,
                      const Eigen::Vector3d &point) -> double;

auto make_pairs(EigenVectorContainer<double, 4> &planes,
                std::vector<IndicesPtr> &inliers,
                EigenVectorContainer<double, 3> &centroids,
                const double threshold = 0.6,
                const double new_plane_offset = 0.5)
    -> std::vector<std::pair<size_t, size_t>>;

auto force_orthogonal_planes(EigenVectorContainer<double, 4> &planes,
                             const double threshold = 0.1,
                             const Eigen::Matrix<double, 3, 1> &up =
                                 Eigen::Matrix<double, 3, 1>(0, 0, 1))
    -> EigenVectorContainer<double, 4>;

auto compute_number_of_inliers(CloudConstPtr cloud,
                               Eigen::Vector4d const &plane,
                               IndicesConstPtr indices,
                               const float threshold = 0.2) -> size_t;

auto merge_planes(EigenVectorContainer<double, 4> const &planes_,
                  std::vector<IndicesPtr> const &inliers_,
                  EigenVectorContainer<double, 3> const &centroids_,
                  CloudConstPtr cloud, const double angle_threshold = 0.1,
                  const double distance_threshold = 0.5,
                  const double min_overlap = 0.8)
    -> std::tuple<EigenVectorContainer<double, 4>, std::vector<IndicesPtr>,
                  EigenVectorContainer<double, 3>>;

auto separate_planes(const EigenVectorContainer<double, 4> &planes,
                     const Eigen::Vector3d &up = Eigen::Vector3d(0, 0, 1),
                     const double epsilon = 0.1)
    -> std::tuple<std::vector<size_t>, std::vector<size_t>>;

} // namespace ReUseX::geometry
