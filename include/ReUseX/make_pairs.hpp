// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <Eigen/StdVector>
#include <spdlog/spdlog.h>

using Indices = pcl::Indices;
using IndicesPtr = pcl::IndicesPtr;
using IndicesConstPtr = pcl::IndicesConstPtr;
template <typename Scalar, int Rows>
using EigenVectorContainer =
    std::vector<Eigen::Matrix<Scalar, Rows, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<Scalar, Rows, 1>>>;

namespace ReUseX {
template <typename NType = double>
auto make_pairs(EigenVectorContainer<NType, 4> &planes,
                std::vector<IndicesPtr> &inliers,
                EigenVectorContainer<NType, 3> &centroids,
                const double threshold = 0.6,
                const double new_plane_offset = 0.5) {
  spdlog::trace("Find plane pairs with threshold {} and add new planes at a "
                "distance of {}",
                threshold, new_plane_offset);

  using Vector3 = Eigen::Matrix<NType, 3, 1>;
  using Vector4 = Eigen::Matrix<NType, 4, 1>;

  std::vector<std::pair<size_t, size_t>> pairs;
  std::vector<bool> paired(planes.size(), false);

  for (size_t i = 0; i < planes.size(); ++i) {
    if (paired[i])
      continue;

    auto &plane_i = planes[i];
    auto &centroid_i = centroids[i];

    size_t best_j = planes.size(); // invalid
    double best_diff = threshold;

    // Inline loop instead of ranges pipeline
    for (size_t j = i + 1; j < planes.size(); ++j) {
      if (paired[j])
        continue;

      auto &plane_j = planes[j];
      auto &centroid_j = centroids[j];

      const double dot =
          plane_i.template head<3>().dot(plane_j.template head<3>());
      if (std::abs(dot + 1.0) >= 0.1)
        continue; // not opposite normals

      const double dist_i =
          ReUseX::planePointDist(plane_i, centroid_j.template head<3>());
      const double dist_j =
          ReUseX::planePointDist(plane_j, centroid_i.template head<3>());
      const double diff = (std::abs(dist_i) + std::abs(dist_j)) * 0.5;

      if (diff < best_diff) {
        best_diff = diff;
        best_j = j;
      }
    }

    if (best_j != planes.size()) {
      pairs.emplace_back(i, best_j);
      paired[best_j] = true;
    } else {
      // Create new plane in-place
      Vector4 new_plane = -plane_i;
      new_plane[3] -= new_plane_offset;

      auto p = centroid_i.template head<3>();
      float dist = ReUseX::planePointDist(new_plane, p);

      Vector3 new_centroid = p - dist * new_plane.template head<3>();

      planes.push_back(new_plane);
      inliers.push_back(IndicesPtr(new Indices{}));
      centroids.push_back(new_centroid);

      pairs.emplace_back(i, planes.size() - 1);
      paired.push_back(true);
    }
  }

  return pairs;
}
} // namespace ReUseX
