// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <Eigen/StdVector>

#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <spdlog/spdlog.h>

#include <range/v3/view/zip.hpp>

using Indices = pcl::Indices;
using IndicesPtr = pcl::IndicesPtr;
using IndicesConstPtr = pcl::IndicesConstPtr;
template <typename Scalar, int Rows>
using EigenVectorContainer =
    std::vector<Eigen::Matrix<Scalar, Rows, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<Scalar, Rows, 1>>>;

namespace ReUseX {

template <typename PointT, typename NType = double>
inline size_t
compute_number_of_inliers(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                          Eigen::Matrix<NType, 4, 1> const &plane,
                          pcl::IndicesConstPtr indices,
                          const float threshold = 0.2) {
  const auto &n = plane.template head<3>();
  const auto &d = plane[3];
  // If normal not guaranteed normalized, uncomment:
  // const float invNorm = 1.0f / n.norm();
  // const float scaledThreshold = threshold * invNorm;

  return std::ranges::count_if(*indices, [&](int idx) {
    const auto &p = cloud->points[idx]
                        .getVector3fMap()
                        .template cast<NType>(); // directly 3f map
    float dist = n.dot(p) + d;
    return std::abs(dist) < threshold; // or scaledThreshold
  });
}

template <typename PointT, typename NType = double>
auto merge_planes(EigenVectorContainer<NType, 4> const &planes_,
                  std::vector<IndicesPtr> const &inliers_,
                  EigenVectorContainer<NType, 3> const &centroids_,
                  typename pcl::PointCloud<PointT>::ConstPtr cloud,
                  const double angle_threshold = 0.1,
                  const double distance_threshold = 0.5,
                  const double min_overlap = 0.8) {
  spdlog::trace("Merge planes with angle threshold {} and distance threshold "
                "{} and min overlap {}",
                angle_threshold, distance_threshold, min_overlap);

  assert(planes_.size() == inliers_.size() &&
         "Planes and inliers must have the same size");
  assert(planes_.size() == centroids_.size() &&
         "Planes and centroids must have the same size");

  // Set up output containers
  auto Pm = EigenVectorContainer<NType, 4>();
  auto Im = std::vector<IndicesPtr>();
  auto Cm = EigenVectorContainer<NType, 3>();

  pcl::PCA<PointT> pca;
  pca.setInputCloud(cloud);

  for (size_t i = 0; i < planes_.size(); ++i) {
    bool match = false;

    // Get current plane data
    auto const Pi = planes_[i];
    auto const Ii = inliers_[i];
    auto const Ci = centroids_[i];

    for (size_t j = 0; j < Pm.size(); ++j) {

      // Get next plane data
      auto const Pj = Pm[j];
      auto const Ij = Im[j];

      // Check orientation
      if (std::abs(Pi.template head<3>().dot(Pj.template head<3>()) - 1.0) >
          angle_threshold)
        continue; // Not same orientation

      // Overlap
      size_t Oj =
          compute_number_of_inliers<PointT>(cloud, Pi, Ij, distance_threshold);

      size_t Oi =
          compute_number_of_inliers<PointT>(cloud, Pj, Ii, distance_threshold);

      const double overlap = static_cast<double>(Oi + Oj) /
                             static_cast<double>(Ii->size() + Ij->size());

      if (overlap < min_overlap)
        continue; // Not enough inlier overlap

      // Merge planes
      IndicesPtr inliers(new Indices);
      inliers->reserve(Ii->size() + Ij->size());
      inliers->insert(inliers->end(), Ii->begin(), Ii->end());
      inliers->insert(inliers->end(), Ij->begin(), Ij->end());

      pca.setIndices(inliers);

      auto normal = pca.getEigenVectors().col(2);
      auto centroid = pca.getMean().template head<3>().template cast<NType>();

      if (normal.template cast<NType>().dot(Pi.template head<3>()) < 0)
        normal = -normal;

      // Set values
      Pm[j].template head<3>() = normal.template cast<NType>();
      Pm[j][3] =
          -normal.template cast<NType>().dot(centroid.template head<3>());

      Im[j] = inliers;
      Cm[j] = centroid;

      match = true;
      break;
    }
    if (!match) {
      Pm.push_back(Pi);
      Im.push_back(Ii);
      Cm.push_back(Ci);
    }
  }
  return std::make_tuple(Pm, Im, Cm);
}

} // namespace ReUseX
