// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/geometry/utils.hpp>

namespace ReUseX::geometry {
auto dist_plane_point(const Eigen::Vector4d &plane,
                      const Eigen::Vector3d &point) -> double {
  return (plane.head<3>().dot(point) + plane[3]) /
         plane.head<3>().squaredNorm();
}

auto make_pairs(EigenVectorContainer<double, 4> &planes,
                std::vector<IndicesPtr> &inliers,
                EigenVectorContainer<double, 3> &centroids,
                const double threshold, const double new_plane_offset)
    -> std::vector<std::pair<size_t, size_t>> {

  using Vector3 = Eigen::Vector3d;
  using Vector4 = Eigen::Vector4d;

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

      const double dot = plane_i.head<3>().dot(plane_j.head<3>());
      if (std::abs(dot + 1.0) >= 0.1)
        continue; // not opposite normals

      const double dist_i = dist_plane_point(plane_i, centroid_j.head<3>());
      const double dist_j = dist_plane_point(plane_j, centroid_i.head<3>());
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

      auto p = centroid_i.head<3>();
      float dist = dist_plane_point(new_plane, p);

      Vector3 new_centroid = p - dist * new_plane.head<3>();

      planes.push_back(new_plane);
      inliers.push_back(IndicesPtr(new Indices{}));
      centroids.push_back(new_centroid);

      pairs.emplace_back(i, planes.size() - 1);
      paired.push_back(true);
    }
  }

  return pairs;
}

auto force_orthogonal_planes(EigenVectorContainer<double, 4> &planes,
                             const double threshold, const Eigen::Vector3d &up)
    -> EigenVectorContainer<double, 4> {
  spdlog::trace("Force planes to be orthogonal");

  for (auto &plane : planes) {
    auto normal = plane.head<3>(); // This is a block, no copy
    const double scalar = normal.dot(up);

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
      spdlog::warn("Plane with normal ({:3f}, {:3f}, {:3f}) is not vertical or "
                   "horizontal",
                   normal.x(), normal.y(), normal.z());
    }
  }

  return planes;
}

auto compute_number_of_inliers(CloudConstPtr cloud,
                               Eigen::Vector4d const &plane,
                               IndicesConstPtr indices, const float threshold)
    -> size_t {
  const auto &n = plane.head<3>();
  const auto &d = plane[3];
  // If normal not guaranteed normalized, uncomment:
  // const float invNorm = 1.0f / n.norm();
  // const float scaledThreshold = threshold * invNorm;

  return std::ranges::count_if(*indices, [&](int idx) {
    const auto &p =
        cloud->points[idx].getVector3fMap().cast<double>(); // directly 3f map
    float dist = n.dot(p) + d;
    return std::abs(dist) < threshold; // or scaledThreshold
  });
}

auto merge_planes(EigenVectorContainer<double, 4> const &planes_,
                  std::vector<IndicesPtr> const &inliers_,
                  EigenVectorContainer<double, 3> const &centroids_,
                  CloudConstPtr cloud, const double angle_threshold,
                  const double distance_threshold, const double min_overlap)
    -> std::tuple<EigenVectorContainer<double, 4>, std::vector<IndicesPtr>,
                  EigenVectorContainer<double, 3>> {
  spdlog::trace("Merge planes with angle threshold {} and distance threshold"
                "{} and min overlap {}",
                angle_threshold, distance_threshold, min_overlap);

  assert(planes_.size() == inliers_.size() &&
         "Planes and inliers must have the same size");
  assert(planes_.size() == centroids_.size() &&
         "Planes and centroids must have the same size");

  // Set up output containers
  auto Pm = EigenVectorContainer<double, 4>();
  auto Im = std::vector<IndicesPtr>();
  auto Cm = EigenVectorContainer<double, 3>();

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
      if (std::abs(Pi.head<3>().dot(Pj.head<3>()) - 1.0) > angle_threshold)
        continue; // Not same orientation

      // Overlap
      size_t Oj = compute_number_of_inliers(cloud, Pi, Ij, distance_threshold);

      size_t Oi = compute_number_of_inliers(cloud, Pj, Ii, distance_threshold);

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
      auto centroid = pca.getMean().head<3>().cast<double>();

      if (normal.cast<double>().dot(Pi.head<3>()) < 0)
        normal = -normal;

      // Set values
      Pm[j].head<3>() = normal.cast<double>();
      Pm[j][3] = -normal.cast<double>().dot(centroid.head<3>());

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

auto separate_planes(const EigenVectorContainer<double, 4> &planes,
                     const Eigen::Vector3d &up, const double epsilon)
    -> std::tuple<std::vector<size_t>, std::vector<size_t>> {

  spdlog::trace("Separate planes into vertical and horizontal planes");

  std::vector<size_t> vertical{};
  std::vector<size_t> horizontal{};

  for (size_t i = 0; i < planes.size(); ++i) {
    auto &plane = planes[i];
    const double dot_prod = plane.head<3>().dot(up);

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

} // namespace ReUseX::geometry
