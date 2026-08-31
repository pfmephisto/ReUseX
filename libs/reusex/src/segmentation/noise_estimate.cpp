// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segmentation/noise_estimate.hpp"
#include "core/logging.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

namespace reusex::geometry {

namespace {

/// MAD → standard-deviation consistency constant for Gaussian data.
constexpr float kMadToSigma = 1.4826F;

/// Median of a non-empty vector (partial-sorts in place, order destroyed).
float median_inplace(std::vector<float> &values) {
  const std::size_t n = values.size();
  const std::size_t mid = n / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  const float hi = values[mid];
  if ((n % 2) != 0)
    return hi;
  // Even count: average the two central order statistics.
  const float lo = *std::max_element(values.begin(), values.begin() + mid);
  return 0.5F * (lo + hi);
}

} // namespace

auto estimate_cloud_noise(CloudConstPtr cloud, CloudNConstPtr /*normals*/,
                          const NoiseEstimateOptions &options)
    -> NoiseEstimate {
  if (!cloud || cloud->empty())
    throw std::runtime_error(
        "estimate_cloud_noise: input cloud is null or empty.");

  const std::size_t n_points = cloud->size();
  // Need at least a handful of points to fit any local plane.
  const int k = std::max(3, options.k_neighbors);
  if (n_points < static_cast<std::size_t>(k) + 1) {
    reusex::warn("estimate_cloud_noise: cloud has {} points (< k+1={}); "
                 "returning zero estimate.",
                 n_points, k + 1);
    return NoiseEstimate{0.0F, 0.0F, 0};
  }

  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud);

  // Deterministic seed-point sampling (STANDARDS §6): fixed-seed PRNG over the
  // point index range. With replacement is fine — duplicates are rare and
  // harmless for a robust median.
  std::mt19937 gen(options.seed);
  std::uniform_int_distribution<std::size_t> pick(0, n_points - 1);

  const std::size_t n_seeds = std::min(options.seed_count, n_points);

  std::vector<float> per_seed_residual; // median |residual| per patch
  std::vector<float> nn_spacing;        // nearest-neighbour distance per seed
  per_seed_residual.reserve(n_seeds);
  nn_spacing.reserve(n_seeds);

  std::vector<int> indices(k);
  std::vector<float> sq_dists(k);
  std::vector<float> patch_residuals(k);

  for (std::size_t s = 0; s < n_seeds; ++s) {
    const std::size_t seed_idx = pick(gen);
    const PointT &q = cloud->points[seed_idx];
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z))
      continue;

    const int found = kdtree.nearestKSearch(q, k, indices, sq_dists);
    if (found < 3)
      continue;

    // Nearest-neighbour spacing: closest neighbour that is not the seed itself.
    // nearestKSearch returns results sorted by distance; index 0 is the seed
    // (distance 0), so index 1 is the true nearest neighbour when present.
    for (int j = 0; j < found; ++j) {
      if (indices[j] != static_cast<int>(seed_idx) && sq_dists[j] > 0.0F) {
        nn_spacing.push_back(std::sqrt(sq_dists[j]));
        break;
      }
    }

    // Local PCA: centroid + covariance of the k-neighbourhood.
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int j = 0; j < found; ++j)
      centroid += cloud->points[indices[j]].getVector3fMap();
    centroid /= static_cast<float>(found);

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int j = 0; j < found; ++j) {
      const Eigen::Vector3f d =
          cloud->points[indices[j]].getVector3fMap() - centroid;
      cov += d * d.transpose();
    }
    cov /= static_cast<float>(found);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success)
      continue;
    // Smallest eigenvalue → plane normal (direction of least variance).
    const Eigen::Vector3f normal = solver.eigenvectors().col(0);

    // Point-to-plane residuals within the patch; robustify via the median.
    patch_residuals.clear();
    for (int j = 0; j < found; ++j) {
      const Eigen::Vector3f d =
          cloud->points[indices[j]].getVector3fMap() - centroid;
      patch_residuals.push_back(std::abs(normal.dot(d)));
    }
    if (patch_residuals.empty())
      continue;
    per_seed_residual.push_back(median_inplace(patch_residuals));
  }

  NoiseEstimate est;
  est.samples = per_seed_residual.size();

  if (per_seed_residual.empty()) {
    reusex::warn("estimate_cloud_noise: no valid patches sampled; returning "
                 "zero estimate.");
    return est;
  }

  // sigma = MAD-scaled median of the per-patch median residuals.
  est.sigma = kMadToSigma * median_inplace(per_seed_residual);
  est.density = nn_spacing.empty() ? 0.0F : median_inplace(nn_spacing);

  reusex::debug("estimate_cloud_noise: sigma={:.4f} m, density(nn)={:.4f} m "
                "from {} patches (k={}, seeds={})",
                est.sigma, est.density, est.samples, k, n_seeds);
  return est;
}

} // namespace reusex::geometry
