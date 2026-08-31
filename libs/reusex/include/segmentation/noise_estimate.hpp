// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types/point_types.hpp"

#include <cstdint>

namespace reusex::geometry {

/// Options controlling the deterministic local-PCA cloud-noise estimator.
///
/// Determinism (docs/STANDARDS.md §6): the estimator draws @ref seed_count
/// random seed points from a fixed-seed PRNG, so the same inputs, options and
/// cloud always yield the same estimate.
struct NoiseEstimateOptions {
  /// Number of random seed points at which a local plane is fit.
  std::size_t seed_count = 1000;
  /// Number of nearest neighbours per seed used for the local PCA fit.
  int k_neighbors = 20;
  /// PRNG seed for deterministic seed-point sampling (STANDARDS §6).
  unsigned seed = 42;
};

/// Result of @ref estimate_cloud_noise.
struct NoiseEstimate {
  /// Estimated sensor noise standard deviation [m], derived from the median
  /// absolute point-to-plane residual of local PCA patches (MAD * 1.4826).
  float sigma = 0.0F;
  /// Median nearest-neighbour spacing [m] — a robust point-density proxy.
  float density = 0.0F;
  /// Number of seed neighbourhoods that contributed a valid residual.
  std::size_t samples = 0;
};

/// Estimate per-cloud sensor noise (sigma) and point density from local
/// PCA patches.
///
/// For each of @ref NoiseEstimateOptions::seed_count random seed points, the
/// @ref NoiseEstimateOptions::k_neighbors nearest neighbours are found via a
/// KdTree and a plane is fit by PCA (smallest-eigenvalue eigenvector = normal).
/// The per-seed residual is the median absolute point-to-plane distance within
/// the patch; sigma is the median of those residuals scaled by 1.4826 (the
/// MAD→sigma consistency constant for Gaussian noise). Density is the median
/// nearest-neighbour spacing across the seeds.
///
/// Deterministic for fixed inputs and @p options (STANDARDS §6).
///
/// @param cloud   Input point cloud (must be non-empty; needs enough points
///                for at least a few k-neighbour patches).
/// @param normals Point normals (currently unused by the fit but kept for
///                interface symmetry with the segmentation pipeline).
/// @param options Sampling/neighbourhood parameters.
/// @return sigma, density and the number of contributing samples.
/// @throws std::runtime_error if @p cloud is empty.
auto estimate_cloud_noise(CloudConstPtr cloud, CloudNConstPtr normals,
                          const NoiseEstimateOptions &options =
                              NoiseEstimateOptions{}) -> NoiseEstimate;

} // namespace reusex::geometry
