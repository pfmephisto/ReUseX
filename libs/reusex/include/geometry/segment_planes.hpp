// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/core/logging.hpp"
#include "reusex/core/processing_observer.hpp"
#include "reusex/types/point_types.hpp"
#include "reusex/utils/fmt_formatter.hpp"

#include <fmt/format.h>
#include <pcl/common/colors.h>
#include <pcl/filters/filter.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/planar_region_growing.hpp>

#include <atomic>
#include <optional>

namespace reusex::geometry {

struct SegmentPlanesOptions {
  IndicesConstPtr filter = nullptr; // Optional filter to limit processing

  float angle_threshold = 25.0F;

  // Distance and inlier thresholds are derived per-cloud from the measured
  // noise/density when `adaptive` is true (issue #214), UNLESS the caller
  // supplies an explicit override below. An explicit override always wins over
  // adaptivity for that one parameter (e.g. a `-d` CLI flag pins the distance
  // threshold while `min_inliers` still adapts). When `adaptive` is false the
  // legacy fixed defaults are used.
  float plane_dist_threshold = 0.07F;
  int min_inliers = 1000;

  float radius = 0.5F;
  float interval_0 = 16.0F;
  float interval_factor = 1.5F;

  /// Enable noise-adaptive threshold derivation (default ON, issue #214).
  bool adaptive = true;
  /// Explicit distance-threshold override [m]; bypasses adaptivity when set.
  std::optional<float> plane_dist_threshold_override;
  /// Explicit min-inliers override; bypasses adaptivity when set.
  std::optional<int> min_inliers_override;
  /// Deterministic seed for the noise estimator (STANDARDS §6).
  unsigned noise_seed = 42;

  // --- Adaptive scaling constants (issue #214) ------------------------------
  /// plane_dist_threshold = clamp(dist_sigma_factor * sigma, dist_min,
  /// dist_max)
  float dist_sigma_factor = 3.0F;
  float dist_min = 0.01F; ///< lower clamp for adaptive distance threshold [m]
  float dist_max = 0.15F; ///< upper clamp for adaptive distance threshold [m]
  /// Target minimum planar surface area [m^2] used to scale min_inliers by the
  /// measured point density (points ≈ area / spacing^2).
  float min_surface_area = 0.5F;
  int min_inliers_floor = 100;    ///< lower clamp for adaptive min_inliers
  int min_inliers_ceiling = 5000; ///< upper clamp for adaptive min_inliers

  // --- Post-segmentation sanity thresholds (issue #214) ---------------------
  /// Warn when the detected plane count exceeds this (over-fragmentation).
  int warn_plane_count = 60;
  /// Warn when the labeled point fraction falls below this (under-coverage).
  float warn_labeled_fraction = 0.30F;

  // Optional cancellation flag. Caller retains ownership and must keep this
  // alive for the full duration of the segment_planes(...) call.
  const std::atomic_bool *cancel_token = nullptr;
};

auto segment_planes_impl(CloudConstPtr cloud, CloudNConstPtr normals,
                         const SegmentPlanesOptions &options)
    -> std::tuple<CloudLPtr, CloudLocPtr, CloudNPtr>;

auto segment_planes(
    CloudConstPtr cloud, CloudNConstPtr normals,
    const SegmentPlanesOptions &options = SegmentPlanesOptions{})
    -> std::tuple<CloudLPtr, CloudLocPtr, CloudNPtr>;

} // namespace reusex::geometry
