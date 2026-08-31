// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types.hpp"

#include <cstddef>
#include <string>

namespace reusex::geometry {

/// Options for ground-truth accuracy scoring.
struct AccuracyMetricsOptions {
  /// Inlier distance threshold for precision/recall/F-score [m].
  float threshold = 0.05f;
  /// Voxel leaf size used to downsample the ground-truth cloud before
  /// scoring [m]. Values <= 0 disable downsampling (the raw GT is used).
  float gt_voxel = 0.01f;
};

/// Ground-truth accuracy report for a reconstructed point cloud.
///
/// Standard reconstruction-benchmark definitions, all distances in meters:
/// - accuracy:     for each reconstruction point, distance to the nearest
///                 ground-truth point. Measures how far the reconstruction
///                 strays from the reference surface (lower is better).
/// - completeness: for each ground-truth point, distance to the nearest
///                 reconstruction point. Measures how much of the reference
///                 surface was actually reconstructed (lower is better).
/// - chamfer:      mean of accuracy_mean and completeness_mean.
/// - precision:    fraction of reconstruction points within @c threshold of
///                 the ground truth.
/// - recall:       fraction of ground-truth points within @c threshold of the
///                 reconstruction.
/// - fscore:       harmonic mean 2*P*R/(P+R) (0 when P+R == 0).
///
/// Unlike the ground-truth-free QualityReport, this requires an external
/// reference cloud (e.g. a Faro survey), and both clouds must share a common
/// coordinate frame.
struct AccuracyReport {
  std::size_t cloud_points = 0; ///< finite reconstruction points scored
  std::size_t gt_points = 0;    ///< finite (post-downsample) GT points scored

  double accuracy_mean = 0.0;   ///< mean recon->GT distance [m]
  double accuracy_median = 0.0; ///< median recon->GT distance [m]

  double completeness_mean = 0.0;   ///< mean GT->recon distance [m]
  double completeness_median = 0.0; ///< median GT->recon distance [m]

  double chamfer = 0.0; ///< (accuracy_mean + completeness_mean) / 2 [m]

  double precision = 0.0; ///< fraction of recon points within threshold
  double recall = 0.0;    ///< fraction of GT points within threshold
  double fscore = 0.0;    ///< 2*P*R/(P+R)

  float threshold = 0.05f; ///< inlier threshold used [m]

  [[nodiscard]] std::string to_json(int indent = 2) const;
};

/// Score a reconstructed cloud against a ground-truth point cloud.
///
/// Nearest-neighbour distances are computed with a KdTree; the ground truth is
/// optionally voxel-downsampled first (@c opt.gt_voxel). Non-finite points in
/// either cloud are skipped.
///
/// @throws std::invalid_argument if either input is empty, or if every point
///         of either input is non-finite (nothing left to score).
AccuracyReport compute_accuracy(const Cloud &cloud, const CloudLoc &gt,
                                const AccuracyMetricsOptions &opt = {});

} // namespace reusex::geometry
