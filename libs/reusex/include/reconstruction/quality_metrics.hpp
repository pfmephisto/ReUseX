// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace reusex::geometry {

/// Quality statistics for a single detected plane.
struct PlaneQuality {
  uint32_t label = 0;
  std::size_t point_count = 0;
  double rms = 0.0; ///< RMS point-to-plane residual [m]
  double p90 = 0.0; ///< 90th percentile of |residual| [m]
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
};

/// Ground-truth-free reconstruction quality report.
///
/// Measures internal-consistency properties that pose error destroys:
/// - flatness: RMS point-to-plane residual of each detected plane. Pose
///   drift smears wall points, inflating the residual.
/// - thickness: 90th percentile of |residual|. Misaligned passes over the
///   same surface produce doubled walls; p90 grows much faster than RMS
///   when the residual distribution becomes bimodal.
///
/// These metrics compare *relative* reconstruction quality across pipeline
/// changes (SLAM parameters, pose refinement) without external ground truth.
struct QualityReport {
  std::size_t total_points = 0;
  std::size_t labeled_points = 0; ///< points with plane label > 0
  std::vector<PlaneQuality> planes;

  double flatness_rms = 0.0;     ///< point-weighted mean of per-plane RMS [m]
  double flatness_rms_max = 0.0; ///< worst per-plane RMS [m]
  double thickness_p90 = 0.0;    ///< point-weighted mean of per-plane p90 [m]

  [[nodiscard]] std::string to_json(int indent = 2) const;
};

struct QualityMetricsOptions {
  /// Planes supported by fewer points are excluded from the report.
  std::size_t min_points = 100;
};

/// Compute per-plane flatness/thickness quality metrics.
///
/// Each plane (points sharing a label > 0 in @p labels) is refit by least
/// squares (PCA), independent of any stored plane parameters, and residuals
/// are measured against the refit plane.
///
/// @throws std::invalid_argument if cloud and labels sizes differ or the
///         cloud is empty.
QualityReport compute_plane_quality(const Cloud &cloud, const CloudL &labels,
                                    const QualityMetricsOptions &options = {});

} // namespace reusex::geometry
