// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// SAM3-on-360: segment an equirectangular panorama by tiling it into
// perspective (pinhole) views the segmenter consumes at its native
// resolution, running the UNMODIFIED SAM3 model on each tile, and stitching
// the per-tile CV_32S label maps back into one equirect label image.
//
// This reuses geometry::overlapping_views / stitch_labels_to_equirect for the
// spherical geometry and the tensor_rt SAM3 forward() path for inference; it
// does not reimplement either.

#pragma once

#include "reusex/vision/IModel.hpp"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace reusex::vision {

/// Options controlling how a panorama is tiled and segmented.
struct SegmentPanoramaOptions {
  int n_yaw = 8;           ///< number of equator tiles around the sphere
  double fov_deg = 90.0;   ///< per-tile horizontal FOV (overlap hides seams)
  int tile = 1008;         ///< tile size in px (matches SAM3 native input)
  float confidence = 0.5f; ///< detection confidence threshold
  /// Text prompts (one class each). Empty => use the SAM3 default prompt list.
  std::vector<std::string> prompts;
};

/// Segment a 360 equirectangular panorama with a SAM3 model.
///
/// @param model         SAM3 model handle (created via BackendFactory).
/// @param equirect_bgr  Equirectangular panorama (BGR, CV_8UC3).
/// @param opts          Tiling / prompt / confidence options.
/// @return A CV_32S equirect label map the same size as @p equirect_bgr,
///         with -1 for background. Class ids are stable across tiles because a
///         single model instance assigns each prompt text a fixed class id.
cv::Mat segment_panorama(IModel &model, const cv::Mat &equirect_bgr,
                         const SegmentPanoramaOptions &opts);

} // namespace reusex::vision
