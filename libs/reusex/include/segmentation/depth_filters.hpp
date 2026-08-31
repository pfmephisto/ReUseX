// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <opencv2/core/mat.hpp>

namespace reusex::geometry {

/// Remove "flying pixels" at depth discontinuity edges.
///
/// Computes depth gradients via Sobel operators and zeros out pixels with
/// gradient magnitudes exceeding @p gradient_threshold.
///
/// @param depth  Input/output depth map (single-channel), modified in-place.
/// @param confidence  Input/output confidence map, modified in-place (may be
/// empty).
/// @param gradient_threshold  Maximum allowed gradient in meters per pixel.
void apply_depth_discontinuity_filter(cv::Mat &depth, cv::Mat &confidence,
                                      float gradient_threshold = 0.5f);

/// Remove isolated noisy depth measurements.
///
/// Compares each pixel to the median of its 5x5 neighbourhood and zeros out
/// pixels deviating by more than @p consistency_threshold.
///
/// @param depth  Input/output depth map (single-channel), modified in-place.
/// @param confidence  Input/output confidence map, modified in-place (may be
/// empty).
/// @param consistency_threshold  Maximum allowed deviation from neighbourhood
/// median in meters.
void apply_ray_consistency_filter(cv::Mat &depth, cv::Mat &confidence,
                                  float consistency_threshold = 0.2f);

} // namespace reusex::geometry
