// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <filesystem>

#include <opencv2/core.hpp>

namespace reusex::io {

/// Stitch a dual-fisheye Insta360 X4 frame into an equirectangular BGR image.
///
/// The calibration is baked from the Hugin reference template in
/// docs/guides/Process walk through.md Phase 5. Input must be a dual-fisheye
/// cv::Mat with width == 2×height (two fisheye circles side by side; left half
/// = front lens, right half = back lens). Resolution scaling is handled
/// automatically relative to the reference resolution (5888×2944).
///
/// Remap maps are computed on the first call for a given input resolution and
/// cached for subsequent calls — construction is ~10 ms at 5888×2944.
///
/// Lens model: equidistant fisheye, ~200° FoV, no polynomial distortion
/// correction (a=b=c=0 in the reference .pto). Gain compensation aligns
/// brightness between hemispheres; a multi-band (Laplacian-pyramid) blend
/// hides the seam.
///
/// @param dual_fisheye  BGR (or BGRA/grey) cv::Mat with dual-fisheye pixels.
/// @returns             Equirectangular BGR cv::Mat; width == 2×height.
/// @throws std::runtime_error if input dimensions are unexpected.
cv::Mat stitch_insta360_x4(const cv::Mat &dual_fisheye);

/// Returns true iff the file extension is ".insp" (case-insensitive).
/// ".insp" is the Insta360 proprietary dual-fisheye capture extension; every
/// file with this extension is routed through stitch_insta360_x4 by the
/// import path.
bool is_insta360_dual_fisheye(const std::filesystem::path &path);

} // namespace reusex::io
