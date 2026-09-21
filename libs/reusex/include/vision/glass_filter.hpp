// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <opencv2/core.hpp>
#include <string_view>
#include <vector>

namespace reusex::vision {

/// Returns the canonical list of glass/mirror/transparent-surface prompt
/// strings used by the glass depth filter. These are appended to the
/// structural prompt list when --glass-filter is requested.
const std::vector<std::string> &glass_prompt_list();

/// Returns true when class_name matches one of the glass prompt strings.
bool is_glass_class(std::string_view class_name);

/// Build a per-frame binary glass confidence map from a semantic label image.
///
/// glass_ids: the 0-based class IDs that correspond to glass/mirror classes in
/// the label image (these are the positions of glass prompts in the merged
/// prompt list).  label_image must be CV_32S (-1 = background, 0..N = class).
///
/// Returns CV_8U mat the same size as label_image:
///   255 = trust depth (non-glass or unlabeled)
///     0 = suppress depth (glass/mirror detected)
///
/// An empty glass_ids or empty label_image returns an all-255 map.
cv::Mat build_glass_confidence_map(const cv::Mat &label_image,
                                   const std::vector<int> &glass_ids);

} // namespace reusex::vision
