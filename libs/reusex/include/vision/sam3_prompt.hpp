// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Backend-agnostic SAM3 prompt type shared between the vision library and
// the GUI layer. No OpenCV, no backend headers — only the standard library.

#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

namespace reusex::vision {

/// A bounding-box prompt: label ("pos" or "neg") and pixel coords
/// [x1,y1,x2,y2].
using SegmentBox = std::pair<std::string, std::array<float, 4>>;

/// One SAM3 prompt: text class name plus optional positive/negative bounding
/// boxes.
///
/// For a point-click prompt from a viewport, emulate it as a small box around
/// the click (e.g. click ± 8 px). Native point-prompt support is a follow-up
/// to #409.
struct Sam3Prompt {
  std::string text;
  std::vector<SegmentBox> boxes;
  /// Per-prompt confidence override. Negative ⟹ use the global threshold.
  float confidence = -1.0f;

  explicit Sam3Prompt(std::string t, std::vector<SegmentBox> b = {},
                      float conf = -1.0f)
      : text(std::move(t)), boxes(std::move(b)), confidence(conf) {}
};

} // namespace reusex::vision
