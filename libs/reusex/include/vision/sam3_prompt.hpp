// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Backend-agnostic SAM3 prompt type shared between the vision library and
// the GUI layer. No OpenCV, no backend headers — only the standard library.

#pragma once

#include <array>
#include <string>
#include <string_view>
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
/// Text sent to SAM3 for a prompt that carries boxes but no class name. This
/// is the upstream SAM3 convention for a geometry-only prompt: Meta's
/// `Sam3Processor.add_geometric_prompt` encodes the text "visual" when only
/// geometry is given, so the model relies on the boxes. Shared by the GUI
/// endpoint and `rux create segment-frame` so both send the same text.
inline constexpr std::string_view kGeometryOnlyPromptText = "visual";

/// The value a backend writes into the label map for one prompt's
/// detections. With @p by_prompt_index (single-image callers such as
/// segment_image / segment_panorama) it is the prompt's position in the
/// request, so label k always means prompt k and two prompts with the same
/// text stay distinct. Without it (the `rux create annotate` dataset path) it
/// is @p cache_id, the backend's per-text id that stays stable across every
/// frame of a run. A negative @p prompt_index (no prompt list) always falls
/// back to @p cache_id.
inline int sam3_label_value(bool by_prompt_index, int prompt_index,
                            int cache_id) noexcept {
  return by_prompt_index && prompt_index >= 0 ? prompt_index : cache_id;
}

/// A click-point prompt: pixel [x, y].
using SegmentPoint = std::array<float, 2>;

struct Sam3Prompt {
  std::string text;
  std::vector<SegmentBox> boxes;
  /// Positive click points. SAM3's exported detector takes boxes only, so a
  /// backend with a geometry encoder feeds each point as a small exemplar box
  /// (sam3_point_exemplar_box) and keeps the detections whose mask covers a
  /// point; a backend without one clips the prompt's pixels to that box.
  std::vector<SegmentPoint> points;
  /// Per-prompt confidence override. Negative ⟹ use the global threshold.
  float confidence = -1.0f;

  explicit Sam3Prompt(std::string t, std::vector<SegmentBox> b = {},
                      float conf = -1.0f)
      : text(std::move(t)), boxes(std::move(b)), confidence(conf) {}
};

} // namespace reusex::vision
