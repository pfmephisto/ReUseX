// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Backend-agnostic helpers for SAM3 geometry (box) prompts: the box encoding
// the geometry encoder expects and the per-prompt token layout the decoder
// consumes. Standard library only, so they are unit-testable without a GPU.

#pragma once

#include "reusex/vision/sam3_prompt.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace reusex::vision {

/// Pixel box [x1, y1, x2, y2] (corners in any order) -> the normalised
/// [cx, cy, w, h] the SAM3 geometry encoder takes, clamped to the
/// @p image_w x @p image_h image. A degenerate image size yields all zeros.
std::array<float, 4> sam3_box_cxcywh(const std::array<float, 4> &xyxy,
                                     int image_w, int image_h);

/// SAM3 box label: 1 for a positive ("pos") box, 0 for a negative one.
std::int64_t sam3_box_label(std::string_view polarity) noexcept;

/// Geometry tokens one prompt slot holds when the largest prompt in a decoder
/// batch has @p max_boxes boxes: the geometry encoder emits one token per box
/// plus a CLS token, so max_boxes + 1, or 0 when no prompt has a box.
int sam3_geometry_slot_len(int max_boxes) noexcept;

/// Validity of one prompt's geometry slot of @p slot_len tokens holding
/// @p n_boxes encoded boxes: 1 == a token the decoder attends to, 0 ==
/// padding. The first n_boxes + 1 tokens (boxes + CLS) are valid, the rest
/// padding. A prompt with no boxes masks its whole slot — not even the CLS
/// token — so it decodes exactly as on the text-only path. Convert to the
/// engines' bool polarity with sam3_mask_value().

std::vector<std::uint8_t> sam3_geometry_slot_mask(int n_boxes, int slot_len);

/// Which way an export's bool token masks point, read off the text encoder's
/// own output for an input whose first @p real_tokens tokens are real and the
/// rest padding. The released sam3.1-onnx-v1 graphs use True == valid token
/// (their text_mask is `attention_mask > 0`); the in-repo exporter's wrappers
/// write True == padding (`attention_mask == 0`). The decoder reads its
/// prompt_mask in the text encoder's polarity, since the text path copies
/// text_mask straight in. Returns true for True == valid, false for True ==
/// padding, and std::nullopt when the mask does not split cleanly into the
/// two runs (a broken probe: do not guess).
std::optional<bool>
sam3_mask_true_is_valid(const std::vector<std::uint8_t> &mask, int real_tokens);

/// The bool an engine mask holds for a token that is @p valid, under the
/// polarity sam3_mask_true_is_valid() found.
constexpr bool sam3_mask_value(bool valid, bool true_is_valid) noexcept {
  return valid == true_is_valid;
}

/// Boxes actually fed for a prompt with @p requested boxes when the model
/// accepts at most @p capacity (0 = geometry disabled).
int sam3_geometry_box_count(std::size_t requested, int capacity) noexcept;

/// Most boxes per prompt the engines can take, or 0 when geometry prompting
/// is unusable. @p model_cap is the backend's own cap; @p encoder_min_boxes /
/// @p encoder_max_boxes the geometry encoder's num_boxes profile range;
/// @p text_len the text tokens per prompt and @p decoder_max_len the
/// decoder's prompt_len profile max. Needs an encoder that accepts one box and
/// a decoder with room for at least one box token plus the CLS token.
int sam3_geometry_capacity(int model_cap, int encoder_min_boxes,
                           int encoder_max_boxes, int text_len,
                           int decoder_max_len) noexcept;

/// Does a detection with pixel box @p det_xyxy belong to a box-prompted
/// selection? SAM3 treats a prompt box as an exemplar and returns every
/// similar object; a selection keeps only what the boxes point at:
///   * with positive boxes, a detection is kept when one of them covers at
///     least half of the detection (the box drawn around the object) or at
///     least half of that box lies inside the detection (a small box drawn
///     on a larger object);
///   * a detection at least half inside a negative box is dropped;
///   * with no positive box and no point, everything not dropped is kept;
///   * a detection that covers one of @p points (click points) is kept;
///     "covers" is @p covers(x, y) — the detection's mask — or, without it,
///     its box.
/// A zero-area detection is never kept.
bool sam3_detection_selected(
    const std::array<float, 4> &det_xyxy, const std::vector<SegmentBox> &boxes,
    const std::vector<std::array<float, 2>> &points = {},
    const std::function<bool(float, float)> &covers = {});

/// Exemplar box half-sizes, as fractions of the image's shorter side, a click
/// point is tried at. SAM3's exported detector takes boxes only, and a box is
/// an exemplar: the detection it yields at the point tracks the box's size (a
/// small box finds the patch under the click, a large one the surrounding
/// furniture). Trying several sizes and keeping the most confident detection
/// under the point picks the object's own scale (measured on NewOffice frame
/// 1500: monitor, chair, bag and pillar each win at a different size).
const std::vector<float> &sam3_point_exemplar_fracs();

/// The exemplar box [x1, y1, x2, y2] for a click point: half-size @p frac of
/// the image's shorter side, clamped to the image.
std::array<float, 4> sam3_point_exemplar_box(const std::array<float, 2> &point,
                                             int image_w, int image_h,
                                             float frac);

/// Which detections a click selects: for each point, the single highest-
/// scoring detection that covers it (@p covers[d][p]: detection d covers
/// point p). Returns the chosen detection indices, ascending, without
/// duplicates; a point no detection covers selects nothing.
std::vector<std::size_t>
sam3_best_detection_per_point(const std::vector<float> &scores,
                              const std::vector<std::vector<bool>> &covers);

/// The detections a prompt with click points keeps: the UNION of
///   * per point, the single most confident detection covering it
///     (sam3_best_detection_per_point over @p scores / @p covers), and
///   * every detection its boxes select (@p box_selected[d]; the caller
///     counts each box hit once, not once per exemplar-size pass).
/// A detection that is both is listed once. Returns ascending indices.
std::vector<std::size_t>
sam3_point_prompt_selection(const std::vector<float> &scores,
                            const std::vector<std::vector<bool>> &covers,
                            const std::vector<bool> &box_selected);

} // namespace reusex::vision
