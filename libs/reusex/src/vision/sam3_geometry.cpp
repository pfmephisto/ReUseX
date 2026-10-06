// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/sam3_geometry.hpp"

#include <algorithm>
#include <optional>

namespace reusex::vision {

std::array<float, 4> sam3_box_cxcywh(const std::array<float, 4> &xyxy,
                                     int image_w, int image_h) {
  if (image_w <= 0 || image_h <= 0)
    return {0.f, 0.f, 0.f, 0.f};
  const float w = static_cast<float>(image_w);
  const float h = static_cast<float>(image_h);
  const float x1 = std::clamp(std::min(xyxy[0], xyxy[2]), 0.f, w);
  const float x2 = std::clamp(std::max(xyxy[0], xyxy[2]), 0.f, w);
  const float y1 = std::clamp(std::min(xyxy[1], xyxy[3]), 0.f, h);
  const float y2 = std::clamp(std::max(xyxy[1], xyxy[3]), 0.f, h);
  return {(x1 + x2) * 0.5f / w, (y1 + y2) * 0.5f / h, (x2 - x1) / w,
          (y2 - y1) / h};
}

std::int64_t sam3_box_label(std::string_view polarity) noexcept {
  return polarity == "pos" ? 1 : 0;
}

int sam3_geometry_slot_len(int max_boxes) noexcept {
  return max_boxes > 0 ? max_boxes + 1 : 0;
}

std::vector<std::uint8_t> sam3_geometry_slot_mask(int n_boxes, int slot_len) {
  std::vector<std::uint8_t> mask(
      static_cast<std::size_t>(std::max(0, slot_len)), 0);
  if (n_boxes <= 0)
    return mask;
  const int valid = std::min(n_boxes + 1, slot_len);
  std::fill_n(mask.begin(), std::max(0, valid), std::uint8_t{1});
  return mask;
}

int sam3_geometry_box_count(std::size_t requested, int capacity) noexcept {
  if (capacity <= 0)
    return 0;
  return static_cast<int>(
      std::min(requested, static_cast<std::size_t>(capacity)));
}

int sam3_geometry_capacity(int model_cap, int encoder_min_boxes,
                           int encoder_max_boxes, int text_len,
                           int decoder_max_len) noexcept {
  if (encoder_min_boxes > 1)
    return 0;
  const int decoder_room = decoder_max_len - text_len - 1; // minus CLS
  const int cap = std::min({model_cap, encoder_max_boxes, decoder_room});
  return std::max(0, cap);
}

namespace {

struct Rect {
  float x1, y1, x2, y2;
  float area() const { return std::max(0.f, x2 - x1) * std::max(0.f, y2 - y1); }
};

Rect normalised(const std::array<float, 4> &b) {
  return {std::min(b[0], b[2]), std::min(b[1], b[3]), std::max(b[0], b[2]),
          std::max(b[1], b[3])};
}

float intersection(const Rect &a, const Rect &b) {
  return Rect{std::max(a.x1, b.x1), std::max(a.y1, b.y1), std::min(a.x2, b.x2),
              std::min(a.y2, b.y2)}
      .area();
}

} // namespace

bool sam3_detection_selected(const std::array<float, 4> &det_xyxy,
                             const std::vector<SegmentBox> &boxes,
                             const std::vector<std::array<float, 2>> &points,
                             const std::function<bool(float, float)> &covers) {
  constexpr float kHalf = 0.5f;
  const Rect det = normalised(det_xyxy);
  const float det_area = det.area();
  if (det_area <= 0.f)
    return false;

  bool any_pos = false;
  bool hit = false;
  for (const auto &[polarity, xyxy] : boxes) {
    const Rect box = normalised(xyxy);
    const float inter = intersection(det, box);
    if (polarity == "pos") {
      any_pos = true;
      const float box_area = box.area();
      if (inter >= kHalf * det_area ||
          (box_area > 0.f && inter >= kHalf * box_area))
        hit = true;
    } else if (inter >= kHalf * det_area) {
      return false; // mostly inside a negative box
    }
  }
  for (const auto &[x, y] : points) {
    const bool in_box =
        x >= det.x1 && x <= det.x2 && y >= det.y1 && y <= det.y2;
    if (in_box && (!covers || covers(x, y)))
      hit = true;
  }
  return (!any_pos && points.empty()) || hit;
}

const std::vector<float> &sam3_point_exemplar_fracs() {
  static const std::vector<float> fracs{0.03f, 0.05f, 0.08f, 0.12f};
  return fracs;
}

std::array<float, 4> sam3_point_exemplar_box(const std::array<float, 2> &point,
                                             int image_w, int image_h,
                                             float frac) {
  const float r =
      frac * static_cast<float>(std::max(1, std::min(image_w, image_h)));
  const float w = static_cast<float>(std::max(0, image_w));
  const float h = static_cast<float>(std::max(0, image_h));
  return {std::clamp(point[0] - r, 0.f, w), std::clamp(point[1] - r, 0.f, h),
          std::clamp(point[0] + r, 0.f, w), std::clamp(point[1] + r, 0.f, h)};
}

std::vector<std::size_t>
sam3_best_detection_per_point(const std::vector<float> &scores,
                              const std::vector<std::vector<bool>> &covers) {
  std::size_t n_points = 0;
  for (const auto &row : covers)
    n_points = std::max(n_points, row.size());
  std::vector<std::size_t> picked;
  for (std::size_t p = 0; p < n_points; ++p) {
    std::optional<std::size_t> best;
    for (std::size_t d = 0; d < covers.size() && d < scores.size(); ++d)
      if (p < covers[d].size() && covers[d][p] &&
          (!best || scores[d] > scores[*best]))
        best = d;
    if (best)
      picked.push_back(*best);
  }
  std::sort(picked.begin(), picked.end());
  picked.erase(std::unique(picked.begin(), picked.end()), picked.end());
  return picked;
}

} // namespace reusex::vision
