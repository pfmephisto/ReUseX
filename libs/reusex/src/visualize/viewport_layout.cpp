// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "visualize/viewport_layout.hpp"

namespace reusex::visualize {

ViewportBounds viewport_bounds(std::size_t index, std::size_t total, float left,
                               float top, float right, float bottom,
                               bool split_horizontal) {

  // Ratio used for odd viewport counts: first viewport gets this much space
  constexpr float ODD_VIEWPORT_RATIO = 0.6f;

  // Base case: single viewport gets full bounds
  if (total == 1) {
    return {left, top, right, bottom};
  }

  // Even count: binary split
  if (total % 2 == 0) {
    std::size_t half = total / 2;

    if (split_horizontal) {
      // Split horizontally (left/right regions)
      float mid = left + (right - left) * 0.5f;

      if (index < half) {
        // First half: left region, recurse with vertical split
        return viewport_bounds(index, half, left, top, mid, bottom, false);
      } else {
        // Second half: right region, recurse with vertical split
        return viewport_bounds(index - half, half, mid, top, right, bottom,
                               false);
      }
    } else {
      // Split vertically (top/bottom regions)
      float mid = top + (bottom - top) * 0.5f;

      if (index < half) {
        // First half: top region, recurse with horizontal split
        return viewport_bounds(index, half, left, top, right, mid, true);
      } else {
        // Second half: bottom region, recurse with horizontal split
        return viewport_bounds(index - half, half, left, mid, right, bottom,
                               true);
      }
    }
  }

  // Odd count: first viewport larger (60%), rest split recursively
  if (index == 0) {
    // First viewport gets the larger portion
    if (split_horizontal) {
      float mid = left + (right - left) * ODD_VIEWPORT_RATIO;
      return {left, top, mid, bottom};
    } else {
      float mid = top + (bottom - top) * ODD_VIEWPORT_RATIO;
      return {left, top, right, mid};
    }
  }

  // Remaining viewports recursively split the smaller region
  if (split_horizontal) {
    float mid = left + (right - left) * ODD_VIEWPORT_RATIO;
    return viewport_bounds(index - 1, total - 1, mid, top, right, bottom,
                           false);
  } else {
    float mid = top + (bottom - top) * ODD_VIEWPORT_RATIO;
    return viewport_bounds(index - 1, total - 1, left, mid, right, bottom,
                           true);
  }
}

} // namespace reusex::visualize
