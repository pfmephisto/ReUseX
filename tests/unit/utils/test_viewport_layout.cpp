// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <reusex/visualize/viewport_layout.hpp>

#include <vector>

using Catch::Approx;
using reusex::visualize::viewport_bounds;
using reusex::visualize::ViewportBounds;

namespace {

// True if two 1D intervals overlap by more than a small epsilon (touching
// edges are allowed, since adjacent viewports share a boundary).
bool intervals_overlap(float a_lo, float a_hi, float b_lo, float b_hi) {
  constexpr float eps = 1e-4f;
  return (a_hi - b_lo) > eps && (b_hi - a_lo) > eps;
}

// True if two viewport rectangles overlap in area (shared edges allowed).
bool rects_overlap(const ViewportBounds &a, const ViewportBounds &b) {
  return intervals_overlap(a.left, a.right, b.left, b.right) &&
         intervals_overlap(a.top, a.bottom, b.top, b.bottom);
}

} // namespace

TEST_CASE("viewport_bounds: single viewport covers the whole window",
          "[visualize][viewport]") {
  auto vb = viewport_bounds(0, 1);
  REQUIRE(vb.left == Approx(0.0f));
  REQUIRE(vb.top == Approx(0.0f));
  REQUIRE(vb.right == Approx(1.0f));
  REQUIRE(vb.bottom == Approx(1.0f));
}

TEST_CASE("viewport_bounds: all bounds stay within [0, 1]",
          "[visualize][viewport]") {
  for (std::size_t total = 1; total <= 9; ++total) {
    for (std::size_t i = 0; i < total; ++i) {
      auto vb = viewport_bounds(i, total);
      INFO("total=" << total << " index=" << i);
      REQUIRE(vb.left >= Approx(0.0f).margin(1e-6));
      REQUIRE(vb.top >= Approx(0.0f).margin(1e-6));
      REQUIRE(vb.right <= Approx(1.0f).margin(1e-6));
      REQUIRE(vb.bottom <= Approx(1.0f).margin(1e-6));
      // Non-degenerate: each viewport has positive area.
      REQUIRE(vb.right > vb.left);
      REQUIRE(vb.bottom > vb.top);
    }
  }
}

TEST_CASE("viewport_bounds: partitions are non-overlapping",
          "[visualize][viewport]") {
  for (std::size_t total = 1; total <= 9; ++total) {
    std::vector<ViewportBounds> boxes;
    for (std::size_t i = 0; i < total; ++i)
      boxes.push_back(viewport_bounds(i, total));

    for (std::size_t i = 0; i < total; ++i) {
      for (std::size_t j = i + 1; j < total; ++j) {
        INFO("total=" << total << " i=" << i << " j=" << j);
        REQUIRE_FALSE(rects_overlap(boxes[i], boxes[j]));
      }
    }
  }
}

TEST_CASE("viewport_bounds: even split halves the window",
          "[visualize][viewport]") {
  // Two viewports split left/right at x = 0.5.
  auto a = viewport_bounds(0, 2);
  auto b = viewport_bounds(1, 2);

  REQUIRE(a.left == Approx(0.0f));
  REQUIRE(a.right == Approx(0.5f));
  REQUIRE(b.left == Approx(0.5f));
  REQUIRE(b.right == Approx(1.0f));

  // Both span the full vertical extent.
  REQUIRE(a.top == Approx(0.0f));
  REQUIRE(a.bottom == Approx(1.0f));
  REQUIRE(b.top == Approx(0.0f));
  REQUIRE(b.bottom == Approx(1.0f));
}

TEST_CASE("viewport_bounds: odd count gives the first viewport 60% width",
          "[visualize][viewport]") {
  // First of three viewports gets the larger (60%) horizontal region.
  auto first = viewport_bounds(0, 3);
  REQUIRE(first.left == Approx(0.0f));
  REQUIRE(first.right == Approx(0.6f));
  REQUIRE(first.top == Approx(0.0f));
  REQUIRE(first.bottom == Approx(1.0f));
}

TEST_CASE("viewport_bounds: four-way split tiles the unit square",
          "[visualize][viewport]") {
  // Total area of all four viewports should sum to 1 with no overlap.
  float area = 0.0f;
  for (std::size_t i = 0; i < 4; ++i) {
    auto vb = viewport_bounds(i, 4);
    area += (vb.right - vb.left) * (vb.bottom - vb.top);
  }
  REQUIRE(area == Approx(1.0f));
}
