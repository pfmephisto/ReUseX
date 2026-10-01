// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Instance highlight colouring for evidence renders (Kortlægning).

#include <catch2/catch_test_macros.hpp>
#include <visualize/highlight.hpp>

#include <stdexcept>

using namespace reusex::visualize;

TEST_CASE("ApplyInstanceHighlight_PaintsInstance_DimsRest",
          "[visualize][highlight]") {
  std::vector<unsigned char> rgb{100, 100, 100, 200, 200, 200, 50, 60, 70};
  const std::vector<std::uint32_t> labels{2, 7, 2};
  CHECK(apply_instance_highlight(rgb, labels, 2) == 2);
  CHECK(rgb[0] == kHighlightRgb[0]);
  CHECK(rgb[1] == kHighlightRgb[1]);
  CHECK(rgb[8] == kHighlightRgb[2]);
  CHECK(rgb[3] == 60); // 200 * 0.3
}

TEST_CASE("ApplyInstanceHighlight_AbsentInstance_ReturnsZero_AndDimsAll",
          "[visualize][highlight]") {
  std::vector<unsigned char> rgb{100, 100, 100};
  CHECK(apply_instance_highlight(rgb, {5}, 9) == 0);
  CHECK(rgb[0] == 30);
}

TEST_CASE("ApplyInstanceHighlight_SizeMismatch_Throws",
          "[visualize][highlight]") {
  std::vector<unsigned char> rgb{1, 2, 3};
  CHECK_THROWS_AS(apply_instance_highlight(rgb, {1, 2}, 1),
                  std::invalid_argument);
}
