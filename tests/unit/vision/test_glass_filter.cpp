// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <opencv2/core.hpp>

#include <vision/glass_filter.hpp>

using namespace reusex::vision;

// ── glass_prompt_list / is_glass_class ───────────────────────────────────────

TEST_CASE("GlassPromptList_ReturnsExpectedItems", "[vision][glass_filter]") {
  const auto &pl = glass_prompt_list();
  REQUIRE(pl.size() == 4);
  REQUIRE(std::find(pl.begin(), pl.end(), "glass") != pl.end());
  REQUIRE(std::find(pl.begin(), pl.end(), "mirror") != pl.end());
  REQUIRE(std::find(pl.begin(), pl.end(), "window pane") != pl.end());
  REQUIRE(std::find(pl.begin(), pl.end(), "transparent surface") != pl.end());
}

TEST_CASE("IsGlassClass_GlassNames_ReturnsTrue", "[vision][glass_filter]") {
  REQUIRE(is_glass_class("glass"));
  REQUIRE(is_glass_class("mirror"));
  REQUIRE(is_glass_class("window pane"));
  REQUIRE(is_glass_class("transparent surface"));
}

TEST_CASE("IsGlassClass_StructuralNames_ReturnsFalse",
          "[vision][glass_filter]") {
  REQUIRE_FALSE(is_glass_class("wall"));
  REQUIRE_FALSE(is_glass_class("floor"));
  REQUIRE_FALSE(is_glass_class("ceiling"));
  REQUIRE_FALSE(is_glass_class("door"));
  REQUIRE_FALSE(is_glass_class(""));
}

// ── build_glass_confidence_map
// ────────────────────────────────────────────────

TEST_CASE("BuildGlassConfidenceMap_EmptyGlassIds_AllTrust",
          "[vision][glass_filter]") {
  cv::Mat label(4, 4, CV_32S, cv::Scalar(0)); // all class 0
  cv::Mat conf = build_glass_confidence_map(label, {});
  REQUIRE(conf.type() == CV_8U);
  REQUIRE(conf.size() == label.size());
  // All pixels should be 255 (trust) since glass_ids is empty.
  REQUIRE(cv::countNonZero(conf != 255) == 0);
}

TEST_CASE("BuildGlassConfidenceMap_EmptyLabelImage_AllTrust",
          "[vision][glass_filter]") {
  cv::Mat empty;
  cv::Mat conf = build_glass_confidence_map(empty, {2, 3});
  REQUIRE(conf.empty());
}

TEST_CASE("BuildGlassConfidenceMap_GlassPixelsSuppressed",
          "[vision][glass_filter]") {
  // 4×4 label image: left 2 columns = class 0 (structural),
  //                  right 2 columns = class 5 (glass).
  cv::Mat label(4, 4, CV_32S, cv::Scalar(0));
  for (int r = 0; r < 4; ++r)
    for (int c = 2; c < 4; ++c)
      label.at<int>(r, c) = 5;

  cv::Mat conf = build_glass_confidence_map(label, {5});

  REQUIRE(conf.type() == CV_8U);
  // Left 2 columns (non-glass) must be 255.
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 2; ++c)
      REQUIRE(conf.at<uchar>(r, c) == 255);
  // Right 2 columns (glass class 5) must be 0.
  for (int r = 0; r < 4; ++r)
    for (int c = 2; c < 4; ++c)
      REQUIRE(conf.at<uchar>(r, c) == 0);
}

TEST_CASE("BuildGlassConfidenceMap_MultipleGlassIds_UnionSuppressed",
          "[vision][glass_filter]") {
  // 4 pixels, each with a different label: 0, 3, 7, 10.
  // Glass ids: {3, 7}.
  cv::Mat label(1, 4, CV_32S);
  label.at<int>(0, 0) = 0;
  label.at<int>(0, 1) = 3;
  label.at<int>(0, 2) = 7;
  label.at<int>(0, 3) = 10;

  cv::Mat conf = build_glass_confidence_map(label, {3, 7});

  REQUIRE(conf.at<uchar>(0, 0) == 255); // structural → trust
  REQUIRE(conf.at<uchar>(0, 1) == 0);   // class 3 → suppress
  REQUIRE(conf.at<uchar>(0, 2) == 0);   // class 7 → suppress
  REQUIRE(conf.at<uchar>(0, 3) == 255); // class 10 → trust
}

// ── find_duplicate_prompt_concept
// ────────────────────────────────────────────

TEST_CASE("FindDuplicatePromptConcept_EmptyOrUnique_ReturnsEmpty",
          "[vision][glass_filter]") {
  CHECK(find_duplicate_prompt_concept({}).empty());
  CHECK(find_duplicate_prompt_concept({"wall", "floor", "ceiling"}).empty());
  CHECK(find_duplicate_prompt_concept({"wall:0.3", "floor:0.5"}).empty());
  // Different concepts whose names happen to share a prefix are not duplicates.
  CHECK(find_duplicate_prompt_concept({"window", "window pane"}).empty());
}

TEST_CASE("FindDuplicatePromptConcept_PlainDuplicate_ReturnsDuplicateText",
          "[vision][glass_filter]") {
  auto dup = find_duplicate_prompt_concept({"wall", "floor", "wall"});
  REQUIRE(!dup.empty());
  CHECK(dup == "wall");
}

TEST_CASE("FindDuplicatePromptConcept_DuplicateWithDifferentThresholds_"
          "ReturnsConcept",
          "[vision][glass_filter]") {
  // "wall:0.3" and "wall:0.5" are the same concept — only the threshold
  // differs; the model deduplicates on concept text alone.
  auto dup = find_duplicate_prompt_concept({"wall:0.3", "floor", "wall:0.5"});
  REQUIRE(!dup.empty());
  CHECK(dup == "wall");
}

TEST_CASE("FindDuplicatePromptConcept_DuplicateWithAndWithoutThreshold_"
          "ReturnsConcept",
          "[vision][glass_filter]") {
  auto dup = find_duplicate_prompt_concept({"wall", "floor", "wall:0.4"});
  REQUIRE(!dup.empty());
  CHECK(dup == "wall");
}

TEST_CASE("BuildGlassConfidenceMap_BackgroundMinusOne_NotSuppressed",
          "[vision][glass_filter]") {
  // Background pixels (-1) must not be suppressed; glass IDs are non-negative.
  cv::Mat label(1, 3, CV_32S);
  label.at<int>(0, 0) = -1; // background
  label.at<int>(0, 1) = 2;  // glass
  label.at<int>(0, 2) = 0;  // structural

  cv::Mat conf = build_glass_confidence_map(label, {2});

  REQUIRE(conf.at<uchar>(0, 0) == 255); // background → trust (not glass)
  REQUIRE(conf.at<uchar>(0, 1) == 0);   // glass → suppress
  REQUIRE(conf.at<uchar>(0, 2) == 255); // structural → trust
}
