// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the backend-agnostic SAM3 geometry-prompt helpers
// (vision/sam3_geometry.hpp): pixel box -> normalised cxcywh, box polarity,
// and the per-prompt geometry slot layout/mask the TensorRT decoder consumes.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <reusex/vision/sam3_geometry.hpp>

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

namespace vision = reusex::vision;
using Catch::Approx;

TEST_CASE("Sam3Geometry_BoxCxcywh_NormalisesPixelBox", "[vision][sam3]") {
  const auto b = vision::sam3_box_cxcywh({100.f, 50.f, 300.f, 150.f}, 400, 200);
  CHECK(b[0] == Approx(0.5f)); // cx = 200 / 400
  CHECK(b[1] == Approx(0.5f)); // cy = 100 / 200
  CHECK(b[2] == Approx(0.5f)); // w  = 200 / 400
  CHECK(b[3] == Approx(0.5f)); // h  = 100 / 200
}

TEST_CASE("Sam3Geometry_BoxCxcywh_OrdersCornersAndClampsToImage",
          "[vision][sam3]") {
  // Corners given bottom-right first, and partly outside the image.
  const auto b =
      vision::sam3_box_cxcywh({500.f, 300.f, 300.f, -100.f}, 400, 200);
  // Clamped to x in [300, 400], y in [0, 200].
  CHECK(b[0] == Approx(350.f / 400.f));
  CHECK(b[1] == Approx(100.f / 200.f));
  CHECK(b[2] == Approx(100.f / 400.f));
  CHECK(b[3] == Approx(200.f / 200.f));
}

TEST_CASE("Sam3Geometry_BoxCxcywh_DegenerateImageIsZero", "[vision][sam3]") {
  const auto b = vision::sam3_box_cxcywh({1.f, 2.f, 3.f, 4.f}, 0, 0);
  CHECK(b == std::array<float, 4>{0.f, 0.f, 0.f, 0.f});
}

TEST_CASE("Sam3Geometry_BoxLabel_PositiveIsOneEverythingElseZero",
          "[vision][sam3]") {
  CHECK(vision::sam3_box_label("pos") == 1);
  CHECK(vision::sam3_box_label("neg") == 0);
  CHECK(vision::sam3_box_label("") == 0);
}

TEST_CASE("Sam3Geometry_SlotLen_IsBoxesPlusClsOrNothing", "[vision][sam3]") {
  CHECK(vision::sam3_geometry_slot_len(0) == 0);
  CHECK(vision::sam3_geometry_slot_len(1) == 2);
  CHECK(vision::sam3_geometry_slot_len(8) == 9);
}

TEST_CASE("Sam3Geometry_SlotMask_MarksBoxesAndClsValidRestPadding",
          "[vision][sam3]") {
  // 2 boxes in a slot sized for 4 boxes (+ CLS = 5 tokens): the encoder
  // writes 2 box tokens + CLS (valid = 1), the 2 trailing tokens are
  // padding (0). The exported graphs use True == valid.
  CHECK(vision::sam3_geometry_slot_mask(2, 5) ==
        std::vector<std::uint8_t>{1, 1, 1, 0, 0});
  // A full slot has no padding.
  CHECK(vision::sam3_geometry_slot_mask(4, 5) ==
        std::vector<std::uint8_t>{1, 1, 1, 1, 1});
}

TEST_CASE("Sam3Geometry_SlotMask_TextOnlyPromptMasksWholeSlot",
          "[vision][sam3]") {
  // A text-only prompt batched with a boxed one must see no geometry tokens
  // (not even the CLS), so it decodes exactly as on the text-only path.
  CHECK(vision::sam3_geometry_slot_mask(0, 3) ==
        std::vector<std::uint8_t>{0, 0, 0});
  CHECK(vision::sam3_geometry_slot_mask(0, 0).empty());
}

TEST_CASE("Sam3Geometry_SlotMask_ClampsOverfullCount", "[vision][sam3]") {
  CHECK(vision::sam3_geometry_slot_mask(7, 3) ==
        std::vector<std::uint8_t>{1, 1, 1});
}

TEST_CASE("Sam3Geometry_BoxCount_ClampsToCapacity", "[vision][sam3]") {
  CHECK(vision::sam3_geometry_box_count(0, 8) == 0);
  CHECK(vision::sam3_geometry_box_count(3, 8) == 3);
  CHECK(vision::sam3_geometry_box_count(12, 8) == 8);
  CHECK(vision::sam3_geometry_box_count(3, 0) == 0); // geometry disabled
}

TEST_CASE("Sam3Geometry_Capacity_LimitedByEncoderAndDecoderProfiles",
          "[vision][sam3]") {
  // Recipe v2: encoder N in [1, 8], decoder L up to 32 + 8 + 1.
  CHECK(vision::sam3_geometry_capacity(20, 1, 8, 32, 41) == 8);
  // The decoder caps it below the encoder.
  CHECK(vision::sam3_geometry_capacity(20, 1, 8, 32, 36) == 3);
  // Recipe v1: decoder fixed at L=32 -> no room for a single geometry token.
  CHECK(vision::sam3_geometry_capacity(20, 8, 8, 32, 32) == 0);
  // An encoder that cannot take one box (static N=8) is unusable.
  CHECK(vision::sam3_geometry_capacity(20, 8, 8, 32, 41) == 0);
  // The model-side cap still applies.
  CHECK(vision::sam3_geometry_capacity(2, 1, 8, 32, 41) == 2);
}

// --- selecting the detections a box prompt points at ------------------------
//
// SAM3 treats a box as an exemplar and returns every similar object in the
// image; a selection UI wants the object(s) the box actually covers.

namespace {
using Boxes = std::vector<vision::SegmentBox>;
vision::SegmentBox pos(float x1, float y1, float x2, float y2) {
  return {"pos", {x1, y1, x2, y2}};
}
vision::SegmentBox neg(float x1, float y1, float x2, float y2) {
  return {"neg", {x1, y1, x2, y2}};
}
} // namespace

TEST_CASE("Sam3Geometry_DetectionSelected_DetectionMostlyInsidePosBox",
          "[vision][sam3]") {
  const Boxes b{pos(100, 100, 200, 200)};
  CHECK(vision::sam3_detection_selected({110, 110, 190, 190}, b));
  // Slightly larger than the drawn box still counts (>= half inside).
  CHECK(vision::sam3_detection_selected({90, 90, 205, 205}, b));
  // A similar object elsewhere in the image does not.
  CHECK_FALSE(vision::sam3_detection_selected({400, 100, 500, 200}, b));
}

TEST_CASE("Sam3Geometry_DetectionSelected_SmallBoxInsideDetection",
          "[vision][sam3]") {
  // A clicked point (small box) selects the detection that contains it.
  const Boxes b{pos(150, 150, 166, 166)};
  CHECK(vision::sam3_detection_selected({100, 100, 300, 300}, b));
  CHECK_FALSE(vision::sam3_detection_selected({200, 200, 300, 300}, b));
}

TEST_CASE("Sam3Geometry_DetectionSelected_NegativeBoxExcludes",
          "[vision][sam3]") {
  const Boxes b{pos(0, 0, 400, 400), neg(0, 0, 100, 100)};
  CHECK(vision::sam3_detection_selected({200, 200, 300, 300}, b));
  CHECK_FALSE(vision::sam3_detection_selected({10, 10, 90, 90}, b));
}

TEST_CASE("Sam3Geometry_DetectionSelected_NoPositiveBoxKeepsAllButNeg",
          "[vision][sam3]") {
  CHECK(vision::sam3_detection_selected({0, 0, 10, 10}, Boxes{}));
  const Boxes only_neg{neg(0, 0, 100, 100)};
  CHECK(vision::sam3_detection_selected({200, 200, 300, 300}, only_neg));
  CHECK_FALSE(vision::sam3_detection_selected({10, 10, 90, 90}, only_neg));
}

TEST_CASE("Sam3Geometry_DetectionSelected_DegenerateDetectionDropped",
          "[vision][sam3]") {
  const Boxes b{pos(0, 0, 100, 100)};
  CHECK_FALSE(vision::sam3_detection_selected({50, 50, 50, 80}, b));
}

TEST_CASE("Sam3Geometry_DetectionSelected_PointMustBeCovered",
          "[vision][sam3]") {
  const std::vector<std::array<float, 2>> pts{{150.f, 150.f}};
  // Box-contained point: kept.
  CHECK(vision::sam3_detection_selected({100, 100, 200, 200}, Boxes{}, pts));
  // A detection elsewhere is NOT kept just because no point hit it.
  CHECK_FALSE(
      vision::sam3_detection_selected({300, 300, 400, 400}, Boxes{}, pts));
  // Inside the box but outside the mask: dropped.
  CHECK_FALSE(vision::sam3_detection_selected(
      {100, 100, 200, 200}, Boxes{}, pts, [](float, float) { return false; }));
}

TEST_CASE("Sam3Geometry_PointExemplarBox_ScalesWithShorterSideAndClamps",
          "[vision][sam3]") {
  // 720 x 960 image, frac 0.05 -> half-size 36 px.
  const auto b =
      vision::sam3_point_exemplar_box({100.f, 200.f}, 720, 960, 0.05f);
  CHECK(b[0] == Approx(64.f));
  CHECK(b[1] == Approx(164.f));
  CHECK(b[2] == Approx(136.f));
  CHECK(b[3] == Approx(236.f));
  const auto c = vision::sam3_point_exemplar_box({5.f, 955.f}, 720, 960, 0.05f);
  CHECK(c[0] == Approx(0.f));
  CHECK(c[3] == Approx(960.f));
  // Several increasing scales are tried.
  const auto &f = vision::sam3_point_exemplar_fracs();
  REQUIRE(f.size() >= 2);
  CHECK(std::is_sorted(f.begin(), f.end()));
}

TEST_CASE("Sam3Geometry_BestDetectionPerPoint_PicksMostConfidentCovering",
          "[vision][sam3]") {
  // Detections 0..3 (e.g. the same click at four exemplar sizes); one point.
  const std::vector<float> scores{0.86f, 0.90f, 0.96f, 0.92f};
  const std::vector<std::vector<bool>> covers{{true}, {true}, {true}, {false}};
  CHECK(vision::sam3_best_detection_per_point(scores, covers) ==
        std::vector<std::size_t>{2});
}

TEST_CASE("Sam3Geometry_BestDetectionPerPoint_OnePerPointNoDuplicates",
          "[vision][sam3]") {
  const std::vector<float> scores{0.5f, 0.9f, 0.7f};
  // Point 0 covered by 0 and 1; point 1 by 1 and 2; point 2 by none.
  const std::vector<std::vector<bool>> covers{
      {true, false, false}, {true, true, false}, {false, true, false}};
  CHECK(vision::sam3_best_detection_per_point(scores, covers) ==
        std::vector<std::size_t>{1});
  CHECK(vision::sam3_best_detection_per_point({}, {}).empty());
}

TEST_CASE("Sam3Geometry_MaskPolarity_ReadFromTheTextEncodersOwnMask",
          "[vision][sam3]") {
  // Released sam3.1-onnx-v1: text_mask = attention_mask > 0 (True == valid).
  CHECK(vision::sam3_mask_true_is_valid({1, 1, 1, 0, 0, 0}, 3) ==
        std::optional<bool>{true});
  // In-repo exporter: text_mask = attention_mask == 0 (True == padding).
  CHECK(vision::sam3_mask_true_is_valid({0, 0, 0, 1, 1, 1}, 3) ==
        std::optional<bool>{false});
  // Not a clean split: refuse to guess.
  CHECK_FALSE(vision::sam3_mask_true_is_valid({1, 0, 1, 0, 0, 0}, 3));
  CHECK_FALSE(vision::sam3_mask_true_is_valid({1, 1, 1, 1, 1, 1}, 3));
  CHECK_FALSE(vision::sam3_mask_true_is_valid({1, 1, 1}, 3)); // no padding
  CHECK_FALSE(vision::sam3_mask_true_is_valid({}, 0));
}

TEST_CASE("Sam3Geometry_MaskValue_FollowsThePolarity", "[vision][sam3]") {
  CHECK(vision::sam3_mask_value(true, true));
  CHECK_FALSE(vision::sam3_mask_value(false, true));
  CHECK_FALSE(vision::sam3_mask_value(true, false));
  CHECK(vision::sam3_mask_value(false, false));
}

TEST_CASE("Sam3Geometry_PointPromptSelection_IsTheUnionOfPointAndBoxPicks",
          "[vision][sam3]") {
  // d0: best on the point, d1: also on the point but weaker AND selected by
  // the box, d2: selected by the box only, d3: neither.
  const std::vector<float> scores{0.95f, 0.80f, 0.70f, 0.99f};
  const std::vector<std::vector<bool>> covers{{true}, {true}, {false}, {false}};
  const std::vector<bool> box{false, true, true, false};
  // A box hit is never dropped for losing the point contest.
  CHECK(vision::sam3_point_prompt_selection(scores, covers, box) ==
        std::vector<std::size_t>{0, 1, 2});
  // Without box hits it is just the best per point.
  CHECK(vision::sam3_point_prompt_selection(scores, covers,
                                            {false, false, false, false}) ==
        std::vector<std::size_t>{0});
  // A detection that is both is listed once.
  CHECK(vision::sam3_point_prompt_selection(scores, covers,
                                            {true, false, false, false}) ==
        std::vector<std::size_t>{0});
}
