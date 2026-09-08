// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Covers reusex::vision box post-processing (#205): xyxy_to_xywh(),
// xywh_to_xyxy(), nms() and non_max_suppression(). Every case is a
// hand-constructed CPU float32 tensor with an IoU worked out by hand, so no
// model weights and no GPU are involved.
//
// UNTESTED HERE, and why:
//   * The CUDA path. nms() TORCH_CHECKs that its inputs are on the CPU, and
//     non_max_suppression() inherits the device of its input. Building a CUDA
//     tensor to watch that check fire would need a device (and an [gpu] tag,
//     serializing the case behind the ctest RESOURCE_LOCK) while only
//     re-testing ATen's own dispatch, so the dtype guard stands in for the
//     input-validation contract and this file stays CPU-only and untagged.
//   * The producer of the predictions tensor (Yolo inference) — needs weights.
//     The layout it must emit ([bs, 4 + nc + 32, anchors], xywh boxes) is
//     pinned here by construction instead.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <vision/nms.hpp>

#include <cstdint>
#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::vision::nms;
using reusex::vision::non_max_suppression;
using reusex::vision::xywh_to_xyxy;
using reusex::vision::xyxy_to_xywh;

namespace {

const auto f32 = torch::TensorOptions().dtype(torch::kFloat32);

/// Build an [N, 4] xyxy box tensor from a flat initializer list.
torch::Tensor boxes(std::vector<float> xyxy) {
  const auto n = static_cast<int64_t>(xyxy.size() / 4);
  return torch::from_blob(xyxy.data(), {n, 4}, f32).clone();
}

torch::Tensor scores(std::vector<float> s) {
  const auto n = static_cast<int64_t>(s.size());
  return torch::from_blob(s.data(), {n}, f32).clone();
}

/// Kept indices, in the order nms() returned them.
std::vector<int64_t> kept(const torch::Tensor &keep) {
  std::vector<int64_t> out;
  auto flat = keep.contiguous();
  for (int64_t i = 0; i < flat.numel(); ++i)
    out.push_back(flat[i].item<int64_t>());
  return out;
}

} // namespace

// ── coordinate conversions ────────────────────────────────────────────────

TEST_CASE("xyxy_to_xywh computes centre and extent", "[vision][nms]") {
  auto b = boxes({2.0F, 4.0F, 10.0F, 8.0F});
  auto c = xyxy_to_xywh(b);

  REQUIRE_THAT(c[0][0].item<float>(), WithinAbs(6.0F, 1e-5)); // cx
  REQUIRE_THAT(c[0][1].item<float>(), WithinAbs(6.0F, 1e-5)); // cy
  REQUIRE_THAT(c[0][2].item<float>(), WithinAbs(8.0F, 1e-5)); // w
  REQUIRE_THAT(c[0][3].item<float>(), WithinAbs(4.0F, 1e-5)); // h
}

TEST_CASE("xywh_to_xyxy is the inverse of xyxy_to_xywh", "[vision][nms]") {
  auto original = boxes({2.0F, 4.0F, 10.0F, 8.0F, -3.0F, 0.5F, 1.0F, 6.5F});
  auto round_trip = xywh_to_xyxy(xyxy_to_xywh(original));

  REQUIRE(torch::allclose(round_trip, original, 1e-5, 1e-5));
}

// ── nms: degenerate inputs ────────────────────────────────────────────────

TEST_CASE("nms on empty input returns an empty long tensor", "[vision][nms]") {
  auto b = torch::empty({0, 4}, f32);
  auto s = torch::empty({0}, f32);

  auto keep = nms(b, s, 0.45F);

  REQUIRE(keep.numel() == 0);
  REQUIRE(keep.dtype() == torch::kLong);
}

TEST_CASE("nms on a single box keeps it", "[vision][nms]") {
  auto keep = nms(boxes({0.0F, 0.0F, 10.0F, 10.0F}), scores({0.9F}), 0.45F);
  REQUIRE(kept(keep) == std::vector<int64_t>{0});
}

TEST_CASE("nms keeps a single survivor when every box is identical",
          "[vision][nms]") {
  // IoU 1.0 between all pairs, so everything but the top-scoring box goes.
  auto b = boxes({0, 0, 4, 4, 0, 0, 4, 4, 0, 0, 4, 4});
  auto keep = nms(b, scores({0.3F, 0.9F, 0.6F}), 0.45F);
  REQUIRE(kept(keep) == std::vector<int64_t>{1});
}

// ── nms: known overlaps -> known survivors ────────────────────────────────

TEST_CASE("nms suppresses an overlapping box and keeps a disjoint one",
          "[vision][nms]") {
  // A = (0,0,10,10) area 100, score .9
  // B = (1,1,11,11) area 100, score .8 -> inter 9*9=81, union 119, IoU 0.681
  // C = (20,20,30,30)          score .7 -> disjoint from both, IoU 0
  auto b = boxes({0, 0, 10, 10, 1, 1, 11, 11, 20, 20, 30, 30});
  auto keep = nms(b, scores({0.9F, 0.8F, 0.7F}), 0.45F);

  REQUIRE(kept(keep) == std::vector<int64_t>{0, 2});
}

TEST_CASE("nms keeps by score, not by input order", "[vision][nms]") {
  // Same two heavily-overlapping boxes, but the low score comes first.
  auto b = boxes({0, 0, 10, 10, 1, 1, 11, 11});
  auto keep = nms(b, scores({0.1F, 0.9F}), 0.45F);

  REQUIRE(kept(keep) == std::vector<int64_t>{1});
}

TEST_CASE("nms breaks score ties by lowest index", "[vision][nms]") {
  // The sort is stable, so equal scores preserve input order and index 0 wins.
  auto b = boxes({0, 0, 10, 10, 1, 1, 11, 11});
  auto keep = nms(b, scores({0.5F, 0.5F}), 0.45F);

  REQUIRE(kept(keep) == std::vector<int64_t>{0});
}

TEST_CASE("nms suppression is transitive through the kept box",
          "[vision][nms]") {
  // A(.9) suppresses B(.8); C(.7) overlaps B but not A, so C survives —
  // suppression radiates from kept boxes only.
  //   A = (0,0,10,10)            C = (12,0,22,10) -> IoU(A,C) = 0
  //   B = (6,0,16,10) -> IoU(A,B) = 4*10/(100+100-40) = 0.25
  //                     IoU(B,C) = 4*10/(100+100-40) = 0.25
  auto b = boxes({0, 0, 10, 10, 6, 0, 16, 10, 12, 0, 22, 10});
  auto keep = nms(b, scores({0.9F, 0.8F, 0.7F}), 0.20F);

  REQUIRE(kept(keep) == std::vector<int64_t>{0, 2});
}

// ── nms: IoU threshold edges ──────────────────────────────────────────────

TEST_CASE("nms treats the IoU threshold as exclusive", "[vision][nms]") {
  // A = (0,0,3,2) area 6, B = (1,0,4,2) area 6.
  // inter = 2*2 = 4, union = 6 + 6 - 4 = 8, IoU = exactly 0.5 in float.
  auto b = boxes({0, 0, 3, 2, 1, 0, 4, 2});
  auto s = scores({0.9F, 0.8F});

  SECTION("IoU exactly at the threshold does not suppress (ovr > thr)") {
    REQUIRE(kept(nms(b, s, 0.5F)) == std::vector<int64_t>{0, 1});
  }
  SECTION("just below the threshold suppresses") {
    REQUIRE(kept(nms(b, s, 0.49F)) == std::vector<int64_t>{0});
  }
  SECTION("just above the threshold does not suppress") {
    REQUIRE(kept(nms(b, s, 0.51F)) == std::vector<int64_t>{0, 1});
  }
}

TEST_CASE("nms keeps edge-touching boxes even at threshold zero",
          "[vision][nms]") {
  // Shared edge at x=2 gives zero intersection area, so IoU is 0 and the
  // exclusive comparison (0 > 0 is false) keeps both.
  auto b = boxes({0, 0, 2, 2, 2, 0, 4, 2});
  auto keep = nms(b, scores({0.9F, 0.8F}), 0.0F);

  REQUIRE(kept(keep) == std::vector<int64_t>{0, 1});
}

TEST_CASE("nms with threshold zero suppresses any real overlap",
          "[vision][nms]") {
  auto b = boxes({0, 0, 2, 2, 1, 1, 3, 3});
  auto keep = nms(b, scores({0.9F, 0.8F}), 0.0F);

  REQUIRE(kept(keep) == std::vector<int64_t>{0});
}

TEST_CASE("nms with threshold one suppresses nothing at all", "[vision][nms]") {
  // IoU can never exceed 1 and the comparison is exclusive, so not even the
  // exact duplicate at index 2 is dropped.
  auto b = boxes({0, 0, 10, 10, 1, 1, 11, 11, 0, 0, 10, 10});
  auto keep = nms(b, scores({0.9F, 0.8F, 0.7F}), 1.0F);

  REQUIRE(kept(keep) == std::vector<int64_t>{0, 1, 2});
}

TEST_CASE("nms rejects non-float32 input loudly", "[vision][nms]") {
  auto b = torch::zeros({2, 4}, torch::TensorOptions().dtype(torch::kFloat64));
  auto s = torch::zeros({2}, torch::TensorOptions().dtype(torch::kFloat64));

  // TORCH_CHECK raises c10::Error, which derives from std::exception.
  REQUIRE_THROWS_AS(nms(b, s, 0.45F), std::exception);
}

// ── shape validation, gained with the torchvision kernel (#141) ───────────
//
// The kernel vendored in #141 validates box/score geometry before it starts
// indexing raw pointers; the previous hand-written transcription had dropped
// those checks and would have walked off the end of the buffer instead. These
// cases pin the diagnostics down so a future re-sync cannot quietly lose them.

TEST_CASE("nms rejects malformed box and score shapes", "[vision][nms]") {
  SECTION("boxes must be 2-dimensional") {
    REQUIRE_THROWS_AS(
        nms(torch::zeros({3, 4, 1}, f32), torch::zeros({3}, f32), 0.45F),
        std::exception);
  }

  SECTION("boxes must have exactly four columns") {
    REQUIRE_THROWS_AS(
        nms(torch::zeros({3, 5}, f32), torch::zeros({3}, f32), 0.45F),
        std::exception);
  }

  SECTION("scores must be 1-dimensional") {
    REQUIRE_THROWS_AS(
        nms(torch::zeros({3, 4}, f32), torch::zeros({3, 1}, f32), 0.45F),
        std::exception);
  }

  SECTION("boxes and scores must agree on N") {
    REQUIRE_THROWS_AS(
        nms(torch::zeros({3, 4}, f32), torch::zeros({2}, f32), 0.45F),
        std::exception);
  }
}

// ── scale ─────────────────────────────────────────────────────────────────

TEST_CASE("nms handles more than a thousand boxes", "[vision][nms]") {
  // A grid of 40 x 30 = 1200 boxes on a 100-unit pitch: every box is disjoint
  // from every other, so nothing may be suppressed however large N gets. This
  // exercises the kernel past the small hand-built cases above, where an
  // indexing or narrow() bug would otherwise hide.
  constexpr int64_t kCols = 40;
  constexpr int64_t kRows = 30;
  constexpr int64_t kN = kCols * kRows;

  std::vector<float> xyxy;
  std::vector<float> conf;
  xyxy.reserve(static_cast<size_t>(kN) * 4);
  conf.reserve(static_cast<size_t>(kN));
  for (int64_t r = 0; r < kRows; ++r) {
    for (int64_t c = 0; c < kCols; ++c) {
      const auto x = static_cast<float>(c * 100);
      const auto y = static_cast<float>(r * 100);
      xyxy.insert(xyxy.end(), {x, y, x + 50.0F, y + 50.0F});
      // Strictly decreasing scores, so the expected output order is exactly
      // the input order and we can compare against it directly.
      conf.push_back(1.0F - static_cast<float>(r * kCols + c) / (2.0F * kN));
    }
  }

  auto keep = nms(boxes(xyxy), scores(conf), 0.45F);
  REQUIRE(keep.numel() == kN);

  std::vector<int64_t> expected(kN);
  for (int64_t i = 0; i < kN; ++i)
    expected[i] = i;
  REQUIRE(kept(keep) == expected);

  SECTION("and collapses a large fully-overlapping set to one box") {
    // Same count, but every box identical: the opposite extreme, where the
    // inner suppression loop has to run to completion for the first box only.
    std::vector<float> same(static_cast<size_t>(kN) * 4);
    for (int64_t i = 0; i < kN; ++i) {
      same[static_cast<size_t>(i) * 4 + 2] = 10.0F;
      same[static_cast<size_t>(i) * 4 + 3] = 10.0F;
    }
    auto one = nms(boxes(same), scores(conf), 0.45F);
    REQUIRE(kept(one) == std::vector<int64_t>{0});
  }
}

// ── non_max_suppression: the YOLO-seg wrapper ─────────────────────────────

namespace {

constexpr int64_t kMaskCoeffs = 32; // YOLO-seg default, hard-coded in nms.cpp

/// Assemble a [1, 4 + nc + 32, N] YOLO-seg style prediction tensor — the exact
/// layout non_max_suppression() documents as its input.
/// @param xywh    per-anchor box in (cx, cy, w, h), 4 entries each
/// @param cls     per-anchor class scores, nc entries each
/// @param nc      number of classes
torch::Tensor predictions(const std::vector<std::vector<float>> &xywh,
                          const std::vector<std::vector<float>> &cls,
                          int64_t nc) {
  const auto n = static_cast<int64_t>(xywh.size());
  const int64_t channels = 4 + nc + kMaskCoeffs;
  // Channel-major, matching the [1, C, N] layout.
  std::vector<float> data(static_cast<size_t>(channels * n), 0.0F);
  const auto at = [&](int64_t c, int64_t a) -> float & {
    return data[static_cast<size_t>(c * n + a)];
  };

  for (int64_t a = 0; a < n; ++a) {
    for (int64_t k = 0; k < 4; ++k)
      at(k, a) = xywh[static_cast<size_t>(a)][static_cast<size_t>(k)];
    for (int64_t c = 0; c < nc; ++c)
      at(4 + c, a) = cls[static_cast<size_t>(a)][static_cast<size_t>(c)];
    // Mask coefficients are carried through untouched; stamp the anchor index
    // into the first one so we can prove rows are not reordered or blended.
    at(4 + nc, a) = static_cast<float>(a);
  }

  return torch::from_blob(data.data(), {1, channels, n}, f32).clone();
}

} // namespace

TEST_CASE("non_max_suppression returns the documented output shape",
          "[vision][nms]") {
  const int64_t nc = 2;
  auto p = predictions({{5, 5, 4, 4}}, {{0.9F, 0.1F}}, nc);

  auto out = non_max_suppression(p, 0.25F, 0.45F, /*maxDetections=*/10);

  REQUIRE(out.size(0) == 1);
  REQUIRE(out.size(1) == 10);
  REQUIRE(out.size(2) == 6 + kMaskCoeffs);
}

TEST_CASE("non_max_suppression emits xyxy boxes with class and confidence",
          "[vision][nms]") {
  const int64_t nc = 2;
  // One anchor, class 1 with confidence 0.8, box centred (5,5) sized 4x4.
  auto p = predictions({{5, 5, 4, 4}}, {{0.1F, 0.8F}}, nc);

  auto out = non_max_suppression(p, 0.25F, 0.45F, 10);
  auto row = out[0][0];

  REQUIRE_THAT(row[0].item<float>(), WithinAbs(3.0F, 1e-5)); // x1
  REQUIRE_THAT(row[1].item<float>(), WithinAbs(3.0F, 1e-5)); // y1
  REQUIRE_THAT(row[2].item<float>(), WithinAbs(7.0F, 1e-5)); // x2
  REQUIRE_THAT(row[3].item<float>(), WithinAbs(7.0F, 1e-5)); // y2
  REQUIRE_THAT(row[4].item<float>(), WithinAbs(0.8F, 1e-5)); // confidence
  REQUIRE_THAT(row[5].item<float>(), WithinAbs(1.0F, 1e-5)); // class id

  // Unfilled detection slots stay zeroed.
  REQUIRE_THAT(out[0][1][4].item<float>(), WithinAbs(0.0F, 1e-5));
}

TEST_CASE("non_max_suppression drops anchors below the confidence threshold",
          "[vision][nms]") {
  const int64_t nc = 2;
  auto p = predictions({{5, 5, 4, 4}, {50, 50, 4, 4}},
                       {{0.9F, 0.0F}, {0.1F, 0.05F}}, nc);

  auto out = non_max_suppression(p, /*confThreshold=*/0.25F, 0.45F, 10);

  // Only the high-confidence anchor survives; slot 1 is untouched zeros.
  REQUIRE_THAT(out[0][0][4].item<float>(), WithinAbs(0.9F, 1e-5));
  REQUIRE_THAT(out[0][1][4].item<float>(), WithinAbs(0.0F, 1e-5));
}

TEST_CASE("non_max_suppression yields nothing when all anchors are below "
          "threshold",
          "[vision][nms]") {
  const int64_t nc = 2;
  auto p = predictions({{5, 5, 4, 4}, {6, 6, 4, 4}},
                       {{0.1F, 0.05F}, {0.2F, 0.0F}}, nc);

  auto out = non_max_suppression(p, 0.25F, 0.45F, 10);

  REQUIRE_THAT(out.abs().sum().item<float>(), WithinAbs(0.0F, 1e-5));
}

TEST_CASE("non_max_suppression is class-aware", "[vision][nms]") {
  const int64_t nc = 2;
  // Two anchors on the *same* box. Same class -> one survives; different
  // classes -> both survive, because boxes are offset per class before NMS.
  SECTION("same class collapses to one detection") {
    auto p = predictions({{5, 5, 4, 4}, {5, 5, 4, 4}},
                         {{0.9F, 0.0F}, {0.8F, 0.0F}}, nc);
    auto out = non_max_suppression(p, 0.25F, 0.45F, 10);

    REQUIRE_THAT(out[0][0][4].item<float>(), WithinAbs(0.9F, 1e-5));
    REQUIRE_THAT(out[0][1][4].item<float>(), WithinAbs(0.0F, 1e-5));
  }
  SECTION("different classes both survive") {
    auto p = predictions({{5, 5, 4, 4}, {5, 5, 4, 4}},
                         {{0.9F, 0.0F}, {0.0F, 0.8F}}, nc);
    auto out = non_max_suppression(p, 0.25F, 0.45F, 10);

    REQUIRE_THAT(out[0][0][5].item<float>(), WithinAbs(0.0F, 1e-5));
    REQUIRE_THAT(out[0][1][5].item<float>(), WithinAbs(1.0F, 1e-5));
    // The emitted boxes are the original ones — the class offset used for
    // suppression must not leak into the output.
    REQUIRE_THAT(out[0][1][0].item<float>(), WithinAbs(3.0F, 1e-5));
  }
}

TEST_CASE("non_max_suppression carries mask coefficients through unchanged",
          "[vision][nms]") {
  const int64_t nc = 2;
  // Anchor 1 outscores anchor 0 and they are disjoint, so both survive in
  // score order: row 0 must carry anchor 1's stamp, row 1 anchor 0's.
  auto p = predictions({{5, 5, 4, 4}, {50, 50, 4, 4}},
                       {{0.6F, 0.0F}, {0.9F, 0.0F}}, nc);

  auto out = non_max_suppression(p, 0.25F, 0.45F, 10);

  REQUIRE_THAT(out[0][0][6].item<float>(), WithinAbs(1.0F, 1e-5));
  REQUIRE_THAT(out[0][1][6].item<float>(), WithinAbs(0.0F, 1e-5));
}

TEST_CASE("non_max_suppression honours maxDetections", "[vision][nms]") {
  const int64_t nc = 1;
  // Four disjoint, confident boxes but only two slots.
  auto p =
      predictions({{5, 5, 4, 4}, {50, 5, 4, 4}, {5, 50, 4, 4}, {50, 50, 4, 4}},
                  {{0.9F}, {0.8F}, {0.7F}, {0.6F}}, nc);

  auto out = non_max_suppression(p, 0.25F, 0.45F, /*maxDetections=*/2);

  REQUIRE(out.size(1) == 2);
  REQUIRE_THAT(out[0][0][4].item<float>(), WithinAbs(0.9F, 1e-5));
  REQUIRE_THAT(out[0][1][4].item<float>(), WithinAbs(0.8F, 1e-5));
}

TEST_CASE("non_max_suppression handles a batch of more than one image",
          "[vision][nms]") {
  const int64_t nc = 1;
  auto a = predictions({{5, 5, 4, 4}}, {{0.9F}}, nc);
  auto b = predictions({{9, 9, 2, 2}}, {{0.4F}}, nc);
  auto p = torch::cat({a, b}, 0);

  auto out = non_max_suppression(p, 0.25F, 0.45F, 10);

  REQUIRE(out.size(0) == 2);
  REQUIRE_THAT(out[0][0][4].item<float>(), WithinAbs(0.9F, 1e-5));
  REQUIRE_THAT(out[1][0][4].item<float>(), WithinAbs(0.4F, 1e-5));
  REQUIRE_THAT(out[1][0][0].item<float>(), WithinAbs(8.0F, 1e-5));
}
