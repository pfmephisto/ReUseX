// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Direct tests for the structural-similarity loss (src/gsplat/ssim.cpp).
//
// SSIM is one of the two terms the trainer actually optimizes
// (`loss = (1-l)*L1 + l*(1 - SSIM)`) and it is the number the held-out
// evaluation reports alongside PSNR, so a sign error or a mis-sized window
// would not crash anything — it would quietly change what "quality" means for
// every figure in docs/research/gaussian-splatting.md. Reference pairs with
// known answers are the only way to catch that.
//
// Deliberately NOT tagged [gpu], for the same reason as test_optimizer.cpp:
// `ssim` is pure torch (a grouped conv2d and some elementwise arithmetic) with
// no rasterizer call in it, so it runs on CPU tensors and needs no device. The
// file lives under cuda/ only because it links torch, which is what puts it in
// the heavy test binary.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "gsplat/ssim.hpp"

#include <cstdint>

using namespace reusex::gsplat::detail;
using Catch::Approx;

namespace {

/// A deterministic [1,3,H,W] "image" in [0,1] with structure in every channel —
/// a plaid of two incommensurable frequencies, so neighbouring windows differ
/// and SSIM has something to measure. A flat image would make every variance
/// term zero and the test vacuous.
torch::Tensor make_image(int64_t h = 48, int64_t w = 64, double phase = 0.0) {
  const auto f32 = torch::TensorOptions(torch::kFloat32);
  auto y = torch::arange(h, f32).unsqueeze(1); // [H,1]
  auto x = torch::arange(w, f32).unsqueeze(0); // [1,W]

  auto c0 = 0.5 + 0.4 * torch::sin(0.21 * x + 0.13 * y + phase);
  auto c1 = 0.5 + 0.3 * torch::cos(0.07 * x - 0.31 * y + phase);
  auto c2 = 0.5 + 0.35 * torch::sin(0.11 * (x + y) + phase);

  return torch::stack({c0.expand({h, w}), c1.expand({h, w}), c2.expand({h, w})})
      .unsqueeze(0) // [1,3,H,W]
      .contiguous();
}

/// A coarse checkerboard of the same shape — structure that shares nothing
/// local with the plaid above.
torch::Tensor make_checkerboard(int64_t h = 48, int64_t w = 64,
                                double cell = 8.0) {
  const auto f32 = torch::TensorOptions(torch::kFloat32);
  auto y = torch::arange(h, f32).unsqueeze(1);
  auto x = torch::arange(w, f32).unsqueeze(0);
  auto tile = (torch::floor(y / cell) + torch::floor(x / cell)).remainder(2.0);
  return tile.unsqueeze(0).expand({3, h, w}).unsqueeze(0).contiguous();
}

double value(const torch::Tensor &t) { return t.item<double>(); }

} // namespace

TEST_CASE("Ssim_IdenticalImages_ReturnsOne", "[gsplat]") {
  auto a = make_image();

  // The upper bound of the metric, and the only value it can take when the
  // means, variances and covariance all agree exactly.
  REQUIRE(value(ssim(a, a)) == Approx(1.0).margin(1e-5));
}

TEST_CASE("Ssim_SwappedArguments_ReturnsSameScore", "[gsplat]") {
  auto a = make_image();
  auto b = make_image(48, 64, /*phase=*/1.7);

  REQUIRE(value(ssim(a, b)) == Approx(value(ssim(b, a))).epsilon(1e-6));
}

TEST_CASE("Ssim_StructurallyUnrelatedImages_ReturnsScoreWellBelowOne",
          "[gsplat]") {
  auto a = make_image();
  // Structure destroyed, not merely shifted: a coarse checkerboard shares no
  // local pattern with the plaid.
  const double s = value(ssim(a, make_checkerboard()));
  INFO("ssim(plaid, checkerboard) = " << s);
  REQUIRE(s < 0.5);
  // Still a bounded similarity, not a diverging number.
  REQUIRE(s > -1.0);
  REQUIRE(s < 1.0);
}

TEST_CASE("Ssim_IncreasingNoise_ReturnsMonotonicallyLowerScore", "[gsplat]") {
  torch::manual_seed(1234);
  auto a = make_image();
  auto noise = torch::randn_like(a);

  const double mild = value(ssim(a, (a + 0.02 * noise).clamp(0.0, 1.0)));
  const double harsh = value(ssim(a, (a + 0.40 * noise).clamp(0.0, 1.0)));

  INFO("mild " << mild << "  harsh " << harsh);
  REQUIRE(mild < 1.0);
  REQUIRE(harsh < mild);
}

TEST_CASE("Ssim_UniformBrightnessShift_StaysHighUnlikeInvertedStructure",
          "[gsplat]") {
  auto a = make_image();
  // A constant offset moves every local mean by the same amount and leaves
  // every variance and the covariance untouched, so the structure term is
  // unchanged and only the luminance term moves — slightly, since C1 is small
  // relative to the signal. This is the defining behaviour that separates SSIM
  // from an L1/PSNR term, and it is why the trainer carries both.
  const double shifted = value(ssim(a, a + 0.05));
  const double inverted = value(ssim(a, 1.0 - a));

  INFO("shifted " << shifted << "  inverted " << inverted);
  REQUIRE(shifted < 1.0);
  REQUIRE(shifted > 0.7);
  // Inverting the image keeps the *magnitude* of the local structure but flips
  // its sign, so the covariance term goes negative and the score collapses —
  // nothing like the near-1.0 a mere brightness shift scores.
  REQUIRE(inverted < 0.2);
  REQUIRE(inverted < shifted);
}
