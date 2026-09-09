// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Correctness (and rough cost) of the device-side CHW -> HWC transpose that
// replaced the host round-trip in TensorRTSam3p1::rearrange_chw_to_hwc (#254).
//
// The kernel is pure data movement, so the bar is bit-for-bit equality with the
// host loop it replaced — that reference loop is kept verbatim below.

#include <catch2/catch_test_macros.hpp>

#include <vision/tensor_rt/kernels/transpose.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

using reusex::vision::tensor_rt::chw_to_hwc_device_roundtrip;

namespace {

// The exact host transpose that lived in Sam3p1.cpp before #254:
//   spatial [C,H,W] (c*H*W + h*W + w) -> seq-major [H*W,C] ((h*W+w)*C + c)
void reference_chw_to_hwc(const float *src, float *dst, int c, int h, int w) {
  const std::size_t hw = static_cast<std::size_t>(h) * w;
  for (int ci = 0; ci < c; ++ci) {
    const float *row = src + static_cast<std::size_t>(ci) * hw;
    for (std::size_t p = 0; p < hw; ++p)
      dst[p * c + ci] = row[p];
  }
}

// Fixed-seed deterministic fixture. Values are drawn from a wide range so a
// mis-indexed element cannot coincidentally match its neighbour, and the run is
// reproducible across machines (mt19937 is specified exactly).
std::vector<float> make_input(std::size_t n, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist(-1000.0f, 1000.0f);
  std::vector<float> v(n);
  for (std::size_t i = 0; i < n; ++i)
    v[i] = dist(rng);
  return v;
}

// Bit-for-bit comparison (memcmp, not ==): the kernel only moves bytes, so any
// difference at all is a bug, including a signed-zero or NaN payload change.
void check_shape(int c, int h, int w, std::uint32_t seed) {
  const std::size_t n = static_cast<std::size_t>(c) * h * w;
  const std::vector<float> src = make_input(n, seed);

  std::vector<float> expected(n, std::numeric_limits<float>::quiet_NaN());
  reference_chw_to_hwc(src.data(), expected.data(), c, h, w);

  std::vector<float> actual(n, 0.0f);
  const double ms =
      chw_to_hwc_device_roundtrip(src.data(), actual.data(), c, h, w, 1);
  if (ms < 0.0)
    SKIP("no usable CUDA device");

  const bool identical =
      std::memcmp(expected.data(), actual.data(), n * sizeof(float)) == 0;
  INFO("shape C=" << c << " H=" << h << " W=" << w);
  REQUIRE(identical);
}

} // namespace

TEST_CASE("ChwToHwcDeviceRoundtrip_VariousShapes_MatchesHostTransposeBitForBit",
          "[vision][transpose][gpu]") {
  SECTION("degenerate 1x1x1") { check_shape(1, 1, 1, 1u); }
  SECTION("single channel") { check_shape(1, 5, 7, 2u); }
  SECTION("single spatial element") { check_shape(19, 1, 1, 3u); }
  SECTION("tiny non-square") { check_shape(3, 4, 5, 4u); }
  SECTION("odd sizes, sub-tile") { check_shape(7, 13, 11, 5u); }
  SECTION("odd sizes straddling one tile") { check_shape(33, 31, 1, 6u); }
  SECTION("exact tile multiple") { check_shape(32, 32, 32, 7u); }
  SECTION("one past a tile multiple") { check_shape(33, 33, 1, 8u); }
  SECTION("wide and shallow") { check_shape(2, 97, 89, 9u); }
  SECTION("tall and narrow") { check_shape(257, 3, 2, 10u); }
  // The real Sam3p1 memory-conditioning shape: fpn_feat_2 is [256, 72, 72],
  // i.e. 5184 tokens x 256 channels = 1,327,104 floats.
  SECTION("Sam3p1 fpn_feat_2 shape") { check_shape(256, 72, 72, 11u); }
}

TEST_CASE(
    "ChwToHwcDeviceRoundtrip_Sam3p1FeatureShape_RunsFasterThanHostTranspose",
    "[vision][transpose][gpu]") {
  // Rough, informational timing on the real shape. The device number is
  // kernel-only (CUDA events); the host number is the CPU transpose pass alone
  // and therefore *excludes* the D2H/H2D copies and the stream sync that the
  // old path also paid — so it understates the win.
  constexpr int kC = 256;
  constexpr int kH = 72;
  constexpr int kW = 72;
  constexpr int kIters = 200;
  const std::size_t n = static_cast<std::size_t>(kC) * kH * kW;

  const std::vector<float> src = make_input(n, 42u);
  std::vector<float> dst(n, 0.0f);

  const double device_ms =
      chw_to_hwc_device_roundtrip(src.data(), dst.data(), kC, kH, kW, kIters);
  if (device_ms < 0.0)
    SKIP("no usable CUDA device");

  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < kIters; ++i)
    reference_chw_to_hwc(src.data(), dst.data(), kC, kH, kW);
  const auto t1 = std::chrono::steady_clock::now();
  const double host_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count() / kIters;

  std::cout << "[transpose] C=" << kC << " H=" << kH << " W=" << kW << " (" << n
            << " floats), " << kIters << " iterations\n"
            << "[transpose]   device kernel : " << device_ms << " ms/call\n"
            << "[transpose]   host CPU pass : " << host_ms
            << " ms/call (copies excluded)\n"
            << "[transpose]   speedup       : " << (host_ms / device_ms)
            << "x\n";

  // Not a performance gate — just a sanity check that the timing ran.
  REQUIRE(device_ms > 0.0);
  REQUIRE(host_ms > 0.0);
}
