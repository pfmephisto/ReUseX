// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Direct tests for the CPU view-sampling helpers
// (src/gsplat/view_sampling.cpp).
//
// These three functions decide what a training run is measured against:
// `split_views` is the only thing that makes "held-out PSNR" a real claim,
// `stride_sample` decides whether an evaluation pass looks at the whole
// trajectory or at a contiguous prefix of it, and `scene_extent` scales the
// position learning rate. All three have exact, checkable answers — and until
// #332 none of them could be asserted on except through a full CUDA training
// run, which is to say not in CI at all.
//
// Deliberately in the LIGHT test binary and NOT tagged [gpu]: this file pulls
// in no torch and no rasterizer, only reusex_gsplat_common, so it compiles and
// runs in the CPU-only build CI actually performs. It reaches into the
// module-private header the same way tests/unit/gsplat/cuda/test_optimizer.cpp
// does — `libs/reusex/src` is on the test target's include path.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "gsplat/view_sampling.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

using namespace reusex::gsplat;
using Catch::Approx;

namespace {

/// A `TrainingView` whose camera centre is exactly @p centre.
///
/// `T_cw` is world -> camera, so the centre enters as `t = -R * c`; with
/// @p R = I that is simply `-c`. Building the pose explicitly (rather than
/// composing a look-at) keeps the expected extent a hand-computable number.
TrainingView view_at(const Eigen::Vector3d &centre,
                     const Eigen::Matrix3d &R = Eigen::Matrix3d::Identity()) {
  TrainingView v;
  v.T_cw = Eigen::Matrix4d::Identity();
  v.T_cw.block<3, 3>(0, 0) = R;
  v.T_cw.block<3, 1>(0, 3) = -R * centre;
  return v;
}

/// `n` cameras evenly spaced on a circle of radius `r` in the z = 0 plane,
/// centred on the origin. The centroid of the ring is the origin, so
/// `scene_extent` must come back as exactly `r`.
std::vector<TrainingView> camera_ring(int n, double r) {
  std::vector<TrainingView> views;
  views.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * M_PI * i / n;
    views.push_back(view_at({r * std::cos(a), r * std::sin(a), 0.0}));
  }
  return views;
}

/// 0..n-1, the index list `stride_sample` is normally handed.
std::vector<std::size_t> iota(std::size_t n) {
  std::vector<std::size_t> v(n);
  std::iota(v.begin(), v.end(), std::size_t{0});
  return v;
}

} // namespace

// ---------------------------------------------------------------------------
// split_views
// ---------------------------------------------------------------------------

TEST_CASE("SplitViews_PositiveHoldoutEvery_HoldsOutEveryNthViewIndex",
          "[gsplat]") {
  const auto s = detail::split_views(20, 4);

  // Indices 0, 4, 8, 12, 16 are held out; everything else trains.
  REQUIRE(s.holdout == std::vector<std::size_t>{0, 4, 8, 12, 16});
  REQUIRE(s.train.size() == 15);
  REQUIRE(s.train.size() + s.holdout.size() == 20);

  // The two sets are disjoint — the whole point of the split.
  for (const std::size_t h : s.holdout)
    REQUIRE(std::find(s.train.begin(), s.train.end(), h) == s.train.end());
  for (const std::size_t t : s.train)
    REQUIRE(t % 4 != 0);
}

TEST_CASE("SplitViews_HoldoutEveryZero_TrainsOnEveryView", "[gsplat]") {
  const auto s = detail::split_views(7, 0);

  REQUIRE(s.holdout.empty());
  REQUIRE(s.train == iota(7));
}

TEST_CASE("SplitViews_DegenerateSplit_FallsBackToTrainingOnEveryView",
          "[gsplat]") {
  SECTION("holdout_every == 1 would hold out every view") {
    const auto s = detail::split_views(6, 1);
    // The degenerate fallback: train on all views, report no held-out metric.
    REQUIRE(s.train == iota(6));
    REQUIRE(s.holdout.empty());
  }

  SECTION("a single view cannot be both trained on and held out") {
    const auto s = detail::split_views(1, 8);
    REQUIRE(s.train == std::vector<std::size_t>{0});
    REQUIRE(s.holdout.empty());
  }
}

TEST_CASE("SplitViews_RepeatedCalls_ReturnIdenticalSplit", "[gsplat]") {
  // Membership is a pure function of the view's position in the list, with no
  // RNG anywhere in it (STANDARDS §6) — which is what lets a prune-only run and
  // an MCMC run be compared on identical held-out images.
  const auto a = detail::split_views(37, 8);
  const auto b = detail::split_views(37, 8);

  REQUIRE(a.train == b.train);
  REQUIRE(a.holdout == b.holdout);
}

// ---------------------------------------------------------------------------
// stride_sample
// ---------------------------------------------------------------------------

TEST_CASE("StrideSample_InputWithinBudget_ReturnsInputUnchanged", "[gsplat]") {
  const auto src = iota(10);

  SECTION("under budget") { REQUIRE(detail::stride_sample(src, 32) == src); }
  SECTION("exactly at budget") {
    REQUIRE(detail::stride_sample(src, 10) == src);
  }
  SECTION("max_n <= 0 means no budget at all") {
    REQUIRE(detail::stride_sample(src, 0) == src);
    REQUIRE(detail::stride_sample(src, -1) == src);
  }
  SECTION("an empty input stays empty") {
    REQUIRE(detail::stride_sample({}, 8).empty());
  }
}

TEST_CASE("StrideSample_InputOverBudget_SpreadsSamplesAcrossWholeRange",
          "[gsplat]") {
  const auto out = detail::stride_sample(iota(100), 10);

  REQUIRE(out.size() == 10);
  // Keeps the first entry, so iteration 0 of an evaluation always sees the
  // same anchor view.
  REQUIRE(out.front() == 0);

  // Strictly increasing: a stride sampler must not repeat or reorder.
  for (std::size_t i = 1; i < out.size(); ++i)
    REQUIRE(out[i] > out[i - 1]);

  // The point of striding rather than truncating: the last sample has to come
  // from the far end of the trajectory, not from a contiguous prefix. With
  // step = 100/10 = 10 the last index is 90.
  REQUIRE(out.back() == 90);
  REQUIRE(out.back() >= 50);
}

TEST_CASE("StrideSample_RaggedBudget_ReturnsExactlyMaxNEntries", "[gsplat]") {
  // 37 / 8 = 4 (integer division), so the stride is 4 and the sampler must
  // stop at 8 entries rather than running off the end of the input.
  const auto out = detail::stride_sample(iota(37), 8);

  REQUIRE(out.size() == 8);
  REQUIRE(out.front() == 0);
  REQUIRE(out.back() == 28);
  for (const std::size_t i : out)
    REQUIRE(i < 37);
}

// ---------------------------------------------------------------------------
// scene_extent
// ---------------------------------------------------------------------------

TEST_CASE("SceneExtent_CameraRing_ReturnsRingRadius", "[gsplat]") {
  constexpr double r = 3.5;
  REQUIRE(detail::scene_extent(camera_ring(12, r)) == Approx(r).epsilon(1e-9));
}

TEST_CASE("SceneExtent_RotatedCamerasInPlace_ReturnsUnchangedRadius",
          "[gsplat]") {
  // The extent is a property of where the cameras *are*, not where they look:
  // it comes from -R^T t, so rotating each camera in place must not move it.
  constexpr double r = 2.0;
  auto views = camera_ring(6, r);
  for (std::size_t i = 0; i < views.size(); ++i) {
    const Eigen::Matrix3d R(Eigen::AngleAxisd(0.3 * static_cast<double>(i),
                                              Eigen::Vector3d::UnitY())
                                .toRotationMatrix());
    const Eigen::Vector3d centre =
        -views[i].T_cw.block<3, 3>(0, 0).transpose() *
        views[i].T_cw.block<3, 1>(0, 3);
    views[i] = view_at(centre, R);
  }
  REQUIRE(detail::scene_extent(views) == Approx(r).epsilon(1e-9));
}

TEST_CASE("SceneExtent_TwoCameras_ReturnsHalfTheirSeparation", "[gsplat]") {
  // The centroid sits midway between them, so the radius is d/2.
  const std::vector<TrainingView> views = {view_at({-4.0, 0.0, 0.0}),
                                           view_at({4.0, 0.0, 0.0})};
  REQUIRE(detail::scene_extent(views) == Approx(4.0).epsilon(1e-9));
}

TEST_CASE("SceneExtent_DegenerateCapture_ReturnsMinimumFloor", "[gsplat]") {
  // A single viewpoint has zero radius, and the trainer multiplies the position
  // learning rate by this number — a literal 0 would freeze the means for the
  // whole run. The documented floor is 1e-3 m.
  SECTION("one view") {
    REQUIRE(detail::scene_extent({view_at({10.0, -2.0, 7.0})}) ==
            Approx(1e-3).epsilon(1e-9));
  }
  SECTION("several co-located views") {
    const std::vector<TrainingView> views = {view_at({1.0, 1.0, 1.0}),
                                             view_at({1.0, 1.0, 1.0}),
                                             view_at({1.0, 1.0, 1.0})};
    REQUIRE(detail::scene_extent(views) == Approx(1e-3).epsilon(1e-9));
  }
  SECTION("no views at all") {
    REQUIRE(detail::scene_extent({}) == Approx(1e-3).epsilon(1e-9));
  }
}

// --- SH warm-up schedule (#240) --------------------------------------------
// `active_sh_degree` decides how expressive the model is allowed to be at each
// iteration. It lives here, with the other pure schedule helpers, so the
// CPU-only CI build can assert on it — the alternative is inferring it from a
// CUDA training log.

TEST_CASE("ActiveShDegree_UnlocksOneBandPerInterval", "[gsplat]") {
  // Degree 3, one band per 1000 iterations: DC only until 1000, then up.
  REQUIRE(detail::active_sh_degree(0, 3, 1000) == 0);
  REQUIRE(detail::active_sh_degree(999, 3, 1000) == 0);
  REQUIRE(detail::active_sh_degree(1000, 3, 1000) == 1);
  REQUIRE(detail::active_sh_degree(1999, 3, 1000) == 1);
  REQUIRE(detail::active_sh_degree(2000, 3, 1000) == 2);
  REQUIRE(detail::active_sh_degree(3000, 3, 1000) == 3);
  // Never past the model's own degree, however long the run.
  REQUIRE(detail::active_sh_degree(3001, 3, 1000) == 3);
  REQUIRE(detail::active_sh_degree(1'000'000, 3, 1000) == 3);
}

TEST_CASE("ActiveShDegree_DegreeZeroModel_StaysAtZero", "[gsplat]") {
  // The default model has no higher bands to unlock; no schedule setting may
  // conjure one, or render() would be asked for a degree the tensors cannot
  // supply.
  for (const int interval : {0, 1, 1000})
    for (const int it : {0, 1, 500, 100000})
      REQUIRE(detail::active_sh_degree(it, 0, interval) == 0);
  REQUIRE(detail::active_sh_degree(10, -1, 100) == 0);
}

TEST_CASE("ActiveShDegree_NoWarmUp_EveryBandLiveImmediately", "[gsplat]") {
  // interval <= 0 is the documented "train all bands from iteration 0" escape
  // hatch, and it must not divide by zero on the way there.
  REQUIRE(detail::active_sh_degree(0, 3, 0) == 3);
  REQUIRE(detail::active_sh_degree(0, 3, -5) == 3);
  REQUIRE(detail::active_sh_degree(12345, 2, 0) == 2);
}

TEST_CASE("ActiveShDegree_Monotonic_NeverRegresses", "[gsplat]") {
  // A band that unlocks must stay unlocked: dropping back would silently zero
  // out coefficients the optimizer had already fitted.
  int previous = 0;
  for (int it = 0; it <= 5000; ++it) {
    const int d = detail::active_sh_degree(it, 3, 700);
    REQUIRE(d >= previous);
    REQUIRE(d <= 3);
    previous = d;
  }
  REQUIRE(previous == 3);
}
