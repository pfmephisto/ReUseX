// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the plane-fit measurement noise model (issue #225).
//
// `plane_fit_sigmas` decides how much authority a single plane observation
// gets in the pose graph. Before this model existed, one scalar
// `sqrt(median_N / N)` scaled BOTH the normal and the distance channel, so the
// only quality signal was inlier count. These tests pin the three properties
// that motivated the change:
//
//   * support   — more inliers tighten both channels as 1/sqrt(N);
//   * roughness — a rougher fit loosens both channels in proportion;
//   * extent    — a smaller footprint loosens ONLY the normal.
//
// The last one is the whole point: two detections with identical inlier count
// and identical residual RMS but very different in-plane extent are equally
// certain about WHERE the plane is and very unequally certain about HOW it is
// oriented. A single shared scalar cannot express that.
//
// The function is pure and gtsam-free, so these cases are exact and fast.

#include <reusex/slam/PlaneGraphOptimizer.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using Catch::Matchers::WithinRel;
using reusex::geometry::plane_fit_sigmas;
using reusex::geometry::PlaneFitQuality;

namespace {

// A representative, well-behaved detection: 1000 surfels on a 0.8 m-extent
// surface fitted to 8 mm RMS — roughly what an iPad-LiDAR frame yields on a
// painted wall at a couple of metres.
PlaneFitQuality nominal() {
  PlaneFitQuality q;
  q.inliers = 1000;
  q.residual_rms = 0.008;
  q.extent_minor = 0.8;
  return q;
}

} // namespace

TEST_CASE("PlaneFitSigmas_QuadruplingSupport_HalvesBothSigmas",
          "[plane_graph][noise]") {
  // Both channels are averages over the inliers, so both tighten as 1/sqrt(N).
  // This is the part the legacy inlier-count model already got right.
  const auto base = plane_fit_sigmas(nominal());

  PlaneFitQuality denser = nominal();
  denser.inliers = 4 * nominal().inliers;
  const auto dense = plane_fit_sigmas(denser);

  REQUIRE_THAT(dense.normal, WithinRel(base.normal / 2.0, 1e-12));
  REQUIRE_THAT(dense.distance, WithinRel(base.distance / 2.0, 1e-12));
}

TEST_CASE("PlaneFitSigmas_RougherFit_LoosensBothChannelsEqually",
          "[plane_graph][noise]") {
  // Residual RMS is the point-noise term sigma, which enters both channels
  // linearly — so doubling it doubles both and leaves their ratio alone. This
  // signal was previously discarded entirely: a cluttered bookshelf front and a
  // bare wall with the same inlier count used to carry identical authority.
  const auto base = plane_fit_sigmas(nominal());

  PlaneFitQuality rough = nominal();
  rough.residual_rms = 2.0 * nominal().residual_rms;
  const auto worse = plane_fit_sigmas(rough);

  REQUIRE_THAT(worse.normal, WithinRel(2.0 * base.normal, 1e-12));
  REQUIRE_THAT(worse.distance, WithinRel(2.0 * base.distance, 1e-12));
  REQUIRE_THAT(worse.normal / worse.distance,
               WithinRel(base.normal / base.distance, 1e-12));
}

TEST_CASE("PlaneFitSigmas_SmallerFootprint_LoosensOnlyNormalSigma",
          "[plane_graph][noise]") {
  // The defining property of the model, and the one no single scalar can
  // express. Extent is a lever arm: it divides the tilt uncertainty and does
  // not appear in the offset uncertainty at all.
  const auto wall = plane_fit_sigmas(nominal());

  PlaneFitQuality patch = nominal(); // same support, same roughness
  patch.extent_minor = nominal().extent_minor / 8.0;
  const auto small = plane_fit_sigmas(patch);

  // Normal: 8x less certain.
  REQUIRE_THAT(small.normal, WithinRel(8.0 * wall.normal, 1e-12));
  // Distance: unchanged. A small patch still knows exactly where it is.
  REQUIRE_THAT(small.distance, WithinRel(wall.distance, 1e-12));
}

TEST_CASE("PlaneFitSigmas_UniformRoughnessAndExtent_"
          "MatchesLegacyInlierWeighting",
          "[plane_graph][noise]") {
  // The legacy model scaled every observation by sqrt(median_N / N). That is
  // exactly what this model reduces to when roughness and extent are uniform
  // across detections — which is why sqrt(N) was a defensible first
  // approximation, and why switching models needs no retuning of
  // plane_sigma_normal / plane_sigma_distance.
  PlaneFitQuality a = nominal();
  PlaneFitQuality b = nominal();
  b.inliers = 250; // 1/4 the support, everything else identical

  const auto sa = plane_fit_sigmas(a);
  const auto sb = plane_fit_sigmas(b);

  const double legacy_ratio = std::sqrt(static_cast<double>(a.inliers) /
                                        static_cast<double>(b.inliers));
  REQUIRE_THAT(sb.normal / sa.normal, WithinRel(legacy_ratio, 1e-12));
  REQUIRE_THAT(sb.distance / sa.distance, WithinRel(legacy_ratio, 1e-12));
}

TEST_CASE("PlaneFitSigmas_DegenerateFits_ClampsToFinitePositiveSigmas",
          "[plane_graph][noise]") {
  // A zero extent or zero residual is not a real detection, but the graph must
  // never see inf/NaN/0 sigmas: a zero sigma is an infinitely trusted factor,
  // which would silently dominate the entire solve.
  SECTION("no inliers") {
    PlaneFitQuality q = nominal();
    q.inliers = 0;
    const auto s = plane_fit_sigmas(q);
    REQUIRE(std::isfinite(s.normal));
    REQUIRE(std::isfinite(s.distance));
    REQUIRE(s.normal > 0.0);
    REQUIRE(s.distance > 0.0);
  }

  SECTION("a perfectly planar fit still gets a positive sigma") {
    PlaneFitQuality q = nominal();
    q.residual_rms = 0.0;
    const auto s = plane_fit_sigmas(q);
    REQUIRE(s.normal > 0.0);
    REQUIRE(s.distance > 0.0);
  }

  SECTION("a degenerate (zero-extent) footprint gets a bounded normal sigma") {
    PlaneFitQuality q = nominal();
    q.extent_minor = 0.0;
    const auto s = plane_fit_sigmas(q);
    REQUIRE(std::isfinite(s.normal));
    REQUIRE(s.normal > 0.0);
    // Still strictly worse than a real footprint — the clamp bounds it, it does
    // not rehabilitate it.
    REQUIRE(s.normal > plane_fit_sigmas(nominal()).normal);
  }

  SECTION("negative inputs cannot produce a negative sigma") {
    PlaneFitQuality q;
    q.inliers = -5;
    q.residual_rms = -1.0;
    q.extent_minor = -1.0;
    const auto s = plane_fit_sigmas(q);
    REQUIRE(s.normal > 0.0);
    REQUIRE(s.distance > 0.0);
    REQUIRE(std::isfinite(s.normal));
    REQUIRE(std::isfinite(s.distance));
  }
}
