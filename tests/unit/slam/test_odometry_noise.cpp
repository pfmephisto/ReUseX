// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the per-edge odometry noise model added by the #225
// odometry-trust experiment (`odometry_motion_scales`).

#include <reusex/slam/PlaneGraphOptimizer.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::geometry::odometry_motion_scales;
using reusex::geometry::OdometryEdgeMotion;

namespace {

std::vector<OdometryEdgeMotion> motions(const std::vector<double> &trans,
                                        const std::vector<double> &rot) {
  std::vector<OdometryEdgeMotion> m(trans.size());
  for (size_t i = 0; i < trans.size(); ++i) {
    m[i].translation = trans[i];
    m[i].rotation = rot[i];
  }
  return m;
}

} // namespace

TEST_CASE("odometry_motion_scales: uniform motion degenerates to the fixed "
          "model",
          "[slam][odometry][noise]") {
  // The load-bearing property: `motion` must be a strict generalisation of
  // `fixed`. If every edge moved the same amount there is nothing to
  // redistribute, so every scale must be exactly 1 and the resulting graph
  // byte-identical to the shipped default.
  const auto m = motions({0.2, 0.2, 0.2, 0.2}, {0.05, 0.05, 0.05, 0.05});
  const auto s = odometry_motion_scales(m, 0.5, 3.0);

  REQUIRE(s.size() == 4);
  for (const auto &x : s) {
    CHECK_THAT(x.translation, WithinAbs(1.0, 1e-12));
    CHECK_THAT(x.rotation, WithinAbs(1.0, 1e-12));
  }
}

TEST_CASE("odometry_motion_scales: sigma scales with the edge's own motion",
          "[slam][odometry][noise]") {
  // Median translation is 0.2. Edges are normalised against it, so a
  // half-median edge earns half the sigma (more trust) and a double-median
  // edge twice (less trust).
  const auto m = motions({0.1, 0.2, 0.2, 0.4}, {0.1, 0.2, 0.2, 0.4});
  const auto s = odometry_motion_scales(m, 0.1, 10.0);

  REQUIRE(s.size() == 4);
  CHECK_THAT(s[0].translation, WithinAbs(0.5, 1e-12));
  CHECK_THAT(s[1].translation, WithinAbs(1.0, 1e-12));
  CHECK_THAT(s[3].translation, WithinAbs(2.0, 1e-12));
  CHECK_THAT(s[0].rotation, WithinAbs(0.5, 1e-12));
  CHECK_THAT(s[3].rotation, WithinAbs(2.0, 1e-12));
}

TEST_CASE("odometry_motion_scales: monotone in motion",
          "[slam][odometry][noise]") {
  // A larger motion must never earn a *smaller* sigma — otherwise the model
  // would be rewarding the less reliable measurement.
  const auto m =
      motions({0.05, 0.1, 0.2, 0.4, 0.8}, {0.4, 0.2, 0.1, 0.05, 0.8});
  const auto s = odometry_motion_scales(m, 0.01, 100.0);

  REQUIRE(s.size() == 5);
  for (size_t i = 1; i < s.size(); ++i)
    CHECK(s[i].translation >= s[i - 1].translation);
}

TEST_CASE("odometry_motion_scales: channels are normalised independently",
          "[slam][odometry][noise]") {
  // A pure-rotation sweep and a pure-translation dolly are different failure
  // modes of the seed, so a big rotation must not loosen the translation
  // channel (or vice versa).
  const auto m = motions({0.2, 0.2, 0.2}, {0.05, 0.05, 0.50});
  const auto s = odometry_motion_scales(m, 0.5, 20.0);

  REQUIRE(s.size() == 3);
  for (const auto &x : s)
    CHECK_THAT(x.translation, WithinAbs(1.0, 1e-12));
  CHECK_THAT(s[2].rotation, WithinAbs(10.0, 1e-12));
  CHECK_THAT(s[0].rotation, WithinAbs(1.0, 1e-12));
}

TEST_CASE("odometry_motion_scales: the clamp binds at both ends",
          "[slam][odometry][noise]") {
  const auto m = motions({0.001, 0.2, 0.2, 50.0}, {0.001, 0.2, 0.2, 50.0});
  const auto s = odometry_motion_scales(m, 0.5, 3.0);

  REQUIRE(s.size() == 4);
  CHECK_THAT(s[0].translation, WithinAbs(0.5, 1e-12));
  CHECK_THAT(s[3].translation, WithinAbs(3.0, 1e-12));
  CHECK_THAT(s[0].rotation, WithinAbs(0.5, 1e-12));
  CHECK_THAT(s[3].rotation, WithinAbs(3.0, 1e-12));
}

TEST_CASE("odometry_motion_scales: a stationary run is not a divide-by-zero",
          "[slam][odometry][noise]") {
  // Zero median motion (a tripod capture, or a channel that never moved) must
  // fall back to the fixed model rather than emit inf/NaN sigmas into GTSAM.
  const auto m = motions({0.0, 0.0, 0.0}, {0.0, 0.0, 0.1});
  const auto s = odometry_motion_scales(m, 0.5, 3.0);

  REQUIRE(s.size() == 3);
  for (const auto &x : s) {
    CHECK(std::isfinite(x.translation));
    CHECK(std::isfinite(x.rotation));
    CHECK_THAT(x.translation, WithinAbs(1.0, 1e-12));
    CHECK_THAT(x.rotation, WithinAbs(1.0, 1e-12));
  }
}

TEST_CASE("odometry_motion_scales: degenerate inputs stay at the fixed model",
          "[slam][odometry][noise]") {
  SECTION("empty input") {
    CHECK(odometry_motion_scales({}, 0.5, 3.0).empty());
  }
  SECTION("inverted clamp") {
    const auto s =
        odometry_motion_scales(motions({0.1, 0.4}, {0.1, 0.4}), 3.0, 0.5);
    REQUIRE(s.size() == 2);
    for (const auto &x : s) {
      CHECK_THAT(x.translation, WithinAbs(1.0, 1e-12));
      CHECK_THAT(x.rotation, WithinAbs(1.0, 1e-12));
    }
  }
  SECTION("non-finite motion is ignored, not propagated") {
    const double inf = std::numeric_limits<double>::infinity();
    const auto s = odometry_motion_scales(
        motions({0.2, 0.2, inf}, {0.1, 0.1, 0.1}), 0.5, 3.0);
    REQUIRE(s.size() == 3);
    for (const auto &x : s)
      CHECK(std::isfinite(x.translation));
  }
}

TEST_CASE("odometry_motion_scales: sign of the motion does not matter",
          "[slam][odometry][noise]") {
  // Magnitudes are what the noise model is about; a sign flip can only arise
  // from caller error and must be inert rather than inverting the ordering.
  const auto signed_scales = odometry_motion_scales(
      motions({-0.4, 0.2, 0.2, 0.4}, {-0.4, 0.2, 0.2, 0.4}), 0.1, 10.0);
  const auto abs_scales = odometry_motion_scales(
      motions({0.4, 0.2, 0.2, 0.4}, {0.4, 0.2, 0.2, 0.4}), 0.1, 10.0);

  REQUIRE(signed_scales.size() == 4);
  REQUIRE(abs_scales.size() == 4);
  for (size_t i = 0; i < signed_scales.size(); ++i) {
    CHECK_THAT(signed_scales[i].translation,
               WithinAbs(abs_scales[i].translation, 1e-12));
    CHECK_THAT(signed_scales[i].rotation,
               WithinAbs(abs_scales[i].rotation, 1e-12));
  }
}
