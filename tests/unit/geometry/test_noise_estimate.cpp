// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Validates the deterministic local-PCA cloud-noise/density estimator
// (issue #214). The estimator must recover the injected Gaussian position
// noise sigma on synthetic rooms across a range of noise levels, and must be
// deterministic for a fixed seed (docs/STANDARDS.md §6). These estimates drive
// the adaptive plane_dist_threshold / min_inliers derivation, so a wrong sigma
// silently mis-tunes the whole segmentation.

#include "../../support/synthetic_scene.hpp"

#include <reusex/geometry/noise_estimate.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using reusex::geometry::estimate_cloud_noise;
using reusex::geometry::NoiseEstimate;
using reusex::geometry::NoiseEstimateOptions;
using reusex::test_support::make_room;

TEST_CASE("noise estimate: recovers injected sigma on synthetic rooms",
          "[geometry][noise]") {
  // Tolerance ±40% (issue #214). Local-PCA residual on a k-neighbourhood is a
  // slightly biased estimator of the true along-normal sigma (patch curvature
  // + in-plane spread leak in), so we allow a generous band.
  constexpr float kSpacing = 0.05F;

  struct Case {
    float sigma;
  };
  const Case cases[] = {{0.000F}, {0.002F}, {0.005F}, {0.015F}};

  for (const auto &c : cases) {
    const auto scene = make_room(4.0F, 3.0F, 2.5F, kSpacing, c.sigma);
    const NoiseEstimate est = estimate_cloud_noise(scene.cloud, scene.normals,
                                                   NoiseEstimateOptions{});

    INFO("injected sigma = " << c.sigma << " m, estimated = " << est.sigma
                             << " m");
    REQUIRE(est.samples > 0);
    // Density (median NN spacing) must be close to the sample spacing.
    CHECK(est.density > 0.0F);
    CHECK(std::abs(est.density - kSpacing) < 0.4F * kSpacing + 1e-4F);

    if (c.sigma == 0.0F) {
      // No injected noise: residual should be tiny (below one spacing).
      CHECK(est.sigma < kSpacing);
    } else {
      CHECK(est.sigma > 0.6F * c.sigma);
      CHECK(est.sigma < 1.4F * c.sigma);
    }
  }
}

TEST_CASE("noise estimate: deterministic for a fixed seed",
          "[geometry][noise]") {
  const auto scene = make_room(4.0F, 3.0F, 2.5F, 0.05F, 0.005F);
  const NoiseEstimate a =
      estimate_cloud_noise(scene.cloud, scene.normals, NoiseEstimateOptions{});
  const NoiseEstimate b =
      estimate_cloud_noise(scene.cloud, scene.normals, NoiseEstimateOptions{});
  CHECK(a.sigma == b.sigma);
  CHECK(a.density == b.density);
  CHECK(a.samples == b.samples);
}

TEST_CASE("noise estimate: empty cloud throws", "[geometry][noise]") {
  auto empty = std::make_shared<reusex::Cloud>();
  auto empty_n = std::make_shared<reusex::CloudN>();
  CHECK_THROWS(estimate_cloud_noise(empty, empty_n, NoiseEstimateOptions{}));
}
