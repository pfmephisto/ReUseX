// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Validates the ground-truth-free quality metrics (issue #221, Tier 4):
// each metric must increase monotonically with injected pose error,
// otherwise it cannot be trusted to rank SLAM/registration changes.

#include "../../support/synthetic_scene.hpp"

#include <reusex/geometry/quality_metrics.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using reusex::geometry::compute_plane_quality;
using reusex::test_support::make_room;
using reusex::test_support::perturb_chunks;

namespace {
// One "frame" worth of points for pose-error simulation. A 4x3x2.5m room
// at 5cm spacing has ~28k points across 6 faces (~4.7k per face), so 500
// points per chunk yields ~9 chunks per face — enough independent "poses"
// to smear each plane.
constexpr std::size_t kChunkSize = 500;
} // namespace

TEST_CASE("quality metrics: perfect room has near-zero residuals",
          "[geometry][quality]") {
  const auto scene = make_room(4.0F, 3.0F, 2.5F, 0.05F, /*sigma=*/0.0F);
  const auto report = compute_plane_quality(*scene.cloud, *scene.labels);

  REQUIRE(report.planes.size() == 6);
  REQUIRE(report.labeled_points == scene.cloud->size());
  CHECK(report.flatness_rms < 1e-6);
  CHECK(report.thickness_p90 < 1e-6);
}

TEST_CASE("quality metrics: sensor noise is recovered as flatness RMS",
          "[geometry][quality]") {
  // 5mm Gaussian noise along the normal must appear as ~5mm RMS.
  const auto scene = make_room(4.0F, 3.0F, 2.5F, 0.05F, /*sigma=*/0.005F);
  const auto report = compute_plane_quality(*scene.cloud, *scene.labels);

  REQUIRE(report.planes.size() == 6);
  CHECK(report.flatness_rms > 0.004);
  CHECK(report.flatness_rms < 0.006);
  // For pure Gaussian residuals p90 ~= 1.64 sigma.
  CHECK(report.thickness_p90 > 0.006);
  CHECK(report.thickness_p90 < 0.010);
}

TEST_CASE("quality metrics: increase monotonically with injected pose error",
          "[geometry][quality]") {
  const auto clean = make_room();

  const auto drift_small =
      perturb_chunks(clean, kChunkSize, /*sigma_trans=*/0.01F,
                     /*sigma_rot_rad=*/0.005F, /*seed=*/7);
  const auto drift_large =
      perturb_chunks(clean, kChunkSize, /*sigma_trans=*/0.03F,
                     /*sigma_rot_rad=*/0.015F, /*seed=*/7);

  const auto r0 = compute_plane_quality(*clean.cloud, *clean.labels);
  const auto r1 =
      compute_plane_quality(*drift_small.cloud, *drift_small.labels);
  const auto r2 =
      compute_plane_quality(*drift_large.cloud, *drift_large.labels);

  INFO("flatness_rms: clean=" << r0.flatness_rms << " small=" << r1.flatness_rms
                              << " large=" << r2.flatness_rms);
  CHECK(r0.flatness_rms < r1.flatness_rms);
  CHECK(r1.flatness_rms < r2.flatness_rms);

  INFO("thickness_p90: clean=" << r0.thickness_p90
                               << " small=" << r1.thickness_p90
                               << " large=" << r2.thickness_p90);
  CHECK(r0.thickness_p90 < r1.thickness_p90);
  CHECK(r1.thickness_p90 < r2.thickness_p90);

  // The injected error must dominate the sensor noise floor, not drown in
  // it: 1cm translation drift should at least double the clean RMS.
  CHECK(r1.flatness_rms > 2.0 * r0.flatness_rms);
}

TEST_CASE("quality metrics: input validation", "[geometry][quality]") {
  const auto scene = make_room(2.0F, 2.0F, 2.0F, 0.1F);

  SECTION("empty cloud throws") {
    reusex::Cloud empty_cloud;
    reusex::CloudL empty_labels;
    CHECK_THROWS_AS(compute_plane_quality(empty_cloud, empty_labels),
                    std::invalid_argument);
  }

  SECTION("size mismatch throws") {
    reusex::CloudL short_labels(*scene.labels);
    short_labels.points.pop_back();
    short_labels.width = static_cast<uint32_t>(short_labels.points.size());
    CHECK_THROWS_AS(compute_plane_quality(*scene.cloud, short_labels),
                    std::invalid_argument);
  }

  SECTION("min_points filters small planes") {
    reusex::geometry::QualityMetricsOptions options;
    options.min_points = scene.cloud->size(); // larger than any single plane
    const auto report =
        compute_plane_quality(*scene.cloud, *scene.labels, options);
    CHECK(report.planes.empty());
    CHECK(report.flatness_rms == 0.0);
  }
}
