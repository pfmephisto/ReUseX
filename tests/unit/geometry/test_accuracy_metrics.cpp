// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Validates the ground-truth accuracy metrics (issue #221, Tier 2):
// accuracy/completeness/chamfer and precision/recall/F-score against a
// reference point cloud must match the standard benchmark definitions.

#include "../../support/synthetic_scene.hpp"

#include <reusex/geometry/accuracy_metrics.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <random>

using reusex::geometry::AccuracyMetricsOptions;
using reusex::geometry::compute_accuracy;
using reusex::test_support::make_room;

namespace {

// Convert an XYZRGB reconstruction cloud into a plain XYZ ground-truth cloud.
reusex::CloudLoc to_loc(const reusex::Cloud &cloud) {
  reusex::CloudLoc out;
  out.reserve(cloud.size());
  for (const auto &p : cloud.points)
    out.emplace_back(p.x, p.y, p.z);
  out.width = static_cast<uint32_t>(out.size());
  out.height = 1;
  return out;
}

} // namespace

TEST_CASE("accuracy metrics: identical clouds are perfect",
          "[geometry][accuracy]") {
  const auto scene = make_room(4.0F, 3.0F, 2.5F, 0.05F, /*sigma=*/0.0F);
  const reusex::CloudLoc gt = to_loc(*scene.cloud);

  // Disable GT downsampling so the two clouds stay point-for-point identical.
  AccuracyMetricsOptions opt;
  opt.gt_voxel = 0.0F;
  const auto report = compute_accuracy(*scene.cloud, gt, opt);

  CHECK(report.cloud_points == scene.cloud->size());
  CHECK(report.gt_points == scene.cloud->size());
  CHECK(report.accuracy_mean < 1e-6);
  CHECK(report.completeness_mean < 1e-6);
  CHECK(report.chamfer < 1e-6);
  CHECK_THAT(report.precision, Catch::Matchers::WithinAbs(1.0, 1e-9));
  CHECK_THAT(report.recall, Catch::Matchers::WithinAbs(1.0, 1e-9));
  CHECK_THAT(report.fscore, Catch::Matchers::WithinAbs(1.0, 1e-9));
}

TEST_CASE("accuracy metrics: 5mm noise recovers as ~5mm accuracy",
          "[geometry][accuracy]") {
  const auto clean = make_room(4.0F, 3.0F, 2.5F, 0.05F, /*sigma=*/0.0F);
  const reusex::CloudLoc gt = to_loc(*clean.cloud);

  // Reconstruction = GT + isotropic 5mm Gaussian noise (fixed seed).
  auto noisy = std::make_shared<reusex::Cloud>(*clean.cloud);
  std::mt19937 gen(1234);
  std::normal_distribution<float> noise(0.0F, 0.005F);
  for (auto &p : noisy->points) {
    p.x += noise(gen);
    p.y += noise(gen);
    p.z += noise(gen);
  }

  AccuracyMetricsOptions opt;
  opt.gt_voxel = 0.0F; // keep GT dense so noise is measured, not smoothed
  opt.threshold = 0.05F;
  const auto report = compute_accuracy(*noisy, gt, opt);

  // Mean 3D magnitude of isotropic per-axis N(0,5mm) noise is
  // sigma*sqrt(8/pi) ~= 8mm; GT samples are 5cm apart so nearest-neighbour
  // snapping barely reduces it. Expect roughly 7-9mm.
  CHECK(report.accuracy_mean > 0.006);
  CHECK(report.accuracy_mean < 0.010);
  // 5mm noise is far inside a 5cm threshold: essentially all inliers.
  CHECK(report.fscore > 0.99);
}

TEST_CASE("accuracy metrics: missing half degrades recall",
          "[geometry][accuracy]") {
  const auto scene = make_room(4.0F, 3.0F, 2.5F, 0.05F, /*sigma=*/0.0F);
  const reusex::CloudLoc gt = to_loc(*scene.cloud);

  // Reconstruction covers only half the ground truth (every other point).
  auto half = std::make_shared<reusex::Cloud>();
  for (std::size_t i = 0; i < scene.cloud->size(); i += 2)
    half->push_back(scene.cloud->points[i]);
  half->width = static_cast<uint32_t>(half->size());
  half->height = 1;

  AccuracyMetricsOptions opt;
  opt.gt_voxel = 0.0F;
  opt.threshold = 0.02F; // < the 5cm sample spacing, so drops matter
  const auto report = compute_accuracy(*half, gt, opt);

  // Every reconstruction point coincides with a GT point -> precision stays
  // high; but half the GT has no nearby reconstruction -> recall ~0.5.
  CHECK(report.precision > 0.9);
  CHECK(report.recall > 0.4);
  CHECK(report.recall < 0.6);
  // Completeness (GT->recon) degrades relative to the full case.
  CHECK(report.completeness_mean > report.accuracy_mean);
}

TEST_CASE("accuracy metrics: input validation", "[geometry][accuracy]") {
  const auto scene = make_room(2.0F, 2.0F, 2.0F, 0.1F);
  const reusex::CloudLoc gt = to_loc(*scene.cloud);

  SECTION("empty reconstruction throws") {
    reusex::Cloud empty_cloud;
    CHECK_THROWS_AS(compute_accuracy(empty_cloud, gt), std::invalid_argument);
  }

  SECTION("empty ground truth throws") {
    reusex::CloudLoc empty_gt;
    CHECK_THROWS_AS(compute_accuracy(*scene.cloud, empty_gt),
                    std::invalid_argument);
  }
}
