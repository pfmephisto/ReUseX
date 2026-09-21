// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Tests for the glass depth-filter plumbing:
//   - ProjectDB glass confidence image round-trip (save / has / get / ids)
//   - reconstruct_point_clouds fails fast when --glass-filter is set but no
//     glass confidence images exist in the database
//   - reconstruct_point_clouds suppresses glass pixels: synthetic frame with a
//     half-glass confidence map produces fewer points than without the filter

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <segmentation/reconstruct.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <stdexcept>

using namespace reusex;
using namespace reusex::geometry;

namespace {

// Identity pose (camera at world origin, no rotation).
constexpr std::array<double, 16> kIdentityPose{1, 0, 0, 0, 0, 1, 0, 0,
                                               0, 0, 1, 0, 0, 0, 0, 1};

// Minimal pinhole intrinsics for an 8×4 depth image.
core::SensorIntrinsics make_intrinsics(int w, int h) {
  core::SensorIntrinsics intr;
  intr.fx = 1.0;
  intr.fy = 1.0;
  intr.cx = 0.0;
  intr.cy = 0.0;
  intr.width = w;
  intr.height = h;
  // local_transform: identity (optical == sensor frame)
  intr.local_transform = kIdentityPose;
  return intr;
}

// Add a single synthetic sensor frame to the DB.
void add_synthetic_frame(ProjectDB &db, int node_id, int w, int h,
                         float depth_m) {
  cv::Mat color(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat depth16(h, w, CV_16UC1,
                  cv::Scalar(static_cast<uint16_t>(depth_m * 1000.0f)));
  cv::Mat conf(h, w, CV_8UC1, cv::Scalar(255)); // full confidence
  db.save_sensor_frame(node_id, color, depth16, conf, kIdentityPose,
                       make_intrinsics(w, h));
}

struct TempDir : reusex::test_support::TempDir {
  TempDir() : reusex::test_support::TempDir("test_glass_depth_filter") {}
};

} // namespace

// ── ProjectDB glass confidence image API ─────────────────────────────────────

TEST_CASE("GlassConfidenceImage_RoundTrip", "[segmentation][glass_filter]") {
  TempDir tmp;
  ProjectDB db(tmp.path / "test.rux");

  constexpr int node_id = 42;
  add_synthetic_frame(db, node_id, 4, 4, 1.0f);

  // No image yet.
  REQUIRE_FALSE(db.has_glass_confidence_image(node_id));
  REQUIRE(db.glass_confidence_image_ids().empty());

  // Save a simple CV_8U confidence map: top half=0 (glass), bottom half=255.
  cv::Mat conf_in(4, 4, CV_8U, cv::Scalar(255));
  conf_in.rowRange(0, 2).setTo(0); // top 2 rows = glass / suppress

  db.save_glass_confidence_image(node_id, conf_in);

  REQUIRE(db.has_glass_confidence_image(node_id));
  auto ids = db.glass_confidence_image_ids();
  REQUIRE(ids.size() == 1);
  REQUIRE(ids[0] == node_id);

  // Retrieve and verify pixel values survive the PNG round-trip.
  cv::Mat conf_out = db.glass_confidence_image(node_id);
  REQUIRE_FALSE(conf_out.empty());
  REQUIRE(conf_out.type() == CV_8U);
  REQUIRE(conf_out.size() == conf_in.size());

  for (int r = 0; r < 2; ++r)
    for (int c = 0; c < 4; ++c)
      REQUIRE(conf_out.at<uchar>(r, c) == 0);

  for (int r = 2; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      REQUIRE(conf_out.at<uchar>(r, c) == 255);
}

TEST_CASE("GlassConfidenceImage_Upsert_OverwritesPrevious",
          "[segmentation][glass_filter]") {
  TempDir tmp;
  ProjectDB db(tmp.path / "test.rux");

  constexpr int node_id = 1;
  add_synthetic_frame(db, node_id, 2, 2, 1.0f);

  cv::Mat first(2, 2, CV_8U, cv::Scalar(0));
  db.save_glass_confidence_image(node_id, first);

  cv::Mat second(2, 2, CV_8U, cv::Scalar(255));
  db.save_glass_confidence_image(node_id, second); // upsert

  cv::Mat retrieved = db.glass_confidence_image(node_id);
  REQUIRE(cv::countNonZero(retrieved != 255) == 0);     // second value wins
  REQUIRE(db.glass_confidence_image_ids().size() == 1); // still one row
}

// ── reconstruct_point_clouds fail-fast ───────────────────────────────────────

TEST_CASE("ReconstructPointClouds_GlassFilterWithNoImages_Throws",
          "[segmentation][glass_filter]") {
  TempDir tmp;
  ProjectDB db(tmp.path / "test.rux");

  // Add a sensor frame without any glass confidence images.
  add_synthetic_frame(db, 1, 4, 4, 1.0f);

  ReconstructionParams params;
  params.glass_filter = true;

  REQUIRE_THROWS_AS(reconstruct_point_clouds(db, params), std::runtime_error);
}

// ── reconstruct_point_clouds glass suppression ───────────────────────────────

TEST_CASE("ReconstructPointClouds_GlassFilterHalvesPoints",
          "[segmentation][glass_filter]") {
  // 64×64 frame: all depth = 1m, identity pose.
  // Intrinsics: fx=fy=100, cx=cy=32 → pixel spacing ≈ 0.01m at z=1m.
  // resolution=0.01m: radius=0.02m, which covers adjacent pixels so they
  // survive statistical/radius outlier removal.
  // Glass confidence: right 32 columns = 0 (suppress), left 32 = 255 (trust).
  // Expected: filtered cloud ~half of the full cloud.

  const int W = 64, H = 64;

  auto make_dense_intrinsics = [&](int w, int h) {
    core::SensorIntrinsics intr;
    intr.fx = 100.0;
    intr.fy = 100.0;
    intr.cx = w / 2.0;
    intr.cy = h / 2.0;
    intr.width = w;
    intr.height = h;
    intr.local_transform = kIdentityPose;
    return intr;
  };

  auto add_dense_frame = [&](ProjectDB &db, int node_id) {
    cv::Mat color(H, W, CV_8UC3, cv::Scalar(100, 100, 100));
    cv::Mat depth16(H, W, CV_16UC1, cv::Scalar(1000)); // 1000 mm = 1 m
    cv::Mat conf(H, W, CV_8UC1, cv::Scalar(255));
    db.save_sensor_frame(node_id, color, depth16, conf, kIdentityPose,
                         make_dense_intrinsics(W, H));
  };

  TempDir tmp;
  ReconstructionParams base_params;
  base_params.resolution = 0.01f; // 1cm voxels — pixel spacing at z=1m
  base_params.min_distance = 0.0f;
  base_params.max_distance = 10.0f;
  base_params.sampling_factor = 1;

  // ── Baseline: no glass filter ─────────────────────────────────────────────
  std::size_t full_count = 0;
  {
    ProjectDB db(tmp.path / "no_glass.rux");
    add_dense_frame(db, 1);
    reconstruct_point_clouds(db, base_params);
    auto cloud = db.point_cloud_xyzrgb("cloud");
    REQUIRE(cloud != nullptr);
    full_count = cloud->size();
    REQUIRE(full_count > 0);
  }

  // ── With glass filter masking the right half ──────────────────────────────
  std::size_t filtered_count = 0;
  {
    ProjectDB db(tmp.path / "with_glass.rux");
    add_dense_frame(db, 1);

    // Left half: trust (255), right half: glass / suppress (0).
    cv::Mat glass_conf(H, W, CV_8U, cv::Scalar(255));
    glass_conf.colRange(W / 2, W).setTo(0);
    db.save_glass_confidence_image(1, glass_conf);

    ReconstructionParams params = base_params;
    params.glass_filter = true;
    params.glass_threshold = 0.5f;

    reconstruct_point_clouds(db, params);
    auto cloud = db.point_cloud_xyzrgb("cloud");
    REQUIRE(cloud != nullptr);
    filtered_count = cloud->size();
  }

  // Filtered cloud must be smaller than the baseline.
  REQUIRE(filtered_count < full_count);
  // Expect roughly half, allowing ±30% for outlier removal effects on edges.
  const double ratio =
      static_cast<double>(filtered_count) / static_cast<double>(full_count);
  REQUIRE(ratio > 0.25);
  REQUIRE(ratio < 0.75);
}
