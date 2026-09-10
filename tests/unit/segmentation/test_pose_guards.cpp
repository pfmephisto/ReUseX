// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Degenerate-pose guards in the segmentation stages (#336).
//
// `ProjectDB::sensor_frame_pose()` returns identity for a frame with no stored
// pose, and hands back an all-zero or NaN transform verbatim. Both
// `reconstruct_point_clouds()` and `extract_frame_surfels()` used to consume
// that silently: back-projected depth landed at the world origin (or collapsed
// to a point) and merged into the fused cloud, indistinguishable from real
// geometry for every stage downstream. They now gate on
// `has_sensor_frame_pose()`.
//
// The assertions are on geometry, not on log lines. Each test poses the
// SURVIVING frame far from the origin, so "the skipped frame's points are
// absent" and "the skipped frame's points landed at the origin" are different
// numbers rather than the same one — the old behaviour fails these tests.
//
// The regression that would make the fix wrong in the other direction is a
// *genuinely stored* identity pose, which is legitimate (a scan may put its
// first frame at the origin) and must be kept. There is a test for that in
// each half.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/segmentation/Surfel.hpp>
#include <reusex/segmentation/reconstruct.hpp>
#include <reusex/segmentation/surfel_extraction.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <limits>
#include <string>

using namespace reusex;
using namespace reusex::test_support;

namespace {

// A frame big and dense enough to survive reconstruct_point_clouds()'s
// post-processing: the voxel grid at 5 cm, then SOR (meanK=50) and ROR
// (5 neighbours within 10 cm). 64x48 at 1.5 m with fx=fy=64 spans roughly
// 1.5 m x 1.1 m at ~2.3 cm sample spacing, which voxelises to ~700 points —
// comfortably above every one of those thresholds.
constexpr int kW = 64;
constexpr int kH = 48;
constexpr double kDepthM = 1.5;

core::SensorIntrinsics
frame_intrinsics(std::array<double, 16> local = identity16()) {
  return make_intrinsics(64.0, 64.0, kW / 2.0, kH / 2.0, kW, kH, local);
}

void seed_frame(ProjectDB &db, int node_id,
                const std::array<double, 16> &pose) {
  db.save_sensor_frame(node_id, make_color(kW, kH), make_depth(kW, kH, kDepthM),
                       cv::Mat(), pose, frame_intrinsics());
}

/// Reconstruction parameters pinned rather than defaulted, so a moving default
/// cannot silently retune these bounds (STANDARDS §4/§6). Only the sampling
/// factor differs from the pipeline defaults: every pixel, to keep the fixture
/// dense enough for the outlier filters.
geometry::ReconstructionParams test_params() {
  geometry::ReconstructionParams p;
  p.resolution = 0.05f;
  p.min_distance = 0.0f;
  p.max_distance = 4.0f;
  p.sampling_factor = 1;
  p.confidence_threshold = 2;
  return p;
}

/// Smallest x over the reconstructed cloud. The discriminator throughout:
/// a frame consumed through the identity fallback contributes points around
/// x = 0, a frame that was skipped contributes none.
double min_x(const ProjectDB &db) {
  auto cloud = db.point_cloud_xyzrgb("cloud");
  REQUIRE(cloud);
  REQUIRE_FALSE(cloud->empty());
  double lo = std::numeric_limits<double>::max();
  for (const auto &p : *cloud)
    lo = std::min(lo, static_cast<double>(p.x));
  return lo;
}

/// x = 20 m: far enough that the two frames' point sets cannot overlap, so
/// `min_x` alone distinguishes "frame 2 was skipped" from "frame 2 landed at
/// the origin".
constexpr double kFarX = 20.0;

} // namespace

// ===========================================================================
// reconstruct_point_clouds
// ===========================================================================

TEST_CASE(
    "ReconstructPointClouds_FrameWithUnusablePose_OmitsItsPointsFromCloud",
    "[segmentation][reconstruct][pose]") {
  TempPath tmp("test_reconstruct_pose_guard");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 1, translation_pose(kFarX, 0.0, 0.0));
    seed_frame(db, 2, translation_pose(kFarX, 0.0, 0.0));
  }

  // Every shape of "unusable" the guard has to reject. All three used to be
  // consumed as an identity (or collapsed) camera at the world origin.
  SECTION("NULL transform") { clear_sensor_frame_pose(tmp.path, 2); }
  SECTION("all-zero transform") {
    set_raw_sensor_frame_pose(tmp.path, 2, zero_pose());
  }
  SECTION("NaN translation") {
    set_raw_sensor_frame_pose(tmp.path, 2, nan_translation_pose());
  }

  ProjectDB db(tmp.path);
  geometry::reconstruct_point_clouds(db, test_params());

  // Frame 1 alone: everything sits around x = 20, nothing near the origin.
  REQUIRE(db.has_point_cloud("cloud"));
  CHECK(min_x(db) > kFarX / 2.0);
}

TEST_CASE("ReconstructPointClouds_FrameWithStoredIdentityPose_KeepsItsPoints",
          "[segmentation][reconstruct][pose]") {
  // The regression guard for the fix itself: a stored identity pose is a
  // legitimate placement at the origin, not an absence of one.
  TempPath tmp("test_reconstruct_identity_pose");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 1, translation_pose(kFarX, 0.0, 0.0));
    seed_frame(db, 2, identity16());
  }

  ProjectDB db(tmp.path);
  geometry::reconstruct_point_clouds(db, test_params());

  REQUIRE(db.has_point_cloud("cloud"));
  CHECK(min_x(db) < kFarX / 2.0);
}

TEST_CASE("ReconstructPointClouds_AllFramesPoseless_ProducesNoCloud",
          "[segmentation][reconstruct][pose]") {
  TempPath tmp("test_reconstruct_all_poseless");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 1, translation_pose(kFarX, 0.0, 0.0));
    seed_frame(db, 2, translation_pose(kFarX, 0.0, 0.0));
  }
  clear_sensor_frame_pose(tmp.path, 1);
  clear_sensor_frame_pose(tmp.path, 2);

  ProjectDB db(tmp.path);
  geometry::reconstruct_point_clouds(db, test_params());

  // Nothing was reconstructed, and — the point of the fix — nothing plausible
  // was invented at the origin either.
  CHECK_FALSE(db.has_point_cloud("cloud"));
}

// ===========================================================================
// extract_frame_surfels
// ===========================================================================
//
// The surfel path feeds `rux optimize` and `rux register`, both of which write
// their answer straight back to the pose column. Seeding a frame from the
// identity fallback would let the optimiser drag the rest of the graph toward
// a camera that was never there.

namespace {

geometry::SurfelExtractionParams surfel_params() {
  geometry::SurfelExtractionParams p;
  p.min_distance = 0.0f;
  p.max_distance = 4.0f;
  p.sampling_factor = 2;
  p.confidence_threshold = 2;
  p.normal_radius = 0.1f;
  p.voxel_size = 0.0f; // no downsample: keep the fixture's point count exact
  p.apply_depth_filters = false;
  return p;
}

} // namespace

TEST_CASE("ExtractFrameSurfels_FrameWithUnusablePose_ReturnsNullopt",
          "[segmentation][surfels][pose]") {
  TempPath tmp("test_surfels_pose_guard");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 7, translation_pose(kFarX, 0.0, 0.0));
  }

  SECTION("NULL transform") { clear_sensor_frame_pose(tmp.path, 7); }
  SECTION("all-zero transform") {
    set_raw_sensor_frame_pose(tmp.path, 7, zero_pose());
  }
  SECTION("NaN translation") {
    set_raw_sensor_frame_pose(tmp.path, 7, nan_translation_pose());
  }

  ProjectDB db(tmp.path);
  CHECK_FALSE(
      geometry::extract_frame_surfels(db, 7, surfel_params()).has_value());
}

TEST_CASE("ExtractFrameSurfels_FrameWithStoredIdentityPose_ReturnsSurfels",
          "[segmentation][surfels][pose]") {
  TempPath tmp("test_surfels_identity_pose");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 7, identity16());
  }

  ProjectDB db(tmp.path);
  auto surfels = geometry::extract_frame_surfels(db, 7, surfel_params());
  REQUIRE(surfels.has_value());
  CHECK(surfels->node_id == 7);
  REQUIRE(surfels->points);
  CHECK_FALSE(surfels->points->empty());
  // world_pose = worldTf * localTf; both identity here.
  CHECK(surfels->world_pose.matrix().isApprox(Eigen::Matrix4f::Identity(),
                                              1e-6F));
}

TEST_CASE("ExtractFrameSurfels_PosedFrame_SeedsWorldPoseFromStoredTransform",
          "[segmentation][surfels][pose]") {
  // The seed the guard exists to protect: a kept frame's world_pose must still
  // be the stored transform, not something the guard sanitised.
  TempPath tmp("test_surfels_seed_pose");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 7, translation_pose(kFarX, 1.0, -2.0));
  }

  ProjectDB db(tmp.path);
  auto surfels = geometry::extract_frame_surfels(db, 7, surfel_params());
  REQUIRE(surfels.has_value());
  const Eigen::Vector3f t = surfels->world_pose.translation();
  CHECK_THAT(t.x(), Catch::Matchers::WithinAbs(kFarX, 1e-4));
  CHECK_THAT(t.y(), Catch::Matchers::WithinAbs(1.0, 1e-4));
  CHECK_THAT(t.z(), Catch::Matchers::WithinAbs(-2.0, 1e-4));
}
