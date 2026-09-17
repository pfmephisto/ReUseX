// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Deterministic pinhole projection tests for instance cropping (#373).

#include <catch2/catch_test_macros.hpp>

#include <core/SensorIntrinsics.hpp>
#include <vision/instance_crop.hpp>

#include <Eigen/Core>

#include <array>
#include <vector>

using reusex::core::SensorIntrinsics;
using reusex::vision::project_points_to_box;

namespace {

// Identity pose (camera at world origin, looking down +Z), simple intrinsics.
SensorIntrinsics makeIntrinsics() {
  SensorIntrinsics k;
  k.fx = 500;
  k.fy = 500;
  k.cx = 320;
  k.cy = 240;
  k.width = 640;
  k.height = 480;
  // local_transform defaults to identity.
  return k;
}

const std::array<double, 16> kIdentityPose = {1, 0, 0, 0, 0, 1, 0, 0,
                                              0, 0, 1, 0, 0, 0, 0, 1};

} // namespace

TEST_CASE("ProjectPointsToBox_PointOnOpticalAxis_LandsAtPrincipalPoint",
          "[vision][crop]") {
  auto k = makeIntrinsics();
  std::vector<Eigen::Vector3f> pts = {{0.f, 0.f, 2.f}};
  auto box = project_points_to_box(pts, kIdentityPose, k, 640, 480);
  REQUIRE(box.has_value());
  CHECK(box->in_bounds == 1);
  CHECK(box->x_min == 320);
  CHECK(box->y_min == 240);
}

TEST_CASE("ProjectPointsToBox_ClusterOfPoints_ComputesEnclosingBox",
          "[vision][crop]") {
  auto k = makeIntrinsics();
  // Points at z=2: x = px offset * z / fx. For px offsets +-100 -> x = +-0.4.
  std::vector<Eigen::Vector3f> pts = {
      {-0.4f, -0.24f, 2.f}, {0.4f, 0.24f, 2.f}, {0.f, 0.f, 2.f}};
  auto box = project_points_to_box(pts, kIdentityPose, k, 640, 480);
  REQUIRE(box.has_value());
  CHECK(box->in_bounds == 3);
  CHECK(box->x_min == 220);
  CHECK(box->x_max == 420);
  CHECK(box->y_min == 180);
  CHECK(box->y_max == 300);
}

TEST_CASE("ProjectPointsToBox_PointsBehindCamera_AreExcluded",
          "[vision][crop]") {
  auto k = makeIntrinsics();
  std::vector<Eigen::Vector3f> pts = {{0.f, 0.f, -2.f}, {0.f, 0.f, 2.f}};
  auto box = project_points_to_box(pts, kIdentityPose, k, 640, 480);
  REQUIRE(box.has_value());
  CHECK(box->in_bounds == 1); // only the +Z point counts
}

TEST_CASE("ProjectPointsToBox_AllPointsOutOfFrame_ReturnsNullopt",
          "[vision][crop]") {
  auto k = makeIntrinsics();
  // Far off to the side: x huge relative to z -> px well outside 0..640.
  std::vector<Eigen::Vector3f> pts = {{100.f, 0.f, 1.f}};
  auto box = project_points_to_box(pts, kIdentityPose, k, 640, 480);
  CHECK_FALSE(box.has_value());
}

TEST_CASE("ProjectPointsToBox_ScaledImageSize_ScalesIntrinsics",
          "[vision][crop]") {
  auto k = makeIntrinsics();
  std::vector<Eigen::Vector3f> pts = {{0.f, 0.f, 2.f}};
  // Half resolution: principal point should halve to (160, 120).
  auto box = project_points_to_box(pts, kIdentityPose, k, 320, 240);
  REQUIRE(box.has_value());
  CHECK(box->x_min == 160);
  CHECK(box->y_min == 120);
}

TEST_CASE("ProjectPointsToBox_EmptyInputOrBadIntrinsics_ReturnsNullopt",
          "[vision][crop]") {
  auto k = makeIntrinsics();
  CHECK_FALSE(
      project_points_to_box({}, kIdentityPose, k, 640, 480).has_value());

  SensorIntrinsics bad;
  std::vector<Eigen::Vector3f> pts = {{0.f, 0.f, 2.f}};
  CHECK_FALSE(
      project_points_to_box(pts, kIdentityPose, bad, 640, 480).has_value());
}
