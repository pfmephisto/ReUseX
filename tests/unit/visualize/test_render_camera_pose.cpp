// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// `camera_from_sensor_frame()` and degenerate stored poses (#336).
//
// This is the one audited `sensor_frame_pose()` consumer with nothing to skip:
// it returns exactly one camera, so `rux render --view frame:<id>` on a
// poseless frame would silently render the scene from the world origin and
// hand back a PNG that looks like a real answer. It refuses instead, the same
// way it already refuses a frame with unusable intrinsics.
//
// No rendering happens here — `camera_from_sensor_frame` is pure composition
// and rescaling, so these tests need no GL context or display.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/visualize/render_view.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <stdexcept>

using namespace reusex;
using namespace reusex::test_support;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

namespace {

constexpr int kW = 32;
constexpr int kH = 24;
constexpr double kFarX = 12.0;

void seed_frame(ProjectDB &db, int node_id,
                const std::array<double, 16> &pose) {
  db.save_sensor_frame(node_id, make_color(kW, kH), cv::Mat(), cv::Mat(), pose,
                       make_intrinsics(16.0, 16.0, kW / 2.0, kH / 2.0, kW, kH));
}

} // namespace

TEST_CASE("CameraFromSensorFrame_PosedFrame_ReturnsStoredPoseAndScaledK",
          "[visualize][render][pose]") {
  TempPath tmp("test_render_camera_posed");

  ProjectDB db(tmp.path);
  seed_frame(db, 1, translation_pose(kFarX, 0.0, 0.0));

  // Render at twice the captured size: intrinsics scale, the pose does not.
  const auto spec = visualize::camera_from_sensor_frame(db, 1, kW * 2, kH * 2);

  CHECK_THAT(spec.pose[3], WithinAbs(kFarX, 1e-9));
  CHECK_THAT(spec.fx, WithinAbs(32.0, 1e-9));
  CHECK_THAT(spec.fy, WithinAbs(32.0, 1e-9));
}

TEST_CASE("CameraFromSensorFrame_FrameWithUnusablePose_Throws",
          "[visualize][render][pose]") {
  TempPath tmp("test_render_camera_poseless");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 1, translation_pose(kFarX, 0.0, 0.0));
  }

  SECTION("NULL transform") { clear_sensor_frame_pose(tmp.path, 1); }
  SECTION("all-zero transform") {
    set_raw_sensor_frame_pose(tmp.path, 1, zero_pose());
  }
  SECTION("NaN translation") {
    set_raw_sensor_frame_pose(tmp.path, 1, nan_translation_pose());
  }

  ProjectDB db(tmp.path);
  REQUIRE_THROWS_AS(visualize::camera_from_sensor_frame(db, 1, kW, kH),
                    std::runtime_error);
  // The message has to name the frame — that is what makes the failure
  // actionable rather than just loud (STANDARDS §5).
  REQUIRE_THROWS_WITH(visualize::camera_from_sensor_frame(db, 1, kW, kH),
                      ContainsSubstring("sensor frame 1") &&
                          ContainsSubstring("no usable stored pose"));
}

TEST_CASE("CameraFromSensorFrame_FrameWithStoredIdentityPose_ReturnsCamera",
          "[visualize][render][pose]") {
  // A scan may legitimately put a frame at the origin; that is a pose, not the
  // absence of one, and rendering from it must keep working.
  TempPath tmp("test_render_camera_identity");

  ProjectDB db(tmp.path);
  seed_frame(db, 1, identity16());

  const auto spec = visualize::camera_from_sensor_frame(db, 1, kW, kH);
  CHECK_THAT(spec.pose[3], WithinAbs(0.0, 1e-9));
  CHECK_THAT(spec.fx, WithinAbs(16.0, 1e-9));
}
