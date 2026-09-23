// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Point -> frame visibility cross-reference (#453).
//
// `visible_frames()` projects a world point into every posed sensor frame and
// ranks the frames that see it by how centrally it appears. The fixture below
// is a pinhole camera (fx=fy=100, principal point at the image centre, camera
// == base so the local transform is identity) placed at four world poses, all
// looking down the optical +z axis, and a single world point at (0, 0, 2):
//
//   * frame 1 — camera at the origin: the point lands dead centre (centrality
//     ~0), the ideal source image;
//   * frame 2 — camera shifted +0.3 m in x: the point is off to one side but
//     still inside the image, so it ranks below frame 1;
//   * frame 3 — camera 5 m *past* the point along +z: the point is behind the
//     camera and must be rejected (the in-front test);
//   * frame 4 — camera 5 m off in x: the point projects far outside the image
//     and must be rejected (the frustum test).
//
// So the discriminating claim is that exactly {1, 2} are returned, in that
// order, with frame 1 strictly more central than frame 2.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <core/frame_visibility.hpp>

#include "../../support/temp_path.hpp"

#include <Eigen/Core>

#include <opencv2/core.hpp>

#include <array>

using reusex::ProjectDB;
using reusex::core::VisibilityQuery;
using reusex::core::visible_frames;
using namespace reusex::test_support;

namespace {

constexpr int kWidth = 128;
constexpr int kHeight = 128;

reusex::core::SensorIntrinsics make_intrinsics() {
  reusex::core::SensorIntrinsics i;
  i.fx = i.fy = 100.0;
  i.cx = i.cy = 64.0;
  i.width = kWidth;
  i.height = kHeight;
  // Identity local transform: the camera optical frame coincides with the
  // stored base frame, so the world pose alone places the camera.
  i.local_transform = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  return i;
}

/// Row-major identity pose translated by (x, y, z).
std::array<double, 16> pose_at(double x, double y, double z) {
  return {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1};
}

void save_frame(ProjectDB &db, int node_id,
                const std::array<double, 16> &pose) {
  cv::Mat color(kHeight, kWidth, CV_8UC3, cv::Scalar(40, 90, 160));
  db.save_sensor_frame(node_id, color, cv::Mat(), cv::Mat(), pose,
                       make_intrinsics(), static_cast<double>(node_id), -1);
}

} // namespace

TEST_CASE("visible_frames ranks source frames by centrality", "[core]") {
  const TempPath tmp("frame_visibility");
  const auto &path = tmp.path;
  {
    ProjectDB db(path);
    save_frame(db, 1, pose_at(0.0, 0.0, 0.0)); // dead centre
    save_frame(db, 2, pose_at(0.3, 0.0, 0.0)); // off to one side, in frame
    save_frame(db, 3, pose_at(0.0, 0.0, 5.0)); // point is behind the camera
    save_frame(db, 4, pose_at(5.0, 0.0, 0.0)); // point projects out of frame
  }

  ProjectDB db(path);
  const Eigen::Vector3d point(0.0, 0.0, 2.0);

  SECTION("only in-front, in-frustum frames are returned, best-first") {
    const auto frames = visible_frames(db, point);

    REQUIRE(frames.size() == 2);
    CHECK(frames[0].frame_id == 1);
    CHECK(frames[1].frame_id == 2);

    // Frame 1 sees the point at the principal point: essentially centrality 0.
    CHECK(frames[0].centrality < 1e-9);
    // Frame 2 is off-centre, so strictly less central than frame 1.
    CHECK(frames[1].centrality > frames[0].centrality);

    // Depth is the optical-axis distance, 2 m for both.
    CHECK(frames[0].depth == Catch::Approx(2.0));
    CHECK(frames[1].depth == Catch::Approx(2.0));

    // Frame 1's projection is the principal point.
    CHECK(frames[0].u == Catch::Approx(64.0));
    CHECK(frames[0].v == Catch::Approx(64.0));
    // Frame 2: u = fx * (-0.3) / 2 + cx = -15 + 64 = 49.
    CHECK(frames[1].u == Catch::Approx(49.0));
    CHECK(frames[1].v == Catch::Approx(64.0));
  }

  SECTION("max_depth rejects frames beyond the range limit") {
    VisibilityQuery q;
    q.max_depth = 1.5; // the point sits at 2 m, so nothing survives
    CHECK(visible_frames(db, point, q).empty());
  }

  SECTION("a point behind every camera is visible nowhere") {
    // Far behind all four cameras along -z.
    CHECK(visible_frames(db, Eigen::Vector3d(0.0, 0.0, -10.0)).empty());
  }
}
