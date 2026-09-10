// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Degenerate-pose guard in `reusex::vision::project()` (#336).
//
// `project()` casts each frame's 2D segmentation image onto the 3D cloud
// through that frame's stored world pose. A frame with no usable pose came
// back from `ProjectDB::sensor_frame_pose()` as identity, so its labels were
// written onto whichever cloud points happened to sit in front of the world
// origin — confident semantic labels on the wrong geometry, and nothing in the
// log to attribute them to. It now skips such frames, leaving those points
// unlabeled, which is recoverable.
//
// The scene mirrors test_project_labels.cpp (fronto-parallel wall, pinhole
// camera looking down +z) with one deliberate change: the frame's stored pose
// puts the camera **20 m along +x**, and the guard test then places the wall
// at the ORIGIN. That is what makes the assertion discriminating rather than
// vacuous:
//
//   * through the frame's stored pose, the origin wall is 20 m off-axis and
//     projects far outside the image — nothing is labelled, which is also what
//     skipping the frame produces;
//   * through the identity fallback the old code used, the origin wall is
//     dead ahead and gets labelled.
//
// So "0 labelled points" is a claim only the fixed code can satisfy. The
// positive control above it puts the wall in front of the camera's stored
// position, proving the fixture can label anything at all.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <core/label_semantics.hpp>
#include <types/point_types.hpp>
#include <vision/project.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>

#include <array>
#include <cstddef>

using reusex::ProjectDB;
using reusex::vision::project;
using namespace reusex::test_support;

namespace {

constexpr int kWidth = 128;
constexpr int kHeight = 128;
constexpr double kFx = 128.0;
constexpr double kFy = 128.0;
constexpr float kDepth = 2.0F;
constexpr int kPatchLo = 40;
constexpr int kPatchHi = 88;
constexpr int kPatchClass = 7;

/// Where the frame's stored pose puts the camera, along +x.
constexpr float kCameraX = 20.0F;

cv::Mat make_label_image() {
  cv::Mat labels(kHeight, kWidth, CV_32S,
                 cv::Scalar(reusex::core::kBackgroundApi));
  labels(cv::Range(kPatchLo, kPatchHi), cv::Range(kPatchLo, kPatchHi))
      .setTo(kPatchClass);
  return labels;
}

/// A fronto-parallel wall centred at `centre_x`. `kCameraX` puts it in front
/// of the camera's stored position; `0` puts it in front of the world origin,
/// i.e. where the identity fallback would look.
reusex::CloudPtr make_wall(float centre_x) {
  reusex::CloudPtr cloud(new reusex::Cloud);
  for (int i = 0; i < 73; ++i) {
    for (int j = 0; j < 73; ++j) {
      reusex::PointT p;
      p.x = centre_x - 0.9F + 0.025F * static_cast<float>(i);
      p.y = -0.9F + 0.025F * static_cast<float>(j);
      p.z = kDepth;
      p.r = p.g = p.b = 200;
      cloud->push_back(p);
    }
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

void seed_frame(ProjectDB &db, int node_id) {
  cv::Mat color(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0));
  db.save_sensor_frame(
      node_id, color, cv::Mat(), cv::Mat(),
      translation_pose(kCameraX, 0.0, 0.0),
      make_intrinsics(kFx, kFy, kWidth / 2.0, kHeight / 2.0, kWidth, kHeight));
  db.save_segmentation_image(node_id, make_label_image());
}

std::size_t labelled_count(const reusex::CloudLPtr &labels) {
  std::size_t n = 0;
  for (const auto &l : *labels)
    if (l.label != reusex::core::kUnlabeled)
      ++n;
  return n;
}

} // namespace

TEST_CASE("Project_PosedFrame_LabelsThePointsItSees",
          "[vision][project][pose]") {
  // Positive control: with the pose intact the labelled square still lands on
  // the wall, so the guard below is measuring the guard and not the fixture.
  TempPath tmp("test_project_posed");

  ProjectDB db(tmp.path);
  seed_frame(db, 1);

  auto labels = project(db, make_wall(kCameraX));
  CHECK(labelled_count(labels) > 0);
}

TEST_CASE("Project_PosedFrame_LeavesGeometryOutsideItsViewUnlabeled",
          "[vision][project][pose]") {
  // The other half of the control: through the STORED pose, a wall at the
  // origin is out of frame. This is the number the guard test has to match —
  // it is what proves that test's 0 comes from the pose, not from the wall
  // being unlabelable.
  TempPath tmp("test_project_posed_offaxis");

  ProjectDB db(tmp.path);
  seed_frame(db, 1);

  auto labels = project(db, make_wall(0.0F));
  CHECK(labelled_count(labels) == 0);
}

TEST_CASE("Project_FrameWithUnusablePose_LeavesAllPointsUnlabeled",
          "[vision][project][pose]") {
  TempPath tmp("test_project_poseless");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 1);
  }

  SECTION("NULL transform") { clear_sensor_frame_pose(tmp.path, 1); }
  SECTION("all-zero transform") {
    set_raw_sensor_frame_pose(tmp.path, 1, zero_pose());
  }
  SECTION("NaN translation") {
    set_raw_sensor_frame_pose(tmp.path, 1, nan_translation_pose());
  }

  ProjectDB db(tmp.path);
  // The wall sits at the origin — exactly where the identity fallback would
  // look, and exactly where the stored pose does not.
  auto labels = project(db, make_wall(0.0F));

  // The frame is skipped entirely, so every point keeps the 0 that means
  // "unlabeled" (STANDARDS §3).
  REQUIRE(labels);
  CHECK(labelled_count(labels) == 0);
}
