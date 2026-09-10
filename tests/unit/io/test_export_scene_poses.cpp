// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Degenerate-pose guards in `gather_export_scene()` (#336).
//
// Two entries in an exported scene are placed from a linked sensor frame's
// stored pose: a 360 panorama with no content-aligned pose of its own, and a
// material passport (a survey photo) linked to the frame it was taken from.
// Both used to read the pose through `ProjectDB::sensor_frame_pose()` after
// checking only that the frame EXISTS — so a frame with a NULL, all-zero or
// NaN transform placed the marker at the world origin, or at NaN, and the
// exported scene showed it as a real position.
//
// Each test below is written so the old behaviour produces a *different*
// number than the new one, rather than the same origin:
//
//   * the panorama case stores a NaN transform, which the old code propagated
//     straight into the entry's coordinates;
//   * the material case gives the frame a non-identity `local_transform`, so
//     the old `pose * local_transform` composition was non-identity even when
//     `pose` fell back to identity.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/MaterialPassport.hpp>
#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/io/export_scene.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::test_support;
using Catch::Matchers::WithinAbs;

namespace {

constexpr int kW = 16;
constexpr int kH = 12;

/// x = 7 m: a placement no fallback would produce by accident.
constexpr double kSeedX = 7.0;

/// A camera-to-base transform with a translation in it. This is what makes
/// the material-passport test discriminating: the exporter composes
/// `pose * local_transform`, so under the old identity fallback the result was
/// `local_transform` — visibly not identity.
std::array<double, 16> offset_local_transform() {
  return translation_pose(0.0, 0.0, 3.0);
}

void seed_frame(ProjectDB &db, int node_id, const std::array<double, 16> &pose,
                std::array<double, 16> local = identity16()) {
  db.save_sensor_frame(
      node_id, make_color(kW, kH), cv::Mat(), cv::Mat(), pose,
      make_intrinsics(8.0, 8.0, kW / 2.0, kH / 2.0, kW, kH, local));
}

/// A minimal JPEG payload. `save_panoramic_image` stores the bytes without
/// decoding them, and `gather_export_scene` never decodes them either — only
/// the row's `node_id` and pose columns matter here.
std::vector<std::uint8_t> fake_jpeg() { return {0xFF, 0xD8, 0xFF, 0xD9}; }

const io::ExportScene::PanoEntry &find_pano(const io::ExportScene &scene,
                                            std::string_view name) {
  auto it = std::find_if(scene.panoramas.begin(), scene.panoramas.end(),
                         [&](const io::ExportScene::PanoEntry &e) {
                           return e.image_name == name;
                         });
  REQUIRE(it != scene.panoramas.end());
  return *it;
}

} // namespace

// ===========================================================================
// 360 panoramas
// ===========================================================================

TEST_CASE("GatherExportScene_PanoramaSeedFrameHasPose_PlacesPanoramaAtThatPose",
          "[io][export_scene][pose]") {
  // The positive control: the fallback path still works when the seed frame
  // genuinely has a pose.
  TempPath tmp("test_export_scene_pano_posed");

  ProjectDB db(tmp.path);
  seed_frame(db, 3, translation_pose(kSeedX, 0.0, 0.0));
  db.save_panoramic_image("pano.jpg", fake_jpeg(), -1.0, 3);

  const auto scene = io::gather_export_scene(db);
  const auto &entry = find_pano(scene, "pano.jpg");
  CHECK_THAT(entry.x, WithinAbs(kSeedX, 1e-9));
}

TEST_CASE("GatherExportScene_PanoramaSeedFrameWithNaNPose_LeavesItUnplaced",
          "[io][export_scene][pose]") {
  TempPath tmp("test_export_scene_pano_nan");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 3, translation_pose(kSeedX, 0.0, 0.0));
    db.save_panoramic_image("pano.jpg", fake_jpeg(), -1.0, 3);
  }
  set_raw_sensor_frame_pose(tmp.path, 3, nan_translation_pose());

  ProjectDB db(tmp.path);
  const auto scene = io::gather_export_scene(db);
  const auto &entry = find_pano(scene, "pano.jpg");

  // The old code propagated the stored NaN into the entry. The panorama is now
  // exported unplaced — the same state one with no linked frame at all gets.
  CHECK(std::isfinite(entry.x));
  CHECK_THAT(entry.x, WithinAbs(0.0, 1e-9));
  CHECK_THAT(entry.y, WithinAbs(0.0, 1e-9));
  CHECK_THAT(entry.z, WithinAbs(0.0, 1e-9));
}

TEST_CASE("GatherExportScene_PanoramaSeedFrameWithoutPose_LeavesItUnplaced",
          "[io][export_scene][pose]") {
  TempPath tmp("test_export_scene_pano_poseless");

  {
    ProjectDB db(tmp.path);
    seed_frame(db, 3, translation_pose(kSeedX, 0.0, 0.0));
    db.save_panoramic_image("pano.jpg", fake_jpeg(), -1.0, 3);
  }

  SECTION("NULL transform") { clear_sensor_frame_pose(tmp.path, 3); }
  SECTION("all-zero transform") {
    set_raw_sensor_frame_pose(tmp.path, 3, zero_pose());
  }

  ProjectDB db(tmp.path);
  const auto scene = io::gather_export_scene(db);
  const auto &entry = find_pano(scene, "pano.jpg");
  CHECK_THAT(entry.x, WithinAbs(0.0, 1e-9));
}

// ===========================================================================
// Material passports
// ===========================================================================

namespace {

/// Store a passport linked to `node_id`. The three-argument overload writes
/// the node id into `material_passports.id`, which is what
/// `passport_linked_node_id()` reads back.
std::string seed_linked_passport(ProjectDB &db, int node_id) {
  core::MaterialPassport passport;
  passport.metadata.document_guid = "guid-" + std::to_string(node_id);
  passport.description.designation = "photo-" + std::to_string(node_id);
  db.add_material_passport(passport, "proj", std::to_string(node_id));
  return passport.description.designation;
}

const io::ExportScene::MaterialEntry &
find_material(const io::ExportScene &scene, const std::string &name) {
  auto it = std::find_if(
      scene.materials.begin(), scene.materials.end(),
      [&](const io::ExportScene::MaterialEntry &e) { return e.name == name; });
  REQUIRE(it != scene.materials.end());
  return *it;
}

} // namespace

TEST_CASE("GatherExportScene_MaterialLinkedToPosedFrame_PlacesItAtThatPose",
          "[io][export_scene][pose]") {
  TempPath tmp("test_export_scene_material_posed");

  ProjectDB db(tmp.path);
  seed_frame(db, 5, translation_pose(kSeedX, 0.0, 0.0),
             offset_local_transform());
  const std::string name = seed_linked_passport(db, 5);

  const auto scene = io::gather_export_scene(db);
  const auto &entry = find_material(scene, name);

  // transform = pose * local_transform: (7,0,0) composed with (0,0,3).
  CHECK_THAT(entry.x, WithinAbs(kSeedX, 1e-9));
  CHECK_THAT(entry.z, WithinAbs(3.0, 1e-9));
}

TEST_CASE("GatherExportScene_MaterialLinkedToPoselessFrame_LeavesItUnplaced",
          "[io][export_scene][pose]") {
  TempPath tmp("test_export_scene_material_poseless");

  std::string name;
  {
    ProjectDB db(tmp.path);
    seed_frame(db, 5, translation_pose(kSeedX, 0.0, 0.0),
               offset_local_transform());
    name = seed_linked_passport(db, 5);
  }

  SECTION("NULL transform") { clear_sensor_frame_pose(tmp.path, 5); }
  SECTION("all-zero transform") {
    set_raw_sensor_frame_pose(tmp.path, 5, zero_pose());
  }
  SECTION("NaN translation") {
    set_raw_sensor_frame_pose(tmp.path, 5, nan_translation_pose());
  }

  ProjectDB db(tmp.path);
  const auto scene = io::gather_export_scene(db);
  const auto &entry = find_material(scene, name);

  // Old behaviour composed the identity fallback with the frame's non-identity
  // local_transform and reported a camera 3 m up. The entry now keeps the
  // identity default an unlinked passport gets.
  CHECK_THAT(entry.x, WithinAbs(0.0, 1e-9));
  CHECK_THAT(entry.y, WithinAbs(0.0, 1e-9));
  CHECK_THAT(entry.z, WithinAbs(0.0, 1e-9));
  CHECK_THAT(entry.transform[11], WithinAbs(0.0, 1e-9));
}
