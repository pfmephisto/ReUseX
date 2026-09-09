// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Tests for `load_training_views` (src/gsplat/TrainingViews.cpp) — the step
// that turns a `.rux` project into the posed image set a 3DGS run is trained
// against. Everything it decides is silent: which frames are kept
// (--frame-stride, --first-frame/--last-frame, --max-views), what resolution
// they arrive at, what `K` says about that resolution, and where each camera
// sits in the world. A mistake in any of those produces a smeared
// reconstruction with a perfectly healthy-looking loss curve, so the
// assertions here are on numbers, not on log lines.
//
// Two things in particular:
//
//   * #330 — a frame whose `transform` column is NULL used to enter the
//     training set as a camera at the world origin, because
//     `ProjectDB::sensor_frame_pose()` returns identity for a frame that has
//     no pose at all. The loader now asks `has_sensor_frame_pose()` first.
//     The regression that would make that fix wrong in the other direction is
//     a *genuinely stored* identity pose, which is legitimate and must be
//     kept; there is a test for exactly that below.
//   * #332 item 3 — before this file `TrainingViews` had no unit tests at all.
//
// Deliberately in the LIGHT test binary: this file includes
// <reusex/gsplat/TrainingViews.hpp> and nothing else from the module, so it
// links only `reusex_gsplat_common` (the torch-free half, #332) and compiles
// in the CPU-only nix build CI actually performs. Nothing here may reference
// torch, CUDA, `train_gaussians`, `render_view` or `has_cuda_device`.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/gsplat/TrainingViews.hpp>

#include "../../support/temp_path.hpp"

#include <opencv2/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sqlite3.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

using namespace reusex;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace fs = std::filesystem;

namespace {

// ---------------------------------------------------------------------------
// Fixture helpers
// ---------------------------------------------------------------------------

std::array<double, 16> identity16() {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

std::array<double, 16> from_eigen(const Eigen::Matrix4d &m) {
  std::array<double, 16> out{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out[static_cast<std::size_t>(r) * 4 + c] = m(r, c);
  return out;
}

Eigen::Matrix4d to_eigen(const std::array<double, 16> &m) {
  Eigen::Matrix4d out;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out(r, c) = m[static_cast<std::size_t>(r) * 4 + c];
  return out;
}

/// Identity rotation, translation (x, y, z) — a distinct, valid pose per
/// frame so a test can tell the retained frames apart by their camera centre.
std::array<double, 16> translation_pose(double x, double y, double z) {
  auto p = identity16();
  p[3] = x;
  p[7] = y;
  p[11] = z;
  return p;
}

cv::Mat make_image(int w, int h, cv::Vec3b fill = {40, 90, 160}) {
  return cv::Mat(h, w, CV_8UC3, fill);
}

core::SensorIntrinsics
make_intrinsics(double fx, double fy, double cx, double cy, int w, int h,
                std::array<double, 16> local = identity16()) {
  core::SensorIntrinsics intr;
  intr.fx = fx;
  intr.fy = fy;
  intr.cx = cx;
  intr.cy = cy;
  intr.width = w;
  intr.height = h;
  intr.local_transform = local;
  return intr;
}

/// A small but valid frame: 8x6 pixels with intrinsics that agree with it.
core::SensorIntrinsics tiny_intrinsics() {
  return make_intrinsics(6.0, 6.0, 4.0, 3.0, 8, 6);
}

void seed_frame(ProjectDB &db, int id, const std::array<double, 16> &pose,
                const core::SensorIntrinsics &intr, const cv::Mat &img) {
  db.save_sensor_frame(id, img, cv::Mat(), cv::Mat(), pose, intr);
}

void seed_frame(ProjectDB &db, int id, const std::array<double, 16> &pose,
                const core::SensorIntrinsics &intr) {
  seed_frame(db, id, pose, intr, make_image(intr.width, intr.height));
}

/// Seed `ids` frames, each with a distinct translation-only pose.
void seed_frames(ProjectDB &db, const std::vector<int> &ids) {
  for (int id : ids)
    seed_frame(db, id, translation_pose(id * 0.5, 0.0, 0.0), tiny_intrinsics());
}

std::vector<int> view_ids(const std::vector<gsplat::TrainingView> &views) {
  std::vector<int> ids;
  ids.reserve(views.size());
  for (const auto &v : views)
    ids.push_back(v.id);
  return ids;
}

// ---------------------------------------------------------------------------
// Raw-SQLite mutation
// ---------------------------------------------------------------------------
//
// These helpers exist because there is no public API that can produce the rows
// under test. `ProjectDB::save_sensor_frame` always binds a well-formed
// 128-byte `transform` blob and a non-empty JPEG `color` blob (it throws on an
// empty image), so "frame with no pose" and "frame with no colour image" —
// both of which real projects get from importers that never had the data —
// are unreachable through the C++ interface. The only honest way to build the
// fixture is to write the row directly.
//
// The caller must have closed its `ProjectDB` first: the database runs in WAL
// mode, so reopening after the write is what makes the change visible to a new
// connection.

void exec_raw(const fs::path &db_path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(db_path.string().c_str(), &raw) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, &err);
  const std::string message = err ? err : "";
  sqlite3_free(err);
  const int changed = sqlite3_changes(raw);
  sqlite3_close(raw);
  INFO(message);
  REQUIRE(rc == SQLITE_OK);
  REQUIRE(changed == 1);
}

/// Make a frame poseless the way an importer without poses leaves it.
void clear_sensor_frame_pose(const fs::path &db_path, int node_id) {
  exec_raw(db_path, "UPDATE sensor_frames SET transform = NULL WHERE node_id "
                    "= " +
                        std::to_string(node_id) + ";");
}

/// Drop the colour blob, leaving the pose and intrinsics intact.
void clear_sensor_frame_image(const fs::path &db_path, int node_id) {
  exec_raw(db_path, "UPDATE sensor_frames SET color = NULL WHERE node_id = " +
                        std::to_string(node_id) + ";");
}

} // namespace

// ===========================================================================
// #330 — poseless frames
// ===========================================================================

TEST_CASE("LoadTrainingViews_FrameWithoutStoredPose_SkipsFrame", "[gsplat]") {
  test_support::TempPath tmp("test_training_views");

  {
    ProjectDB db(tmp.path);
    seed_frames(db, {1, 2, 3, 4});
  }
  // Frames 2 and 3 lose their pose; 1 and 4 keep theirs.
  clear_sensor_frame_pose(tmp.path, 2);
  clear_sensor_frame_pose(tmp.path, 3);

  ProjectDB db(tmp.path);
  auto views = gsplat::load_training_views(db, {});

  REQUIRE(view_ids(views) == std::vector<int>{1, 4});

  // The survivors are the frames we posed, at the centres we put them:
  // T_bc is identity here, so the camera centre is the pose translation.
  for (const auto &v : views) {
    const Eigen::Matrix3d R = v.T_cw.block<3, 3>(0, 0);
    const Eigen::Vector3d centre = -R.transpose() * v.T_cw.block<3, 1>(0, 3);
    REQUIRE_THAT(centre.x(), WithinAbs(v.id * 0.5, 1e-9));
    REQUIRE_THAT(centre.y(), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(centre.z(), WithinAbs(0.0, 1e-9));
  }
}

TEST_CASE("LoadTrainingViews_AllFramesPoseless_ThrowsNamingSkipCounts",
          "[gsplat]") {
  test_support::TempPath tmp("test_training_views");

  {
    ProjectDB db(tmp.path);
    seed_frames(db, {1, 2, 3});
  }
  for (int id : {1, 2, 3})
    clear_sensor_frame_pose(tmp.path, id);

  ProjectDB db(tmp.path);
  REQUIRE_THROWS_AS(gsplat::load_training_views(db, {}), std::runtime_error);
  REQUIRE_THROWS_WITH(
      gsplat::load_training_views(db, {}),
      ContainsSubstring("no usable training views out of 3 sensor frames") &&
          ContainsSubstring("skipped 3 without a stored pose"));
}

TEST_CASE("LoadTrainingViews_GenuinelyStoredIdentityPose_KeepsFrame",
          "[gsplat]") {
  // The regression that would make the #330 fix wrong in the other direction:
  // a scan may legitimately put a frame at the world origin, and an identity
  // pose that was actually written is a pose. Only the absence of a blob is a
  // missing pose.
  test_support::TempPath tmp("test_training_views");

  ProjectDB db(tmp.path);
  seed_frame(db, 11, identity16(), tiny_intrinsics());

  auto views = gsplat::load_training_views(db, {});

  REQUIRE(views.size() == 1);
  REQUIRE(views[0].id == 11);
  // T_wb = T_bc = I, so T_cw = I too.
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      REQUIRE_THAT(views[0].T_cw(r, c), WithinAbs(r == c ? 1.0 : 0.0, 1e-12));
}

TEST_CASE("LoadTrainingViews_FewerThanTwoPosedFrames_Throws", "[gsplat]") {
  // One view cannot triangulate: every Gaussian is free to slide along its
  // viewing ray, so the run would produce a confident-looking result that
  // means nothing. Refuse instead — but only because poses were the reason.
  test_support::TempPath tmp("test_training_views");

  {
    ProjectDB db(tmp.path);
    seed_frames(db, {1, 2, 3});
  }
  clear_sensor_frame_pose(tmp.path, 2);
  clear_sensor_frame_pose(tmp.path, 3);

  ProjectDB db(tmp.path);
  REQUIRE_THROWS_WITH(
      gsplat::load_training_views(db, {}),
      ContainsSubstring(
          "only 1 usable training view(s) after skipping 2 of 3"));
}

// ===========================================================================
// Frame selection
// ===========================================================================

TEST_CASE("LoadTrainingViews_FrameStride_KeepsEveryNthFrame", "[gsplat]") {
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);
  seed_frames(db, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10});

  gsplat::TrainingViewOptions opt;
  opt.frame_stride = 3;
  auto views = gsplat::load_training_views(db, opt);

  REQUIRE(view_ids(views) == std::vector<int>{1, 4, 7, 10});
}

TEST_CASE("LoadTrainingViews_FirstAndLastFrame_ClipsToInclusiveRange",
          "[gsplat]") {
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);
  seed_frames(db, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10});

  SECTION("both bounds") {
    gsplat::TrainingViewOptions opt;
    opt.first_frame = 3;
    opt.last_frame = 6;
    REQUIRE(view_ids(gsplat::load_training_views(db, opt)) ==
            std::vector<int>{3, 4, 5, 6});
  }
  SECTION("lower bound only") {
    gsplat::TrainingViewOptions opt;
    opt.first_frame = 8;
    REQUIRE(view_ids(gsplat::load_training_views(db, opt)) ==
            std::vector<int>{8, 9, 10});
  }
  SECTION("upper bound only") {
    gsplat::TrainingViewOptions opt;
    opt.last_frame = 2;
    REQUIRE(view_ids(gsplat::load_training_views(db, opt)) ==
            std::vector<int>{1, 2});
  }
  SECTION("-1 means unbounded") {
    gsplat::TrainingViewOptions opt;
    opt.first_frame = -1;
    opt.last_frame = -1;
    REQUIRE(gsplat::load_training_views(db, opt).size() == 10);
  }
}

TEST_CASE("LoadTrainingViews_ClippedRangeWithStride_"
          "StridesWithinRegionNotGlobalIndex",
          "[gsplat]") {
  // The ordering property the source calls out: region selection runs first,
  // striding second, "striding before clipping would make the retained frames
  // depend on where the range starts". Both sections below stride by 2 over
  // ids 1..12; the only difference is the lower bound, and each one has to
  // begin at its own first in-range frame.
  //
  // These expectations are what discriminates the two orders. Striding first
  // would keep the odd ids {1,3,5,7,9,11} globally and then clip, giving
  // {5,7,9,11} for BOTH sections — the "range start does not matter" bug.
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);
  seed_frames(db, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12});

  gsplat::TrainingViewOptions opt;
  opt.frame_stride = 2;
  opt.last_frame = 12;

  SECTION("range starting on an even id") {
    opt.first_frame = 4;
    REQUIRE(view_ids(gsplat::load_training_views(db, opt)) ==
            std::vector<int>{4, 6, 8, 10, 12});
  }
  SECTION("range starting on an odd id") {
    opt.first_frame = 5;
    REQUIRE(view_ids(gsplat::load_training_views(db, opt)) ==
            std::vector<int>{5, 7, 9, 11});
  }
}

TEST_CASE("LoadTrainingViews_MaxViews_CapsViewCountAfterStriding", "[gsplat]") {
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);
  seed_frames(db, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10});

  SECTION("cap below the available count") {
    gsplat::TrainingViewOptions opt;
    opt.max_views = 3;
    REQUIRE(view_ids(gsplat::load_training_views(db, opt)) ==
            std::vector<int>{1, 2, 3});
  }
  SECTION("cap applies to the strided sequence, not the raw ids") {
    gsplat::TrainingViewOptions opt;
    opt.frame_stride = 4;
    opt.max_views = 2;
    REQUIRE(view_ids(gsplat::load_training_views(db, opt)) ==
            std::vector<int>{1, 5});
  }
  SECTION("0 means unlimited") {
    gsplat::TrainingViewOptions opt;
    opt.max_views = 0;
    REQUIRE(gsplat::load_training_views(db, opt).size() == 10);
  }
}

TEST_CASE("LoadTrainingViews_FrameStrideBelowOne_Throws", "[gsplat]") {
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);
  seed_frames(db, {1, 2});

  gsplat::TrainingViewOptions opt;
  SECTION("zero") { opt.frame_stride = 0; }
  SECTION("negative") { opt.frame_stride = -3; }

  REQUIRE_THROWS_WITH(gsplat::load_training_views(db, opt),
                      ContainsSubstring("frame_stride must be >= 1"));
}

// ===========================================================================
// Downscale + intrinsics rescale
// ===========================================================================

TEST_CASE("LoadTrainingViews_ImageAboveMaxSize_DownscalesAndRescalesK",
          "[gsplat]") {
  // 400x200 at max_image_size 100 is an exact quarter, so both the resulting
  // size and every entry of K are hand-computable.
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);

  auto intr = make_intrinsics(350.0, 310.0, 201.0, 97.0, 400, 200);
  seed_frame(db, 1, identity16(), intr);

  gsplat::TrainingViewOptions opt;
  opt.max_image_size = 100;
  auto views = gsplat::load_training_views(db, opt);

  REQUIRE(views.size() == 1);
  const auto &v = views[0];
  REQUIRE(v.width() == 100);
  REQUIRE(v.height() == 50);

  REQUIRE_THAT(v.K(0, 0), WithinAbs(350.0 * 0.25, 1e-9));
  REQUIRE_THAT(v.K(1, 1), WithinAbs(310.0 * 0.25, 1e-9));
  REQUIRE_THAT(v.K(0, 2), WithinAbs(201.0 * 0.25, 1e-9));
  REQUIRE_THAT(v.K(1, 2), WithinAbs(97.0 * 0.25, 1e-9));

  // The invariants that must survive any resize: a pinhole camera's field of
  // view and principal-point placement are ratios against the image, so these
  // four are what "K still describes this image" actually means.
  REQUIRE_THAT(v.K(0, 0) / v.width(), WithinRel(350.0 / 400.0, 1e-12));
  REQUIRE_THAT(v.K(1, 1) / v.height(), WithinRel(310.0 / 200.0, 1e-12));
  REQUIRE_THAT(v.K(0, 2) / v.width(), WithinRel(201.0 / 400.0, 1e-12));
  REQUIRE_THAT(v.K(1, 2) / v.height(), WithinRel(97.0 / 200.0, 1e-12));
}

TEST_CASE("LoadTrainingViews_RaggedDownscale_RescalesKByRealisedRatioPerAxis",
          "[gsplat]") {
  // 401x201 at max 100 rounds to 100x50, so the realised ratios differ per
  // axis (100/401 != 50/201). Scaling K by the *requested* ratio instead of
  // the realised one leaves fy/cy wrong by ~0.25% — invisible in a rendered
  // frame, fatal to the geometry. The four ratio invariants below are exactly
  // what catches that.
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);

  auto intr = make_intrinsics(410.0, 395.0, 200.5, 100.5, 401, 201);
  seed_frame(db, 1, identity16(), intr);

  gsplat::TrainingViewOptions opt;
  opt.max_image_size = 100;
  auto views = gsplat::load_training_views(db, opt);

  REQUIRE(views.size() == 1);
  const auto &v = views[0];
  REQUIRE(v.width() == 100);
  REQUIRE(v.height() == 50);

  REQUIRE_THAT(v.K(0, 0) / v.width(), WithinRel(410.0 / 401.0, 1e-12));
  REQUIRE_THAT(v.K(1, 1) / v.height(), WithinRel(395.0 / 201.0, 1e-12));
  REQUIRE_THAT(v.K(0, 2) / v.width(), WithinRel(200.5 / 401.0, 1e-12));
  REQUIRE_THAT(v.K(1, 2) / v.height(), WithinRel(100.5 / 201.0, 1e-12));

  // ...and the per-axis ratios really are different here, so the assertions
  // above are not accidentally satisfied by a single-scalar implementation.
  const double sx = v.K(0, 0) / 410.0;
  const double sy = v.K(1, 1) / 395.0;
  REQUIRE(std::abs(sx - sy) > 1e-6);
}

TEST_CASE("LoadTrainingViews_NoDownscaleNeeded_LeavesImageAndKUntouched",
          "[gsplat]") {
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);

  auto intr = make_intrinsics(350.0, 310.0, 201.0, 97.0, 400, 200);
  seed_frame(db, 1, identity16(), intr);

  gsplat::TrainingViewOptions opt;
  SECTION("max_image_size = 0 disables resizing") { opt.max_image_size = 0; }
  SECTION("image already within the budget") { opt.max_image_size = 400; }
  SECTION("image comfortably below the budget") { opt.max_image_size = 4096; }

  auto views = gsplat::load_training_views(db, opt);
  REQUIRE(views.size() == 1);
  const auto &v = views[0];
  REQUIRE(v.width() == 400);
  REQUIRE(v.height() == 200);
  REQUIRE_THAT(v.K(0, 0), WithinAbs(350.0, 1e-12));
  REQUIRE_THAT(v.K(1, 1), WithinAbs(310.0, 1e-12));
  REQUIRE_THAT(v.K(0, 2), WithinAbs(201.0, 1e-12));
  REQUIRE_THAT(v.K(1, 2), WithinAbs(97.0, 1e-12));
}

TEST_CASE("LoadTrainingViews_IntrinsicsResolutionDiffersFromImage_"
          "CorrectsKToTheImage",
          "[gsplat]") {
  // The stored intrinsics describe the sensor's native resolution; if the
  // stored image was written at another one, the image wins.
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);

  // Intrinsics claim 640x480, the actual stored image is 320x240.
  auto intr = make_intrinsics(500.0, 480.0, 320.0, 240.0, 640, 480);
  seed_frame(db, 1, identity16(), intr, make_image(320, 240));

  auto views = gsplat::load_training_views(db, {});

  REQUIRE(views.size() == 1);
  const auto &v = views[0];
  REQUIRE(v.width() == 320);
  REQUIRE(v.height() == 240);
  REQUIRE_THAT(v.K(0, 0), WithinAbs(250.0, 1e-9));
  REQUIRE_THAT(v.K(1, 1), WithinAbs(240.0, 1e-9));
  REQUIRE_THAT(v.K(0, 2), WithinAbs(160.0, 1e-9));
  REQUIRE_THAT(v.K(1, 2), WithinAbs(120.0, 1e-9));
}

TEST_CASE("LoadTrainingViews_MismatchedIntrinsicsThenDownscale_"
          "AppliesBothCorrections",
          "[gsplat]") {
  // Both corrections compose: K is first rescaled from the claimed 640x480 to
  // the stored 320x240, then again by the downscale to 160x120.
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);

  auto intr = make_intrinsics(500.0, 480.0, 320.0, 240.0, 640, 480);
  seed_frame(db, 1, identity16(), intr, make_image(320, 240));

  gsplat::TrainingViewOptions opt;
  opt.max_image_size = 160;
  auto views = gsplat::load_training_views(db, opt);

  REQUIRE(views.size() == 1);
  const auto &v = views[0];
  REQUIRE(v.width() == 160);
  REQUIRE(v.height() == 120);
  REQUIRE_THAT(v.K(0, 0) / v.width(), WithinRel(500.0 / 640.0, 1e-12));
  REQUIRE_THAT(v.K(1, 1) / v.height(), WithinRel(480.0 / 480.0, 1e-12));
  REQUIRE_THAT(v.K(0, 2) / v.width(), WithinRel(320.0 / 640.0, 1e-12));
  REQUIRE_THAT(v.K(1, 2) / v.height(), WithinRel(240.0 / 480.0, 1e-12));
}

// ===========================================================================
// Pre-existing skip paths
// ===========================================================================

TEST_CASE("LoadTrainingViews_FrameWithEmptyColourImage_SkipsFrame",
          "[gsplat]") {
  test_support::TempPath tmp("test_training_views");

  {
    ProjectDB db(tmp.path);
    seed_frames(db, {1, 2, 3});
  }
  clear_sensor_frame_image(tmp.path, 2);

  ProjectDB db(tmp.path);
  REQUIRE(view_ids(gsplat::load_training_views(db, {})) ==
          std::vector<int>{1, 3});
}

TEST_CASE("LoadTrainingViews_FrameWithInvalidIntrinsics_SkipsFrame",
          "[gsplat]") {
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);

  seed_frame(db, 1, translation_pose(0.5, 0, 0), tiny_intrinsics());
  seed_frame(db, 3, translation_pose(1.5, 0, 0), tiny_intrinsics());

  // Frame 2 has a perfectly good image and pose; only its intrinsics are
  // unusable, so the image has to be supplied independently of them.
  const cv::Mat img = make_image(8, 6);
  SECTION("fx is zero") {
    seed_frame(db, 2, translation_pose(1.0, 0, 0),
               make_intrinsics(0.0, 6.0, 4.0, 3.0, 8, 6), img);
  }
  SECTION("fy is negative") {
    seed_frame(db, 2, translation_pose(1.0, 0, 0),
               make_intrinsics(6.0, -6.0, 4.0, 3.0, 8, 6), img);
  }
  SECTION("width is zero") {
    seed_frame(db, 2, translation_pose(1.0, 0, 0),
               make_intrinsics(6.0, 6.0, 4.0, 3.0, 0, 6), img);
  }
  SECTION("height is negative") {
    seed_frame(db, 2, translation_pose(1.0, 0, 0),
               make_intrinsics(6.0, 6.0, 4.0, 3.0, 8, -6), img);
  }

  REQUIRE(view_ids(gsplat::load_training_views(db, {})) ==
          std::vector<int>{1, 3});
}

// ===========================================================================
// Pose composition
// ===========================================================================

TEST_CASE("LoadTrainingViews_KnownPoseAndLocalTransform_"
          "ComposesTcwAsInverseOfTwbTbc",
          "[gsplat]") {
  // ProjectDB stores pose = T_wb (sensor base in world) and the intrinsics
  // carry local_transform = T_bc (camera in base), so T_wc = T_wb * T_bc and
  // T_cw = (T_wc)^-1. Both rotations are deliberately non-trivial: with
  // identity rotations the test would pass for any implementation that merely
  // negated a translation.
  test_support::TempPath tmp("test_training_views");
  ProjectDB db(tmp.path);

  Eigen::Matrix4d T_wb = Eigen::Matrix4d::Identity();
  T_wb.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(0.63, Eigen::Vector3d(1, 2, 3).normalized())
          .toRotationMatrix();
  T_wb.block<3, 1>(0, 3) = Eigen::Vector3d(1.5, -0.75, 2.25);

  Eigen::Matrix4d T_bc = Eigen::Matrix4d::Identity();
  T_bc.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(-0.91, Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
  T_bc.block<3, 1>(0, 3) = Eigen::Vector3d(0.02, 0.11, -0.03);

  const Eigen::Matrix4d T_wc = T_wb * T_bc;

  seed_frame(db, 5, from_eigen(T_wb),
             make_intrinsics(6.0, 6.0, 4.0, 3.0, 8, 6, from_eigen(T_bc)));

  auto views = gsplat::load_training_views(db, {});
  REQUIRE(views.size() == 1);
  const Eigen::Matrix4d T_cw = views[0].T_cw;

  // 1. The camera centre recovered from T_cw is where T_wc puts it.
  const Eigen::Matrix3d R_cw = T_cw.block<3, 3>(0, 0);
  const Eigen::Vector3d centre = -R_cw.transpose() * T_cw.block<3, 1>(0, 3);
  const Eigen::Vector3d expected_centre = T_wc.block<3, 1>(0, 3);
  REQUIRE_THAT(centre.x(), WithinAbs(expected_centre.x(), 1e-9));
  REQUIRE_THAT(centre.y(), WithinAbs(expected_centre.y(), 1e-9));
  REQUIRE_THAT(centre.z(), WithinAbs(expected_centre.z(), 1e-9));

  // 2. ...and the rotation is right too: T_cw * T_wc == I.
  const Eigen::Matrix4d round_trip = T_cw * T_wc;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      REQUIRE_THAT(round_trip(r, c), WithinAbs(r == c ? 1.0 : 0.0, 1e-9));

  // 3. The stored pose really was the non-trivial one we asked for, i.e. the
  //    fixture is not silently round-tripping an identity.
  const Eigen::Matrix4d stored = to_eigen(db.sensor_frame_pose(5));
  REQUIRE((stored - Eigen::Matrix4d::Identity()).norm() > 0.1);
  REQUIRE_FALSE(views[0].from_panorama);
}
