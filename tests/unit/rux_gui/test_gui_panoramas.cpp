// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Contract tests for the panorama routes of `rux gui` (#265, Phase 5).
//
// Two things are pinned here, both of which the panorama mode of the viewport
// depends on and neither of which the Phase 1 contract had:
//
//   * `frame_pose` — where an *unaligned* panorama can be drawn, without the
//     client resolving `node_id` itself once per panorama. Every panorama in a
//     project that has only been imported is in that state, so this is the
//     normal case rather than the edge case.
//   * `max_size` on the image route — a stored equirect is routinely 8192x4096
//     and several megabytes, which a thumbnail strip cannot fetch.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <gui/api.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>

#include <opencv2/imgcodecs.hpp>

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

using json = nlohmann::json;
using namespace rux::gui;

namespace {

using reusex::test_support::TempPath;

Params params_of(const std::map<std::string, std::string> &values) {
  Params params;
  for (const auto &[key, value] : values)
    params.set(key, value);
  return params;
}

/// A recognisable, non-identity row-major 4x4 pose.
std::array<double, 16> pose_at(double x, double y, double z) {
  return {1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1};
}

/// Store a sensor frame carrying a real pose (the four-argument overload
/// leaves `transform` NULL, which is a different case and tested separately).
void save_posed_frame(reusex::ProjectDB &db, int id,
                      const std::array<double, 16> &pose) {
  const cv::Mat color(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
  const cv::Mat depth(4, 4, CV_16U, cv::Scalar(1200));
  const cv::Mat confidence(4, 4, CV_8U, cv::Scalar(2));
  reusex::core::SensorIntrinsics intrinsics;
  intrinsics.fx = 2.0;
  intrinsics.fy = 2.0;
  intrinsics.cx = 2.0;
  intrinsics.cy = 2.0;
  intrinsics.width = 4;
  intrinsics.height = 4;
  db.save_sensor_frame(id, color, depth, confidence, pose, intrinsics, 1000.0);
}

/// Store a panorama of @p width x @p height as a real JPEG.
void save_panorama(reusex::ProjectDB &db, const std::string &filename,
                   int width, int height, int node_id) {
  cv::Mat equirect(height, width, CV_8UC3, cv::Scalar(30, 60, 90));
  // A gradient rather than a flat fill: JPEG-encoding a constant image is a
  // degenerate case, and a resize of it cannot be told from a crop.
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      equirect.at<cv::Vec3b>(y, x)[1] =
          static_cast<uint8_t>((x * 255) / std::max(1, width - 1));

  std::vector<uint8_t> jpeg;
  REQUIRE(cv::imencode(".jpg", equirect, jpeg));
  db.save_panoramic_image(filename, jpeg, /*timestamp=*/1000.0, node_id);
}

/// The single entry of a one-panorama project.
json only_panorama(const reusex::ProjectDB &db) {
  const auto body = panoramas_json(db, Params{});
  REQUIRE(body.at("panoramas").size() == 1);
  return body.at("panoramas").at(0);
}

} // namespace

// ===========================================================================
// frame_pose: placing a panorama that `rux align 360` has not touched
// ===========================================================================

TEST_CASE("PanoramasJson_UnalignedPanoramaMatchedToPosedFrame_"
          "ReportsTheFramePoseSeparatelyFromTheAlignedOne",
          "[gui][panorama]") {
  TempPath project("test_gui_panoramas");
  reusex::ProjectDB db(project.path);

  save_posed_frame(db, 7, pose_at(1.5, -2.5, 0.75));
  save_panorama(db, "R0010001.JPG", 64, 32, /*node_id=*/7);

  const auto entry = only_panorama(db);

  // The aligned pose is absent and says so. Its value is the identity
  // `PanoramicImage` defaults to, which is exactly why `has_pose` exists.
  CHECK(entry.at("has_pose") == false);
  CHECK(entry.at("pose_source") == "timestamp");

  // The borrowed one is present, and is the *frame's* pose rather than a
  // sanitised copy of it.
  REQUIRE(entry.at("has_frame_pose") == true);
  REQUIRE(entry.contains("frame_pose"));
  const auto frame_pose = entry.at("frame_pose").get<std::vector<double>>();
  REQUIRE(frame_pose.size() == 16);
  CHECK(frame_pose[3] == Catch::Approx(1.5));
  CHECK(frame_pose[7] == Catch::Approx(-2.5));
  CHECK(frame_pose[11] == Catch::Approx(0.75));

  // And the two never merge: reading `pose` must not silently hand back the
  // frame's translation, or a client can no longer tell a resected panorama
  // from a borrowed one.
  const auto pose = entry.at("pose").get<std::vector<double>>();
  CHECK(pose[3] == Catch::Approx(0.0));
  CHECK(pose[7] == Catch::Approx(0.0));
  CHECK(pose[11] == Catch::Approx(0.0));
}

TEST_CASE("PanoramasJson_PanoramaMatchedToNoFrame_HasNoFramePose",
          "[gui][panorama]") {
  TempPath project("test_gui_panoramas");
  reusex::ProjectDB db(project.path);

  save_panorama(db, "R0010001.JPG", 64, 32, /*node_id=*/-1);

  const auto entry = only_panorama(db);
  CHECK(entry.at("node_id") == -1);
  CHECK(entry.at("has_frame_pose") == false);
  // Absent, not identity: an identity `frame_pose` would put the panorama at
  // the world origin, which is a place it demonstrably is not.
  CHECK_FALSE(entry.contains("frame_pose"));
}

TEST_CASE("PanoramasJson_MatchedFrameCarriesNoPose_HasNoFramePose",
          "[gui][panorama]") {
  TempPath project("test_gui_panoramas");
  reusex::ProjectDB db(project.path);

  // A colour-only import: the row exists but `transform` is NULL, so
  // `sensor_frame_pose()` would answer identity (#336) and a client trusting
  // it would stack every such panorama on the origin.
  const cv::Mat color(4, 4, CV_8UC3, cv::Scalar(1, 2, 3));
  db.save_sensor_frame(9, color);
  save_panorama(db, "R0010001.JPG", 64, 32, /*node_id=*/9);

  const auto entry = only_panorama(db);
  CHECK(entry.at("node_id") == 9);
  CHECK(entry.at("has_frame_pose") == false);
  CHECK_FALSE(entry.contains("frame_pose"));
}

TEST_CASE("PanoramaJson_AlignedPanorama_ReportsTheResectedPoseAndItsQuality",
          "[gui][panorama]") {
  TempPath project("test_gui_panoramas");
  reusex::ProjectDB db(project.path);

  save_posed_frame(db, 7, pose_at(1.5, -2.5, 0.75));
  save_panorama(db, "R0010001.JPG", 64, 32, /*node_id=*/7);

  const int id = db.list_panoramic_images().at(0).id;
  db.save_panorama_pose(id, pose_at(3.0, 4.0, 1.0), /*inliers=*/182,
                        /*rms=*/0.42);

  const auto entry = panorama_json(db, id);
  CHECK(entry.at("has_pose") == true);
  CHECK(entry.at("pose_source") == "aligned");
  CHECK(entry.at("align_inliers") == 182);
  CHECK(entry.at("align_rms") == Catch::Approx(0.42));
  CHECK(entry.at("pose").get<std::vector<double>>()[3] == Catch::Approx(3.0));

  // Still reported alongside, and still distinct: the difference between the
  // two is what `rux align 360` actually did, and a UI comparing them is the
  // cheapest sanity check on the alignment there is.
  REQUIRE(entry.at("has_frame_pose") == true);
  CHECK(entry.at("frame_pose").get<std::vector<double>>()[3] ==
        Catch::Approx(1.5));
}

// ===========================================================================
// max_size on the image route
// ===========================================================================

TEST_CASE("PanoramaImageBlob_MaxSize_DownscalesToTheRequestedLongestEdge",
          "[gui][panorama]") {
  TempPath project("test_gui_panoramas");
  reusex::ProjectDB db(project.path);
  save_panorama(db, "R0010001.JPG", 512, 256, /*node_id=*/-1);
  const int id = db.list_panoramic_images().at(0).id;

  const Blob blob =
      panorama_image_blob(db, id, params_of({{"max_size", "64"}}));
  CHECK(blob.content_type == "image/jpeg");

  const cv::Mat decoded = cv::imdecode(blob.data, cv::IMREAD_COLOR);
  REQUIRE_FALSE(decoded.empty());
  CHECK(decoded.cols == 64);
  // Aspect ratio preserved — a squashed equirect maps onto the sphere with the
  // horizon in the wrong place, which looks like a levelling error.
  CHECK(decoded.rows == 32);
}

TEST_CASE("PanoramaImageBlob_MaxSizeAboveTheStoredSize_DoesNotUpscale",
          "[gui][panorama]") {
  TempPath project("test_gui_panoramas");
  reusex::ProjectDB db(project.path);
  save_panorama(db, "R0010001.JPG", 64, 32, /*node_id=*/-1);
  const int id = db.list_panoramic_images().at(0).id;

  const Blob blob =
      panorama_image_blob(db, id, params_of({{"max_size", "4096"}}));
  const cv::Mat decoded = cv::imdecode(blob.data, cv::IMREAD_COLOR);
  REQUIRE_FALSE(decoded.empty());
  CHECK(decoded.cols == 64);
  CHECK(decoded.rows == 32);
}

TEST_CASE("PanoramaImageBlob_NoMaxSize_ReturnsTheStoredResolution",
          "[gui][panorama]") {
  TempPath project("test_gui_panoramas");
  reusex::ProjectDB db(project.path);
  save_panorama(db, "R0010001.JPG", 128, 64, /*node_id=*/-1);
  const int id = db.list_panoramic_images().at(0).id;

  const cv::Mat decoded = cv::imdecode(
      panorama_image_blob(db, id, Params{}).data, cv::IMREAD_COLOR);
  REQUIRE_FALSE(decoded.empty());
  CHECK(decoded.cols == 128);
  CHECK(decoded.rows == 64);
}

TEST_CASE("PanoramaImageBlob_MaxSizeOutOfRange_IsRejectedAsABadRequest",
          "[gui][panorama]") {
  TempPath project("test_gui_panoramas");
  reusex::ProjectDB db(project.path);
  save_panorama(db, "R0010001.JPG", 64, 32, /*node_id=*/-1);
  const int id = db.list_panoramic_images().at(0).id;

  try {
    panorama_image_blob(db, id, params_of({{"max_size", "100000"}}));
    FAIL("expected HttpError");
  } catch (const HttpError &e) {
    CHECK(e.status() == 400);
    CHECK(std::string(e.what()).find("max_size") != std::string::npos);
  }
}

TEST_CASE("PanoramaImageBlob_UnknownId_Throws404", "[gui][panorama]") {
  TempPath project("test_gui_panoramas");
  reusex::ProjectDB db(project.path);

  try {
    panorama_image_blob(db, 404, Params{});
    FAIL("expected HttpError");
  } catch (const HttpError &e) {
    CHECK(e.status() == 404);
  }
}
