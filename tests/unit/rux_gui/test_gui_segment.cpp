// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the SAM3 segment endpoints (#409, #448, #467).
//
// Coverage goals (no GPU required — all inference is mocked):
//
//   * parse_segment_frame_request / parse_segment_panorama_request —
//     body validation, field defaults, use_cuda handling.
//   * execute_segment_frame — 503 when segmenter is nullptr, mock segmenter
//     records use_cuda correctly, save=true writes to DB.
//   * execute_segment_panorama — 503 when segmenter is nullptr, save=true.
//
// Real SAM3 inference is [gpu]-tagged and lives elsewhere.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <gui/api.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>

#include <opencv2/imgcodecs.hpp>

#include <string>
#include <vector>

using json = nlohmann::json;
using namespace rux::gui;
using reusex::test_support::TempPath;

namespace {

// ---------------------------------------------------------------------------
// Mock segmenters — no ML backend needed.
// ---------------------------------------------------------------------------

/// Records the last use_cuda passed, returns an all-background label map.
class MockFrameSegmenter : public IFrameSegmenter {
    public:
  bool last_use_cuda = true;
  std::string last_model_path;
  bool was_called = false;

  SegmentFrameResult segment(const cv::Mat &image_bgr,
                             const std::vector<reusex::vision::Sam3Prompt> &,
                             float, const std::string &model_path,
                             bool use_cuda) override {
    last_use_cuda = use_cuda;
    last_model_path = model_path;
    was_called = true;
    // Return a non-empty label map (all label 0 = "class a").
    cv::Mat lm(image_bgr.rows, image_bgr.cols, CV_32S, cv::Scalar(0));
    return {std::move(lm), {"class_a"}};
  }
};

/// Returns an all-background label map for panorama requests.
class MockPanoramaSegmenter : public IPanoramaSegmenter {
    public:
  bool last_use_cuda = true;
  bool was_called = false;

  SegmentPanoramaResult segment(const cv::Mat &equirect_bgr,
                                const std::vector<reusex::vision::Sam3Prompt> &,
                                float, int, double, const std::string &,
                                bool use_cuda) override {
    last_use_cuda = use_cuda;
    was_called = true;
    cv::Mat lm(equirect_bgr.rows, equirect_bgr.cols, CV_32S, cv::Scalar(0));
    return {std::move(lm), {"class_a"}};
  }
};

/// Save a minimal sensor frame (colour only) so execute_segment_frame can
/// load it.
void save_color_frame(reusex::ProjectDB &db, int id) {
  const cv::Mat color(8, 8, CV_8UC3, cv::Scalar(10, 20, 30));
  db.save_sensor_frame(id, color);
}

/// Save a small panorama so execute_segment_panorama can load it.
void save_pano(reusex::ProjectDB &db) {
  cv::Mat equirect(16, 32, CV_8UC3, cv::Scalar(30, 60, 90));
  std::vector<uint8_t> jpeg;
  REQUIRE(cv::imencode(".jpg", equirect, jpeg));
  db.save_panoramic_image("R0010001.JPG", jpeg, /*timestamp=*/1.0,
                          /*node_id=*/-1);
}

} // namespace

// ===========================================================================
// parse_segment_frame_request
// ===========================================================================

TEST_CASE("ParseSegmentFrameRequest_ValidBody_ParsesAllFields",
          "[gui][segment][parse]") {
  const auto req = parse_segment_frame_request(
      R"({"model_path":"/models/sam3","confidence":0.7,"save":false,
          "use_cuda":false,"prompts":[{"text":"wall"}]})",
      /*server_cuda_default=*/true);

  CHECK(req.model_path == "/models/sam3");
  CHECK(req.confidence == Catch::Approx(0.7f));
  CHECK(req.save == false);
  CHECK(req.use_cuda == false);
  REQUIRE(req.prompts.size() == 1);
  CHECK(req.prompts[0].text == "wall");
}

TEST_CASE("ParseSegmentFrameRequest_UseCudaAbsent_UsesServerDefault",
          "[gui][segment][parse]") {
  const auto req_true = parse_segment_frame_request(
      R"({"model_path":"/m"})", /*server_cuda_default=*/true);
  CHECK(req_true.use_cuda == true);

  const auto req_false = parse_segment_frame_request(
      R"({"model_path":"/m"})", /*server_cuda_default=*/false);
  CHECK(req_false.use_cuda == false);
}

TEST_CASE("ParseSegmentFrameRequest_UseCudaInBody_OverridesServerDefault",
          "[gui][segment][parse]") {
  // Body says false, server says true → body wins.
  const auto req = parse_segment_frame_request(
      R"({"model_path":"/m","use_cuda":false})", /*server_cuda_default=*/true);
  CHECK(req.use_cuda == false);
}

TEST_CASE("ParseSegmentFrameRequest_MissingModelPath_Is400",
          "[gui][segment][parse]") {
  try {
    parse_segment_frame_request(R"({})", true);
    FAIL("expected HttpError");
  } catch (const HttpError &e) {
    CHECK(e.status() == 400);
    CHECK(std::string(e.what()).find("model_path") != std::string::npos);
  }
}

TEST_CASE("ParseSegmentFrameRequest_MalformedJson_Is400",
          "[gui][segment][parse]") {
  REQUIRE_THROWS_AS(parse_segment_frame_request("not json", true), HttpError);
  try {
    parse_segment_frame_request("not json", true);
    FAIL("expected HttpError");
  } catch (const HttpError &e) {
    CHECK(e.status() == 400);
  }
}

TEST_CASE("ParseSegmentFrameRequest_BadBoxLabel_Is400",
          "[gui][segment][parse]") {
  // Box label must be "pos" or "neg".
  try {
    parse_segment_frame_request(
        R"({"model_path":"/m","prompts":[{"text":"wall","boxes":[["bad",[0,0,10,10]]]}]})",
        true);
    FAIL("expected HttpError");
  } catch (const HttpError &e) {
    CHECK(e.status() == 400);
  }
}

TEST_CASE("ParseSegmentFrameRequest_Defaults_AreCorrect",
          "[gui][segment][parse]") {
  const auto req = parse_segment_frame_request(R"({"model_path":"/m"})", false);
  CHECK(req.confidence == Catch::Approx(0.5f));
  CHECK(req.save == true);
  CHECK(req.prompts.empty());
}

// ===========================================================================
// parse_segment_panorama_request
// ===========================================================================

TEST_CASE("ParseSegmentPanoramaRequest_ValidBody_ParsesAllFields",
          "[gui][segment][parse][panorama]") {
  const auto req = parse_segment_panorama_request(
      R"({"model_path":"/m","confidence":0.6,"save":false,"use_cuda":false,
          "n_yaw":12,"fov_deg":80.0,"prompts":[{"text":"ceiling"}]})",
      true);

  CHECK(req.model_path == "/m");
  CHECK(req.confidence == Catch::Approx(0.6f));
  CHECK(req.save == false);
  CHECK(req.use_cuda == false);
  CHECK(req.n_yaw == 12);
  CHECK(req.fov_deg == Catch::Approx(80.0));
  REQUIRE(req.prompts.size() == 1);
  CHECK(req.prompts[0].text == "ceiling");
}

TEST_CASE("ParseSegmentPanoramaRequest_UseCudaAbsent_UsesServerDefault",
          "[gui][segment][parse][panorama]") {
  const auto req = parse_segment_panorama_request(
      R"({"model_path":"/m"})", /*server_cuda_default=*/false);
  CHECK(req.use_cuda == false);
}

TEST_CASE("ParseSegmentPanoramaRequest_MissingModelPath_Is400",
          "[gui][segment][parse][panorama]") {
  try {
    parse_segment_panorama_request(R"({"n_yaw":8})", true);
    FAIL("expected HttpError");
  } catch (const HttpError &e) {
    CHECK(e.status() == 400);
  }
}

TEST_CASE("ParseSegmentPanoramaRequest_Defaults_AreCorrect",
          "[gui][segment][parse][panorama]") {
  const auto req =
      parse_segment_panorama_request(R"({"model_path":"/m"})", true);
  CHECK(req.n_yaw == 8);
  CHECK(req.fov_deg == Catch::Approx(90.0));
  CHECK(req.save == true);
  CHECK(req.prompts.empty());
}

// ===========================================================================
// execute_segment_frame
// ===========================================================================

TEST_CASE("ExecuteSegmentFrame_NullSegmenter_Throws503",
          "[gui][segment][execute]") {
  TempPath project("test_segment_exec_503");
  reusex::ProjectDB db(project.path);

  SegmentFrameRequest req;
  req.model_path = "/models/sam3";

  try {
    execute_segment_frame(db, 1, req, nullptr);
    FAIL("expected HttpError(503)");
  } catch (const HttpError &e) {
    CHECK(e.status() == 503);
  }
}

TEST_CASE("ExecuteSegmentFrame_UnknownFrameId_Throws404",
          "[gui][segment][execute]") {
  TempPath project("test_segment_exec_404");
  reusex::ProjectDB db(project.path);

  MockFrameSegmenter mock;
  SegmentFrameRequest req;
  req.model_path = "/models/sam3";

  try {
    execute_segment_frame(db, 999, req, &mock);
    FAIL("expected HttpError(404)");
  } catch (const HttpError &e) {
    CHECK(e.status() == 404);
  }
}

TEST_CASE("ExecuteSegmentFrame_UseCudaFalse_PassedToSegmenter",
          "[gui][segment][execute]") {
  TempPath project("test_segment_exec_cuda");
  reusex::ProjectDB db(project.path);
  save_color_frame(db, 1);

  MockFrameSegmenter mock;

  SegmentFrameRequest req;
  req.model_path = "/models/sam3";
  req.use_cuda = false;
  req.save = false;

  execute_segment_frame(db, 1, req, &mock);

  REQUIRE(mock.was_called);
  CHECK(mock.last_use_cuda == false);
  CHECK(mock.last_model_path == "/models/sam3");
}

TEST_CASE("ExecuteSegmentFrame_UseCudaTrue_PassedToSegmenter",
          "[gui][segment][execute]") {
  TempPath project("test_segment_exec_cuda_true");
  reusex::ProjectDB db(project.path);
  save_color_frame(db, 1);

  MockFrameSegmenter mock;

  SegmentFrameRequest req;
  req.model_path = "/models/sam3";
  req.use_cuda = true;
  req.save = false;

  execute_segment_frame(db, 1, req, &mock);

  REQUIRE(mock.was_called);
  CHECK(mock.last_use_cuda == true);
}

TEST_CASE("ExecuteSegmentFrame_SaveTrue_WritesSegmentationImage",
          "[gui][segment][execute]") {
  TempPath project("test_segment_exec_save");
  reusex::ProjectDB db(project.path);
  save_color_frame(db, 1);

  REQUIRE_FALSE(db.has_segmentation_image(1));

  MockFrameSegmenter mock;
  SegmentFrameRequest req;
  req.model_path = "/models/sam3";
  req.save = true;

  const auto body = execute_segment_frame(db, 1, req, &mock);

  CHECK(body.at("frame_id") == 1);
  CHECK(body.at("saved") == true);
  // The mock returns an all-zero (label 0) label map, so labeled_pixels > 0.
  CHECK(body.at("labeled_pixels").get<int>() > 0);
  // Verify the segmentation image was actually written to the DB.
  CHECK(db.has_segmentation_image(1));
}

TEST_CASE("ExecuteSegmentFrame_SaveFalse_DoesNotWriteToDb",
          "[gui][segment][execute]") {
  TempPath project("test_segment_exec_nosave");
  reusex::ProjectDB db(project.path);
  save_color_frame(db, 1);

  MockFrameSegmenter mock;
  SegmentFrameRequest req;
  req.model_path = "/models/sam3";
  req.save = false;

  const auto body = execute_segment_frame(db, 1, req, &mock);

  CHECK(body.at("saved") == false);
  CHECK_FALSE(db.has_segmentation_image(1));
}

// ===========================================================================
// execute_segment_panorama
// ===========================================================================

TEST_CASE("ExecuteSegmentPanorama_NullSegmenter_Throws503",
          "[gui][segment][execute][panorama]") {
  TempPath project("test_segment_pano_503");
  reusex::ProjectDB db(project.path);

  SegmentPanoramaRequest req;
  req.model_path = "/models/sam3";

  try {
    execute_segment_panorama(db, 1, req, nullptr);
    FAIL("expected HttpError(503)");
  } catch (const HttpError &e) {
    CHECK(e.status() == 503);
  }
}

TEST_CASE("ExecuteSegmentPanorama_UseCudaFromRequest_PassedToSegmenter",
          "[gui][segment][execute][panorama]") {
  TempPath project("test_segment_pano_cuda");
  reusex::ProjectDB db(project.path);
  save_pano(db);
  const int pano_id = db.list_panoramic_images().at(0).id;

  MockPanoramaSegmenter mock;
  SegmentPanoramaRequest req;
  req.model_path = "/models/sam3";
  req.use_cuda = false;
  req.save = false;

  execute_segment_panorama(db, pano_id, req, &mock);

  REQUIRE(mock.was_called);
  CHECK(mock.last_use_cuda == false);
}

TEST_CASE("ExecuteSegmentPanorama_SaveTrue_WritesToDb",
          "[gui][segment][execute][panorama]") {
  TempPath project("test_segment_pano_save");
  reusex::ProjectDB db(project.path);
  save_pano(db);
  const int pano_id = db.list_panoramic_images().at(0).id;

  MockPanoramaSegmenter mock;
  SegmentPanoramaRequest req;
  req.model_path = "/models/sam3";
  req.save = true;

  const auto body = execute_segment_panorama(db, pano_id, req, &mock);

  CHECK(body.at("pano_id") == pano_id);
  CHECK(body.at("saved") == true);
  CHECK(body.at("labeled_pixels").get<int>() > 0);
}
