// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Contract tests for the `rux gui` editor endpoints and frame-browser reads
// (#265, Phase 4).
//
// Covers the surface commit 03b51ad added: Params::boolean, the `segmented`
// frame filter, the `max_size` / `normalize` renderings of a frame image, the
// cloud label legend, the derived component fields, and the two sparse PATCH
// handlers. Everything runs against a real (and deliberately degenerate)
// ProjectDB; nothing here needs a socket.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <gui/api.hpp>
#include <gui/edits.hpp>

#include "../../support/temp_path.hpp"

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <core/component_record.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using json = nlohmann::json;
using namespace rux::gui;
using Catch::Approx;

namespace {

/// Temp project path that auto-cleans, including the sqlite sidecars (#262).
struct TempDB : reusex::test_support::TempPath {
  TempDB() : TempPath("test_gui_edits") {}
};

Params params_of(const std::map<std::string, std::string> &values) {
  Params params;
  for (const auto &[key, value] : values)
    params.set(key, value);
  return params;
}

reusex::core::SensorIntrinsics pinhole(int width, int height) {
  reusex::core::SensorIntrinsics intrinsics;
  intrinsics.fx = intrinsics.fy = 100.0;
  intrinsics.cx = width / 2.0;
  intrinsics.cy = height / 2.0;
  intrinsics.width = width;
  intrinsics.height = height;
  return intrinsics;
}

/// Store one sensor frame; `depth` / `confidence` may be empty matrices.
void save_frame(reusex::ProjectDB &db, int id, const cv::Mat &color,
                const cv::Mat &depth = cv::Mat(),
                const cv::Mat &confidence = cv::Mat()) {
  const std::array<double, 16> identity = {1, 0, 0, 0, 0, 1, 0, 0,
                                           0, 0, 1, 0, 0, 0, 0, 1};
  db.save_sensor_frame(id, color, depth, confidence, identity,
                       pinhole(color.cols, color.rows));
}

cv::Mat decode(const Blob &blob) {
  REQUIRE(blob.content_type == "image/png");
  const cv::Mat encoded(1, static_cast<int>(blob.data.size()), CV_8UC1,
                        const_cast<uint8_t *>(blob.data.data()));
  cv::Mat image = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
  REQUIRE_FALSE(image.empty());
  return image;
}

/// Pack xyz triples the way `building_components.vertex_data` stores them:
/// little-endian float64, three per vertex (core/component_record.hpp).
std::vector<uint8_t> pack(const std::vector<std::array<double, 3>> &vertices) {
  std::vector<uint8_t> bytes(vertices.size() * 3 * sizeof(double));
  for (size_t i = 0; i < vertices.size(); ++i)
    std::memcpy(bytes.data() + i * 3 * sizeof(double), vertices[i].data(),
                3 * sizeof(double));
  return bytes;
}

void save_raw_component(reusex::ProjectDB &db, const std::string &name,
                        std::vector<uint8_t> vertex_data,
                        const std::string &metadata = {}) {
  reusex::core::ComponentRecord record;
  record.name = name;
  record.guid = "guid-" + name;
  record.type = "window";
  record.vertex_data = std::move(vertex_data);
  record.plane = {0.0, 0.0, 1.0, 0.0};
  record.confidence = 0.5;
  record.metadata = metadata;
  db.save_component_record(record);
}

void save_component(reusex::ProjectDB &db, const std::string &name,
                    const std::vector<std::array<double, 3>> &vertices,
                    const std::string &metadata = {}) {
  save_raw_component(db, name, pack(vertices), metadata);
}

/// The status of the HttpError @p callable throws, or 0 when it throws none.
template <typename Callable> int status_of(Callable &&callable) {
  try {
    callable();
  } catch (const HttpError &e) {
    return e.status();
  }
  return 0;
}

std::string add_passport(reusex::ProjectDB &db, const std::string &guid) {
  reusex::core::MaterialPassport passport;
  passport.metadata.document_guid = guid;
  passport.metadata.creation_date = "2026-01-15T10:30:00Z";
  passport.metadata.version_number = "1.0.0";
  passport.description.designation = "Test Material";
  db.add_material_passport(passport, "test-project");
  return guid;
}

} // namespace

// ===========================================================================
// Params::boolean
// ===========================================================================

TEST_CASE("ParamsBoolean_DocumentedSpellings_ParsesAnyCaseAndRejectsOthers",
          "[gui][params]") {
  Params params;
  for (const char *truthy : {"true", "TRUE", "True", "1", "yes", "YES"}) {
    params.set("flag", truthy);
    INFO("value: " << truthy);
    REQUIRE(params.boolean("flag") == std::optional<bool>(true));
  }
  for (const char *falsy : {"false", "FALSE", "False", "0", "no", "No"}) {
    params.set("flag", falsy);
    INFO("value: " << falsy);
    REQUIRE(params.boolean("flag") == std::optional<bool>(false));
  }

  SECTION("absent and empty are indistinguishable from unset, not from false") {
    Params sparse;
    sparse.set("empty", "");
    CHECK_FALSE(sparse.boolean("missing").has_value());
    CHECK_FALSE(sparse.boolean("empty").has_value());
  }

  SECTION("anything else is a 400 naming the parameter, not a silent false") {
    for (const char *bogus : {"maybe", "2", "tru", "y", "on", "-1"}) {
      Params bad;
      bad.set("segmented", bogus);
      INFO("value: " << bogus);
      try {
        bad.boolean("segmented");
        FAIL("expected HttpError");
      } catch (const HttpError &e) {
        CHECK(e.status() == 400);
        CHECK(std::string(e.what()).find("segmented") != std::string::npos);
      }
    }
  }
}

// ===========================================================================
// GET /frames — the `segmented` filter
// ===========================================================================

TEST_CASE("FramesJson_SegmentedFilter_NarrowsIdsButCountsDescribeWholeScan",
          "[gui][frames]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);

  const cv::Mat color(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
  for (int id : {1, 2, 3})
    save_frame(db, id, color);

  cv::Mat labels(4, 4, CV_32S, cv::Scalar(-1));
  labels.at<int32_t>(0, 0) = 1;
  db.save_segmentation_image(2, labels);

  SECTION("without the filter every frame is listed") {
    const auto body = frames_json(db, Params{});
    CHECK(body.at("ids") == json::array({1, 2, 3}));
    CHECK(body.at("total_count") == 3);
    CHECK(body.at("segmented_count") == 1);
  }

  SECTION("segmented=true lists only the annotated frames") {
    const auto body = frames_json(db, params_of({{"segmented", "true"}}));
    CHECK(body.at("ids") == json::array({2}));
    // The denominator is the whole scan, so a UI can say "1 of 3".
    CHECK(body.at("total_count") == 3);
    CHECK(body.at("segmented_count") == 1);
  }

  SECTION("segmented=false lists exactly the complement") {
    const auto body = frames_json(db, params_of({{"segmented", "no"}}));
    CHECK(body.at("ids") == json::array({1, 3}));
    CHECK(body.at("total_count") == 3);
    CHECK(body.at("segmented_count") == 1);
  }

  SECTION("an unparseable filter is a 400 rather than an unfiltered list") {
    CHECK(status_of([&] {
            return frames_json(db, params_of({{"segmented", "sometimes"}}));
          }) == 400);
  }
}

TEST_CASE("FramesJson_EmptyScan_ReturnsEmptyList", "[gui][frames]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);

  for (const char *value : {"true", "false"}) {
    INFO("segmented=" << value);
    const auto body = frames_json(db, params_of({{"segmented", value}}));
    CHECK(body.at("ids").empty());
    CHECK(body.at("total_count") == 0);
    CHECK(body.at("segmented_count") == 0);
  }
}

// ===========================================================================
// GET /frames/{id}/image
// ===========================================================================

TEST_CASE("FrameImage_MaxSize_ShrinksLongestEdgeAndNeverUpscales",
          "[gui][frames]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  save_frame(db, 1, cv::Mat(20, 40, CV_8UC3, cv::Scalar(10, 20, 30)));

  SECTION("a thumbnail keeps the aspect ratio") {
    const auto response = frame_image(db, 1, params_of({{"max_size", "10"}}));
    const cv::Mat image = decode(response.blob);
    CHECK(image.cols == 10);
    CHECK(image.rows == 5);
  }

  SECTION("a cap above the image never upscales it") {
    for (const char *cap : {"0", "40", "1000"}) {
      INFO("max_size=" << cap);
      const cv::Mat image =
          decode(frame_image(db, 1, params_of({{"max_size", cap}})).blob);
      CHECK(image.cols == 40);
      CHECK(image.rows == 20);
    }
  }

  SECTION("a nonsensical cap is a 400, not a clamped guess") {
    for (const char *bogus : {"-1", "4097", "999999"}) {
      INFO("max_size=" << bogus);
      CHECK(status_of([&] {
              return frame_image(db, 1, params_of({{"max_size", bogus}}));
            }) == 400);
    }
  }
}

TEST_CASE("FrameImage_NormalizeDepth_RendersDisplayableAndReportsRange",
          "[gui][frames]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);

  // Millimetres, as stored. 0 is "no return" and must not anchor the low end.
  cv::Mat depth(2, 2, CV_16U, cv::Scalar(0));
  depth.at<uint16_t>(0, 1) = 1000;
  depth.at<uint16_t>(1, 0) = 2000;
  depth.at<uint16_t>(1, 1) = 3000;
  save_frame(db, 1, cv::Mat(2, 2, CV_8UC3, cv::Scalar(0, 0, 0)), depth);

  SECTION("the raw image carries measurements and no range") {
    const auto response = frame_image(db, 1, params_of({{"kind", "depth"}}));
    CHECK_FALSE(response.range.valid);
    const cv::Mat image = decode(response.blob);
    REQUIRE(image.depth() == CV_16U);
    CHECK(image.at<uint16_t>(1, 1) == 3000);
  }

  SECTION("the normalized image is 8-bit grey stretched over the valid "
          "pixels only") {
    const auto response = frame_image(
        db, 1, params_of({{"kind", "depth"}, {"normalize", "true"}}));
    REQUIRE(response.range.valid);
    CHECK(response.range.min == Approx(1000.0));
    CHECK(response.range.max == Approx(3000.0));

    const cv::Mat image = decode(response.blob);
    REQUIRE(image.depth() == CV_8U);
    REQUIRE(image.channels() == 1);
    CHECK(image.at<uint8_t>(0, 0) == 0);   // no return, blanked
    CHECK(image.at<uint8_t>(1, 1) == 255); // the far end of the range
    CHECK(image.at<uint8_t>(1, 0) > image.at<uint8_t>(0, 1));
  }
}

TEST_CASE("FrameImage_NormalizeDepthWithNoValidPixels_ReturnsFlatBlack",
          "[gui][frames]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);

  const cv::Mat color(2, 2, CV_8UC3, cv::Scalar(0, 0, 0));
  save_frame(db, 1, color, cv::Mat(2, 2, CV_16U, cv::Scalar(0)));
  // A constant non-zero frame: valid pixels, but still no range to stretch.
  save_frame(db, 2, color, cv::Mat(2, 2, CV_16U, cv::Scalar(1500)));

  SECTION("an all-zero depth image reports no range at all") {
    const auto response = frame_image(
        db, 1, params_of({{"kind", "depth"}, {"normalize", "true"}}));
    CHECK_FALSE(response.range.valid);
    const cv::Mat image = decode(response.blob);
    REQUIRE(image.depth() == CV_8U);
    CHECK(cv::countNonZero(image) == 0);
  }

  SECTION("a constant depth image reports a degenerate range and stays flat") {
    const auto response = frame_image(
        db, 2, params_of({{"kind", "depth"}, {"normalize", "true"}}));
    REQUIRE(response.range.valid);
    CHECK(response.range.min == Approx(1500.0));
    CHECK(response.range.max == Approx(1500.0));
    CHECK(cv::countNonZero(decode(response.blob)) == 0);
  }
}

TEST_CASE("FrameImage_NormalizeColor_IsIgnored", "[gui][frames]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  save_frame(db, 1, cv::Mat(8, 8, CV_8UC3, cv::Scalar(10, 20, 30)));

  const auto response =
      frame_image(db, 1, params_of({{"kind", "color"}, {"normalize", "1"}}));
  CHECK_FALSE(response.range.valid);
  const cv::Mat image = decode(response.blob);
  CHECK(image.channels() == 3);
  CHECK(image.cols == 8);
  CHECK(image.rows == 8);
}

TEST_CASE(
    "FrameImage_NormalizeSegmentation_ColourisesPerClassLeavingBackgroundBlack",
    "[gui][frames]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);

  save_frame(db, 9, cv::Mat(4, 4, CV_8UC3, cv::Scalar(0, 0, 0)));
  cv::Mat labels(1, 4, CV_32S, cv::Scalar(-1));
  labels.at<int32_t>(0, 1) = 3;
  labels.at<int32_t>(0, 2) = 3;
  labels.at<int32_t>(0, 3) = 5;
  db.save_segmentation_image(9, labels);

  const auto response = frame_image(
      db, 9, params_of({{"kind", "segmentation"}, {"normalize", "yes"}}));
  const cv::Mat image = decode(response.blob);
  REQUIRE(image.channels() == 3);

  const auto background = image.at<cv::Vec3b>(0, 0);
  CHECK(background == cv::Vec3b(0, 0, 0));
  CHECK_FALSE(image.at<cv::Vec3b>(0, 1) == cv::Vec3b(0, 0, 0));
  // Same class, same colour; different classes, different colours.
  CHECK(image.at<cv::Vec3b>(0, 1) == image.at<cv::Vec3b>(0, 2));
  CHECK_FALSE(image.at<cv::Vec3b>(0, 1) == image.at<cv::Vec3b>(0, 3));
}

TEST_CASE("FrameImage_DownscaledLabelMask_ResamplesIdsWithoutInterpolating",
          "[gui][frames]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);

  // Two 2x4 bands of classes 3 and 5. A blended resample would manufacture 4.
  save_frame(db, 9, cv::Mat(4, 4, CV_8UC3, cv::Scalar(0, 0, 0)));
  cv::Mat labels(4, 4, CV_32S, cv::Scalar(3));
  labels.rowRange(2, 4).setTo(5);
  db.save_segmentation_image(9, labels);

  const cv::Mat image = decode(
      frame_image(db, 9,
                  params_of({{"kind", "segmentation"}, {"max_size", "2"}}))
          .blob);
  REQUIRE(image.cols == 2);
  REQUIRE(image.rows == 2);
  REQUIRE(image.depth() == CV_16U);
  for (int y = 0; y < image.rows; ++y)
    for (int x = 0; x < image.cols; ++x) {
      INFO("pixel " << x << "," << y);
      const uint16_t value = image.at<uint16_t>(y, x);
      CHECK((value == 3 || value == 5));
    }
}

TEST_CASE("FrameImage_UnknownKindOrMissingImage_Throws", "[gui][frames]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  save_frame(db, 1, cv::Mat(2, 2, CV_8UC3, cv::Scalar(1, 2, 3)));

  CHECK(status_of([&] {
          return frame_image(db, 1, params_of({{"kind", "thermal"}}));
        }) == 400);
  CHECK(status_of([&] { return frame_image(db, 42, Params{}); }) == 404);
  // The frame exists but carries no depth blob.
  CHECK(status_of([&] {
          return frame_image(db, 1, params_of({{"kind", "depth"}}));
        }) == 404);
  CHECK(status_of([&] {
          return frame_image(db, 1, params_of({{"kind", "segmentation"}}));
        }) == 404);
}

// ===========================================================================
// GET /clouds/{name}/labels
// ===========================================================================

TEST_CASE("CloudLabelsJson_UnlabeledId_IsOmitted", "[gui][labels]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);

  reusex::CloudL cloud;
  for (uint32_t i = 0; i < 3; ++i) {
    reusex::LabelT label;
    label.label = i;
    cloud.push_back(label);
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  db.save_point_cloud("planes", cloud, "test");
  // A stray row for 0 is stale data, not a class (STANDARDS §3).
  db.save_label_definitions("planes",
                            {{0, "unlabeled"}, {1, "wall"}, {2, "floor"}});

  const auto body = cloud_labels_json(db, "planes");
  REQUIRE(body.at("labels").is_object());
  CHECK(body.at("labels").size() == 2);
  CHECK(body.at("labels").at("1") == "wall");
  CHECK(body.at("labels").at("2") == "floor");
  CHECK_FALSE(body.at("labels").contains("0"));
}

TEST_CASE("CloudLabelsJson_CloudWithoutLegend_ReturnsEmptyMap",
          "[gui][labels]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);

  reusex::CloudL cloud;
  reusex::LabelT label;
  label.label = 1;
  cloud.push_back(label);
  cloud.width = 1;
  cloud.height = 1;
  db.save_point_cloud("planes", cloud, "test");

  const auto body = cloud_labels_json(db, "planes");
  REQUIRE(body.at("labels").is_object());
  CHECK(body.at("labels").empty());

  // A cloud that does not exist at all is a different answer.
  CHECK(status_of([&] { return cloud_labels_json(db, "nope"); }) == 404);
}

// ===========================================================================
// Component derived fields
// ===========================================================================

TEST_CASE("ComponentSummaryJson_DegenerateBoundary_OmitsArea",
          "[gui][components]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);

  save_component(
      db, "rect",
      {{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 1.0, 0.0}, {0.0, 1.0, 0.0}});
  save_component(db, "segment", {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});
  // A blob too short to hold even one xyz triple: zero vertices, and the
  // column is NOT NULL so it cannot simply be absent.
  save_raw_component(db, "empty", std::vector<uint8_t>(16, 0));

  const auto rect = component_json(db, "rect");
  CHECK(rect.at("vertex_count") == 4);
  REQUIRE(rect.contains("area"));
  CHECK(rect.at("area").get<double>() == Approx(2.0));

  SECTION("a boundary that cannot enclose anything reports no area") {
    for (const char *name : {"empty", "segment"}) {
      INFO("component: " << name);
      const auto body = component_json(db, name);
      CHECK_FALSE(body.contains("area"));
      CHECK(body.at("vertices").is_array());
    }
    CHECK(component_json(db, "empty").at("vertex_count") == 0);
    CHECK(component_json(db, "segment").at("vertex_count") == 2);
  }

  SECTION("the list view derives the same fields as the detail view") {
    const auto listed = components_json(db, Params{});
    REQUIRE(listed.at("components").size() == 3);
    for (const auto &entry : listed.at("components")) {
      if (entry.at("name") != "rect")
        continue;
      CHECK(entry.at("area").get<double>() == Approx(2.0));
    }
  }
}

TEST_CASE(
    "ComponentSummaryJson_UnparseableMetadata_StillListsWithoutProvenance",
    "[gui][components]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);

  const std::vector<std::array<double, 3>> square{
      {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0}};

  save_component(db, "linked", square, R"({"source_instance_guid":"inst-7"})");
  save_component(db, "unlinked", square, R"({"style":"casement"})");
  save_component(db, "broken", square, "{not json at all");
  save_component(db, "mistyped", square, R"({"source_instance_guid":42})");
  save_component(db, "bare", square, "");

  CHECK(component_json(db, "linked").at("source_instance_guid") == "inst-7");

  for (const char *name : {"unlinked", "broken", "mistyped", "bare"}) {
    INFO("component: " << name);
    const auto body = component_json(db, name);
    CHECK_FALSE(body.contains("source_instance_guid"));
    // The rest of the record is still served.
    CHECK(body.at("name") == name);
    CHECK(body.at("vertex_count") == 4);
  }

  // And the collection listing survives the malformed row too.
  CHECK(components_json(db, Params{}).at("components").size() == 5);
}

// ===========================================================================
// PATCH /clouds/{name}/labels
// ===========================================================================

namespace {

/// A three-class `planes` cloud, the fixture the label-patch cases share.
void seed_planes(reusex::ProjectDB &db, const std::string &name = "planes") {
  reusex::CloudL cloud;
  for (uint32_t i = 1; i <= 3; ++i) {
    reusex::LabelT label;
    label.label = i;
    cloud.push_back(label);
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  db.save_point_cloud(name, cloud, "test");
  db.save_label_definitions(name, {{1, "wall"}, {2, "floor"}, {3, "ceiling"}});
}

} // namespace

TEST_CASE("PatchCloudLabels_SparsePatch_RenamesOnlyNamedIds", "[gui][edits]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  seed_planes(db);

  const auto body =
      patch_cloud_labels(db, "planes", R"({"labels":{"2":"slab"}})");

  // The response is the full legend, not just the edited entry.
  REQUIRE(body.at("labels").size() == 3);
  CHECK(body.at("labels").at("1") == "wall");
  CHECK(body.at("labels").at("2") == "slab");
  CHECK(body.at("labels").at("3") == "ceiling");

  // And it is what the project now holds, not just what was returned.
  const auto stored = db.label_definitions("planes");
  CHECK(stored.at(1) == "wall");
  CHECK(stored.at(2) == "slab");
  CHECK(stored.at(3) == "ceiling");
}

TEST_CASE("PatchCloudLabels_EmptyPatch_IsNoOpAndReturnsLegend",
          "[gui][edits]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  seed_planes(db);

  const auto before = db.label_definitions("planes");
  const auto body = patch_cloud_labels(db, "planes", R"({"labels":{}})");

  CHECK(body.at("labels").size() == 3);
  CHECK(body.at("labels").at("1") == "wall");
  CHECK(db.label_definitions("planes") == before);
}

TEST_CASE("PatchCloudLabels_InvalidPatch_RejectsWithoutPartialWrite",
          "[gui][edits]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  seed_planes(db);
  const auto before = db.label_definitions("planes");

  const std::vector<std::string> rejected{
      "not json",                        // unparseable
      "[]",                              // not an object
      "{}",                              // no `labels`
      R"({"labels":[]})",                // `labels` not an object
      R"({"labels":"wall"})",            // ditto
      R"({"labels":{"wall":"x"}})",      // non-integer key
      R"({"labels":{"1x":"y"}})",        // trailing junk in the key
      R"({"labels":{"0":"x"}})",         // 0 is unlabeled, not a class
      R"({"labels":{"-1":"x"}})",        // ids are positive
      R"({"labels":{"1":5}})",           // non-string name
      R"({"labels":{"1":null}})",        // ditto
      R"({"labels":{"1":""}})",          // an empty caption is not a rename
      R"({"labels":{"9":"attic"}})",     // not already in the legend
      R"({"labels":{"1":"a","9":"b"}})", // one bad entry poisons the batch
  };

  for (const auto &body : rejected) {
    INFO("body: " << body);
    CHECK(status_of([&] { return patch_cloud_labels(db, "planes", body); }) ==
          400);
    // Storage replaces the whole map, so a half-applied patch would be a
    // rewrite with some edits in and some out.
    CHECK(db.label_definitions("planes") == before);
  }
}

TEST_CASE("PatchCloudLabels_UnknownCloudOrInstancesCloud_Throws",
          "[gui][edits]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  seed_planes(db);
  seed_planes(db, "instances");

  // No such cloud at all.
  CHECK(status_of([&] {
          return patch_cloud_labels(db, "nope", R"({"labels":{"1":"x"}})");
        }) == 404);

  SECTION("a cloud with no legend has nothing to rename") {
    reusex::CloudL cloud;
    reusex::LabelT label;
    label.label = 1;
    cloud.push_back(label);
    cloud.width = 1;
    cloud.height = 1;
    db.save_point_cloud("rooms", cloud, "test");

    CHECK(status_of([&] {
            return patch_cloud_labels(db, "rooms", R"({"labels":{"1":"x"}})");
          }) == 404);
  }

  SECTION("instance names encode a record the pipeline parses back") {
    const auto before = db.label_definitions("instances");
    CHECK(status_of([&] {
            return patch_cloud_labels(db, "instances",
                                      R"({"labels":{"1":"my chair"}})");
          }) == 409);
    CHECK(db.label_definitions("instances") == before);
  }
}

// ===========================================================================
// PATCH /materials/{guid}
// ===========================================================================

TEST_CASE("PatchMaterial_SparsePatch_SetsNamedPropertiesCreatingUnknownOnes",
          "[gui][edits]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  const auto guid = add_passport(db, "guid-patch-1");

  db.set_passport_property(guid, "colour", "red");
  const auto before = db.passport_stored_properties(guid);

  const auto body = patch_material(
      db, guid,
      R"({"properties":{"colour":"blue","dismantling_notes":"crane"}})");

  CHECK(body.at("guid") == guid);
  CHECK(body.at("properties").at("colour") == "blue");
  // An unknown field is auto-created rather than refused: passports arrive
  // from MaterialEPAS carrying fields no GUI form knows about.
  CHECK(body.at("properties").at("dismantling_notes") == "crane");
  CHECK(body.at("property_count").get<size_t>() == before.size() + 1);

  const auto stored = db.passport_stored_properties(guid);
  CHECK(stored.at("colour") == "blue");
  CHECK(stored.at("dismantling_notes") == "crane");
  // Everything the patch did not name is untouched.
  for (const auto &[key, value] : before) {
    INFO("property: " << key);
    if (key == "colour")
      continue;
    CHECK(stored.at(key) == value);
  }
}

TEST_CASE("PatchMaterial_NullValue_ClearsPropertyAndIsIdempotent",
          "[gui][edits]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  const auto guid = add_passport(db, "guid-patch-2");
  db.set_passport_property(guid, "colour", "red");

  const auto cleared =
      patch_material(db, guid, R"({"properties":{"colour":null}})");
  CHECK_FALSE(cleared.at("properties").contains("colour"));
  CHECK_FALSE(db.passport_stored_properties(guid).count("colour"));

  SECTION("the same request run again is idempotent, not an error") {
    const auto again =
        patch_material(db, guid, R"({"properties":{"colour":null}})");
    CHECK_FALSE(again.at("properties").contains("colour"));
  }

  SECTION("clearing a property the passport never carried is also fine") {
    const auto body =
        patch_material(db, guid, R"({"properties":{"never_set":null}})");
    CHECK_FALSE(body.at("properties").contains("never_set"));
  }
}

TEST_CASE("PatchMaterial_EmptyPatch_IsNoOpAndReturnsPassport", "[gui][edits]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  const auto guid = add_passport(db, "guid-patch-3");
  const auto before = db.passport_stored_properties(guid);

  const auto body = patch_material(db, guid, R"({"properties":{}})");
  CHECK(body.at("guid") == guid);
  CHECK(body.at("property_count").get<size_t>() == before.size());
  CHECK(db.passport_stored_properties(guid) == before);
  // The same shape material_json() serves, so a client parses one response.
  CHECK(body == material_json(db, guid));
}

TEST_CASE("PatchMaterial_MalformedBody_Throws", "[gui][edits]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  const auto guid = add_passport(db, "guid-patch-4");
  const auto before = db.passport_stored_properties(guid);

  CHECK(status_of([&] {
          return patch_material(db, "no-such-guid",
                                R"({"properties":{"colour":"red"}})");
        }) == 404);

  const std::vector<std::string> rejected{
      "not json",
      "[]",
      "{}",
      R"({"props":{"colour":"red"}})",
      R"({"properties":[]})",
      R"({"properties":"colour=red"})",
      R"({"properties":{"":"red"}})",
      R"({"properties":{"colour":5}})",
      R"({"properties":{"colour":{"value":"red"}}})",
      R"({"properties":{"colour":"red","":"x"}})",
  };

  for (const auto &body : rejected) {
    INFO("body: " << body);
    CHECK(status_of([&] { return patch_material(db, guid, body); }) == 400);
    CHECK(db.passport_stored_properties(guid) == before);
  }
}
