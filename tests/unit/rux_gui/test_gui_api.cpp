// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Contract tests for the `rux gui` API surface (#265, Phase 1).
//
// These cover the parts of docs/gui/openapi.yaml that are verifiable without a
// browser or a socket: the route table, request-parameter parsing, job
// submission validation, the WebSocket message protocol, and the JSON shape of
// the read endpoints against a real (empty and populated) ProjectDB.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <gui/api.hpp>
#include <gui/assets.hpp>
#include <gui/point_lod.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <pipeline/JobRunner.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <unistd.h>

namespace fs = std::filesystem;
using json = nlohmann::json;
using namespace rux::gui;

namespace {

using reusex::test_support::TempDir;
using reusex::test_support::TempPath;

// Add a material passport with the given document guid to `db`.
void add_passport(reusex::ProjectDB &db, const std::string &guid) {
  reusex::core::MaterialPassport p;
  p.metadata.document_guid = guid;
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  db.add_material_passport(p, "test-project");
}

void write_file(const fs::path &path, std::string_view content) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path, std::ios::binary);
  out << content;
}

Params params_of(const std::map<std::string, std::string> &values) {
  Params params;
  for (const auto &[key, value] : values)
    params.set(key, value);
  return params;
}

/// Assert the shared `offset`/`count`/`total` envelope, and that `count`
/// really is the length of the item array rather than a number that merely
/// looks plausible.
void check_page_envelope(const json &body, std::string_view items_key,
                         size_t offset, size_t count, size_t total) {
  INFO("collection: " << items_key);
  REQUIRE(body.contains("offset"));
  REQUIRE(body.contains("count"));
  REQUIRE(body.contains("total"));
  CHECK(body.at("offset") == offset);
  CHECK(body.at("count") == count);
  CHECK(body.at("total") == total);
  CHECK(body.at(std::string(items_key)).size() == count);
}

/// A cloud of @p points points, so a test can populate a project cheaply.
void save_test_cloud(reusex::ProjectDB &db, const std::string &name,
                     int points) {
  reusex::Cloud cloud;
  for (int i = 0; i < points; ++i) {
    reusex::PointT point;
    point.x = static_cast<float>(i);
    cloud.push_back(point);
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  db.save_point_cloud(name, cloud, "test");
}

} // namespace

// ===========================================================================
// Route table
// ===========================================================================

TEST_CASE("EndpointTable_DocumentedRoutes_MatchesContract", "[gui][routes]") {
  const auto &table = endpoint_table();
  REQUIRE_FALSE(table.empty());

  // The exact set documented in docs/gui/openapi.yaml. Adding a route without
  // documenting it must fail here — that is the point of this test.
  const std::set<std::string> expected{
      "GET /api/v1/health",
      "GET /api/v1/endpoints",
      "GET /api/v1/project",
      "GET /api/v1/projects",
      "PATCH /api/v1/projects/<string>",
      "GET /api/v1/clouds",
      "GET /api/v1/clouds/<string>",
      "GET /api/v1/clouds/<string>/points",
      "GET /api/v1/clouds/<string>/labels",
      "PATCH /api/v1/clouds/<string>/labels",
      "GET /api/v1/clouds/<string>/tiles",
      "GET /api/v1/meshes",
      "GET /api/v1/meshes/<string>",
      "GET /api/v1/meshes/<string>/data",
      "GET /api/v1/meshes/<string>/textures",
      "GET /api/v1/meshes/<string>/textures/<string>",
      "GET /api/v1/gsplats",
      "GET /api/v1/gsplats/<string>",
      "GET /api/v1/gsplats/<string>/data",
      "GET /api/v1/frames",
      "GET /api/v1/frames/visibility",
      "GET /api/v1/frames/<int>",
      "GET /api/v1/frames/<int>/image",
      "POST /api/v1/frames/<int>/segment",
      "GET /api/v1/panoramas",
      "GET /api/v1/panoramas/<int>",
      "GET /api/v1/panoramas/<int>/image",
      "POST /api/v1/panoramas/<int>/segment",
      "GET /api/v1/components",
      "GET /api/v1/components/<string>",
      "GET /api/v1/materials",
      "POST /api/v1/materials",
      "GET /api/v1/materials/<string>",
      "PATCH /api/v1/materials/<string>",
      "DELETE /api/v1/materials/<string>",
      "GET /api/v1/materials/<string>/thumbnail",
      "PUT /api/v1/materials/<string>/thumbnail",
      "GET /api/v1/material-columns",
      "POST /api/v1/material-columns",
      "PATCH /api/v1/material-columns/<string>",
      "DELETE /api/v1/material-columns/<string>",
      "GET /api/v1/instances/<string>",
      "GET /api/v1/instances/<string>/<int>/frames",
      "PUT /api/v1/instances/<string>/<int>/material",
      "GET /api/v1/stages",
      "GET /api/v1/stages/<string>/validation",
      "GET /api/v1/pipeline-log",
      "GET /api/v1/posegraph",
      "DELETE /api/v1/posegraph/edges/<int>/<int>",
      "POST /api/v1/posegraph/edges",
      "GET /api/v1/jobs",
      "POST /api/v1/jobs",
      "GET /api/v1/jobs/<string>",
      "POST /api/v1/jobs/<string>/cancel",
      "GET /api/v1/events",
      "GET /api/v1/reports/ressourcekortlaegning",
      "POST /api/v1/reports/ressourcekortlaegning",
      "GET /api/v1/reports/ressourcekortlaegning/<int>",
      "GET /api/v1/exports/csv",
      "GET /api/v1/export-templates",
      "POST /api/v1/export-templates",
      "GET /api/v1/export-templates/<int>",
      "PATCH /api/v1/export-templates/<int>",
      "DELETE /api/v1/export-templates/<int>",
  };

  std::set<std::string> actual;
  for (const auto &endpoint : table) {
    const auto key = endpoint.method + " " + endpoint.path;
    INFO("duplicate route: " << key);
    REQUIRE(actual.insert(key).second);

    CHECK_FALSE(endpoint.summary.empty());
    INFO("route outside the versioned prefix: " << key);
    CHECK(endpoint.path.rfind(std::string(kApiPrefix), 0) == 0);
  }

  CHECK(actual == expected);
}

TEST_CASE("EndpointsJson_RegisteredRoutes_MatchesEndpointTable",
          "[gui][routes]") {
  const auto body = endpoints_json();
  REQUIRE(body.contains("endpoints"));
  REQUIRE(body["endpoints"].size() == endpoint_table().size());

  for (const auto &entry : body["endpoints"]) {
    CHECK(entry.contains("method"));
    CHECK(entry.contains("path"));
    CHECK(entry.contains("summary"));
  }
}

// ===========================================================================
// Params
// ===========================================================================

TEST_CASE("Params_StrAndInteger_ReturnValueOrDefault", "[gui][params]") {
  Params params;
  params.set("limit", "250");
  params.set("format", "json");
  params.set("empty", "");

  CHECK(params.integer("limit", 100) == 250);
  CHECK(params.integer("missing", 100) == 100);
  CHECK(params.integer("empty", 7) == 7);
  CHECK(params.str("format", "binary") == "json");
  CHECK(params.str("missing", "fallback") == "fallback");
  CHECK(params.str("empty", "fallback") == "fallback");
}

TEST_CASE("ParamsInteger_NonIntegerValue_Throws400", "[gui][params]") {
  Params params;
  params.set("limit", "twelve");
  params.set("offset", "12abc");

  REQUIRE_THROWS_AS(params.integer("limit", 0), HttpError);
  REQUIRE_THROWS_AS(params.integer("offset", 0), HttpError);

  try {
    params.integer("limit", 0);
    FAIL("expected HttpError");
  } catch (const HttpError &e) {
    CHECK(e.status() == 400);
    CHECK(std::string(e.what()).find("limit") != std::string::npos);
  }
}

// ===========================================================================
// Errors
// ===========================================================================

TEST_CASE("ErrorJson_StatusAndMessage_SerializesBoth", "[gui][errors]") {
  const auto body = error_json(404, "no such cloud 'nope'");
  CHECK(body.at("status") == 404);
  CHECK(body.at("error") == "no such cloud 'nope'");
}

// ===========================================================================
// Read endpoints over a real ProjectDB
// ===========================================================================

TEST_CASE("HealthJson_OpenProject_ReportsNameWithoutPath", "[gui][project]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  const auto body = health_json(&db, project.path);
  CHECK(body.at("status") == "ok");
  CHECK(body.at("api_version") == std::string(kApiVersion));
  CHECK(body.at("implementation") == std::string(kImplementation));
  CHECK(body.at("project").at("open") == true);
  CHECK(body.at("project").at("schema_version").get<int>() > 0);

  // Only the file name: a remote implementation must not disclose server paths.
  const std::string reported = body.at("project").at("name");
  CHECK(reported == project.path.filename().string());
  CHECK(reported.find('/') == std::string::npos);
}

TEST_CASE("HealthJson_UnopenableProject_ReportsClosedWithoutSchemaVersion",
          "[gui][project]") {
  const auto body = health_json(nullptr, "/somewhere/broken.rux");
  CHECK(body.at("status") == "ok");
  CHECK(body.at("project").at("open") == false);
  CHECK(body.at("project").at("name") == "broken.rux");
  CHECK_FALSE(body.at("project").contains("schema_version"));
}

TEST_CASE("ProjectDbReadEndpoints_EmptyProject_ReturnEmptyCollections",
          "[gui][project]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  const auto summary = project_summary_json(db);
  CHECK(summary.at("path") == project.path.filename().string());
  CHECK(summary.at("schema_version").get<int>() > 0);
  for (const char *key : {"projects", "clouds", "meshes", "materials"}) {
    INFO("key: " << key);
    REQUIRE(summary.at(key).is_array());
    CHECK(summary.at(key).empty());
  }
  CHECK(summary.at("sensor_frames").at("total_count") == 0);
  CHECK(summary.at("panoramic_images").at("total_count") == 0);
  CHECK(summary.at("components").at("total_count") == 0);

  CHECK(clouds_json(db, Params{}).at("clouds").empty());
  CHECK(meshes_json(db, Params{}).at("meshes").empty());
  CHECK(frames_json(db, Params{}).at("ids").empty());
  CHECK(panoramas_json(db, Params{}).at("panoramas").empty());
  CHECK(materials_json(db, Params{}).at("materials").empty());
  CHECK(pipeline_log_json(db, Params{}).at("entries").empty());
}

TEST_CASE("ProjectsJson_ProjectMetadata_ExposesFields", "[gui][project]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  reusex::ProjectDB::ProjectMetadata metadata;
  metadata.id = "p1";
  metadata.name = "Test Building";
  metadata.building_address = "123 Test Street";
  metadata.year_of_construction = 1950;
  metadata.survey_date = "2026-04-16";
  metadata.survey_organisation = "Test Org";
  metadata.notes = "note";
  db.update_project_metadata(metadata);

  const auto body = projects_json(db, Params{});
  REQUIRE(body.at("projects").size() == 1);
  const auto &entry = body.at("projects").at(0);
  CHECK(entry.at("id") == "p1");
  CHECK(entry.at("name") == "Test Building");
  CHECK(entry.at("building_address") == "123 Test Street");
  CHECK(entry.at("year_of_construction") == 1950);
  CHECK(entry.at("survey_date") == "2026-04-16");
  CHECK(entry.at("survey_organisation") == "Test Org");
  CHECK(entry.at("notes") == "note");

  // The summary carries the same records.
  CHECK(project_summary_json(db).at("projects").size() == 1);
}

TEST_CASE("CloudJson_KnownAndUnknownCloud_ReturnsMetadataOrThrows",
          "[gui][clouds]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  reusex::Cloud cloud;
  for (int i = 0; i < 5; ++i) {
    reusex::PointT point;
    point.x = static_cast<float>(i);
    point.y = 1.0F;
    point.z = 2.0F;
    point.r = 10;
    point.g = 20;
    point.b = 30;
    cloud.push_back(point);
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  db.save_point_cloud("cloud", cloud, "test");

  const auto listed = clouds_json(db, Params{});
  REQUIRE(listed.at("clouds").size() == 1);
  CHECK(listed.at("clouds").at(0).at("name") == "cloud");
  CHECK(listed.at("clouds").at(0).at("type") == "PointXYZRGB");
  CHECK(listed.at("clouds").at(0).at("point_count") == 5);
  CHECK(listed.at("clouds").at(0).at("organized") == false);

  const auto single = cloud_json(db, "cloud");
  CHECK(single.at("name") == "cloud");
  CHECK(single.at("point_count") == 5);

  REQUIRE_THROWS_AS(cloud_json(db, "missing"), HttpError);
}

TEST_CASE("CloudPointsJson_OffsetLimitAndFormat_PagesAndValidatesFormat",
          "[gui][clouds]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  reusex::Cloud cloud;
  for (int i = 0; i < 10; ++i) {
    reusex::PointT point;
    point.x = static_cast<float>(i);
    point.y = 0.0F;
    point.z = 0.0F;
    point.r = static_cast<uint8_t>(i);
    point.g = 0;
    point.b = 0;
    cloud.push_back(point);
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  db.save_point_cloud("cloud", cloud, "test");

  Params page;
  page.set("offset", "3");
  page.set("limit", "4");

  const auto body = cloud_points_json(db, "cloud", page);
  CHECK(body.at("name") == "cloud");
  CHECK(body.at("type") == "PointXYZRGB");
  CHECK(body.at("total") == 10);
  CHECK(body.at("offset") == 3);
  CHECK(body.at("count") == 4);
  CHECK(body.at("fields") == json::array({"x", "y", "z", "r", "g", "b"}));
  REQUIRE(body.at("points").size() == 4);
  CHECK(body.at("points").at(0).at(0).get<double>() == 3.0);
  CHECK(body.at("points").at(3).at(0).get<double>() == 6.0);

  SECTION("an offset past the end yields an empty page, not an error") {
    Params past;
    past.set("offset", "1000");
    const auto empty = cloud_points_json(db, "cloud", past);
    CHECK(empty.at("count") == 0);
    CHECK(empty.at("total") == 10);
  }

  SECTION("format=json goes down the JSON path") {
    Params as_json;
    as_json.set("format", "json");
    const auto response = cloud_points(db, "cloud", as_json);
    REQUIRE(response.body.has_value());
    CHECK_FALSE(response.blob.has_value());
    CHECK(response.headers.empty());
    CHECK(response.body->at("total") == 10);
  }

  SECTION("format=binary answers with a RUXP page (#283)") {
    Params binary;
    binary.set("format", "binary");
    binary.set("offset", "3");
    binary.set("limit", "4");

    const auto response = cloud_points(db, "cloud", binary);
    REQUIRE(response.blob.has_value());
    CHECK_FALSE(response.body.has_value());
    CHECK(response.blob->content_type == "application/octet-stream");

    // Byte-level coverage of the format lives in
    // tests/unit/rux_gui/test_gui_binary_points.cpp; here it is enough that
    // the endpoint hands back a well-formed page of the right window.
    const auto &bytes = response.blob->data;
    REQUIRE(bytes.size() > 4);
    CHECK(std::string(reinterpret_cast<const char *>(bytes.data()), 4) ==
          "RUXP");
    CHECK(bytes.size() == 72 + 4 * 15); // header + 4 PointXYZRGB points

    // The X-Ruxp-* headers mirror the body header; they are a curl-level
    // convenience, not the contract (docs/gui/binary-points.md).
    std::map<std::string, std::string> headers(response.headers.begin(),
                                               response.headers.end());
    CHECK(headers.at("X-Ruxp-Version") == "1");
    CHECK(headers.at("X-Ruxp-Type") == "PointXYZRGB");
    CHECK(headers.at("X-Ruxp-Offset") == "3");
    CHECK(headers.at("X-Ruxp-Count") == "4");
    CHECK(headers.at("X-Ruxp-Total") == "10");
  }

  SECTION("an unknown format is a 400") {
    Params bogus;
    bogus.set("format", "protobuf");
    try {
      cloud_points(db, "cloud", bogus);
      FAIL("expected HttpError");
    } catch (const HttpError &e) {
      CHECK(e.status() == 400);
    }
  }

  SECTION("an unknown cloud is a 404 on both formats") {
    Params binary;
    binary.set("format", "binary");
    REQUIRE_THROWS_AS(cloud_points(db, "missing", binary), HttpError);
    REQUIRE_THROWS_AS(cloud_points(db, "missing", Params{}), HttpError);
  }
}

TEST_CASE("CloudJson_LabelCloud_ExposesLabelDefinitions", "[gui][clouds]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  reusex::CloudL labels;
  for (uint32_t i = 0; i < 4; ++i) {
    reusex::LabelT label;
    label.label = i; // 0 = unlabeled (STANDARDS §3)
    labels.push_back(label);
  }
  labels.width = labels.size();
  labels.height = 1;
  db.save_point_cloud("planes", labels, "test");
  db.save_label_definitions("planes", {{1, "wall"}, {2, "floor"}});

  const auto body = cloud_json(db, "planes");
  CHECK(body.at("type") == "Label");
  REQUIRE(body.contains("labels"));
  CHECK(body.at("labels").at("1") == "wall");
  CHECK(body.at("labels").at("2") == "floor");
  CHECK_FALSE(body.at("labels").contains("0"));

  Params none;
  const auto points = cloud_points_json(db, "planes", none);
  CHECK(points.at("fields") == json::array({"label"}));
  CHECK(points.at("points").at(0).at(0) == 0);
  CHECK(points.at("points").at(3).at(0) == 3);
}

TEST_CASE("CloudJson_LabelCloud_ExposesLabelKind", "[gui][clouds]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  reusex::CloudL labels;
  reusex::LabelT pt;
  pt.label = 1;
  labels.push_back(pt);
  labels.width = 1;
  labels.height = 1;

  // Geometry clouds: planes and rooms.
  db.save_point_cloud("planes", labels, "test");
  db.save_point_cloud("rooms", labels, "test");

  const auto planes = cloud_json(db, "planes");
  REQUIRE(planes.contains("label_kind"));
  CHECK(planes.at("label_kind") == "geometry");

  const auto rooms = cloud_json(db, "rooms");
  REQUIRE(rooms.contains("label_kind"));
  CHECK(rooms.at("label_kind") == "geometry");

  // Semantic clouds: instances and annotation-derived.
  db.save_point_cloud("instances", labels, "test");
  db.save_point_cloud("labels", labels, "test");

  const auto instances = cloud_json(db, "instances");
  REQUIRE(instances.contains("label_kind"));
  CHECK(instances.at("label_kind") == "semantic");

  const auto annot = cloud_json(db, "labels");
  REQUIRE(annot.contains("label_kind"));
  CHECK(annot.at("label_kind") == "semantic");
}

TEST_CASE("CloudTilesJson_NoTileIndex_Returns404", "[gui][clouds][tiles]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);
  save_test_cloud(db, "cloud", 100);

  // A cloud with no computed tile index is a 404 on this route.
  try {
    cloud_tiles_json(db, "cloud");
    FAIL("expected HttpError");
  } catch (const HttpError &e) {
    CHECK(e.status() == 404);
  }

  // An unknown cloud is also a 404.
  REQUIRE_THROWS_AS(cloud_tiles_json(db, "missing"), HttpError);
}

TEST_CASE("CloudTilesJson_WithTileIndex_ReturnsStructure",
          "[gui][clouds][tiles]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  // A cloud big enough for the default K=64 tiles, saved as
  // morton_10bit_bitrev.
  reusex::Cloud cloud;
  for (int i = 0; i < 4096; ++i) {
    reusex::PointT point;
    point.x = static_cast<float>(i % 16);
    point.y = static_cast<float>((i / 16) % 16);
    point.z = static_cast<float>(i / 256);
    cloud.push_back(point);
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  db.save_point_cloud("cloud", cloud, "test",
                      R"({"storage_order":"morton_10bit_bitrev"})");

  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  REQUIRE_FALSE(blob.empty());
  db.save_tile_index("cloud", blob);

  const auto body = cloud_tiles_json(db, "cloud");
  CHECK(body.at("name") == "cloud");
  CHECK(body.at("tile_count") == 64);
  CHECK(body.at("tile_bits") == 6);
  CHECK(body.at("point_count") == 4096);
  REQUIRE(body.at("tiles").is_array());
  REQUIRE(body.at("tiles").size() == 64);

  const auto &tile0 = body.at("tiles").at(0);
  CHECK(tile0.contains("id"));
  CHECK(tile0.contains("count"));
  REQUIRE(tile0.at("min").is_array());
  REQUIRE(tile0.at("min").size() == 3);
  REQUIRE(tile0.at("max").size() == 3);

  // Every point is assigned to exactly one tile.
  uint64_t total = 0;
  for (const auto &t : body.at("tiles"))
    total += t.at("count").get<uint64_t>();
  CHECK(total == 4096);

  // A single-tile points request returns that tile's points at full resolution.
  Params tile;
  tile.set("tile", "0");
  const auto points = cloud_points_json(db, "cloud", tile);
  CHECK(points.at("lod") == false);
  CHECK(points.at("count") == tile0.at("count"));
}

TEST_CASE("PipelineLogJson_MixedEntries_RoundTripsAndRejectsNegativeLimit",
          "[gui][pipeline]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  const int ok = db.log_pipeline_start("planes", R"({"radius":0.5})");
  db.log_pipeline_end(ok, true);
  const int bad = db.log_pipeline_start("rooms", "");
  db.log_pipeline_end(bad, false, "no planes");

  const auto body = pipeline_log_json(db, Params{});
  REQUIRE(body.at("entries").size() == 2);

  bool saw_success = false;
  bool saw_failure = false;
  for (const auto &entry : body.at("entries")) {
    CHECK(entry.contains("id"));
    CHECK_FALSE(entry.at("started_at").get<std::string>().empty());
    if (entry.at("stage") == "planes") {
      saw_success = entry.at("status") == "success";
      CHECK(entry.at("parameters") == R"({"radius":0.5})");
      CHECK(entry.at("error_msg") == "");
    }
    if (entry.at("stage") == "rooms") {
      saw_failure = entry.at("status") == "failed";
      CHECK(entry.at("error_msg") == "no planes");
    }
  }
  CHECK(saw_success);
  CHECK(saw_failure);

  SECTION("a negative limit is rejected") {
    Params negative;
    negative.set("limit", "-1");
    REQUIRE_THROWS_AS(pipeline_log_json(db, negative), HttpError);
  }
}

TEST_CASE("StagesJson_EmptyProject_SeparatesRunnableFromReady",
          "[gui][pipeline]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  const auto body = stages_json(db);
  REQUIRE(body.at("stages").size() == 5);

  std::set<std::string> names;
  for (const auto &stage : body.at("stages")) {
    names.insert(stage.at("stage").get<std::string>());
    CHECK(stage.contains("runnable"));
    CHECK(stage.contains("cancellable"));
    CHECK(stage.contains("ready"));
    CHECK(stage.at("blockers").is_array());
    // Nothing is ready in an empty project, and every unready stage must say
    // why (STANDARDS §5 — no silent failure).
    if (!stage.at("ready").get<bool>())
      CHECK_FALSE(stage.at("blockers").empty());
  }
  CHECK(names == std::set<std::string>{"clouds", "planes", "rooms", "instances",
                                       "mesh"});

  for (const auto &stage : body.at("stages")) {
    const auto name = stage.at("stage").get<std::string>();
    // All stages are now runnable (#265 Phase 3 added the mesh runner).
    CHECK(stage.at("runnable").get<bool>() == true);
    // clouds and mesh run stages that cannot be interrupted mid-run (clouds
    // because it is not interruptible, mesh because the MIP solver has no
    // cancel hook).
    if (name == "clouds" || name == "mesh")
      CHECK(stage.at("cancellable") == false);
    if (name == "planes" || name == "rooms" || name == "instances")
      CHECK(stage.at("cancellable") == true);
  }
}

TEST_CASE("StagesJson_EachStage_IncludesContractHintsAndParameterSchema",
          "[gui][pipeline]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  const auto body = stages_json(db);

  for (const auto &stage : body.at("stages")) {
    const auto name = stage.at("stage").get<std::string>();
    INFO("stage: " << name);

    CHECK_FALSE(stage.at("summary").get<std::string>().empty());
    CHECK_FALSE(stage.at("command").get<std::string>().empty());
    CHECK(stage.at("outputs").is_array());
    CHECK_FALSE(stage.at("outputs").empty());

    // An unready stage must not just say "no": every error-severity issue
    // carries the derived resolution command (#295), which is what lets the
    // UI say "run X first" instead of leaving the user stuck.
    REQUIRE(stage.at("issues").is_array());
    REQUIRE_FALSE(stage.at("issues").empty());
    for (const auto &issue : stage.at("issues")) {
      CHECK_FALSE(issue.at("check").get<std::string>().empty());
      CHECK_FALSE(issue.at("message").get<std::string>().empty());
      const auto severity = issue.at("severity").get<std::string>();
      CHECK((severity == "error" || severity == "warning"));
      if (severity == "error")
        CHECK_FALSE(issue.at("hint").get<std::string>().empty());
    }

    // `blockers` stays a strict summary of the error-severity issues.
    size_t errors = 0;
    for (const auto &issue : stage.at("issues"))
      if (issue.at("severity") == "error")
        ++errors;
    CHECK(stage.at("blockers").size() == errors);

    REQUIRE(stage.at("parameters").is_array());
    // Every runnable stage has at least one parameter knob, and every knob
    // must be renderable without guessing (#265 Phase 3 added the mesh runner).
    CHECK_FALSE(stage.at("parameters").empty());
    for (const auto &parameter : stage.at("parameters")) {
      INFO("parameter: " << parameter.at("key"));
      CHECK_FALSE(parameter.at("key").get<std::string>().empty());
      CHECK_FALSE(parameter.at("label").get<std::string>().empty());
      CHECK_FALSE(parameter.at("description").get<std::string>().empty());
      CHECK(parameter.contains("default"));
      CHECK(parameter.contains("minimum"));
      CHECK(parameter.contains("maximum"));
      CHECK(parameter.at("presence_sensitive").is_boolean());
    }
  }
}

TEST_CASE(
    "StageValidationJson_KnownAndUnknownStage_MatchesCatalogueOrThrows404",
    "[gui][pipeline]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  const auto catalogue = stages_json(db);
  for (const auto &stage : catalogue.at("stages")) {
    const auto name = stage.at("stage").get<std::string>();
    INFO("stage: " << name);
    CHECK(stage_validation_json(db, name) == stage);
  }

  // An unknown stage is a 404, not an empty record that a UI would render as
  // "ready".
  CHECK_THROWS_AS(stage_validation_json(db, "not-a-stage"), HttpError);
  try {
    stage_validation_json(db, "not-a-stage");
  } catch (const HttpError &e) {
    CHECK(e.status() == 404);
    CHECK(std::string(e.what()).find("not-a-stage") != std::string::npos);
  }
}

TEST_CASE("MissingResourceJson_UnknownId_Throws404WithMessage",
          "[gui][errors]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  auto expect_404 = [](auto &&callable) {
    try {
      callable();
      FAIL("expected HttpError");
    } catch (const HttpError &e) {
      CHECK(e.status() == 404);
      CHECK_FALSE(std::string(e.what()).empty());
    }
  };

  expect_404([&] { return cloud_json(db, "nope"); });
  expect_404([&] { return mesh_json(db, "nope"); });
  expect_404([&] { return mesh_textures_json(db, "nope"); });
  expect_404([&] { return frame_json(db, 42); });
  expect_404([&] { return panorama_json(db, 42); });
  expect_404([&] { return component_json(db, "nope"); });
  expect_404([&] { return material_json(db, "nope"); });
  expect_404([&] { return instances_json(db, "nope", Params{}); });
}

TEST_CASE("InstancesJson_SavedInstances_ReportsGuidAndMaterialLink",
          "[gui][instances]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  reusex::CloudL labels;
  for (uint32_t i = 0; i < 3; ++i) {
    reusex::LabelT label;
    label.label = i + 1;
    labels.push_back(label);
  }
  labels.width = labels.size();
  labels.height = 1;
  db.save_point_cloud("instances", labels, "test");

  db.save_instances("instances",
                    {{1, "guid-one", 7, 100}, {2, "guid-two", 7, 200}});

  const auto body = instances_json(db, "instances", Params{});
  CHECK(body.at("cloud") == "instances");
  REQUIRE(body.at("instances").size() == 2);

  const auto &first = body.at("instances").at(0);
  CHECK(first.at("instance_id") == 1);
  CHECK(first.at("guid") == "guid-one");
  CHECK(first.at("semantic_class") == 7);
  CHECK(first.at("point_count") == 100);
  // Unlinked instances report null rather than omitting the key, so a client
  // never has to distinguish "absent" from "unlinked".
  CHECK(first.at("material_guid").is_null());
}

TEST_CASE("LinkInstanceMaterial_UnknownCloud_Is404", "[gui][instances]") {
  TempPath project("test_gui_api_link");
  reusex::ProjectDB db(project.path);
  CHECK_THROWS_AS(
      link_instance_material(db, "no-such-cloud", 1, R"({"guid":"g"})"),
      HttpError);
}

TEST_CASE("LinkInstanceMaterial_UnknownInstance_Is404", "[gui][instances]") {
  TempPath project("test_gui_api_link");
  reusex::ProjectDB db(project.path);

  reusex::CloudL labels;
  reusex::LabelT lbl;
  lbl.label = 1;
  labels.push_back(lbl);
  labels.width = 1;
  labels.height = 1;
  db.save_point_cloud("instances", labels, "test");
  db.save_instances("instances", {{1, "inst-guid", 7, 10}});
  add_passport(db, "mat-guid");

  // instance_id 99 does not exist
  CHECK_THROWS_AS(
      link_instance_material(db, "instances", 99, R"({"guid":"mat-guid"})"),
      HttpError);
}

TEST_CASE("LinkInstanceMaterial_HappyPath_ReturnsMaterialGuid",
          "[gui][instances]") {
  TempPath project("test_gui_api_link");
  reusex::ProjectDB db(project.path);

  reusex::CloudL labels;
  for (uint32_t i = 0; i < 2; ++i) {
    reusex::LabelT lbl;
    lbl.label = i + 1;
    labels.push_back(lbl);
  }
  labels.width = 2;
  labels.height = 1;
  db.save_point_cloud("instances", labels, "test");
  db.save_instances("instances",
                    {{1, "inst-guid-1", 7, 100}, {2, "inst-guid-2", 7, 200}});
  add_passport(db, "mat-guid-A");

  const auto body =
      link_instance_material(db, "instances", 1, R"({"guid":"mat-guid-A"})");

  CHECK(body.at("instance_id") == 1);
  CHECK(body.at("guid") == "inst-guid-1");
  CHECK(body.at("material_guid") == "mat-guid-A");

  // Instance 2 should remain unlinked.
  const auto list = instances_json(db, "instances", Params{});
  CHECK(list.at("instances").at(1).at("material_guid").is_null());
}

TEST_CASE("LinkInstanceMaterial_BadBody_Is400", "[gui][instances]") {
  TempPath project("test_gui_api_link");
  reusex::ProjectDB db(project.path);

  reusex::CloudL labels;
  reusex::LabelT lbl;
  lbl.label = 1;
  labels.push_back(lbl);
  labels.width = 1;
  labels.height = 1;
  db.save_point_cloud("instances", labels, "test");
  db.save_instances("instances", {{1, "inst-guid", 7, 10}});

  CHECK_THROWS_AS(link_instance_material(db, "instances", 1, "not-json"),
                  HttpError);
  CHECK_THROWS_AS(link_instance_material(db, "instances", 1, R"({})"),
                  HttpError);
}

// ===========================================================================
// Jobs
// ===========================================================================

TEST_CASE("ParseJobRequest_VariousBodies_ValidatesBeforeQueuing",
          "[gui][jobs]") {
  SECTION("a well-formed request parses") {
    const auto submission =
        parse_job_request(R"({"stage":"planes","parameters":{"radius":0.5}})");
    CHECK(submission.stage == reusex::pipeline::JobStage::planes);
    CHECK(json::parse(submission.parameters).at("radius") == 0.5);
  }

  SECTION("parameters may be omitted") {
    const auto submission = parse_job_request(R"({"stage":"rooms"})");
    CHECK(submission.stage == reusex::pipeline::JobStage::rooms);
    CHECK(submission.parameters.empty());
  }

  SECTION("null parameters are treated as omitted") {
    const auto submission =
        parse_job_request(R"({"stage":"rooms","parameters":null})");
    CHECK(submission.parameters.empty());
  }

  auto expect_400 = [](std::string_view body) {
    INFO("body: " << body);
    try {
      parse_job_request(body);
      FAIL("expected HttpError");
    } catch (const HttpError &e) {
      CHECK(e.status() == 400);
    }
  };

  SECTION("malformed or incomplete requests are 400s") {
    expect_400("not json");
    expect_400("[]");
    expect_400("{}");
    expect_400(R"({"stage":123})");
    expect_400(R"({"stage":"planes","parameters":"radius=0.5"})");
    expect_400(R"({"stage":"planes","parameters":[1,2]})");
  }

  SECTION("mesh is now a valid submittable stage (#265 Phase 3)") {
    // This must NOT throw: mesh has a runner, so the request is accepted.
    CHECK_NOTHROW(parse_job_request(R"({"stage":"mesh"})"));
  }
}

TEST_CASE("JobJson_RunningJobRecord_SerializesDocumentedShape", "[gui][jobs]") {
  reusex::pipeline::JobRecord record;
  record.id = "job-1";
  record.stage = reusex::pipeline::JobStage::planes;
  record.status = reusex::pipeline::JobStatus::running;
  record.parameters = R"({"radius":0.5})";
  record.submitted_at = "2026-09-08T11:22:30Z";
  record.started_at = "2026-09-08T11:22:31Z";
  record.progress_stage = reusex::core::Stage::region_growing;
  record.progress_current = 25;
  record.progress_total = 100;

  const auto body = job_json(record, "scan.rux");
  CHECK(body.at("id") == "job-1");
  CHECK(body.at("stage") == "planes");
  CHECK(body.at("status") == "running");
  CHECK(body.at("finished_at") == "");
  CHECK(body.at("cancel_requested") == false);
  // Parameters go out as an object, not an escaped string.
  REQUIRE(body.at("parameters").is_object());
  CHECK(body.at("parameters").at("radius") == 0.5);
  // The wire carries a machine token; the display string rides alongside it so
  // a client never has to match on prose.
  CHECK(body.at("progress").at("stage") == "region_growing");
  CHECK(body.at("progress").at("stage_label") == "Region Growing");
  CHECK(body.at("progress").at("current") == 25);
  CHECK(body.at("progress").at("total") == 100);
  CHECK(body.at("progress").at("fraction").get<double>() == 0.25);
}

TEST_CASE("JobJson_ZeroProgressTotal_ReportsNullFraction", "[gui][jobs]") {
  reusex::pipeline::JobRecord record;
  record.id = "job-2";
  record.progress_total = 0;
  record.progress_current = 0;

  const auto body = job_json(record, "scan.rux");
  CHECK(body.at("progress").at("total") == 0);
  CHECK(body.at("progress").at("fraction").is_null());
}

TEST_CASE("JobJson_UnparseableParameters_DegradesToEmptyObject",
          "[gui][jobs]") {
  reusex::pipeline::JobRecord record;
  record.id = "job-3";
  record.parameters = "this is not json";

  const auto body = job_json(record, "scan.rux");
  REQUIRE(body.at("parameters").is_object());
  CHECK(body.at("parameters").empty());
}

TEST_CASE("JobEventJson_FinishedEvent_WrapsFullJobRecord", "[gui][jobs]") {
  reusex::pipeline::JobEvent event;
  event.type = reusex::pipeline::JobEvent::Type::finished;
  event.timestamp = "2026-09-08T11:24:02Z";
  event.job.id = "job-4";
  event.job.status = reusex::pipeline::JobStatus::failed;
  event.job.error = "inputs missing";

  const auto body = job_event_json(event, "scan.rux");
  CHECK(body.at("type") == "job.finished");
  CHECK(body.at("timestamp") == "2026-09-08T11:24:02Z");
  CHECK(body.at("job").at("id") == "job-4");
  CHECK(body.at("job").at("status") == "failed");
  CHECK(body.at("job").at("error") == "inputs missing");
}

TEST_CASE("HelloJson_JobSnapshotAndProjectPath_OmitsServerPath",
          "[gui][websocket]") {
  reusex::pipeline::JobRecord record;
  record.id = "job-5";

  const auto body = hello_json({record}, "/home/user/scans/scan.rux");
  CHECK(body.at("type") == "hello");
  CHECK(body.at("api_version") == std::string(kApiVersion));
  CHECK(body.at("project") == "scan.rux");
  REQUIRE(body.at("jobs").is_array());
  CHECK(body.at("jobs").size() == 1);
  CHECK(body.at("jobs").at(0).at("id") == "job-5");
}

// ===========================================================================
// WebSocket protocol
// ===========================================================================

TEST_CASE("HandleWsMessage_VariousMessageTypes_FollowsDocumentedProtocol",
          "[gui][websocket]") {
  std::optional<std::string> subscription;
  bool subscribe_called = false;
  auto subscribe = [&](std::optional<std::string> job_id) {
    subscribe_called = true;
    subscription = std::move(job_id);
  };

  SECTION("ping is answered with pong") {
    auto reply = handle_ws_message(R"({"type":"ping"})", subscribe);
    REQUIRE(reply.has_value());
    CHECK(reply->at("type") == "pong");
    CHECK_FALSE(subscribe_called);
  }

  SECTION("subscribe sets a filter and needs no reply") {
    auto reply =
        handle_ws_message(R"({"type":"subscribe","job_id":"abc"})", subscribe);
    CHECK_FALSE(reply.has_value());
    CHECK(subscribe_called);
    REQUIRE(subscription.has_value());
    CHECK(*subscription == "abc");
  }

  SECTION("a null job_id clears the filter") {
    subscription = "abc";
    auto reply =
        handle_ws_message(R"({"type":"subscribe","job_id":null})", subscribe);
    CHECK_FALSE(reply.has_value());
    CHECK_FALSE(subscription.has_value());
  }

  SECTION("bad messages get an error frame, never a dropped connection") {
    for (const char *body :
         {"not json", "[]", R"({"no_type":1})", R"({"type":"nonsense"})",
          R"({"type":"subscribe"})", R"({"type":"subscribe","job_id":7})"}) {
      INFO("body: " << body);
      auto reply = handle_ws_message(body, subscribe);
      REQUIRE(reply.has_value());
      CHECK(reply->at("type") == "error");
      CHECK_FALSE(reply->at("error").get<std::string>().empty());
    }
  }
}

TEST_CASE("EventMatchesSubscription_JobIdFilter_MatchesOnlyTargetJob",
          "[gui][websocket]") {
  reusex::pipeline::JobEvent event;
  event.job.id = "job-a";

  CHECK(event_matches_subscription(event, std::nullopt));
  CHECK(event_matches_subscription(event, std::optional<std::string>("job-a")));
  CHECK_FALSE(
      event_matches_subscription(event, std::optional<std::string>("job-b")));
}

// ===========================================================================
// Static assets
// ===========================================================================

TEST_CASE("PlaceholderPage_ProjectName_ProducesSelfDescribingHtml",
          "[gui][assets]") {
  const auto page = placeholder_page("scan.rux");
  CHECK(page.rfind("<!doctype html>", 0) == 0);
  CHECK(page.find("scan.rux") != std::string::npos);
  CHECK(page.find("/api/v1") != std::string::npos);
  // The sentinel must be fully substituted, not left in the output.
  CHECK(page.find("%%PROJECT%%") == std::string::npos);
}

TEST_CASE("ResolveAsset_VariousRequests_ServesOnlyFilesUnderRoot",
          "[gui][assets]") {
  TempDir root("test_gui_assets");
  write_file(root.path / "index.html", "<h1>index</h1>");
  write_file(root.path / "assets" / "app.js", "console.log(1)");

  SECTION("root resolves to index.html") {
    const auto resolved = resolve_asset(root.path, "/");
    REQUIRE_FALSE(resolved.empty());
    CHECK(resolved.filename() == "index.html");
  }

  SECTION("nested files resolve") {
    const auto resolved = resolve_asset(root.path, "/assets/app.js");
    REQUIRE_FALSE(resolved.empty());
    CHECK(resolved.filename() == "app.js");
  }

  SECTION("a query string is ignored") {
    const auto resolved = resolve_asset(root.path, "/assets/app.js?v=2");
    REQUIRE_FALSE(resolved.empty());
    CHECK(resolved.filename() == "app.js");
  }

  SECTION("path traversal is refused") {
    // A server that can be talked into reading outside its bundle is a
    // security bug, so these must all come back empty.
    for (const char *attack :
         {"/../../../etc/passwd", "/assets/../../etc/passwd",
          "/..%2f..%2fetc/passwd", "//etc/passwd"}) {
      INFO("attack: " << attack);
      CHECK(resolve_asset(root.path, attack).empty());
    }
  }

  SECTION("a missing file resolves to nothing") {
    CHECK(resolve_asset(root.path, "/does-not-exist.js").empty());
  }

  SECTION("an empty root never resolves") {
    CHECK(resolve_asset({}, "/index.html").empty());
  }
}

TEST_CASE("MimeTypeFor_VariousExtensions_ReturnsExpectedContentType",
          "[gui][assets]") {
  CHECK(mime_type_for("index.html") == "text/html; charset=utf-8");
  CHECK(mime_type_for("app.js") == "text/javascript; charset=utf-8");
  CHECK(mime_type_for("app.css") == "text/css; charset=utf-8");
  CHECK(mime_type_for("data.json") == "application/json");
  CHECK(mime_type_for("mesh.glb") == "model/gltf-binary");
  CHECK(mime_type_for("logo.SVG") == "image/svg+xml"); // case-insensitive
  CHECK(mime_type_for("archive.tar.zst") == "application/octet-stream");
  CHECK(mime_type_for("noextension") == "application/octet-stream");
}

TEST_CASE("ResolveAssetDir_NotADirectory_Throws", "[gui][assets]") {
  REQUIRE_THROWS_AS(resolve_asset_dir("/definitely/not/a/directory"),
                    std::runtime_error);
}

// ===========================================================================
// Review follow-ups (#274): project identity, limit clamping, SPA fallback
// ===========================================================================

TEST_CASE("JobAndEventJson_AnyRecord_IncludesProjectField",
          "[gui][jobs][project]") {
  // The contract has to survive Phase 6, where one ruxd serves many projects.
  // Adding the field later would be a breaking change; adding it now costs a
  // string and lets a client key its state on it from day one.
  reusex::pipeline::JobRecord record;
  record.id = "job-p";

  CHECK(job_json(record, "scan.rux").at("project") == "scan.rux");
  CHECK(jobs_json({record}, "scan.rux").at("jobs").at(0).at("project") ==
        "scan.rux");

  reusex::pipeline::JobEvent event;
  event.job = record;
  CHECK(job_event_json(event, "scan.rux").at("project") == "scan.rux");
  CHECK(job_event_json(event, "scan.rux").at("job").at("project") ==
        "scan.rux");
}

TEST_CASE("CheckJobProject_VariousProjectValues_AllowsMatchOrRejectsMismatch",
          "[gui][jobs]") {
  SECTION("omitting project is fine — the server has only one open") {
    const auto submission = parse_job_request(R"({"stage":"planes"})");
    CHECK_FALSE(submission.project.has_value());
    REQUIRE_NOTHROW(check_job_project(submission, "scan.rux"));
  }

  SECTION("naming the open project is fine") {
    const auto submission =
        parse_job_request(R"({"stage":"planes","project":"scan.rux"})");
    REQUIRE(submission.project.has_value());
    REQUIRE_NOTHROW(check_job_project(submission, "scan.rux"));
  }

  SECTION("naming a different project is a 409, not a silent wrong run") {
    const auto submission =
        parse_job_request(R"({"stage":"planes","project":"other.rux"})");
    try {
      check_job_project(submission, "scan.rux");
      FAIL("expected HttpError");
    } catch (const HttpError &e) {
      CHECK(e.status() == 409);
      const std::string message = e.what();
      CHECK(message.find("other.rux") != std::string::npos);
      CHECK(message.find("scan.rux") != std::string::npos);
    }
  }

  SECTION("a non-string project is a 400") {
    try {
      parse_job_request(R"({"stage":"planes","project":7})");
      FAIL("expected HttpError");
    } catch (const HttpError &e) {
      CHECK(e.status() == 400);
    }
  }
}

TEST_CASE("JobEventJson_SequenceField_SerializesSeq", "[gui][websocket]") {
  // Events are published without the runner lock, so arrival order is not
  // emission order. `seq` is assigned under the lock and is the authority.
  reusex::pipeline::JobEvent event;
  event.sequence = 42;
  CHECK(job_event_json(event, "scan.rux").at("seq") == 42);
}

TEST_CASE("PipelineLogJson_LimitParameter_ClampsWithinBoundsOrRejectsNegative",
          "[gui][pipeline]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  for (int i = 0; i < 5; ++i) {
    const int id = db.log_pipeline_start("planes", "");
    db.log_pipeline_end(id, true);
  }

  SECTION("an explicit small limit is honoured") {
    Params params;
    params.set("limit", "2");
    CHECK(pipeline_log_json(db, params).at("entries").size() == 2);
  }

  SECTION("limit=0 means the server maximum, not unbounded") {
    // ProjectDB treats 0 as "no limit"; passing it straight through would let
    // one query serialize a whole project's history.
    Params params;
    params.set("limit", "0");
    const auto entries = pipeline_log_json(db, params).at("entries");
    CHECK(entries.size() == 5); // Fewer than the cap, so all of them.
  }

  SECTION("an absurd limit is clamped rather than honoured") {
    Params params;
    params.set("limit", "99999999");
    CHECK(pipeline_log_json(db, params).at("entries").size() == 5);
  }

  SECTION("a negative limit is still a 400") {
    Params params;
    params.set("limit", "-5");
    REQUIRE_THROWS_AS(pipeline_log_json(db, params), HttpError);
  }
}

TEST_CASE("LooksLikeSpaRoute_RoutesVsAssetPaths_DistinguishesCorrectly",
          "[gui][assets]") {
  // Answering a missing /assets/app.js with index.html hands the browser HTML
  // where it expects JavaScript — the failure then surfaces as an inscrutable
  // syntax error instead of the 404 it actually is.
  CHECK(looks_like_spa_route("/"));
  CHECK(looks_like_spa_route("/projects"));
  CHECK(looks_like_spa_route("/projects/42"));
  CHECK(looks_like_spa_route("/projects/42?tab=clouds"));
  CHECK(looks_like_spa_route("/viewer/"));

  CHECK_FALSE(looks_like_spa_route("/assets/app.js"));
  CHECK_FALSE(looks_like_spa_route("/assets/app.4f2c.css"));
  CHECK_FALSE(looks_like_spa_route("/favicon.ico"));
  CHECK_FALSE(looks_like_spa_route("/assets/app.js?v=2"));
}

TEST_CASE("PercentDecode_EncodedTraversal_DecodesBeforeAssetResolution",
          "[gui][assets]") {
  CHECK(percent_decode("/assets/app.js") == "/assets/app.js");
  CHECK(percent_decode("%2e%2e/secret") == "../secret");
  CHECK(percent_decode("%2E%2E%2Fsecret") == "../secret");
  CHECK(percent_decode("a%20b") == "a b");
  // Malformed escapes are left verbatim rather than silently dropped.
  CHECK(percent_decode("100%") == "100%");
  CHECK(percent_decode("%zz") == "%zz");
  CHECK(percent_decode("%4") == "%4");

  TempDir root("test_gui_assets");
  write_file(root.path / "index.html", "<h1>index</h1>");

  // The raw-text traversal check these encodings used to slip past.
  for (const char *attack :
       {"/%2e%2e/%2e%2e/etc/passwd", "/assets/%2E%2E/%2E%2E/etc/passwd",
        "/..%2f..%2fetc/passwd", "/%2e%2e%5c%2e%2e%5cetc%5cpasswd"}) {
    INFO("attack: " << attack);
    CHECK(resolve_asset(root.path, attack).empty());
  }
}

// ===========================================================================
// Paging (#285)
// ===========================================================================

TEST_CASE("ParsePageRequest_ZeroOrOversizedLimit_ClampsToServerMaximum",
          "[gui][paging]") {
  // 0 means "as many as the server will give", which is the maximum -- never
  // "unbounded". Anything larger is clamped rather than refused, so a client
  // asking for more than it can get is not taught to hard-code our cap.
  CHECK(parse_page_request(params_of({{"limit", "0"}}), 25, 100).limit == 100);
  CHECK(parse_page_request(params_of({{"limit", "9999"}}), 25, 100).limit ==
        100);
  CHECK(parse_page_request(params_of({{"limit", "7"}}), 25, 100).limit == 7);

  // Absent means the collection's own default, which is not always the maximum.
  CHECK(parse_page_request(Params{}, 25, 100).limit == 25);
  CHECK(parse_page_request(Params{}, 0, 100).limit == 100);
}

TEST_CASE("ParsePageRequest_NegativeOrNonNumericValues_Rejects",
          "[gui][paging]") {
  REQUIRE_THROWS_AS(parse_page_request(params_of({{"offset", "-1"}}), 0, 100),
                    HttpError);
  REQUIRE_THROWS_AS(parse_page_request(params_of({{"limit", "-1"}}), 0, 100),
                    HttpError);
  REQUIRE_THROWS_AS(parse_page_request(params_of({{"offset", "half"}}), 0, 100),
                    HttpError);
}

TEST_CASE("PageWindow_OffsetPastTheEnd_IsAnEmptyPageNotAnError",
          "[gui][paging]") {
  // A client walking a collection that shrank underneath it should get an
  // honest empty page, not a 404 it has to special-case.
  const auto past = page_window(PageRequest{50, 10}, 5);
  CHECK(past.count() == 0);
  CHECK(past.total == 5);

  // A huge offset paired with a huge limit must not wrap around size_t and
  // return the whole collection.
  const auto huge = page_window(
      PageRequest{static_cast<uint64_t>(-1), static_cast<uint64_t>(-1)}, 5);
  CHECK(huge.count() == 0);

  const auto partial = page_window(PageRequest{3, 10}, 5);
  CHECK(partial.first == 3);
  CHECK(partial.count() == 2);
}

TEST_CASE("PagedCollections_OffsetAndLimit_ReturnRequestedWindowWithEnvelope",
          "[gui][paging]") {
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);
  for (const char *name : {"a", "b", "c", "d", "e"})
    save_test_cloud(db, name, 2);

  SECTION("no parameters returns everything, still enveloped") {
    check_page_envelope(clouds_json(db, Params{}), "clouds", 0, 5, 5);
  }

  SECTION("a window is honoured and total stays the collection size") {
    const auto body =
        clouds_json(db, params_of({{"offset", "1"}, {"limit", "2"}}));
    check_page_envelope(body, "clouds", 1, 2, 5);
  }

  SECTION("a page past the end is empty rather than a 404") {
    const auto body = clouds_json(db, params_of({{"offset", "99"}}));
    check_page_envelope(body, "clouds", 5, 0, 5);
  }
}

TEST_CASE("PagedCollections_EmptyProject_AllCarryTheSharedEnvelope",
          "[gui][paging]") {
  // One idiom, one set of field names. A client writes its "is there more?"
  // logic once, so every paged collection must actually agree.
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);
  save_test_cloud(db, "instances", 1);

  check_page_envelope(clouds_json(db, Params{}), "clouds", 0, 1, 1);
  check_page_envelope(meshes_json(db, Params{}), "meshes", 0, 0, 0);
  check_page_envelope(panoramas_json(db, Params{}), "panoramas", 0, 0, 0);
  check_page_envelope(materials_json(db, Params{}), "materials", 0, 0, 0);
  check_page_envelope(projects_json(db, Params{}), "projects", 0, 0, 0);
  check_page_envelope(components_json(db, Params{}), "components", 0, 0, 0);
  check_page_envelope(pipeline_log_json(db, Params{}), "entries", 0, 0, 0);
  check_page_envelope(instances_json(db, "instances", Params{}), "instances", 0,
                      0, 0);
  check_page_envelope(frames_json(db, Params{}), "ids", 0, 0, 0);
  check_page_envelope(jobs_page_json({}, "scan.rux", Params{}), "jobs", 0, 0,
                      0);
}

TEST_CASE("InstancesJson_Paged_KeepsCloudNameAlongsideTheEnvelope",
          "[gui][paging][instances]") {
  // The `cloud` field is not part of the page; it must survive paging.
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);
  save_test_cloud(db, "instances", 1);

  const auto body =
      instances_json(db, "instances", params_of({{"limit", "1"}}));
  CHECK(body.at("cloud") == "instances");
  REQUIRE(body.contains("total"));
}

TEST_CASE("FramesJson_Paged_KeepsScanTotalsDistinctFromThePageTotal",
          "[gui][paging][frames]") {
  // TWO DIFFERENT TOTALS, deliberately: `total` counts the ids matching the
  // filter (what the client pages through), `total_count` describes the whole
  // scan so a browser can render "N of M" without a second request. Conflating
  // them is the mistake this test exists to catch.
  TempPath project("test_gui_api");
  reusex::ProjectDB db(project.path);

  const auto body = frames_json(db, params_of({{"limit", "1"}}));
  REQUIRE(body.contains("total"));
  REQUIRE(body.contains("total_count"));
  REQUIRE(body.contains("segmented_count"));
  CHECK(body.at("ids").size() == body.at("count"));
}

TEST_CASE("FramesJson_MultiScan_GroupsIdsByScanId", "[gui][frames]") {
  // Frames from two different import sessions must appear in separate `scans`
  // entries in the response (#462). The flat `ids` array still has all ids;
  // the `scans` array provides the per-scan breakdown the frontend needs to
  // render one collapsible group per import session.
  TempPath project("test_gui_api_multiscan");
  reusex::ProjectDB db(project.path);

  const auto scan_a = db.create_scan("path/to/scan_a.db");
  const auto scan_b = db.create_scan("path/to/scan_b.db");

  const auto intr =
      reusex::test_support::make_intrinsics(100.0, 100.0, 64.0, 64.0, 128, 128);
  const std::array<double, 16> identity{1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, 0, 0, 0, 1};
  db.save_sensor_frame(1, reusex::test_support::make_color(128, 128), cv::Mat(),
                       cv::Mat(), identity, intr, 1.0, scan_a.id);
  db.save_sensor_frame(2, reusex::test_support::make_color(128, 128), cv::Mat(),
                       cv::Mat(), identity, intr, 2.0, scan_a.id);
  db.save_sensor_frame(3, reusex::test_support::make_color(128, 128), cv::Mat(),
                       cv::Mat(), identity, intr, 3.0, scan_b.id);

  const auto body = frames_json(db, Params{});
  REQUIRE(body.contains("ids"));
  CHECK(body.at("ids").size() == 3);
  CHECK(body.at("total_count") == 3);

  REQUIRE(body.contains("scans"));
  const auto &scans = body.at("scans");
  REQUIRE(scans.size() == 2);

  CHECK(scans[0].at("scan_id") == scan_a.id);
  CHECK(scans[0].at("source_path") == "path/to/scan_a.db");
  REQUIRE(scans[0].contains("imported_at"));
  REQUIRE(scans[0].at("ids").size() == 2);
  CHECK(scans[0].at("ids")[0] == 1);
  CHECK(scans[0].at("ids")[1] == 2);

  CHECK(scans[1].at("scan_id") == scan_b.id);
  CHECK(scans[1].at("source_path") == "path/to/scan_b.db");
  REQUIRE(scans[1].at("ids").size() == 1);
  CHECK(scans[1].at("ids")[0] == 3);
}

// ===========================================================================
// Job.result (#285)
// ===========================================================================

TEST_CASE("JobJson_SucceededJobWithArtifacts_ReportsWhatItWrote",
          "[gui][jobs]") {
  // A UI refreshes what this run wrote instead of re-fetching every collection
  // on the chance that one of them changed.
  reusex::pipeline::JobRecord record;
  record.id = "job-1";
  record.stage = reusex::pipeline::JobStage::planes;
  record.status = reusex::pipeline::JobStatus::succeeded;
  record.result_summary = "detected 3 plane(s)";
  record.result_outputs = {{"cloud", "planes", 3},
                           {"cloud", "plane_centroids", 3}};

  const auto body = job_json(record, "scan.rux");
  REQUIRE(body.contains("result"));
  CHECK(body.at("result").at("summary") == "detected 3 plane(s)");
  REQUIRE(body.at("result").at("outputs").size() == 2);
  CHECK(body.at("result").at("outputs").at(0).at("kind") == "cloud");
  CHECK(body.at("result").at("outputs").at(0).at("name") == "planes");
  CHECK(body.at("result").at("outputs").at(0).at("count") == 3);
}

TEST_CASE("JobJson_ArtifactWithNoHonestCount_OmitsTheCountField",
          "[gui][jobs]") {
  // An absent count reads as "not measured". Sending 0 would read as "wrote
  // nothing", and -1 would read as a number.
  reusex::pipeline::JobRecord record;
  record.id = "job-2";
  record.status = reusex::pipeline::JobStatus::succeeded;
  record.result_outputs = {{"table", "instances", -1}};

  const auto entry =
      job_json(record, "scan.rux").at("result").at("outputs").at(0);
  CHECK(entry.at("name") == "instances");
  CHECK_FALSE(entry.contains("count"));
}

TEST_CASE("JobJson_UnfinishedOrFailedJob_HasNoResult", "[gui][jobs]") {
  // Only success produces a result. Dressing a failure up as an outcome would
  // invite a UI to render it as one; the reason belongs in `error`.
  reusex::pipeline::JobRecord record;
  record.id = "job-3";

  record.status = reusex::pipeline::JobStatus::queued;
  CHECK_FALSE(job_json(record, "scan.rux").contains("result"));

  record.status = reusex::pipeline::JobStatus::running;
  CHECK_FALSE(job_json(record, "scan.rux").contains("result"));

  record.status = reusex::pipeline::JobStatus::failed;
  record.error = "no input cloud";
  const auto failed = job_json(record, "scan.rux");
  CHECK_FALSE(failed.contains("result"));
  CHECK(failed.at("error") == "no input cloud");

  record.status = reusex::pipeline::JobStatus::cancelled;
  CHECK_FALSE(job_json(record, "scan.rux").contains("result"));
}

// ===========================================================================
// point / frame visibility (#453)
// ===========================================================================

namespace {

// Same synthetic scene the library test uses: a pinhole camera (fx=fy=100,
// principal point at the 128x128 centre, identity local transform) at four
// world poses, all looking down +z. Frame 1 sees the point (0,0,2) dead
// centre, frame 2 off to one side, frames 3 and 4 not at all.
void save_visibility_frame(reusex::ProjectDB &db, int id, double tx, double ty,
                           double tz) {
  const auto intr = reusex::test_support::make_intrinsics(
      /*fx=*/100.0, /*fy=*/100.0, /*cx=*/64.0, /*cy=*/64.0, /*w=*/128,
      /*h=*/128);
  const std::array<double, 16> pose{1, 0, 0, tx, 0, 1, 0, ty,
                                    0, 0, 1, tz, 0, 0, 0, 1};
  db.save_sensor_frame(id, reusex::test_support::make_color(128, 128),
                       cv::Mat(), cv::Mat(), pose, intr,
                       static_cast<double>(id), -1);
}

} // namespace

TEST_CASE("FramesVisibility_PointQuery_RanksByCentrality", "[gui][routes]") {
  const TempPath project("gui_visibility");
  {
    reusex::ProjectDB db(project.path);
    save_visibility_frame(db, 1, 0.0, 0.0, 0.0);
    save_visibility_frame(db, 2, 0.3, 0.0, 0.0);
    save_visibility_frame(db, 3, 0.0, 0.0, 5.0);
    save_visibility_frame(db, 4, 5.0, 0.0, 0.0);
  }
  reusex::ProjectDB db(project.path);

  SECTION("in-front, in-frustum frames come back best-first") {
    const auto body = frames_visibility_json(
        db, params_of({{"x", "0"}, {"y", "0"}, {"z", "2"}}));

    REQUIRE(body.at("total") == 2);
    REQUIRE(body.at("frames").size() == 2);
    CHECK(body["frames"][0].at("frame_id") == 1);
    CHECK(body["frames"][1].at("frame_id") == 2);
    // Frame 1 sees the point at the principal point.
    CHECK(body["frames"][0].at("centrality").get<double>() < 1e-9);
    CHECK(body["frames"][0].at("score").get<double>() ==
          Catch::Approx(1.0).margin(1e-9));
    CHECK(body["frames"][1].at("centrality").get<double>() >
          body["frames"][0].at("centrality").get<double>());
    CHECK(body.at("point") == json::array({0.0, 0.0, 2.0}));
  }

  SECTION("limit bounds the body but total reports the full count") {
    const auto body = frames_visibility_json(
        db, params_of({{"x", "0"}, {"y", "0"}, {"z", "2"}, {"limit", "1"}}));
    CHECK(body.at("total") == 2);
    CHECK(body.at("count") == 1);
    CHECK(body.at("frames").size() == 1);
    CHECK(body["frames"][0].at("frame_id") == 1);
  }

  SECTION("max_depth rejects frames beyond the range limit") {
    const auto body = frames_visibility_json(
        db,
        params_of({{"x", "0"}, {"y", "0"}, {"z", "2"}, {"max_depth", "1.5"}}));
    CHECK(body.at("total") == 0);
    CHECK(body.at("frames").empty());
  }

  SECTION("a missing coordinate is a 400") {
    CHECK_THROWS_AS(frames_visibility_json(db, params_of({{"x", "0"}})),
                    HttpError);
  }

  SECTION("a non-numeric coordinate is a 400") {
    CHECK_THROWS_AS(
        frames_visibility_json(
            db, params_of({{"x", "0"}, {"y", "0"}, {"z", "over-there"}})),
        HttpError);
  }
}

TEST_CASE("InstanceFrames_UnknownCloud_Is404", "[gui][routes]") {
  const TempPath project("gui_visibility_instance");
  reusex::ProjectDB db(project.path);
  CHECK_THROWS_AS(instance_frames_json(db, "nope", 1, Params{}), HttpError);
}

// ===========================================================================
// Pose-graph editor endpoints (#407)
// ===========================================================================

namespace {

/// Seed the pose_graph_edges table with a small graph suitable for editor
/// tests.  Uses save_pose_graph_edges to populate atomically, then we can
/// test add/delete on top.
void seed_pose_graph(reusex::ProjectDB &db) {
  reusex::ProjectDB::PoseGraphEdge e1;
  e1.from_node_id = 1;
  e1.to_node_id = 2;
  e1.edge_type = "odometry";
  e1.residual = 0.1;
  e1.weight = 10.0;

  reusex::ProjectDB::PoseGraphEdge e2;
  e2.from_node_id = 1;
  e2.to_node_id = 50;
  e2.edge_type = "loop_closure";
  e2.residual = 3.5;
  e2.weight = 1.0;

  db.save_pose_graph_edges({e1, e2});
}

} // namespace

TEST_CASE("DeletePoseGraphEdge_ExistingEdge_RemovesAndReports",
          "[gui][posegraph][editor]") {
  const TempPath project("gui_posegraph_delete");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  seed_pose_graph(db);

  // Delete the loop-closure edge.
  const auto body = delete_posegraph_edge(db, 1, 50, "loop_closure");
  CHECK(body.at("deleted") == 1);
  CHECK(body.at("from") == 1);
  CHECK(body.at("to") == 50);

  // Only the odometry edge should remain.
  const auto edges = db.list_pose_graph_edges();
  REQUIRE(edges.size() == 1);
  CHECK(edges[0].edge_type == "odometry");
}

TEST_CASE("DeletePoseGraphEdge_WithoutTypeFilter_DeletesAllMatchingPairs",
          "[gui][posegraph][editor]") {
  const TempPath project("gui_posegraph_delete_all");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);

  // Two edges between the same pair but different types.
  reusex::ProjectDB::PoseGraphEdge ea;
  ea.from_node_id = 5;
  ea.to_node_id = 10;
  ea.edge_type = "odometry";
  ea.residual = 0.0;
  ea.weight = 1.0;
  reusex::ProjectDB::PoseGraphEdge eb = ea;
  eb.edge_type = "loop_closure";
  db.save_pose_graph_edges({ea, eb});

  const auto body = delete_posegraph_edge(db, 5, 10);
  CHECK(body.at("deleted") == 2);
  CHECK(db.list_pose_graph_edges().empty());
}

TEST_CASE("DeletePoseGraphEdge_NoMatch_Is404", "[gui][posegraph][editor]") {
  const TempPath project("gui_posegraph_delete_404");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  seed_pose_graph(db);

  try {
    delete_posegraph_edge(db, 99, 100);
    FAIL("expected HttpError(404)");
  } catch (const HttpError &e) {
    CHECK(e.status() == 404);
  }
}

TEST_CASE("DeletePoseGraphEdge_UnknownType_Is400", "[gui][posegraph][editor]") {
  const TempPath project("gui_posegraph_delete_400");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  seed_pose_graph(db);

  try {
    delete_posegraph_edge(db, 1, 2, "unknown_type");
    FAIL("expected HttpError(400)");
  } catch (const HttpError &e) {
    CHECK(e.status() == 400);
  }
}

TEST_CASE("AddPoseGraphEdge_ValidBody_InsertsAndReturnsEdge",
          "[gui][posegraph][editor]") {
  const TempPath project("gui_posegraph_add");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  seed_pose_graph(db);

  const auto body = add_posegraph_edge(
      db, R"({"from":10,"to":20,"type":"loop_closure","weight":2.5})");

  CHECK(body.at("from") == 10);
  CHECK(body.at("to") == 20);
  CHECK(body.at("type") == "loop_closure");
  CHECK(body.at("residual") == 0.0);
  CHECK(body.at("weight") == 2.5);

  // Confirm it is in the DB.
  const auto edges = db.list_pose_graph_edges();
  REQUIRE(edges.size() == 3); // 2 seeded + 1 added
  const auto &added = edges.back();
  CHECK(added.from_node_id == 10);
  CHECK(added.to_node_id == 20);
  CHECK(added.edge_type == "loop_closure");
  CHECK(added.weight == 2.5);
}

TEST_CASE("AddPoseGraphEdge_DefaultType_IsLoopClosure",
          "[gui][posegraph][editor]") {
  const TempPath project("gui_posegraph_add_default");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  seed_pose_graph(db);

  const auto body = add_posegraph_edge(db, R"({"from":5,"to":95})");
  CHECK(body.at("type") == "loop_closure");
  CHECK(body.at("weight") == 1.0);
}

TEST_CASE("AddPoseGraphEdge_MissingFrom_Is400", "[gui][posegraph][editor]") {
  const TempPath project("gui_posegraph_add_400");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  seed_pose_graph(db);

  try {
    add_posegraph_edge(db, R"({"to":20})");
    FAIL("expected HttpError(400)");
  } catch (const HttpError &e) {
    CHECK(e.status() == 400);
  }
}

TEST_CASE("AddPoseGraphEdge_UnknownType_Is400", "[gui][posegraph][editor]") {
  const TempPath project("gui_posegraph_add_400_type");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  seed_pose_graph(db);

  try {
    add_posegraph_edge(db, R"({"from":1,"to":2,"type":"bad_type"})");
    FAIL("expected HttpError(400)");
  } catch (const HttpError &e) {
    CHECK(e.status() == 400);
  }
}

TEST_CASE("AddPoseGraphEdge_NoTable_Is409", "[gui][posegraph][editor]") {
  // A fresh DB has no pose_graph_edges table until rux optimize has run.
  const TempPath project("gui_posegraph_add_409");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);

  try {
    add_posegraph_edge(db, R"({"from":1,"to":2})");
    FAIL("expected HttpError(409)");
  } catch (const HttpError &e) {
    CHECK(e.status() == 409);
  }
}

// ===========================================================================
// POST /frames/<id>/segment — segment_frame_result_json (#409)
// ===========================================================================

TEST_CASE("SegmentFrameResultJson_WithLabelMap_ReportsLabeledPixels",
          "[gui][segment]") {
  // 4x4 label map: two labels in left/right halves, background (-1) in top row.
  cv::Mat label_map(4, 4, CV_32S, cv::Scalar(0));
  for (int c = 2; c < 4; ++c)
    for (int r = 0; r < 4; ++r)
      label_map.at<int>(r, c) = 1;
  for (int c = 0; c < 4; ++c)
    label_map.at<int>(0, c) = -1;

  const std::vector<std::string> names{"wall", "floor"};
  const auto body = segment_frame_result_json(42, label_map, names, true);

  CHECK(body.at("frame_id") == 42);
  CHECK(body.at("saved") == true);
  // 4x4=16 pixels, top row (-1) = 4 background pixels → 12 labeled.
  CHECK(body.at("labeled_pixels") == 12);
  REQUIRE(body.contains("labels"));
  CHECK(body.at("labels").at("0") == "wall");
  CHECK(body.at("labels").at("1") == "floor");
}

TEST_CASE("SegmentFrameResultJson_EmptyLabelMap_ZeroLabeledPixels",
          "[gui][segment]") {
  const auto body = segment_frame_result_json(
      7, cv::Mat{}, std::vector<std::string>{}, false);

  CHECK(body.at("frame_id") == 7);
  CHECK(body.at("saved") == false);
  CHECK(body.at("labeled_pixels") == 0);
  CHECK(body.at("labels").empty());
}

TEST_CASE("SegmentFrameResultJson_NoClassNames_EmptyLabelsObject",
          "[gui][segment]") {
  cv::Mat label_map(2, 2, CV_32S, cv::Scalar(-1));
  const auto body = segment_frame_result_json(1, label_map, {}, false);

  CHECK(body.at("labeled_pixels") == 0);
  CHECK(body.at("labels").is_object());
  CHECK(body.at("labels").empty());
}

// ===========================================================================
// POST /panoramas/<id>/segment — segment_panorama_result_json (#448)
// ===========================================================================

TEST_CASE("SegmentPanoramaResultJson_WithLabelMap_ReportsLabeledPixels",
          "[gui][segment][panorama]") {
  // 4x8 equirect label map: two classes, background in top row.
  cv::Mat label_map(4, 8, CV_32S, cv::Scalar(0));
  for (int c = 4; c < 8; ++c)
    for (int r = 0; r < 4; ++r)
      label_map.at<int>(r, c) = 1;
  for (int c = 0; c < 8; ++c)
    label_map.at<int>(0, c) = -1;

  const std::vector<std::string> names{"wall", "floor"};
  const auto body = segment_panorama_result_json(5, label_map, names, true);

  CHECK(body.at("pano_id") == 5);
  CHECK(body.at("saved") == true);
  // 4x8=32 pixels, top row (-1) = 8 background → 24 labeled.
  CHECK(body.at("labeled_pixels") == 24);
  REQUIRE(body.contains("labels"));
  CHECK(body.at("labels").at("0") == "wall");
  CHECK(body.at("labels").at("1") == "floor");
}

TEST_CASE("SegmentPanoramaResultJson_EmptyLabelMap_ZeroLabeledPixels",
          "[gui][segment][panorama]") {
  const auto body = segment_panorama_result_json(
      3, cv::Mat{}, std::vector<std::string>{}, false);

  CHECK(body.at("pano_id") == 3);
  CHECK(body.at("saved") == false);
  CHECK(body.at("labeled_pixels") == 0);
  CHECK(body.at("labels").empty());
}

TEST_CASE("SegmentPanoramaResultJson_NoClassNames_EmptyLabelsObject",
          "[gui][segment][panorama]") {
  cv::Mat label_map(2, 4, CV_32S, cv::Scalar(-1));
  const auto body = segment_panorama_result_json(7, label_map, {}, false);

  CHECK(body.at("labeled_pixels") == 0);
  CHECK(body.at("labels").is_object());
  CHECK(body.at("labels").empty());
}
