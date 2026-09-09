// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Contract tests for the `rux gui` API surface (#265, Phase 1).
//
// These cover the parts of docs/gui/openapi.yaml that are verifiable without a
// browser or a socket: the route table, request-parameter parsing, job
// submission validation, the WebSocket message protocol, and the JSON shape of
// the read endpoints against a real (empty and populated) ProjectDB.

#include <catch2/catch_test_macros.hpp>

#include <gui/api.hpp>
#include <gui/assets.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>
#include <pipeline/JobRunner.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
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

void write_file(const fs::path &path, std::string_view content) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path, std::ios::binary);
  out << content;
}

/// The project name the server would report for a temp project.
std::string name_of(const fs::path &project) {
  return project.filename().string();
}

} // namespace

// ===========================================================================
// Route table
// ===========================================================================

TEST_CASE("The route table matches the documented contract", "[gui][routes]") {
  const auto &table = endpoint_table();
  REQUIRE_FALSE(table.empty());

  // The exact set documented in docs/gui/openapi.yaml. Adding a route without
  // documenting it must fail here — that is the point of this test.
  const std::set<std::string> expected{
      "GET /api/v1/health",
      "GET /api/v1/endpoints",
      "GET /api/v1/project",
      "GET /api/v1/projects",
      "GET /api/v1/clouds",
      "GET /api/v1/clouds/<string>",
      "GET /api/v1/clouds/<string>/points",
      "GET /api/v1/meshes",
      "GET /api/v1/meshes/<string>",
      "GET /api/v1/meshes/<string>/data",
      "GET /api/v1/meshes/<string>/textures",
      "GET /api/v1/meshes/<string>/textures/<string>",
      "GET /api/v1/frames",
      "GET /api/v1/frames/<int>",
      "GET /api/v1/frames/<int>/image",
      "GET /api/v1/panoramas",
      "GET /api/v1/panoramas/<int>",
      "GET /api/v1/panoramas/<int>/image",
      "GET /api/v1/components",
      "GET /api/v1/components/<string>",
      "GET /api/v1/materials",
      "GET /api/v1/materials/<string>",
      "GET /api/v1/instances/<string>",
      "GET /api/v1/stages",
      "GET /api/v1/stages/<string>/validation",
      "GET /api/v1/pipeline-log",
      "GET /api/v1/jobs",
      "POST /api/v1/jobs",
      "GET /api/v1/jobs/<string>",
      "POST /api/v1/jobs/<string>/cancel",
      "GET /api/v1/events",
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

TEST_CASE("/endpoints reports the same table it registers", "[gui][routes]") {
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

TEST_CASE("Params reads strings and integers with defaults", "[gui][params]") {
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

TEST_CASE("A non-integer query parameter is a 400, not a silent zero",
          "[gui][params]") {
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

TEST_CASE("error_json carries a message and the status", "[gui][errors]") {
  const auto body = error_json(404, "no such cloud 'nope'");
  CHECK(body.at("status") == 404);
  CHECK(body.at("error") == "no such cloud 'nope'");
}

// ===========================================================================
// Read endpoints over a real ProjectDB
// ===========================================================================

TEST_CASE("health_json reports the project without leaking its path",
          "[gui][project]") {
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

TEST_CASE("health_json degrades gracefully when the project cannot be opened",
          "[gui][project]") {
  const auto body = health_json(nullptr, "/somewhere/broken.rux");
  CHECK(body.at("status") == "ok");
  CHECK(body.at("project").at("open") == false);
  CHECK(body.at("project").at("name") == "broken.rux");
  CHECK_FALSE(body.at("project").contains("schema_version"));
}

TEST_CASE("An empty project serializes to well-formed, empty collections",
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

  CHECK(clouds_json(db).at("clouds").empty());
  CHECK(meshes_json(db).at("meshes").empty());
  CHECK(frames_json(db).at("ids").empty());
  CHECK(panoramas_json(db).at("panoramas").empty());
  CHECK(materials_json(db).at("materials").empty());
  CHECK(pipeline_log_json(db, Params{}).at("entries").empty());
}

TEST_CASE("Project metadata is exposed under /projects", "[gui][project]") {
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

  const auto body = projects_json(db);
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

TEST_CASE("Clouds are listed with type and point count", "[gui][clouds]") {
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

  const auto listed = clouds_json(db);
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

TEST_CASE("Point pages honour offset, limit and field order", "[gui][clouds]") {
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

  SECTION("the binary format is refused with 501 until Phase 2") {
    Params binary;
    binary.set("format", "binary");
    try {
      cloud_points_json(db, "cloud", binary);
      FAIL("expected HttpError");
    } catch (const HttpError &e) {
      CHECK(e.status() == 501);
    }
  }

  SECTION("an unknown format is a 400") {
    Params bogus;
    bogus.set("format", "protobuf");
    try {
      cloud_points_json(db, "cloud", bogus);
      FAIL("expected HttpError");
    } catch (const HttpError &e) {
      CHECK(e.status() == 400);
    }
  }
}

TEST_CASE("Label clouds expose their label definitions", "[gui][clouds]") {
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

TEST_CASE("The pipeline log round-trips through the API shape",
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

TEST_CASE("The stage catalogue separates runnability from readiness",
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
    // `mesh` is documented but has no runner yet.
    CHECK(stage.at("runnable").get<bool>() == (name != "mesh"));
    // clouds runs a stage that cannot be interrupted mid-run.
    if (name == "clouds")
      CHECK(stage.at("cancellable") == false);
    if (name == "planes" || name == "rooms" || name == "instances")
      CHECK(stage.at("cancellable") == true);
  }
}

TEST_CASE("Every stage carries its contract, hints and parameter schema",
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
    // A stage with no runner takes no parameters; a runnable one always has
    // knobs, and every knob must be renderable without guessing.
    CHECK(stage.at("parameters").empty() == (name == "mesh"));
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

TEST_CASE("Per-stage validation returns the same record as the catalogue",
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

TEST_CASE("Missing resources are 404s with a useful message", "[gui][errors]") {
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
  expect_404([&] { return instances_json(db, "nope"); });
}

TEST_CASE("Instance rows carry their stable GUID and material link",
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

  const auto body = instances_json(db, "instances");
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

// ===========================================================================
// Jobs
// ===========================================================================

TEST_CASE("A job submission body is validated before anything is queued",
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

  SECTION("a documented but non-runnable stage is refused with guidance") {
    try {
      parse_job_request(R"({"stage":"mesh"})");
      FAIL("expected HttpError");
    } catch (const HttpError &e) {
      CHECK(e.status() == 400);
      const std::string message = e.what();
      CHECK(message.find("mesh") != std::string::npos);
      // The error must list what IS runnable, not just say no.
      CHECK(message.find("planes") != std::string::npos);
    }
  }
}

TEST_CASE("A job serializes into the documented shape", "[gui][jobs]") {
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

TEST_CASE("Indeterminate progress reports a null fraction", "[gui][jobs]") {
  reusex::pipeline::JobRecord record;
  record.id = "job-2";
  record.progress_total = 0;
  record.progress_current = 0;

  const auto body = job_json(record, "scan.rux");
  CHECK(body.at("progress").at("total") == 0);
  CHECK(body.at("progress").at("fraction").is_null());
}

TEST_CASE("Unparseable stored parameters degrade to an empty object",
          "[gui][jobs]") {
  reusex::pipeline::JobRecord record;
  record.id = "job-3";
  record.parameters = "this is not json";

  const auto body = job_json(record, "scan.rux");
  REQUIRE(body.at("parameters").is_object());
  CHECK(body.at("parameters").empty());
}

TEST_CASE("Job events wrap the full record", "[gui][jobs]") {
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

TEST_CASE("The hello frame carries a job snapshot and no server path",
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

TEST_CASE("WebSocket client messages follow the documented protocol",
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

TEST_CASE("Subscriptions filter events by job id", "[gui][websocket]") {
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

TEST_CASE("The placeholder page is a complete, self-describing document",
          "[gui][assets]") {
  const auto page = placeholder_page("scan.rux");
  CHECK(page.rfind("<!doctype html>", 0) == 0);
  CHECK(page.find("scan.rux") != std::string::npos);
  CHECK(page.find("/api/v1") != std::string::npos);
  // The sentinel must be fully substituted, not left in the output.
  CHECK(page.find("%%PROJECT%%") == std::string::npos);
}

TEST_CASE("Asset resolution serves files under the root and nothing else",
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

TEST_CASE("MIME types cover the frontend bundle's file kinds",
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

TEST_CASE("An explicit --assets that is not a directory fails loudly",
          "[gui][assets]") {
  REQUIRE_THROWS_AS(resolve_asset_dir("/definitely/not/a/directory"),
                    std::runtime_error);
}

// ===========================================================================
// Review follow-ups (#274): project identity, limit clamping, SPA fallback
// ===========================================================================

TEST_CASE("Every job and event names the project it belongs to",
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

TEST_CASE("A job aimed at a different project is refused", "[gui][jobs]") {
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

TEST_CASE("Events carry a monotonic sequence number", "[gui][websocket]") {
  // Events are published without the runner lock, so arrival order is not
  // emission order. `seq` is assigned under the lock and is the authority.
  reusex::pipeline::JobEvent event;
  event.sequence = 42;
  CHECK(job_event_json(event, "scan.rux").at("seq") == 42);
}

TEST_CASE("pipeline-log limit is clamped at both ends", "[gui][pipeline]") {
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

TEST_CASE("The SPA fallback applies to routes, not to missing files",
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

TEST_CASE("Percent-encoded traversal is decoded before it is judged",
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
