// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests for the DB-level stage runner (#265, Phase 1).
//
// These use a real (empty) ProjectDB and assert the failure path — that a stage
// whose input contract is unmet is refused with a reason AND leaves a durable
// pipeline_log record. Running a stage to completion needs real scan data and
// belongs in tests/integration/.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include <nlohmann/json.hpp>
#include <pipeline/stages.hpp>

#include "../../support/temp_path.hpp"

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace reusex::pipeline;

using reusex::test_support::TempPath;

TEST_CASE("A stage with unmet inputs is refused with the reason",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes;

  const auto result = run_stage(db, ctx);

  CHECK_FALSE(result.ok);
  CHECK_FALSE(result.cancelled);
  // The message must name what is missing, not just say "failed".
  CHECK(result.message.find("cloud") != std::string::npos);
  CHECK(result.message.find("normals") != std::string::npos);
}

TEST_CASE("A refused stage still lands in pipeline_log", "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::rooms;
  ctx.parameters = R"({"resolution":1.5})";

  REQUIRE_FALSE(run_stage(db, ctx).ok);

  // The GUI history view reads pipeline_log; an attempt that vanished without
  // trace would be a silent failure (STANDARDS §5).
  const auto log = db.pipeline_log();
  REQUIRE(log.size() == 1);
  // The CLI-compatible name (S8 from the #274 review): the GUI and CLI paths
  // must not fill one table with two names for the same operation.
  CHECK(log.front().stage == "segment_rooms");
  CHECK(log.front().status == "failed");
  CHECK_FALSE(log.front().error_msg.empty());
  CHECK_FALSE(log.front().finished_at.empty());
  // The submitted parameters are persisted verbatim, so a run is reproducible.
  CHECK(log.front().parameters == R"({"resolution":1.5})");
}

TEST_CASE("Malformed stage parameters fail without running anything",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes;
  ctx.parameters = "definitely not json";

  const auto result = run_stage(db, ctx);
  CHECK_FALSE(result.ok);
  CHECK_FALSE(result.message.empty());
  // Parsing happens before the log row is opened, so nothing is recorded.
  CHECK(db.pipeline_log().empty());
}

TEST_CASE("run_stage reports an unopenable project instead of throwing",
          "[pipeline][stages]") {
  StageContext ctx;
  ctx.project = "/definitely/not/a/directory/project.rux";
  ctx.stage = JobStage::planes;

  const auto result = run_stage(ctx);
  CHECK_FALSE(result.ok);
  CHECK(result.message.find("could not open project") != std::string::npos);
}

TEST_CASE("An already-cancelled context short-circuits a cancellable stage",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);

  std::atomic_bool cancelled{true};
  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes;
  ctx.cancel_token = &cancelled;

  // Inputs are unmet, so this still fails — the point is that a cancel token
  // never turns a refusal into a crash, and the token is observed.
  const auto result = run_stage(db, ctx);
  CHECK_FALSE(result.ok);
  CHECK(ctx.is_cancelled());
}

TEST_CASE("The default executor is callable and honours the context",
          "[pipeline][stages]") {
  auto executor = default_stage_executor();
  REQUIRE(static_cast<bool>(executor));

  StageContext ctx;
  ctx.project = "/definitely/not/a/directory/project.rux";
  ctx.stage = JobStage::clouds;

  const auto result = executor(ctx);
  CHECK_FALSE(result.ok);
  CHECK_FALSE(result.message.empty());
}

TEST_CASE("StageResult factories set the flags they claim",
          "[pipeline][stages]") {
  const auto ok = StageResult::success("done");
  CHECK(ok.ok);
  CHECK_FALSE(ok.cancelled);
  CHECK(ok.message == "done");

  const auto bad = StageResult::failure("nope");
  CHECK_FALSE(bad.ok);
  CHECK_FALSE(bad.cancelled);
  CHECK(bad.message == "nope");

  // A cancellation is NOT a success — a caller that only checks `ok` must not
  // mistake a half-finished stage for a completed one.
  const auto stopped = StageResult::cancel();
  CHECK_FALSE(stopped.ok);
  CHECK(stopped.cancelled);
  CHECK(stopped.message == "cancelled");
}

// ===========================================================================
// Review follow-ups (#274)
// ===========================================================================

TEST_CASE("pipeline_log uses the same stage names as the CLI",
          "[pipeline][stages]") {
  // The GUI and the CLI write into ONE pipeline_log table. Logging "rooms"
  // from one path and "segment_rooms" from the other would silently split a
  // project's history in two, and `rux log` would show neither the whole
  // story. The CLI's names win because existing databases are full of them.
  CHECK(pipeline_log_name(JobStage::clouds) == "cloud_reconstruction");
  CHECK(pipeline_log_name(JobStage::planes) == "segment_planes");
  CHECK(pipeline_log_name(JobStage::rooms) == "segment_rooms");
  CHECK(pipeline_log_name(JobStage::instances) == "segment_instances");

  // The wire token stays short and is deliberately NOT the log name.
  CHECK(to_string(JobStage::rooms) == "rooms");
}

TEST_CASE("A stage run records the CLI-compatible name and its job id",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes;
  ctx.parameters = R"({"radius":0.5})";
  ctx.job_id = "job-abc-123";

  REQUIRE_FALSE(run_stage(db, ctx).ok);

  const auto log = db.pipeline_log();
  REQUIRE(log.size() == 1);
  CHECK(log.front().stage == "segment_planes");

  // The job id is folded into the stored parameters, so a client reconnecting
  // after a restart can join its old job ids back to what happened to them.
  const auto params = nlohmann::json::parse(log.front().parameters);
  CHECK(params.at("job_id") == "job-abc-123");
  CHECK(params.at("radius") == 0.5);
}

TEST_CASE("A direct (non-job) run logs no job id", "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::rooms;
  ctx.parameters = R"({"resolution":1.5})";

  REQUIRE_FALSE(run_stage(db, ctx).ok);

  const auto log = db.pipeline_log();
  REQUIRE(log.size() == 1);
  const auto params = nlohmann::json::parse(log.front().parameters);
  CHECK_FALSE(params.contains("job_id"));
  CHECK(params.at("resolution") == 1.5);
}
