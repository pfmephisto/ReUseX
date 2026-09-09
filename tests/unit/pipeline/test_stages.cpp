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
#include <core/processing_observer.hpp>

#include <nlohmann/json.hpp>
#include <pipeline/stages.hpp>
#include <types/point_types.hpp>

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

TEST_CASE("RunStage_UnmetInputs_RefusedWithReason", "[pipeline][stages]") {
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

TEST_CASE("RunStage_RefusedStage_RecordsPipelineLogEntry",
          "[pipeline][stages]") {
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

TEST_CASE("RunStage_MalformedParameters_FailsWithoutRunning",
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

TEST_CASE("RunStage_UnopenableProject_ReportsErrorInsteadOfThrowing",
          "[pipeline][stages]") {
  StageContext ctx;
  ctx.project = "/definitely/not/a/directory/project.rux";
  ctx.stage = JobStage::planes;

  const auto result = run_stage(ctx);
  CHECK_FALSE(result.ok);
  CHECK(result.message.find("could not open project") != std::string::npos);
}

TEST_CASE("RunStage_AlreadyCancelledContext_ShortCircuitsCancellableStage",
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

TEST_CASE("DefaultStageExecutor_UnopenableProject_ReturnsFailureResult",
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

TEST_CASE("StageResult_Factories_SetClaimedFlags", "[pipeline][stages]") {
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

TEST_CASE("PipelineLogName_RunnableStages_MatchesCliNames",
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

TEST_CASE("RunStage_JobRun_RecordsCliNameAndJobId", "[pipeline][stages]") {
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

TEST_CASE("RunStage_DirectRun_LogsNoJobId", "[pipeline][stages]") {
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

// ===========================================================================
// #284 — CLI convergence
//
// The `rux create <stage>` subcommands no longer carry their own copies of
// these stage bodies, so everything they used to do themselves — evaluating a
// -f filter expression, distinguishing "your input was bad" from "it broke
// mid-run" — has to hold here instead.
// ===========================================================================

namespace {

/// Seed a project with the minimum a segmentation stage needs: a small planar
/// patch, its normals, and a semantic label cloud, all index-aligned
/// (STANDARDS §3.2).
void seed_segmentable_project(reusex::ProjectDB &db, int side = 20) {
  reusex::Cloud cloud;
  reusex::CloudN normals;
  reusex::CloudL labels;

  for (int y = 0; y < side; ++y) {
    for (int x = 0; x < side; ++x) {
      reusex::PointT point;
      point.x = 0.02F * static_cast<float>(x);
      point.y = 0.02F * static_cast<float>(y);
      point.z = 0.0F;
      point.r = point.g = point.b = 200;
      cloud.push_back(point);

      reusex::NormalT normal;
      normal.normal_x = 0.0F;
      normal.normal_y = 0.0F;
      normal.normal_z = 1.0F;
      normal.curvature = 0.0F;
      normals.push_back(normal);

      // Two semantic classes split down the middle, so an instance run has
      // something to cluster and a filter has something to select.
      reusex::LabelT label;
      label.label = (x < side / 2) ? 1U : 2U;
      labels.push_back(label);
    }
  }

  db.save_point_cloud("cloud", cloud, "test");
  db.save_point_cloud("normals", normals, "test");
  db.save_point_cloud("labels", labels, "test");
}

/// Records whether the global progress hooks were driven.
class RecordingObserver : public reusex::core::IProgressObserver {
    public:
  int started = 0;
  int finished = 0;

  void on_process_started(reusex::core::Stage, size_t) override { ++started; }
  void on_process_finished(reusex::core::Stage) override { ++finished; }
};

} // namespace

TEST_CASE("RunStage_RefusedStage_ReportsInvalidInputNotFailure",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes;

  const auto result = run_stage(db, ctx);

  // `rux` maps this to INVALID_ARGUMENT; collapsing it into a generic failure
  // would change the exit code the CLI has always returned for an unusable
  // request (#284).
  CHECK_FALSE(result.ok);
  CHECK(result.invalid_input);
  CHECK_FALSE(result.cancelled);
}

TEST_CASE("RunStage_MalformedParameters_ReportsInvalidInput",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes;
  ctx.parameters = "definitely not json";

  const auto result = run_stage(db, ctx);
  CHECK_FALSE(result.ok);
  CHECK(result.invalid_input);
}

TEST_CASE("StageResult_Invalid_SetsOnlyInvalidInputFlag",
          "[pipeline][stages]") {
  const auto bad = StageResult::invalid("bad filter");
  CHECK_FALSE(bad.ok);
  CHECK(bad.invalid_input);
  CHECK_FALSE(bad.cancelled);
  CHECK(bad.message == "bad filter");

  // The other factories must not claim it.
  CHECK_FALSE(StageResult::success("done").invalid_input);
  CHECK_FALSE(StageResult::failure("nope").invalid_input);
  CHECK_FALSE(StageResult::cancel().invalid_input);
}

TEST_CASE("RunStage_SeededPlanesStage_RunsToCompletionAndWritesOutputs",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);
  seed_segmentable_project(db);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes;
  // Pin the thresholds: adaptive derivation measures noise, and this patch is
  // synthetically noise-free.
  ctx.parameters = R"({"adaptive":false,"min_inliers":10})";

  const auto result = run_stage(db, ctx);

  INFO(result.message);
  REQUIRE(result.ok);
  CHECK_FALSE(result.invalid_input);
  CHECK(db.has_point_cloud("planes"));
  CHECK(db.has_point_cloud("plane_centroids"));
  CHECK(db.has_point_cloud("plane_normals"));

  const auto log = db.pipeline_log();
  REQUIRE(log.size() == 1);
  CHECK(log.front().stage == "segment_planes");
  CHECK(log.front().status == "success");
}

TEST_CASE("RunStage_FilterExpression_RestrictsStageAndIsRecorded",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);
  seed_segmentable_project(db);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes;
  ctx.parameters =
      R"({"adaptive":false,"min_inliers":10,"filter":"labels == 1"})";

  const auto result = run_stage(db, ctx);
  INFO(result.message);
  REQUIRE(result.ok);

  // The filter that produced this result is durable, so the run is
  // reproducible from its own history. The CLI's copy never recorded it.
  const auto log = db.pipeline_log();
  REQUIRE(log.size() == 1);
  const auto params = nlohmann::json::parse(log.front().parameters);
  CHECK(params.at("filter") == "labels == 1");
}

TEST_CASE("RunStage_UnusableFilterExpression_RefusedAsInvalidInput",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);
  seed_segmentable_project(db);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes;

  SECTION("a cloud the project does not have") {
    ctx.parameters = R"({"adaptive":false,"filter":"nosuchcloud == 1"})";
  }
  SECTION("syntactic nonsense") {
    ctx.parameters = R"({"adaptive":false,"filter":"labels >>>< "})";
  }

  const auto result = run_stage(db, ctx);

  CHECK_FALSE(result.ok);
  // This is the exit code the subcommand used to return by catching the
  // exception itself; it must survive the move into the library.
  CHECK(result.invalid_input);
  CHECK(result.message.find("filter") != std::string::npos);

  // The refusal is still durable, and it does NOT leave a half-written result.
  CHECK_FALSE(db.has_point_cloud("planes"));
  const auto log = db.pipeline_log();
  REQUIRE(log.size() == 1);
  CHECK(log.front().status == "failed");
}

TEST_CASE("RunStage_FilterOverMisSizedCloud_RefusedRatherThanMisapplied",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);
  seed_segmentable_project(db);

  // A label cloud that does not line up with "cloud": indices drawn from it
  // would silently address the wrong points.
  reusex::CloudL shorter;
  for (int i = 0; i < 10; ++i) {
    reusex::LabelT label;
    label.label = 1U;
    shorter.push_back(label);
  }
  db.save_point_cloud("stale", shorter, "test");

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::planes;
  ctx.parameters = R"({"adaptive":false,"filter":"stale == 1"})";

  const auto result = run_stage(db, ctx);
  CHECK_FALSE(result.ok);
  CHECK(result.invalid_input);
}

TEST_CASE("RunStage_SeededInstancesStage_DrivesGlobalProgressObserver",
          "[pipeline][stages]") {
  TempPath project("test_pipeline_stages");
  reusex::ProjectDB db(project.path);
  seed_segmentable_project(db);

  // `rux` installs its progress bar as a global observer once at startup and
  // never passes it to a stage. Converging the CLI onto run_stage is only safe
  // if the stage bodies still reach that global (#284).
  RecordingObserver observer;
  reusex::core::set_progress_observer(&observer);

  StageContext ctx;
  ctx.project = project.path;
  ctx.stage = JobStage::instances;
  ctx.parameters = R"({"min_cluster_size":1,"cluster_tolerance":0.05})";

  const auto result = run_stage(db, ctx);
  reusex::core::reset_progress_observer();

  INFO(result.message);
  REQUIRE(result.ok);
  CHECK(observer.started > 0);
  CHECK(observer.finished > 0);
}
