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
#include <pipeline/stages.hpp>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace reusex::pipeline;

namespace {

/// A temp-path name unique across both objects AND processes.
///
/// `ctest -j` runs every test case in its own process, and `this` lands at the
/// same stack address in each of them — an address-only name collides. The pid
/// is what makes concurrent runs disjoint.
inline std::string unique_name(std::string_view prefix, const void *self) {
  return std::string(prefix) + std::to_string(::getpid()) + "_" +
         std::to_string(reinterpret_cast<uintptr_t>(self));
}

struct TempProject {
  fs::path path;
  TempProject()
      : path(fs::temp_directory_path() /
             (unique_name("test_pipeline_stages_", this) + ".rux")) {}
  ~TempProject() noexcept {
    std::error_code ec;
    fs::remove(path, ec);
    if (ec)
      std::cerr << "Warning: could not remove " << path << ": " << ec.message()
                << std::endl;
  }
};

} // namespace

TEST_CASE("A stage with unmet inputs is refused with the reason",
          "[pipeline][stages]") {
  TempProject project;
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
  TempProject project;
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
  CHECK(log.front().stage == "rooms");
  CHECK(log.front().status == "failed");
  CHECK_FALSE(log.front().error_msg.empty());
  CHECK_FALSE(log.front().finished_at.empty());
  // The submitted parameters are persisted verbatim, so a run is reproducible.
  CHECK(log.front().parameters == R"({"resolution":1.5})");
}

TEST_CASE("Malformed stage parameters fail without running anything",
          "[pipeline][stages]") {
  TempProject project;
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
  TempProject project;
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
