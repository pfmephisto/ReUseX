// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd's injected stage executor runs `optimize` through the shared library
// reader (pipeline/optimize_stage.hpp, slam/optimize_parameters.hpp), so a
// web-GUI run reads its parameters exactly like `rux optimize` and the Qt
// client. A wrong-typed key is a 400-style invalid input, not an exception
// escaping into the job runner (the old ruxd-local reader let it through).
//
// Lives in tests/unit/ruxd/ → reusex_unit_tests_vision (links ruxd_lib).

#include <catch2/catch_test_macros.hpp>

#include "injected.hpp"

#include <reusex/core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

using reusex::test_support::TempPath;

TEST_CASE("RuxdStageExecutor_Optimize_UsesTheSharedReader",
          "[ruxd][optimize_stage]") {
  TempPath project("test_ruxd_stage_executor");
  {
    reusex::ProjectDB db(project.path);
  }

  const auto exec = ruxd::make_stage_executor();
  reusex::pipeline::StageContext ctx;
  ctx.project = project.path;
  ctx.stage = reusex::pipeline::JobStage::optimize;
  ctx.parameters = R"({"min_observations":"seven"})";

  reusex::pipeline::StageResult r;
  REQUIRE_NOTHROW(r = exec(ctx));
  CHECK_FALSE(r.ok);
  CHECK(r.invalid_input);
  CHECK(r.message.find("min_observations") != std::string::npos);
}
