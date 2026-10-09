// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Smoke test for the installed ReUseX package: open (create) a project and run
// one pipeline stage through the public API. Usage: consumer <project.rux>
#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/version.hpp>
#include <reusex/pipeline/stages.hpp>

#include <cstdio>
#include <filesystem>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <project.rux>\n", argv[0]);
    return 2;
  }
  const std::filesystem::path project = argv[1];
  std::filesystem::remove(project);

  reusex::ProjectDB db(project, /*readOnly=*/false);
  if (!db.is_open() ||
      db.schema_version() != reusex::ProjectDB::latest_schema_version()) {
    std::fprintf(stderr, "ProjectDB did not open at the latest schema\n");
    return 1;
  }

  // An empty project has no cloud, so the planes stage must refuse cleanly
  // (StageResult, never an exception) — this links the whole planes path.
  reusex::pipeline::StageContext ctx;
  ctx.project = project;
  ctx.stage = reusex::pipeline::JobStage::planes;
  const auto result = reusex::pipeline::run_stage(db, ctx);
  if (result.ok || result.message.empty()) {
    std::fprintf(stderr,
                 "planes on an empty project should fail with a "
                 "reason (ok=%d)\n",
                 result.ok);
    return 1;
  }

  std::printf("ReUseX %s package OK: schema v%d, stage '%.*s' -> \"%s\"\n",
              reusex::core::VERSION, db.schema_version(),
              static_cast<int>(reusex::pipeline::to_string(ctx.stage).size()),
              reusex::pipeline::to_string(ctx.stage).data(),
              result.message.c_str());
  return 0;
}
