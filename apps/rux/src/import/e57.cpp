// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/e57.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/e57.hpp>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

void setup_subcommand_import_e57(CLI::App &app,
                                 std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandImportE57Options>();
  auto *sub = app.add_subcommand("e57", "Import point cloud from an E57 file");

  sub->footer(R"(
DESCRIPTION:
  Reads all scan positions from an E57 file and stores them as a single
  merged XYZRGB point cloud in the ReUseX project database. Color, normals,
  and intensity data are extracted when present.

CLOUD NAMING:
  Saved as "cloud" (and "normals" when every scan provides normals), matching
  the canonical names produced by the RTABMap import pipeline. Re-importing
  overwrites the existing entries.

EXAMPLES:
  rux import e57 scan.e57                      # Import to ./project.rux
  rux -p office.rux import e57 building.e57    # Custom project path

NOTES:
  - Intensity-only scans have intensity mapped to grayscale RGB
  - Scan pose transforms are applied so all scans share a common coordinate system
  - Use 'rux view' to inspect the imported point clouds
)");

  sub->add_option("file", opt->input_path, "Path to the E57 file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_e57");
    return run_subcommand_import_e57(*opt, *global_opt);
  });
}

int run_subcommand_import_e57(SubcommandImportE57Options const &opt,
                              const RuxOptions &global_opt) {

  fs::path project_path = global_opt.project_db;
  spdlog::info("Importing E57 file to project: {}", project_path.string());

  reusex::ProjectDB project_db(project_path);
  reusex::io::import_e57(project_db, opt.input_path);

  spdlog::info("E57 import complete. Use 'rux view' to inspect results.");

  return RuxError::SUCCESS;
}
