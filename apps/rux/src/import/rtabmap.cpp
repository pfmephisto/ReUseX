// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/rtabmap.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/rtabmap.hpp>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

void setup_subcommand_import_rtabmap(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandImportRTABMapOptions>();
  auto *sub =
      app.add_subcommand("rtabmap", "Import sensor data from RTABMap database");

  sub->footer(R"(
DESCRIPTION:
  Extracts raw sensor data (RGB images, depth maps, camera poses, intrinsics)
  from an RTABMap SLAM database and stores it in a ReUseX project for
  further processing. This is the first step in the typical reconstruction
  pipeline. No point cloud reconstruction is performed during import.

EXAMPLES:
  rux import rtabmap scan.db           # Import to ./project.rux
  rux -p office.rux import rtabmap scan.db  # Custom project path
  rux import rtabmap ~/scans/building.db    # Absolute path

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor frames
  2. rux create clouds                 # Reconstruct point clouds
  3. rux create annotate --net model   # Optional: ML inference
  4. rux view                          # Visualize results

NOTES:
  - Requires RTABMap database (.db) from RTABMap SLAM application
  - Extracts: RGB images, depth maps, confidence maps, camera poses
  - Camera intrinsics and local transforms preserved from RTABMap
  - Images stored in original orientation (no preprocessing)
  - Use global -p/--project flag to specify output project path
  - After import, run 'rux create clouds' to generate point clouds
)");

  sub->add_option("database", opt->database_path_in,
                  "Path to the RTABMap database file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_rtabmap");
    return run_subcommand_import_rtabmap(*opt, *global_opt);
  });
}

int run_subcommand_import_rtabmap(SubcommandImportRTABMapOptions const &opt, const RuxOptions &global_opt) {

  fs::path project_path = global_opt.project_db;
  spdlog::info("Importing RTABMap database to project: {}",
               project_path.string());

  ReUseX::ProjectDB project_db(project_path);
  ReUseX::io::import_rtabmap(project_db, opt.database_path_in);

  int logId = project_db.log_pipeline_start(
      "import", fmt::format(R"({{"Import ":{}}})", opt.database_path_in));
  spdlog::trace("logId: {}", logId);

  spdlog::info("Import complete. Use 'rux create clouds' to generate "
               "point clouds.");

  return RuxError::SUCCESS;
}
