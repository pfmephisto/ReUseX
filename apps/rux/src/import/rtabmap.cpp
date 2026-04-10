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
      app.add_subcommand("rtabmap", "Import raw sensor data from an RTABMap "
                                    "database into a ReUseX project.");

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
