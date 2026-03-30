// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/rtabmap.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/rtabmap.hpp>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

void setup_subcommand_import_rtabmap(CLI::App &app) {

  auto opt = std::make_shared<SubcommandImportRTABMapOptions>();
  auto *sub =
      app.add_subcommand("rtabmap", "Import raw sensor data from an RTABMap "
                                    "database into a ReUseX project.");

  sub->add_option("database", opt->database_path_in,
                  "Path to the RTABMap database file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("project", opt->project_path_out,
                  "Path to the output .rux project database.")
      ->default_val(opt->project_path_out);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_import_rtabmap");
    return run_subcommand_import_rtabmap(*opt);
  });
}

int run_subcommand_import_rtabmap(SubcommandImportRTABMapOptions const &opt) {

  spdlog::info("Importing RTABMap database to project: {}",
               opt.project_path_out.string());

  ReUseX::ProjectDB project_db(opt.project_path_out);
  ReUseX::io::import_rtabmap(project_db, opt.database_path_in);

  int logId = project_db.log_pipeline_start(
      "import", fmt::format(R"({{"Import ":{}}})", opt.database_path_in));
  spdlog::trace("logId: {}", logId);

  spdlog::info("Import complete. Use 'rux create clouds {}' to generate "
               "point clouds.",
               opt.project_path_out);

  return RuxError::SUCCESS;
}
