// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/rtabmap.hpp"
#include "import.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/rtabmap.hpp>

#include <spdlog/spdlog.h>

void setup_subcommand_import_rtabmap(CLI::App &app, ImportContext &ctx) {
  auto opt = std::make_shared<SubcommandImportRTABMapOptions>();
  auto *sub =
      app.add_subcommand("rtabmap", "Import raw sensor data from an RTABMap "
                                    "database into a ReUseX project.");

  sub->add_option("database", opt->database_path_in,
                  "Path to the RTABMap database file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("--project", opt->project,
                  "Path to the output .rux project database.")
      ->default_val(opt->project);

  sub->callback([opt, &ctx]() {
    spdlog::trace("calling run_subcommand_import_rtabmap");
    return run_subcommand_import_rtabmap(*opt, ctx);
  });
}

int run_subcommand_import_rtabmap(SubcommandImportRTABMapOptions const &opt,
                                  ImportContext & /*ctx*/) {
  spdlog::info("Importing RTABMap database to project: {}",
               opt.project.string());

  ReUseX::ProjectDB projectDb(opt.project);
  ReUseX::io::import_rtabmap(projectDb, opt.database_path_in);

  spdlog::info("Import complete. Use 'rux create clouds {}' to generate "
               "point clouds.",
               opt.project.string());
  return RuxError::SUCCESS;
}
