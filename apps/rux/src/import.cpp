// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import.hpp"
#include "import/materialepas.hpp"
#include "import/rtabmap.hpp"
#include <spdlog/spdlog.h>

void setup_subcommand_import(CLI::App &app) {

  auto *sub = app.add_subcommand(
      "import", "Import data from various sources into point cloud format.");

  setup_subcommand_import_rtabmap(*sub);
  setup_subcommand_import_materialepas(*sub);

  sub->callback([]() {
    spdlog::trace("calling import subcommand");
    return run_subcommand_import();
  });
}

int run_subcommand_import() {
  spdlog::trace("running import subcommand");
  return RuxError::SUCCESS;
}
