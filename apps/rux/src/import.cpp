// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import.hpp"
#include "import/e57.hpp"
#include "import/materialepas.hpp"
#include "import/panorama.hpp"
#include "import/photos.hpp"
#include "import/ply.hpp"
#include "import/rtabmap.hpp"
#include <spdlog/spdlog.h>

void setup_subcommand_import(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {

  auto *sub = app.add_subcommand(
      "import", "Import data from external sources");

  sub->footer(R"(
DESCRIPTION:
  Parent command for importing data from external sources into ReUseX
  project databases. Supports RTABMap SLAM databases, E57 and PLY point
  clouds, and material passport JSON files.

SUBCOMMANDS:
  rtabmap       Import sensor data from RTABMap SLAM database
  e57           Import point cloud from an E57 file
  ply           Import point cloud from a PLY file
  materialepas  Import material passports from JSON file
  360           Import 360 panoramic images
  photos        Import manual survey photos as material passports

EXAMPLES:
  rux import rtabmap scan.db           # Import RTABMap scan
  rux import e57 scan.e57              # Import E57 point cloud
  rux import ply cloud.ply             # Import PLY point cloud
  rux import materialepas data.json    # Import material passports
  rux import 360 /path/to/photos/     # Import 360 panoramic images
  rux import photos ./manual_photos/  # Import survey photos

NOTES:
  - Use 'rux import <subcommand> --help' for detailed options
  - Import is typically the first step in the processing pipeline
  - Data imported into project database specified by -p/--project flag
)");

  setup_subcommand_import_rtabmap(*sub, global_opt);
  setup_subcommand_import_e57(*sub, global_opt);
  setup_subcommand_import_ply(*sub, global_opt);
  setup_subcommand_import_materialepas(*sub, global_opt);
  setup_subcommand_import_panorama(*sub, global_opt);
  setup_subcommand_import_photos(*sub, global_opt);

  sub->callback([]() {
    spdlog::trace("calling import subcommand");
    return run_subcommand_import();
  });
}

int run_subcommand_import() {
  spdlog::trace("running import subcommand");
  return RuxError::SUCCESS;
}
