// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/ply.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/ply.hpp>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

void setup_subcommand_import_ply(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandImportPLYOptions>();
  auto *sub = app.add_subcommand("ply", "Import point cloud from a PLY file");

  sub->footer(R"(
DESCRIPTION:
  Reads a PLY point cloud file and stores all available data in the ReUseX
  project database. The following fields are extracted when present:
    - XYZ position (required)
    - RGB color   → stored as an XYZRGB cloud named after the file stem
    - Normals     → stored as a separate "_normals" cloud
    - Labels      → stored as a separate "_labels" cloud
    - Intensity   → mapped to grayscale RGB when no color data is present

EXAMPLES:
  rux import ply cloud.ply                     # Import to ./project.rux
  rux -p office.rux import ply cloud.ply       # Custom project path

NOTES:
  - Supported PLY variants: ASCII and binary (little- and big-endian)
  - Labels are saved only when at least one non-zero label value is found
  - Use 'rux view' to inspect the imported point cloud
)");

  sub->add_option("file", opt->input_path, "Path to the PLY file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_ply");
    return run_subcommand_import_ply(*opt, *global_opt);
  });
}

int run_subcommand_import_ply(SubcommandImportPLYOptions const &opt, const RuxOptions &global_opt) {

  fs::path project_path = global_opt.project_db;
  spdlog::info("Importing PLY file to project: {}", project_path.string());

  reusex::ProjectDB project_db(project_path);
  reusex::io::import_ply(project_db, opt.input_path);

  spdlog::info("PLY import complete. Use 'rux view' to inspect results.");

  return RuxError::SUCCESS;
}
