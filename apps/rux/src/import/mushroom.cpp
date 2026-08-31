// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/mushroom.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/mushroom.hpp>

#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

void setup_subcommand_import_mushroom(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandImportMushroomOptions>();
  auto *sub = app.add_subcommand(
      "mushroom", "Import a MuSHRoom dataset capture (RGB-D benchmark)");

  sub->footer(R"(
DESCRIPTION:
  Imports one capture of the MuSHRoom benchmark dataset (Ren et al.,
  WACV 2024, CC-BY-4.0): room-scale RGB-D sequences from consumer devices
  with Faro laser-scanned ground-truth meshes. Point the command at a
  capture directory containing images/, depth/ and transformations.json.

  MuSHRoom's iPhone captures use the same sensor class as ReUseX's own
  iPad scans, making them the primary external ground-truth benchmark for
  reconstruction quality (issues #221 / #224).

EXAMPLES:
  rux -p honka.rux import mushroom ~/datasets/mushroom/honka/iphone/long_capture

WORKFLOW:
  1. rux import mushroom <capture_dir>  # Import sensor frames
  2. rux create clouds                  # Reconstruct point clouds
  3. rux create planes                  # Segment planar surfaces
  4. rux analyze quality                # Score against internal metrics

NOTES:
  - Download from Zenodo (records 10151161 / 10222321, CC-BY-4.0)
  - Poses converted from OpenGL camera-to-world to OpenCV optical frame
  - Depth PNGs expected in millimeters (CV_16UC1)
  - No confidence channel in MuSHRoom; imported as fully confident
)");

  sub->add_option("capture_dir", opt->capture_dir,
                  "MuSHRoom capture directory (contains transformations.json)")
      ->required()
      ->check(CLI::ExistingDirectory);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_mushroom");
    return run_subcommand_import_mushroom(*opt, *global_opt);
  });
}

int run_subcommand_import_mushroom(SubcommandImportMushroomOptions const &opt,
                                   const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Importing MuSHRoom capture {} into project {}", opt.capture_dir,
               project_path);

  try {
    reusex::ProjectDB db(project_path);

    int logId = db.log_pipeline_start(
        "import_mushroom",
        fmt::format(R"({{"capture_dir":"{}"}})", opt.capture_dir.string()));

    const auto frames = reusex::io::import_mushroom(db, opt.capture_dir);

    db.log_pipeline_end(logId, true);
    spdlog::info("Imported {} frames. Use 'rux create clouds' to generate "
                 "point clouds.",
                 frames);
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("MuSHRoom import failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
