// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/arkitscenes.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/arkitscenes.hpp>

#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

void setup_subcommand_import_arkitscenes(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandImportArkitscenesOptions>();
  auto *sub = app.add_subcommand(
      "arkitscenes",
      "Import an ARKitScenes scene (lowres iPad-LiDAR RGB-D benchmark)");

  sub->footer(R"(
DESCRIPTION:
  Imports one scene of the ARKitScenes dataset (Apple / Dehghan et al.,
  NeurIPS 2021, research-only license): room-scale RGB-D sequences captured
  with iPad-Pro LiDAR — the same sensor class as ReUseX's own iPad scans,
  which makes them an external reconstruction-quality benchmark (issue #224).

  Point the command at a scene directory. The importer resolves the frames
  directory (the one containing lowres_wide.traj), searching one level of
  nesting if needed, and reads the lowres streams: lowres_wide/ (RGB),
  lowres_depth/ (CV_16UC1 mm), confidence/ (0-2) and lowres_wide_intrinsics/.

EXAMPLES:
  rux -p scene.rux import arkitscenes ~/datasets/arkitscenes/raw/Validation/41069021

WORKFLOW:
  1. rux import arkitscenes <scene_dir>  # Import sensor frames
  2. rux create clouds                   # Reconstruct point clouds
  3. rux create planes                   # Segment planar surfaces
  4. rux analyze quality                 # Score against internal metrics

NOTES:
  - Uses the lowres LiDAR streams (lowres_wide / lowres_depth / confidence)
  - Poses inverted from ARKit world->camera to (optical) camera-to-world
  - Depth PNGs are CV_16UC1 in millimeters (passed through unchanged)
  - Confidence PNGs are CV_8UC1 with values 0-2 (higher = better)
  - ARKitScenes carries a research-only license (see Apple's dataset terms)
)");

  sub->add_option("scene_dir", opt->scene_dir,
                  "ARKitScenes scene directory (contains lowres_wide.traj, "
                  "or a subdir that does)")
      ->required()
      ->check(CLI::ExistingDirectory);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_arkitscenes");
    return run_subcommand_import_arkitscenes(*opt, *global_opt);
  });
}

int run_subcommand_import_arkitscenes(
    SubcommandImportArkitscenesOptions const &opt,
    const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Importing ARKitScenes scene {} into project {}", opt.scene_dir,
               project_path);

  try {
    reusex::ProjectDB db(project_path);

    int logId = db.log_pipeline_start(
        "import_arkitscenes",
        fmt::format(R"({{"scene_dir":"{}"}})", opt.scene_dir.string()));

    const auto frames = reusex::io::import_arkitscenes(db, opt.scene_dir);

    db.log_pipeline_end(logId, true);
    spdlog::info("Imported {} frames. Use 'rux create clouds' to generate "
                 "point clouds.",
                 frames);
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("ARKitScenes import failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
