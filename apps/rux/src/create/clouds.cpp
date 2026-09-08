// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/clouds.hpp"
#include "create/stage_bridge.hpp"
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>

#include <spdlog/spdlog.h>

void setup_subcommand_create_clouds(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateCloudsOptions>();
  auto *sub =
      app.add_subcommand("clouds", "Reconstruct 3D point clouds from depth");

  sub->footer(R"(
DESCRIPTION:
  Generates 3D point clouds by back-projecting depth images from sensor
  frames using camera intrinsics. Applies configurable depth filtering,
  per-pixel subsampling, and voxel grid downsampling to control output
  size and quality. Merges all sensor frames into unified world coordinate.

EXAMPLES:
  rux create clouds                    # Defaults: 5cm grid, 0.2-10m
  rux create clouds -g 0.02            # High detail: 2cm voxel
  rux create clouds --min-distance 0.5 # Filter close noise
  rux create clouds --sampling-factor 2  # Skip every 2nd pixel

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor frames
  2. rux create clouds                 # Reconstruct point clouds
  3. rux create planes                 # Segment planar surfaces
  4. rux view                          # Visualize results

NOTES:
  - Requires sensor frames with depth data (run 'rux import rtabmap' first)
  - Grid resolution affects output size: 0.05m is typical balance
  - Use --confidence to filter low-quality depth measurements
  - Sampling factor reduces per-frame points (1=all, 2=half, etc.)
  - Output saved as 'cloud' and 'normals' in project database
  - Depth range filters invalid measurements and distant noise
)");

  sub->add_option("-g,--grid", opt->resolution,
                  "Voxel grid resolution for downsampling")
      ->default_val(opt->resolution);

  sub->add_option("--min-distance", opt->min_distance,
                  "Minimum depth in meters")
      ->default_val(opt->min_distance);

  sub->add_option("--max-distance", opt->max_distance,
                  "Maximum depth in meters")
      ->default_val(opt->max_distance);

  sub->add_option("--sampling-factor", opt->sampling_factor,
                  "Per-frame pixel subsampling factor")
      ->default_val(opt->sampling_factor);

  sub->add_option("--confidence", opt->confidence_threshold,
                  "Minimum confidence threshold")
      ->default_val(opt->confidence_threshold);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_create_clouds");
    return run_subcommand_create_clouds(*opt, *global_opt);
  });
}

int run_subcommand_create_clouds(SubcommandCreateCloudsOptions const &opt,
                                 const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Reconstructing point clouds from: {}", project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    // Pre-flight validation: check for sensor frames
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::clouds);
        rc != RuxError::SUCCESS)
      return rc;

    reusex::pipeline::StageContext ctx;
    ctx.project = project_path;
    ctx.stage = reusex::pipeline::JobStage::clouds;
    ctx.parameters = rux::StageParams()
                         .set("resolution", opt.resolution)
                         .set("min_distance", opt.min_distance)
                         .set("max_distance", opt.max_distance)
                         .set("sampling_factor", opt.sampling_factor)
                         .set("confidence_threshold", opt.confidence_threshold)
                         .dump();

    // run_stage owns the pipeline_log row, the input-contract check and the
    // failure reporting; it has already logged whatever went wrong.
    const auto result = reusex::pipeline::run_stage(db, ctx);
    if (result.ok)
      spdlog::info("Point cloud reconstruction complete");
    return rux::exit_code_for(result);

  } catch (const std::exception &e) {
    spdlog::error("Point cloud reconstruction failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
