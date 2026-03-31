// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/clouds.hpp"
#include "validation.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/reconstruct.hpp>

#include <spdlog/spdlog.h>

void setup_subcommand_create_clouds(CLI::App &app) {
  auto opt = std::make_shared<SubcommandCreateCloudsOptions>();
  auto *sub =
      app.add_subcommand("clouds", "Generate 3D point clouds by reconstructing "
                                   "depth data from sensor frames");

  sub->add_option("project", opt->project,
                  "Path to the ReUseX project database (.rux).")
      ->default_val(opt->project)
      ->check(CLI::ExistingFile);

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

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_create_clouds");
    return run_subcommand_create_clouds(*opt);
  });
}

int run_subcommand_create_clouds(SubcommandCreateCloudsOptions const &opt) {
  spdlog::info("Reconstructing point clouds from: {}", opt.project.string());

  try {
    ReUseX::ProjectDB db(opt.project);

    // Pre-flight validation: check for sensor frames
    auto validation = rux::validation::validate_clouds_prerequisites(db);
    if (!validation) {
      spdlog::error("{}", validation.error_message);
      spdlog::info("Resolution: {}", validation.resolution_hint);
      return RuxError::INVALID_ARGUMENT;
    }

    int logId = db.log_pipeline_start(
        "cloud_reconstruction",
        fmt::format(
            R"({{"resolution":{},"min_distance":{},"max_distance":{},"sampling_factor":{},"confidence_threshold":{}}})",
            opt.resolution, opt.min_distance, opt.max_distance,
            opt.sampling_factor, opt.confidence_threshold));

    ReUseX::geometry::ReconstructionParams params;
    params.resolution = opt.resolution;
    params.min_distance = opt.min_distance;
    params.max_distance = opt.max_distance;
    params.sampling_factor = opt.sampling_factor;
    params.confidence_threshold = opt.confidence_threshold;

    ReUseX::geometry::reconstruct_point_clouds(db, params);

    db.log_pipeline_end(logId, true);

    spdlog::info("Point cloud reconstruction complete");
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Point cloud reconstruction failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
