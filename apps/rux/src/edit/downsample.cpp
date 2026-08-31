// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "edit/downsample.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/sync_downsample.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>

#include <stdexcept>
#include <string>

void setup_subcommand_edit_downsample(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandEditDownsampleOptions>();
  auto *sub =
      app.add_subcommand("downsample", "Voxel-grid downsample a project cloud");

  sub->footer(R"(
DESCRIPTION:
  Downsamples an XYZRGB point cloud already stored in the project database
  using a voxel grid. Each occupied voxel contributes one centroid point
  in the output. Unlike pcl::VoxelGrid (which uses int32 voxel indices and
  overflows around 2 billion total voxels), this command uses int64
  voxel coordinates and a hash-based partition, so it scales to multi-
  billion-point inputs.

  By default the SAME voxel partition is applied to every index-aligned
  sibling cloud — any point cloud with the same point count as the input
  (e.g. "normals", "labels", "planes", "rooms", "instances"). Sibling
  clouds are transformed and overwritten in place so all derived products
  stay row-aligned with the primary cloud (docs/STANDARDS.md §3.2). Label
  clouds are resolved per voxel by majority vote; their label definitions
  are carried over.

  This means downsampling can never silently desynchronize a project. If
  you deliberately want to touch only the primary cloud, pass --only-primary
  together with --force-desync; the command will then leave the siblings
  stale and log exactly which ones.

EXAMPLES:
  rux edit downsample -r 0.05
      Overwrite 'cloud' with a 5 cm voxel-downsampled version, and
      downsample every same-size sibling with the same partition.

  rux edit downsample -i cloud -o cloud_5cm -r 0.05
      Write the primary to a new name 'cloud_5cm'. Siblings are still
      downsampled (in place) so they stay aligned with the new cloud.

  rux edit downsample -r 0.05 --only-primary --force-desync
      Downsample ONLY 'cloud', deliberately breaking alignment with its
      derived clouds (reported loudly).

NOTES:
  - The input cloud must have type PointXYZRGB.
  - Supported sibling types: PointXYZRGB, Normal, Label. A sibling of any
    other type aborts the operation rather than leaving it desynchronized.
)");

  sub->add_option("-i,--input", opt->input,
                  "Source cloud name in the project DB")
      ->default_val(opt->input);

  sub->add_option(
      "-o,--output", opt->output,
      "Output cloud name for the primary (defaults to the input — overwrites)");

  sub->add_option("-r,--resolution", opt->resolution,
                  "Voxel leaf size in meters")
      ->required()
      ->check(CLI::PositiveNumber);

  sub->add_flag("--only-primary", opt->only_primary,
                "Downsample only the primary cloud, leaving siblings stale. "
                "Refused unless combined with --force-desync.");

  sub->add_flag(
      "--force-desync", opt->force_desync,
      "Permit --only-primary to break alignment with derived clouds.");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_edit_downsample");
    return run_subcommand_edit_downsample(*opt, *global_opt);
  });
}

int run_subcommand_edit_downsample(SubcommandEditDownsampleOptions const &opt,
                                   const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;

  try {
    reusex::ProjectDB db(project_path);

    reusex::geometry::SyncDownsampleOptions sopt;
    sopt.leaf_size = opt.resolution;
    sopt.primary_output = opt.output; // empty => overwrite
    sopt.only_primary = opt.only_primary;
    sopt.force_desync = opt.force_desync;

    spdlog::info("Downsampling '{}' in {} (leaf={} m)", opt.input,
                 project_path.string(), opt.resolution);

    auto result = reusex::geometry::sync_downsample(db, opt.input, sopt);

    spdlog::info("Primary output: {} points", result.output_points);
    if (!result.written_clouds.empty())
      spdlog::info("Wrote {} cloud(s): [{}]", result.written_clouds.size(),
                   fmt::join(result.written_clouds, ", "));
    if (!result.desynced_clouds.empty())
      spdlog::warn("Left {} sibling cloud(s) stale (desynchronized): [{}]",
                   result.desynced_clouds.size(),
                   fmt::join(result.desynced_clouds, ", "));

    spdlog::info("Downsample complete");
    return RuxError::SUCCESS;

  } catch (const std::invalid_argument &e) {
    spdlog::error("Downsample failed: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("Downsample failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
