// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/rtabmap.hpp"
#include "import.hpp"

#include <fmt/format.h>
#include <reusex/core/project_db.hpp>
#include <reusex/io/rtabmap.hpp>
#include <spdlog/spdlog.h>
#include <tuple>

void setup_subcommand_import_rtabmap(CLI::App &app, ImportContext &ctx) {

  // Create the option and subcommand objects.
  auto opt = std::make_shared<SubcommandImportRTABMapOptions>();
  auto *sub = app.add_subcommand(
      "rtabmap", "Import an RTAB-Map database and "
                 "create a point cloud with labels and normals, "
                 "and the trajectory.");

  sub->add_option("database", opt->database_path_in,
                  "Path to the RTAB-Map database file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("trajectory", opt->trajectory_path_out,
                  "Path to the output trajectory file")
      //->check(CLI::NonexistentPath)
      ->default_val(opt->trajectory_path_out);

  sub->add_option("-g,--grid", opt->resulution,
                  "Voxel grid resolution for downsampling the point cloud")
      ->default_val(opt->resulution);

  sub->add_option("--min-distance", opt->min_distance,
                  "Minimum distance points need to be from the camera")
      ->default_val(opt->min_distance);

  sub->add_option("--max-distance", opt->max_distance,
                  "Maximum distance points can be from the camera")
      ->default_val(opt->max_distance);

  sub->add_option("--sampling-factor", opt->sampling_factor,
                  "Factor for downsampling the individual frames")
      ->default_val(opt->sampling_factor);

  sub->add_option("--project", opt->project,
                  "Save imported data to a .rux project database instead of "
                  "PCD files");

  sub->callback([opt, &ctx]() {
    spdlog::trace("calling run_subcommand_import_rtabmap");
    return run_subcommand_import_rtabmap(*opt, ctx);
  });
}

int run_subcommand_import_rtabmap(SubcommandImportRTABMapOptions const &opt,
                                  ImportContext &ctx) {
  std::tie(ctx.cloud, ctx.normals, ctx.labels) =
      ReUseX::io::import_rtabmap_database(opt.database_path_in, opt.resulution,
                                          opt.min_distance, opt.max_distance,
                                          opt.sampling_factor);

  if (opt.project) {
    spdlog::info("Saving imported data to project database: {}",
                 opt.project->string());
    ReUseX::ProjectDB projectDb(*opt.project);

    int logId = projectDb.logPipelineStart("import",
        fmt::format(R"({{"resolution":{}, "min_distance":{}, "max_distance":{}, "sampling_factor":{}}})",
                    opt.resulution, opt.min_distance, opt.max_distance,
                    opt.sampling_factor));
    try {
      if (ctx.cloud && !ctx.cloud->empty())
        projectDb.savePointCloud("cloud", *ctx.cloud, "import");

      if (ctx.normals && !ctx.normals->empty())
        projectDb.savePointCloud("normals", *ctx.normals, "import");

      if (ctx.labels && !ctx.labels->empty())
        projectDb.savePointCloud("labels", *ctx.labels, "import");

      // Import sensor frame color images for annotation pipeline
      ReUseX::io::importSensorFrames(projectDb, opt.database_path_in);

      projectDb.logPipelineEnd(logId, true);
      spdlog::info("Project database saved successfully");
    } catch (const std::exception &e) {
      projectDb.logPipelineEnd(logId, false, e.what());
      throw;
    }
  }

  return RuxError::SUCCESS;
}
