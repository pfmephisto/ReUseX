// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "curand_globals.h"
#include <project.hpp>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <fmt/format.h>
#include <fmt/std.h>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/vision/project.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <range/v3/to_container.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>

namespace fs = std::filesystem;

void setup_subcommand_project(CLI::App &app) {

  auto opt = std::make_shared<SubcommandProjectOptions>();
  auto *sub = app.add_subcommand(
      "project", "Project labels from the dataset onto the point cloud.");

  sub->add_option("project", opt->project,
                  "Path to the .rux project file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_project");
    return run_subcommand_project(*opt);
  });
};

int run_subcommand_project(SubcommandProjectOptions const &opt) {
  spdlog::info("Projecting labels in project: {}", opt.project.string());

  try {
    ReUseX::ProjectDB db(opt.project);

    int logId = db.log_pipeline_start("project_labels");

    spdlog::trace("Loading point cloud from ProjectDB");
    auto cloud = db.point_cloud_xyzrgb("cloud");

    spdlog::trace("Projecting labels from sensor frames");
    CloudLPtr labels = ReUseX::vision::project(opt.project, cloud);

    spdlog::trace("Saving projected labels to ProjectDB");
    db.save_point_cloud("labels", *labels, "project_labels");

    spdlog::info("Label projection complete");

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Label projection failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
