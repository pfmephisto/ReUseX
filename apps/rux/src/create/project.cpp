// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "curand_globals.h"
#include "create/project.hpp"
#include "validation.hpp"

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

void setup_subcommand_create_project(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandProjectOptions>();
  auto *sub = app.add_subcommand(
      "project", "Project 2D segmentation labels from sensor frames onto the 3D point cloud");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_project");
    return run_subcommand_project(*opt, *global_opt);
  });
};

int run_subcommand_project(SubcommandProjectOptions const &opt, const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Projecting labels in project: {}", project_path.string());

  try {
    ReUseX::ProjectDB db(project_path);

    // Pre-flight validation: check for cloud and segmentation images
    auto validation = rux::validation::validate_project_prerequisites(db);
    if (!validation) {
      spdlog::error("{}", validation.error_message);
      spdlog::info("Resolution: {}", validation.resolution_hint);
      return RuxError::INVALID_ARGUMENT;
    }

    int logId = db.log_pipeline_start("project_labels");

    spdlog::trace("Loading point cloud from ProjectDB");
    auto cloud = db.point_cloud_xyzrgb("cloud");

    spdlog::trace("Projecting labels from sensor frames");
    CloudLPtr labels = ReUseX::vision::project(project_path, cloud);

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
