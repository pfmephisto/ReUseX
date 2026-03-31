// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "add/cloud.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <spdlog/spdlog.h>

void setup_subcommand_add_cloud(CLI::App &app) {
  auto opt = std::make_shared<SubcommandAddCloudOptions>();
  auto *sub = app.add_subcommand(
      "cloud", "Add a point cloud from a file (PCD/PLY/E57) to the project database");

  sub->add_option("name", opt->name,
                  "Name for the point cloud in the database")
      ->required();

  sub->add_option("input_file", opt->input_file,
                  "Path to the input point cloud file")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("--project", opt->project,
                  "Path to the ReUseX project database (.rux)")
      ->default_val(opt->project)
      ->check(CLI::ExistingFile);

  sub->add_option("--stage", opt->stage,
                  "Pipeline stage metadata (e.g., 'import', 'preprocessed')")
      ->default_val(opt->stage);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_add_cloud");
    return run_subcommand_add_cloud(*opt);
  });
}

int run_subcommand_add_cloud(SubcommandAddCloudOptions const &opt) {
  spdlog::info("Adding point cloud '{}' from: {}",
               opt.name, opt.input_file.string());

  // TODO: Implement file loading
  // 1. Detect format from extension (.pcd, .ply, .e57)
  // 2. Load cloud using appropriate loader (PCL, E57Format, etc.)
  // 3. Convert to appropriate type (PointXYZRGB, Normal, Label, PointXYZ)
  // 4. Save to ProjectDB using db.save_point_cloud()

  spdlog::error("Point cloud import not yet implemented");
  spdlog::info("To implement: detect format -> load file -> save to database");

  return RuxError::NOT_IMPLEMENTED;
}
