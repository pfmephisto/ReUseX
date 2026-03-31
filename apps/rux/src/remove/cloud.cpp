// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "remove/cloud.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <iostream>

void setup_subcommand_remove_cloud(CLI::App &app) {
  auto opt = std::make_shared<SubcommandRemoveCloudOptions>();
  auto *sub = app.add_subcommand(
      "cloud", "Delete a point cloud from the project database");

  sub->add_option("name", opt->name,
                  "Name of the point cloud to delete")
      ->required();

  sub->add_option("--project", opt->project,
                  "Path to the ReUseX project database (.rux)")
      ->default_val(opt->project)
      ->check(CLI::ExistingFile);

  sub->add_flag("-f,--force", opt->force,
                "Skip confirmation prompt")
      ->default_val(opt->force);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_remove_cloud");
    return run_subcommand_remove_cloud(*opt);
  });
}

int run_subcommand_remove_cloud(SubcommandRemoveCloudOptions const &opt) {
  spdlog::info("Removing point cloud '{}' from: {}",
               opt.name, opt.project.string());

  try {
    ReUseX::ProjectDB db(opt.project);

    // Check if cloud exists
    if (!db.has_point_cloud(opt.name)) {
      spdlog::error("Point cloud '{}' does not exist in project", opt.name);
      return RuxError::INVALID_ARGUMENT;
    }

    // Get cloud info for confirmation
    std::string type = db.point_cloud_type(opt.name);

    // Confirmation prompt unless --force
    if (!opt.force) {
      fmt::print("Warning: This will permanently delete point cloud '{}' (type: {})\n",
                 opt.name, type);
      fmt::print("Are you sure? [y/N]: ");

      std::string response;
      std::getline(std::cin, response);

      if (response != "y" && response != "Y" && response != "yes" && response != "Yes") {
        spdlog::info("Deletion cancelled");
        return RuxError::SUCCESS;
      }
    }

    // Perform deletion
    db.delete_point_cloud(opt.name);
    spdlog::info("Successfully removed point cloud '{}'", opt.name);

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Failed to remove point cloud: {}", e.what());
    return RuxError::IO;
  }
}
