// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "get/clouds.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

void setup_subcommand_get_clouds(CLI::App &app) {
  auto opt = std::make_shared<SubcommandGetCloudsOptions>();
  auto *sub = app.add_subcommand(
      "clouds", "List all point clouds in the project database");

  sub->add_option("project", opt->project,
                  "Path to the ReUseX project database (.rux).")
      ->default_val(opt->project)
      ->check(CLI::ExistingFile);

  sub->add_flag("-j,--json", opt->json_output,
                "Output results in JSON format")
      ->default_val(opt->json_output);

  sub->add_flag("-v,--verbose", opt->verbose,
                "Include detailed information (type, point count)")
      ->default_val(opt->verbose);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_get_clouds");
    return run_subcommand_get_clouds(*opt);
  });
}

int run_subcommand_get_clouds(SubcommandGetCloudsOptions const &opt) {
  spdlog::info("Listing point clouds in: {}", opt.project.string());

  try {
    ReUseX::ProjectDB db(opt.project, /* readOnly */ true);
    auto cloud_names = db.list_point_clouds();

    if (cloud_names.empty()) {
      spdlog::info("No point clouds found in project");
      return RuxError::SUCCESS;
    }

    if (opt.json_output) {
      // TODO: Implement JSON output format
      spdlog::warn("JSON output not yet implemented, falling back to text");
    }

    if (opt.verbose) {
      // Print detailed information
      fmt::print("{:<30} {:<15} {:<12}\n", "Name", "Type", "Point Count");
      fmt::print("{:-<60}\n", "");

      for (const auto &name : cloud_names) {
        std::string type = db.point_cloud_type(name);

        // Get point count based on type
        size_t point_count = 0;
        if (type == "PointXYZRGB") {
          auto cloud = db.point_cloud_xyzrgb(name);
          point_count = cloud ? cloud->size() : 0;
        } else if (type == "Normal") {
          auto cloud = db.point_cloud_normal(name);
          point_count = cloud ? cloud->size() : 0;
        } else if (type == "Label") {
          auto cloud = db.point_cloud_label(name);
          point_count = cloud ? cloud->size() : 0;
        } else if (type == "PointXYZ") {
          auto cloud = db.point_cloud_xyz(name);
          point_count = cloud ? cloud->size() : 0;
        }

        fmt::print("{:<30} {:<15} {:>12}\n", name, type, point_count);
      }
    } else {
      // Print simple list
      for (const auto &name : cloud_names) {
        fmt::print("{}\n", name);
      }
    }

    spdlog::info("Found {} point cloud(s)", cloud_names.size());
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Failed to list point clouds: {}", e.what());
    return RuxError::IO;
  }
}
