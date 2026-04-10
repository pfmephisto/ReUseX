// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export/rhino.hpp"
#include "global-params.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/types.hpp>
#include <reusex/io/rhino.hpp>
#include <fmt/format.h>
#include <pcl/common/io.h>
#include <spdlog/spdlog.h>

#include <filesystem>
namespace fs = std::filesystem;
using namespace ReUseX;

void setup_subcommand_export_rhino(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandExportRhinoOptions>();
  auto *sub = parent.add_subcommand(
      "rhino",
      "Export a labeled point cloud from ProjectDB to Rhino 3DM format.");

  sub->add_option("-c, --cloud-name", opt->cloud_name,
                  "Name of the point cloud in ProjectDB")
      ->default_val(opt->cloud_name);

  sub->add_option("-l, --labels-name", opt->labels_name,
                  "Name of the labels in ProjectDB (optional)");

  sub->add_option("-o, --output", opt->path_out,
                  "Path to the output Rhino 3DM file")
      ->default_val(opt->path_out);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling export rhino subcommand");
    return run_subcommand_export_rhino(*opt, *global_opt);
  });
}

int run_subcommand_export_rhino(SubcommandExportRhinoOptions const &opt, const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Exporting to Rhino from project: {}", project_path.string());

  try {
    ProjectDB db(project_path);

    spdlog::trace("Loading point cloud '{}' from ProjectDB", opt.cloud_name);
    CloudPtr pcl_cloud = db.point_cloud_xyzrgb(opt.cloud_name);

    CloudLPtr pcl_labels;
    if (!opt.labels_name.empty()) {
      spdlog::trace("Loading labels '{}' from ProjectDB", opt.labels_name);
      pcl_labels = db.point_cloud_label(opt.labels_name);
    } else {
      spdlog::trace("No labels specified, creating empty label cloud");
      pcl_labels = CloudLPtr(new CloudL);
      pcl_labels->resize(pcl_cloud->size());
    }

    spdlog::trace("Converting to Rhino format");
    auto model = ReUseX::io::save_rhino_pointcloud(pcl_cloud, pcl_labels);

    spdlog::trace("Writing Rhino model to: {}", opt.path_out.string());
    int version = 0;
    if (!model->Write(opt.path_out.c_str(), version)) {
      spdlog::error("Failed to write Rhino model to: {}", opt.path_out.string());
      return RuxError::IO;
    }

    spdlog::info("Rhino model successfully written to: {}", opt.path_out.string());
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Rhino export failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
