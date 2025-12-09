// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/export.hpp"

#include <ReUseX/io/rhino.hpp>
#include <fmt/format.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <spdlog/spdlog.h>

#include <filesystem>
namespace fs = std::filesystem;
using namespace ReUseX;

void setup_subcommand_export(CLI::App &app) {

  auto opt = std::make_shared<SubcommandExportOptions>();
  auto *sub = app.add_subcommand(
      "export",
      "Export a labeled point cloud to Rhino 3DM format.");

  sub->add_option("cloud", opt->cloud_path_in,
                  "Path to the input point cloud file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("labels", opt->labels_path_in,
                  "Path to the input point cloud labels file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("output", opt->path_out,
                  "Path to the output Rhino 3DM file")
      ->default_val(opt->path_out);

  sub->callback([opt]() {
    spdlog::trace("calling export subcommand");
    return run_subcommand_export(*opt);
  });
}

int run_subcommand_export(SubcommandExportOptions const &opt) {

  spdlog::trace("load pcl point cloud from file: {}", opt.cloud_path_in);
  CloudPtr pcl_cloud(new Cloud);
  pcl::io::loadPCDFile<PointT>(opt.cloud_path_in.c_str(), *pcl_cloud);

  spdlog::trace("load pcl labels from file: {}", opt.labels_path_in);
  CloudLPtr pcl_labels(new CloudL);
  pcl::io::loadPCDFile<LabelT>(opt.labels_path_in.c_str(), *pcl_labels);

  auto model = ReUseX::io::save_rhino_pointcloud(pcl_cloud, pcl_labels);

  spdlog::trace("writeing model to file: {}", opt.path_out.string());
  int version = 0;
  // const char *comment = __FILE__ "write_layers_example()" __DATE__;
  if (!model->Write(opt.path_out.c_str(), version
                    /*, comment, &error_log*/)) {
    spdlog::error("Failed to write model to file: {}", opt.path_out.string());
    return 1;
  }
  spdlog::info("Model successfully written to {}", opt.path_out.string());

  return RuxError::SUCCESS;
}
