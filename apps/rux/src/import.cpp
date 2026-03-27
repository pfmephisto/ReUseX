// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import.hpp"
#include "import/materialepas.hpp"
#include "import/rtabmap.hpp"
#include "spdmon.hpp"

#include <fmt/format.h>
#include <fmt/std.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/colors.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <ranges>
namespace fs = std::filesystem;
// using namespace ReUseX;

void setup_subcommand_import(CLI::App &app) {

  auto opt = std::make_shared<SubcommandImportOptions>();
  auto ctx = std::make_shared<ImportContext>();

  auto *sub = app.add_subcommand(
      "import", "Import data from various sources into point cloud format.");

  // sub->add_option("path", opt->path_in, "Path to the input file.")
  //     ->required()
  //     ->check(CLI::ExistingFile);

  sub->add_option("cloud", opt->cloud_path_out,
                  "Path to save the imported point cloud to.")
      ->default_val(opt->cloud_path_out);

  sub->add_option("normals", opt->normals_path_out,
                  "Path to save the imported point cloud normals to.")
      ->default_val(opt->normals_path_out);

  sub->add_option("labels", opt->labels_path_out,
                  "Path to save the imported point cloud labels to.")
      ->default_val(opt->labels_path_out);

  sub->add_flag("--ascii", opt->ascii, "Save the point cloud in ASCII format")
      ->default_val(opt->ascii);
  setup_subcommand_import_rtabmap(*sub, *ctx);
  setup_subcommand_import_materialepas(*sub);

  sub->callback([opt, ctx]() {
    spdlog::trace("calling import subcommand");
    return run_subcommand_import(*opt, *ctx);
  });
}

int run_subcommand_import(SubcommandImportOptions const &opt,
                          ImportContext &ctx) {
  spdlog::trace("running import subcommand");

  auto cloud = ctx.cloud;
  auto normals = ctx.normals;
  auto labels = ctx.labels;

  if (cloud && !cloud->empty()) {
    spdlog::info("Saving imported point cloud to: {}", opt.cloud_path_out);
    pcl::io::savePCDFile(opt.cloud_path_out.string(), *cloud, !opt.ascii);
  }

  if (normals && !normals->empty()) {
    spdlog::info("Saving imported normals to:     {}", opt.normals_path_out);
    pcl::io::savePCDFile(opt.normals_path_out.string(), *normals, !opt.ascii);
  }

  if (labels && !labels->empty()) {
    spdlog::info("Saving imported labels to:      {}", opt.labels_path_out);
    pcl::io::savePCDFile(opt.labels_path_out.string(), *labels, !opt.ascii);
  }

  return RuxError::SUCCESS;
}
