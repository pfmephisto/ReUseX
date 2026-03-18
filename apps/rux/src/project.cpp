// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "curand_globals.h"
#include <project.hpp>

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

// #include <fmt/color.h>
#include <fmt/std.h>

#include <reusex/io/reusex.hpp>
#include <reusex/vision/project.hpp>
#include <reusex/visualize/Visualizer.hpp>

#include <pcl/common/common.h>
#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <range/v3/to_container.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>

namespace fs = std::filesystem;

void setup_subcommand_project(CLI::App &app) {

  auto opt = std::make_shared<SubcommandProjectOptions>();
  auto *sub = app.add_subcommand(
      "project", "Project labes for the dataset on the the point cloud.");

  sub->add_option("databae", opt->database_path_in,
                  "Path to the database file.")
      ->default_val(opt->database_path_in);

  sub->add_option("cloud", opt->cloud_path_in, "Path to the input cloud file.")
      ->default_val(opt->cloud_path_in);

  sub->add_option("labels", opt->labels_path_out,
                  "Path to the output labels file.")
      ->default_val(opt->labels_path_out);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_project");
    return run_subcommand_project(*opt);
  });
};

int run_subcommand_project(SubcommandProjectOptions const &opt) {

  //// INFO: Setup viewer and vizualiztion queue
  //// INFO: Setup viewer and vizualiztion
  /// queue/std::shared_ptr<ReUseX::visualize::Visualizer> viewer;
  // auto vps = std::make_shared<std::vector<int>>();
  // vps->resize(4);
  // if (opt.display) {
  //   spdlog::trace("Setting up visualization thread and queue");
  //   viewer = std::make_shared<ReUseX::visualize::Visualizer>(vps);
  //   spdlog::debug("VPs: {}", fmt::join(*vps, ", "));
  // }

  assert(fs::exists(opt.database_path_in) &&
         "Input database file does not exist");
  assert(fs::exists(opt.cloud_path_in) &&
         "Input point cloud file does not exist");

  CloudPtr cloud(new Cloud);
  spdlog::trace("Reading {:<8} file: {}", "input", opt.cloud_path_in);
  pcl::io::load<PointT>(opt.cloud_path_in.string(), *cloud);

  CloudLPtr labels = ReUseX::vision::project(opt.database_path_in, cloud);
  pcl::io::save(opt.labels_path_out.string(), *labels);

  return RuxError::SUCCESS;
}
