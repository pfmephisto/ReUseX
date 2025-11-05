// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/view.hpp"
#include <spdlog/spdlog.h>

#include <pcl/common/common.h>
#include <pcl/io/auto_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fmt/format.h>
#include <fmt/std.h>

#include <filesystem>
#include <latch>
#include <mutex>
#include <thread>

void setup_subcommand_view(CLI::App &app) {

  auto opt = std::make_shared<SubcommandViewOptions>();
  auto *sub =
      app.add_subcommand("view", "This tool allows for viewing of the 3D "
                                 "annotated point clouds.");

  sub->add_option("cloud", opt->cloud_path_in, "Path to the input cloud file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("-l, --label", opt->label_paths_in,
                  "Path(s) to the label file to overlay.");

  sub->callback([opt]() {
    spdlog::trace("calling viewer subcommand");
    return run_subcommand_view(*opt);
  });
}

int run_subcommand_view(SubcommandViewOptions const &opt) {

  spdlog::trace("Visualization thread started");
  auto viewer =
      std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");

  // TODO: Make the tilling configurable or automatic create multiple
  // rows/columns
  std::vector<int> viewports(1);
  for (size_t i = 0; i < viewports.size(); ++i) {
    float left = static_cast<float>(i) / static_cast<float>(viewports.size());
    float right =
        static_cast<float>(i + 1) / static_cast<float>(viewports.size());
    viewer->createViewPort(left, 0.0, right, 1.0, viewports[i]);
    viewer->setBackgroundColor(0, 0, 0, 1);
    viewer->addCoordinateSystem(1.0, fmt::format("vp{}", i), viewports[i]);
  }

  viewer->initCameraParameters();

  spdlog::trace("Loading point cloud from {}", opt.cloud_path_in);
  CloudPtr cloud(new Cloud);
  pcl::io::load<PointT>(opt.cloud_path_in.string(), *cloud);

  viewer->addPointCloud<PointT>(cloud, "cloud", viewports[0]);

  std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> label_clouds;
  label_clouds.reserve(opt.label_paths_in.size());
  for (const auto &label_path : opt.label_paths_in) {
    spdlog::trace("Loading label cloud from {}", label_path);
    CloudLPtr labels(new CloudL);
    pcl::io::load<LabelT>(label_path.string(), *labels);

    if (labels->size() != cloud->size()) {
      spdlog::error(
          "Label cloud size ({}) does not match point cloud size ({})",
          labels->size(), cloud->size());
      continue;
    }

    label_clouds.emplace_back(new pcl::PointCloud<pcl::PointXYZL>);
    label_clouds.back()->resize(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
      label_clouds.back()->points[i].x = cloud->points[i].x;
      label_clouds.back()->points[i].y = cloud->points[i].y;
      label_clouds.back()->points[i].z = cloud->points[i].z;
      label_clouds.back()->points[i].label = labels->points[i].label;
    }
  }

  viewer->registerKeyboardCallback(
      [&](const pcl::visualization::KeyboardEvent &event) {
        if (event.getKeySym() == "0" && event.keyDown()) {
          viewer->removeAllPointClouds();
          viewer->addPointCloud<PointT>(cloud, "cloud", viewports[0]);
        }
      });

  for (size_t i = 0; i <= label_clouds.size() && i < 9; ++i) {
    viewer->registerKeyboardCallback(
        [i, &viewer, &label_clouds,
         &viewports](const pcl::visualization::KeyboardEvent &event) {
          if (event.getKeySym() == std::to_string(i + 1) && event.keyDown()) {
            const std::string cloud_name = fmt::format("label_cloud_{}", i);

            // This is already the current view
            if (viewer->contains(cloud_name))
              return;

            spdlog::trace("Adding label cloud {}", i);
            viewer->removeAllPointClouds();
            pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZL>
                color_handler(label_clouds[i]);
            viewer->addPointCloud<pcl::PointXYZL>(
                label_clouds[i], color_handler, cloud_name, viewports[0]);
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
          }
        });
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
