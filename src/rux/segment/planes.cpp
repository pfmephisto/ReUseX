// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/segment/planes.hpp"
#include "pcl/planar_region_growing.hpp"

#include "ReUseX/utils/fmt_formatter.hpp"
#include "spdmon/spdmon.hpp"

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/colors.h>
#include <pcl/filters/filter.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ReUseX/io/reusex.hpp"
namespace fs = std::filesystem;

void setup_subcommand_segment_planes(CLI::App &app) {

  auto opt = std::make_shared<SubcommandSegPlanesOptions>();
  auto *sub = app.add_subcommand("planes", "Segment planes in a point cloud");

  sub->get_formatter()->column_width(40);

  sub->add_option("cloud", opt->cloud_path_in,
                  "Path to the input point cloud file.")
      //->required()
      // ->check(CLI::ExistingFile)
      ->default_val(opt->cloud_path_in);

  sub->add_option("normals", opt->normals_path_in,
                  "Path to the input point cloud file.")
      //->required()
      // ->check(CLI::ExistingFile)
      ->default_val(opt->normals_path_in);

  sub->add_option("planes", opt->planes_path_out,
                  "Path to the output planes file")
      ->default_val(opt->planes_path_out);

  sub->add_option("-a, --angle-threshold", opt->angle_threshold,
                  "Angle threshold for plane fitting (default: 25° aka. "
                  "cos(25) or 0.96592583)")
      ->default_val(opt->angle_threshold)
      ->check(CLI::Range(0.0, 365.0));

  sub->add_option("-d, --plane-dist-threshold", opt->plane_dist_threshold,
                  "Distance threshold for plane fitting (default: 0.1m)")
      ->default_val(opt->plane_dist_threshold)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-m, --min-cluster-size", opt->minInliers,
                  "Minimum cluster size for plane fitting (default: 2sqm in "
                  "2cm resolution of point cloud)")
      ->default_val(opt->minInliers)
      ->check(CLI::Range(3, 1000000));

  sub->add_option("-r, --radius", opt->radius, "Radius for region growing")
      ->default_val(opt->radius)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-i, --interval-0", opt->interval_0,
                  "Initial interval for plane update")
      ->default_val(opt->interval_0)
      ->check(CLI::Range(1.0, 10000.0));

  sub->add_option("-f, --interval-factor", opt->interval_factor,
                  "Factor for interval update")
      ->default_val(opt->interval_factor)
      ->check(CLI::Range(1.0, 10.0));

  sub->add_flag("--visualize", opt->visualize,
                "Visualize the segmentation process")
      ->default_val(opt->visualize);

  sub->callback([opt]() {
    spdlog::trace("calling seg-planes subcommand");
    return run_subcommand_segment_planes(*opt);
  });
}

int run_subcommand_segment_planes(SubcommandSegPlanesOptions const &opt) {

  spdlog::trace("Load the point cloud from disk");
  CloudPtr cloud(new Cloud);
  // pcl::io::loadPCDFile<PointT>(opt.path_in.string(), *cloud);
  pcl::io::load<PointT>(opt.cloud_path_in.string(), *cloud);

  spdlog::trace("Load the normals from disk");
  CloudNPtr normals(new CloudN);
  pcl::io::loadPCDFile<NormalT>(opt.normals_path_in.string(), *normals);

  // pcl::Indices indices;
  // pcl::removeNaNNormalsFromPointCloud(*cloud, *cloud, indices);

  spdlog::trace("Initialize the segmentation algorithm");
  pcl::PlanarRegionGrowing<PointT, NormalT, LabelT> seg;
  seg.setInputCloud(cloud);
  seg.setInputNormals(normals);

  seg.setAngularThreshold(opt.angle_threshold);
  seg.setDistanceThreshold(opt.plane_dist_threshold);
  seg.setMinInliers(opt.minInliers);

  seg.setRadiusSearch(opt.radius);

  seg.setInitialInterval(opt.interval_0);
  seg.setIntervalFactor(opt.interval_factor);

  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (opt.visualize) {
    static int plane_id = 0;
    spdlog::warn("Visualization is an experimental feature.");
    viewer = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("MCL Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    seg.registerVisualizationCallback(
        [&viewer](const pcl::ModelCoefficients &plane,
                  const Eigen::Vector4f &origin) {
          spdlog::trace("Visualization callback called");
          const auto name = "plane" + std::to_string(plane_id);
          const auto color = pcl::GlasbeyLUT::at(plane_id);
          viewer->addPlane(plane, origin.x(), origin.y(), origin.z(), name);
          viewer->setShapeRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_COLOR, color.r / 254,
              color.g / 254, color.b / 254, name);
          viewer->setShapeRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, name);
          plane_id++;
        });

    viewer->addPointCloud<PointT>(cloud, "cloud");
  }

  spdlog::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*cloud, *labels);

  spdlog::trace("Call the segmentation algorithm");
  seg.segment(labels);

  spdlog::info("Found {} clusters", seg.getCentroids().size());
  if (viewer)
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  std::vector<pcl::ModelCoefficients> model_coefficients =
      seg.getModelCoefficients();
  auto centroids = seg.getCentroids();
  auto inlier_indices = seg.getInlierIndices();

  // TODO: Reconsider how to save the planes
  if (!ReUseX::io::save(opt.planes_path_out.string(), model_coefficients,
                        centroids, inlier_indices))
    spdlog::error("Could not save the planes to disk");

  spdlog::trace("Save the point cloud with planes");
  pcl::io::savePCDFileBinary(opt.planes_path_out, *labels);

  return 0;
}
