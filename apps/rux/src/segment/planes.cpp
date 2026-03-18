// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segment/planes.hpp"
#include <reusex/geometry/segment_planes.hpp>

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/colors.h>
#include <pcl/filters/filter.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>

#include <reusex/io/reusex.hpp>
namespace fs = std::filesystem;

/**
 * @brief Setup CLI options for the plane segmentation subcommand.
 * 
 * Configures command-line arguments including cloud paths, thresholds,
 * and segmentation options for plane segmentation.
 * 
 * @param app CLI application to add the subcommand to.
 */
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
                  "Path to the input normals file.")
      //->required()
      // ->check(CLI::ExistingFile)
      ->default_val(opt->normals_path_in);

  sub->add_option("planes", opt->planes_path_out,
                  "Path to save the output plane labels file")
      ->default_val(opt->planes_path_out);

  sub->add_option("centroids", opt->plane_centroids_path_out,
                  "Path to save the output plane centroids file")
      ->default_val(opt->plane_centroids_path_out);

  sub->add_option("plane-normals", opt->plane_normals_path_out,
                  "Path to save the output plane normals file")
      ->default_val(opt->plane_normals_path_out);

  sub->add_option("-a, --angle-threshold", opt->angle_threshold,
                  "Angle threshold for plane fitting "
                  "(default: 25° or cos(25°) = 0.96592583)")
      ->default_val(opt->angle_threshold)
      ->check(CLI::Range(0.0, 365.0));

  sub->add_option("-d, --plane-dist-threshold", opt->plane_dist_threshold,
                  "Distance threshold for plane fitting (default: 0.1m)")
      ->default_val(opt->plane_dist_threshold)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-m, --min-cluster-size", opt->minInliers,
                  "Minimum cluster size for plane fitting "
                  "(default: 2sqm in 2cm resolution)")
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

  sub->callback([opt]() {
    spdlog::trace("calling seg-planes subcommand");
    return run_subcommand_segment_planes(*opt);
  });
}

/**
 * @brief Execute plane segmentation on a point cloud.
 * 
 * Loads point cloud and normals, performs multi-scale region growing plane
 * segmentation, and saves the results (labeled planes, centroids, normals).
 * 
 * @param opt Options containing file paths and segmentation parameters.
 * @return Exit code (RuxError::SUCCESS on success).
 */
int run_subcommand_segment_planes(SubcommandSegPlanesOptions const &opt) {

  spdlog::trace("Load the point cloud from disk");
  CloudPtr cloud(new Cloud);
  // pcl::io::loadPCDFile<PointT>(opt.path_in.string(), *cloud);
  pcl::io::load<PointT>(opt.cloud_path_in.string(), *cloud);

  spdlog::trace("Load the normals from disk");
  CloudNPtr normals(new CloudN);
  pcl::io::loadPCDFile<NormalT>(opt.normals_path_in.string(), *normals);

  using namespace ReUseX::geometry;

  ReUseX::geometry::SegmentPlanesRequest request;
  request.cloud = cloud;
  request.normals = normals;
  request.angle_threshold = opt.angle_threshold;
  request.plane_dist_threshold = opt.plane_dist_threshold;
  request.min_inliers = opt.minInliers;
  request.radius = opt.radius;
  request.interval_0 = opt.interval_0;
  request.interval_factor = opt.interval_factor;

  auto [labels, centroids, plane_normals] =
      ReUseX::geometry::segment_planes(request);

  spdlog::trace("Save the point cloud with planes");
  pcl::io::savePCDFileBinary(opt.planes_path_out, *labels);
  pcl::io::savePCDFileBinary(opt.plane_centroids_path_out.string(), *centroids);
  pcl::io::savePCDFileBinary(opt.plane_normals_path_out.string(),
                             *plane_normals);

  return RuxError::SUCCESS;
}
