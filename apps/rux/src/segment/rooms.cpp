// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segment/rooms.hpp"
#include "ReUseX/io/reusex.hpp"
#include "ReUseX/utils/fmt_formatter.hpp"
#include "pcl/markov_clustering.hpp"
#include "spdmon/spdmon.hpp"

#include "ReUseX/geometry/segment_rooms.hpp"

// GraphBLAS imports complex.h which defines a macro named 'I' that conflicts
// with the type in CLI11
#ifdef I
#undef I
#endif

#include <CLI/CLI.hpp>
#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/pca.h>
#include <pcl/correspondence.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace fs = std::filesystem;

/**
 * @brief Setup CLI options for the room segmentation subcommand.
 * 
 * Configures command-line arguments including paths, MCL parameters,
 * and clustering options for room segmentation.
 * 
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_segment_rooms(CLI::App &app) {

  auto opt = std::make_shared<SubcommandSegRoomsOptions>();
  auto *sub = app.add_subcommand(
      "rooms",
      "Segment rooms using Markov clustering based on visual relationships.");

  sub->add_option("cloud", opt->cloud_path_in,
                  "Path to the input point cloud file.")
      //->required()
      // ->check(CLI::ExistingFile)
      ->default_val(opt->cloud_path_in);

  // sub->add_option("normals", opt->normals_path_in,
  //                 "Path to the input normals file.")
  //     //->required()
  //     // ->check(CLI::ExistingFile)
  //     ->default_val(opt->normals_path_in);

  sub->add_option("planes", opt->planes_path_in,
                  "Path to the input planes file.")
      ->default_val(opt->planes_path_in);

  sub->add_option("plane-centroids", opt->plane_centroids_path_in,
                  "Path to the input plane centroids file.")
      ->default_val(opt->plane_centroids_path_in);

  sub->add_option("plane-normals", opt->plane_normals_path_in,
                  "Path to the input plane normals file.")
      ->default_val(opt->plane_normals_path_in);

  sub->add_option("rooms", opt->rooms_path_out,
                  "Path to save the output room labels file")
      ->default_val(opt->rooms_path_out);

  sub->add_option("-i, --inflation", opt->inflation,
                  "Inflation factor for MCL algorithm. "
                  "Higher values lead to more clusters. "
                  "Common values: 1.4, 1.6, 2.0, 3.0, 4.0")
      ->default_val(opt->inflation)
      ->check(CLI::Range(0.0, 10.0));

  sub->add_option("-e, --expansion", opt->expansion,
                  "Expansion factor for MCL algorithm.")
      ->default_val(opt->expansion)
      ->check(CLI::Range(1, 10));

  sub->add_option("-p, --pruning-threshold", opt->pruning_threshold,
                  "Pruning threshold for MCL algorithm.")
      ->default_val(opt->pruning_threshold)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-c, --convergence-threshold", opt->convergence_threshold,
                  "Convergence threshold for MCL algorithm.")
      ->default_val(opt->convergence_threshold)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-m, --max-iter", opt->max_iter,
                  "Maximum number of iterations for MCL algorithm.")
      ->default_val(opt->max_iter)
      ->check(CLI::Range(1, 1000));

  sub->add_option("-g, --grid-size", opt->grid_size,
                  "Grid size for spatial discretization in MCL algorithm.")
      ->default_val(opt->grid_size)
      ->check(CLI::Range(0.01, 10.0));

  sub->callback([opt]() {
    spdlog::trace("calling seg-rooms subcommand");
    return run_subcommand_segment_rooms(*opt);
  });
}

/**
 * @brief Execute room segmentation using Markov clustering.
 * 
 * Loads point cloud, plane labels, and plane data, then performs room
 * segmentation using MCL algorithm based on visual relations.
 * 
 * @param opt Options containing file paths and MCL parameters.
 * @return Exit code (0 on success).
 */
int run_subcommand_segment_rooms(SubcommandSegRoomsOptions const &opt) {

  spdlog::trace("Load the point cloud from disk");
  CloudPtr cloud(new Cloud);
  pcl::io::load<PointT>(opt.cloud_path_in.string(), *cloud);

  spdlog::trace("Load the planes, plane centroids and plane normals from "
                "disk");
  CloudLPtr planes(new CloudL);
  pcl::io::load<LabelT>(opt.planes_path_in.string(), *planes);

  spdlog::trace("Load the plane centroids and plane normals from disk");
  CloudLocPtr plane_centroids(new CloudLoc);
  pcl::io::load<LocT>(opt.plane_centroids_path_in.string(), *plane_centroids);

  spdlog::trace("Load the plane normals from disk");
  CloudNPtr plane_normals(new CloudN);
  pcl::io::load<NormalT>(opt.plane_normals_path_in.string(), *plane_normals);

  assert(planes->size() == cloud->size() &&
         "Planes and cloud size must be equal");

  std::set<int> unique_labels{};
  for (const auto &point : planes->points)
    unique_labels.insert(point.label);
  spdlog::debug("Unique plane labels found: {}",
                fmt::join(unique_labels, ", "));

  // FIXME: Label encoding offset may cause indexing errors for unlabeled points
  // category=Geometry estimate=2h
  // Current code uses label-1 as index into plane_normals vector (line 169).
  // If unlabeled points (label < 1) exist and are not skipped properly, this
  // could cause off-by-one indexing errors. Need to verify that:
  // 1. All unlabeled points have label < 1 (currently checking this)
  // 2. Plane labels start at 1 (not 0) consistently throughout pipeline
  // 3. plane_normals vector size matches max(label) not unique label count
  // See MEMORY.md for label encoding conventions: -1 for background in API
  spdlog::trace("Create normals cloud for all points based on plane labels");
  CloudNPtr normals(new CloudN);
  normals->resize(planes->size());
  normals->width = planes->width;
  normals->height = planes->height;
  for (size_t i = 0; i < planes->points.size(); ++i) {
    if (planes->points[i].label < 1)
      continue;
    normals->points[i] = plane_normals->points[planes->points[i].label - 1];
  }
  spdlog::trace("Calling segment_rooms");

  using namespace ReUseX::geometry;
  ReUseX::geometry::SegmentRoomsRequest request;
  request.cloud = cloud;
  request.normals = normals;
  request.planes = planes;
  request.grid_size = opt.grid_size;
  request.inflation = opt.inflation;
  request.expansion = opt.expansion;
  request.pruning_threshold = opt.pruning_threshold;
  request.convergence_threshold = opt.convergence_threshold;
  request.max_iter = opt.max_iter;

  auto labels = ReUseX::geometry::segment_rooms(request);

  spdlog::trace("Save the point cloud with planes to dist at: {}",
                opt.rooms_path_out.string());
  pcl::io::savePCDFileBinary(opt.rooms_path_out, *labels);

  return 0;
}
