// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/planes.hpp"
#include "filter_utils.hpp"
#include "validation.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/segment_planes.hpp>

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/colors.h>
#include <pcl/filters/filter.h>

namespace fs = std::filesystem;

/**
 * @brief Setup CLI options for the plane segmentation subcommand.
 * 
 * Configures command-line arguments including cloud paths, thresholds,
 * and segmentation options for plane segmentation.
 * 
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_create_planes(CLI::App &app) {

  auto opt = std::make_shared<SubcommandSegPlanesOptions>();
  auto *sub = app.add_subcommand("planes", "Detect and segment planar surfaces (walls, floors, ceilings) in a point cloud");

  sub->get_formatter()->column_width(40);

  sub->add_option("project", opt->project,
                  "Path to the .rux project file.")
      ->required()
      ->check(CLI::ExistingFile);

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

  sub->add_option("--interval-factor", opt->interval_factor,
                  "Factor for interval update")
      ->default_val(opt->interval_factor)
      ->check(CLI::Range(1.0, 10.0));

  sub->add_option("-f, --filter", opt->filter_expr,
                  "Filter expression to limit processing to specific labeled points.\n"
                  "Syntax: <cloud_name> <op> <value(s)>\n"
                  "Examples:\n"
                  "  -f 'planes in [1, 2, 5]'        # Filter to labels 1, 2, 5 from planes cloud\n"
                  "  -f 'rooms == 3'                 # Only process room 3\n"
                  "  -f 'planes in [1,2] || rooms == 5'  # Combine multiple clouds\n"
                  "  -f 'planes >= 10 && planes <= 20'   # Range filter")
      ->default_val("");

  sub->callback([opt]() {
    spdlog::trace("calling seg-planes subcommand");
    return run_subcommand_segment_planes(*opt);
  });
}

/**
 * @brief Execute plane segmentation on a point cloud.
 *
 * Loads point cloud and normals from ProjectDB, performs multi-scale region
 * growing plane segmentation, and saves results to ProjectDB.
 *
 * @param opt Options containing project path and segmentation parameters.
 * @return Exit code (RuxError::SUCCESS on success).
 */
int run_subcommand_segment_planes(SubcommandSegPlanesOptions const &opt) {
  spdlog::info("Segmenting planes in project: {}", opt.project.string());

  try {
    ReUseX::ProjectDB db(opt.project);

    // Pre-flight validation: check for cloud and normals
    auto validation = rux::validation::validate_planes_prerequisites(db);
    if (!validation) {
      spdlog::error("{}", validation.error_message);
      spdlog::info("Resolution: {}", validation.resolution_hint);
      return RuxError::INVALID_ARGUMENT;
    }

    int logId = db.log_pipeline_start("segment_planes",
        fmt::format(R"({{"angle_threshold":{},"plane_dist_threshold":{},"min_inliers":{},"radius":{},"interval_0":{},"interval_factor":{}}})",
                    opt.angle_threshold, opt.plane_dist_threshold, opt.minInliers,
                    opt.radius, opt.interval_0, opt.interval_factor));

    spdlog::trace("Loading point cloud and normals from ProjectDB");
    auto cloud = db.point_cloud_xyzrgb("cloud");
    auto normals = db.point_cloud_normal("normals");

    using namespace ReUseX::geometry;

    // Build options struct
    ReUseX::geometry::SegmentPlanesOptions options;
    options.angle_threshold = opt.angle_threshold;
    options.plane_dist_threshold = opt.plane_dist_threshold;
    options.min_inliers = opt.minInliers;
    options.radius = opt.radius;
    options.interval_0 = opt.interval_0;
    options.interval_factor = opt.interval_factor;

    // Evaluate filter if provided
    if (!opt.filter_expr.empty()) {
      try {
        options.filter = rux::filters::evaluate_filter(opt.filter_expr, db, cloud->size());
      } catch (const std::exception &e) {
        spdlog::error("Filter evaluation failed: {}", e.what());
        return RuxError::INVALID_ARGUMENT;
      }
    }

    spdlog::trace("Running plane segmentation algorithm");
    auto [labels, centroids, plane_normals] =
        ReUseX::geometry::segment_planes(cloud, normals, options);

    spdlog::trace("Saving results to ProjectDB");
    db.save_point_cloud("planes", *labels, "segment_planes");
    db.save_point_cloud("plane_centroids", *centroids, "segment_planes");
    db.save_point_cloud("plane_normals", *plane_normals, "segment_planes");

    spdlog::info("Plane segmentation complete: {} planes detected", centroids->size());

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Plane segmentation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
