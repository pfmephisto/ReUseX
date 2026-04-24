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
void setup_subcommand_create_planes(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandSegPlanesOptions>();
  auto *sub =
      app.add_subcommand("planes", "Detect and segment planar surfaces");

  sub->footer(R"(
DESCRIPTION:
  Detects and segments planar surfaces (walls, floors, ceilings) in 3D
  point clouds using multi-scale region growing. Identifies major structural
  planes and assigns a unique label to each detected surface. Essential
  preprocessing step for room segmentation and mesh generation.

EXAMPLES:
  rux create planes                    # Default settings
  rux create planes -a 15 -d 0.05      # Tighter angle/distance
  rux create planes -m 1000            # Larger minimum cluster
  rux create planes -r 0.15            # Larger search radius

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct point clouds
  3. rux create planes                 # Segment planar surfaces
  4. rux create rooms                  # Segment rooms from planes
  5. rux view                          # Visualize results

NOTES:
  - Requires 'cloud' and 'normals' in project database
  - Angle threshold: surface normal similarity (degrees, default: 25°)
  - Distance threshold: max point-to-plane distance (meters, default: 0.1m)
  - Minimum cluster size affects noise filtering (default: ~2 sqm @ 2cm res)
  - Output: 'planes', 'plane_centroids', 'plane_normals' saved to project
  - Filter syntax supports expressions like 'rooms == 3'
)");

  sub->get_formatter()->column_width(40);

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
      ->check(CLI::Range(0.0, 5.0));

  sub->add_option("-i, --interval-0", opt->interval_0,
                  "Initial interval for plane update")
      ->default_val(opt->interval_0)
      ->check(CLI::Range(1.0, 10000.0));

  sub->add_option("--interval-factor", opt->interval_factor,
                  "Factor for interval update")
      ->default_val(opt->interval_factor)
      ->check(CLI::Range(1.0, 10.0));

  sub->add_option(
         "-f, --filter", opt->filter_expr,
         "Filter expression to limit processing to specific labeled points.\n"
         "Syntax: <cloud_name> <op> <value(s)>\n"
         "Examples:\n"
         "  -f 'planes in [1, 2, 5]'        # Filter to labels 1, 2, 5 from "
         "planes cloud\n"
         "  -f 'rooms == 3'                 # Only process room 3\n"
         "  -f 'planes in [1,2] || rooms == 5'  # Combine multiple clouds\n"
         "  -f 'planes >= 10 && planes <= 20'   # Range filter")
      ->default_val("");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling seg-planes subcommand");
    return run_subcommand_segment_planes(*opt, *global_opt);
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
int run_subcommand_segment_planes(SubcommandSegPlanesOptions const &opt,
                                  const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Segmenting planes in project: {}", project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    // Pre-flight validation: check for cloud and normals
    auto validation = rux::validation::validate_planes_prerequisites(db);
    if (!validation) {
      spdlog::error("{}", validation.error_message);
      spdlog::info("Resolution: {}", validation.resolution_hint);
      return RuxError::INVALID_ARGUMENT;
    }

    int logId = db.log_pipeline_start(
        "segment_planes",
        fmt::format(
            R"({{"angle_threshold":{},"plane_dist_threshold":{},"min_inliers":{},"radius":{},"interval_0":{},"interval_factor":{}}})",
            opt.angle_threshold, opt.plane_dist_threshold, opt.minInliers,
            opt.radius, opt.interval_0, opt.interval_factor));

    spdlog::trace("Loading point cloud and normals from ProjectDB");
    auto cloud = db.point_cloud_xyzrgb("cloud");
    auto normals = db.point_cloud_normal("normals");

    using namespace reusex::geometry;

    // Build options struct
    reusex::geometry::SegmentPlanesOptions options;
    options.angle_threshold = opt.angle_threshold;
    options.plane_dist_threshold = opt.plane_dist_threshold;
    options.min_inliers = opt.minInliers;
    options.radius = opt.radius;
    options.interval_0 = opt.interval_0;
    options.interval_factor = opt.interval_factor;

    // Evaluate filter if provided
    if (!opt.filter_expr.empty()) {
      try {
        options.filter =
            rux::filters::evaluate_filter(opt.filter_expr, db, cloud->size());
      } catch (const std::exception &e) {
        spdlog::error("Filter evaluation failed: {}", e.what());
        return RuxError::INVALID_ARGUMENT;
      }
    }

    spdlog::trace("Running plane segmentation algorithm");
    auto [labels, centroids, plane_normals] =
        reusex::geometry::segment_planes(cloud, normals, options);

    spdlog::trace("Saving results to ProjectDB");
    db.save_point_cloud("planes", *labels, "segment_planes");
    db.save_point_cloud("plane_centroids", *centroids, "segment_planes");
    db.save_point_cloud("plane_normals", *plane_normals, "segment_planes");

    spdlog::info("Plane segmentation complete: {} planes detected",
                 centroids->size());

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Plane segmentation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
