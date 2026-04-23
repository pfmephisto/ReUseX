// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/rooms.hpp"
#include "filter_utils.hpp"
#include "spdmon.hpp"
#include "validation.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/utils/fmt_formatter.hpp>

#include <reusex/geometry/segment_rooms.hpp>

#include <CLI/CLI.hpp>
#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/pca.h>
#include <pcl/correspondence.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace fs = std::filesystem;

/**
 * @brief Setup CLI options for the room segmentation subcommand.
 *
 * Configures command-line arguments including paths, Leiden parameters,
 * and clustering options for room segmentation.
 *
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_create_rooms(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandSegRoomsOptions>();
  auto *sub = app.add_subcommand(
      "rooms",
      "Segment rooms using Leiden clustering");

  sub->footer(R"(
DESCRIPTION:
  Performs room segmentation using the Leiden community detection algorithm
  based on spatial relationships and visual connections between planar surfaces.
  Groups planes into distinct room volumes using graph-based clustering.

EXAMPLES:
  rux create rooms                     # Default settings (resolution=1.0)
  rux create rooms -r 1.5              # More clusters (higher resolution)
  rux create rooms -r 0.5 -g 0.1       # Fewer clusters, finer grid
  rux create rooms -f 'planes >= 5'    # Filter to specific planes

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct point clouds
  3. rux create planes                 # Segment planar surfaces
  4. rux create rooms                  # Segment rooms from planes
  5. rux create mesh                   # Generate 3D mesh

NOTES:
  - Requires 'cloud', 'planes', 'plane_centroids', 'plane_normals' in project
  - Resolution parameter controls cluster granularity (higher = more rooms)
  - Grid size affects spatial discretization (default: 0.2m)
  - Filter syntax supports boolean expressions: 'planes in [1,2] || rooms == 5'
  - Output saved as 'rooms' label cloud in project database
)");

  sub->add_option("-r, --resolution", opt->resolution,
                  "Leiden resolution parameter. "
                  "Higher values lead to more clusters. "
                  "Common values: 0.5, 1.0, 1.5, 2.0")
      ->default_val(opt->resolution)
      ->check(CLI::Range(0.0, 10.0));

  sub->add_option("-b, --beta", opt->beta,
                  "Leiden beta parameter for refinement phase randomness. "
                  "Lower values = more deterministic.")
      ->default_val(opt->beta)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-m, --max-iter", opt->max_iter,
                  "Maximum number of Leiden iterations. "
                  "Negative value = iterate until convergence.")
      ->default_val(opt->max_iter)
      ->check(CLI::Range(-1, 1000));

  sub->add_option("-g, --grid-size", opt->grid_size,
                  "Grid size for spatial discretization.")
      ->default_val(opt->grid_size)
      ->check(CLI::Range(0.01, 10.0));

  sub->add_option("-f, --filter", opt->filter_expr,
                  "Filter expression to limit processing to specific labeled points.\n"
                  "Syntax: <cloud_name> <op> <value(s)>\n"
                  "Examples:\n"
                  "  -f 'planes in [1, 2, 5]'        # Filter to labels 1, 2, 5 from planes cloud\n"
                  "  -f 'rooms == 3'                 # Only process room 3\n"
                  "  -f 'planes in [1,2] || rooms == 5'  # Combine multiple clouds\n"
                  "  -f 'planes >= 10 && planes <= 20'   # Range filter")
      ->default_val("");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling seg-rooms subcommand");
    return run_subcommand_segment_rooms(*opt, *global_opt);
  });
}

/**
 * @brief Execute room segmentation using Leiden community detection.
 *
 * Loads point cloud, plane labels, and plane data from ProjectDB, then
 * performs room segmentation using Leiden algorithm based on visual relations.
 *
 * @param opt Options containing project path and Leiden parameters.
 * @return Exit code (RuxError::SUCCESS on success).
 */
int run_subcommand_segment_rooms(SubcommandSegRoomsOptions const &opt, const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Segmenting rooms in project: {}", project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    // Pre-flight validation: check for planes prerequisites
    auto validation = rux::validation::validate_rooms_prerequisites(db);
    if (!validation) {
      spdlog::error("{}", validation.error_message);
      spdlog::info("Resolution: {}", validation.resolution_hint);
      return RuxError::INVALID_ARGUMENT;
    }

    int logId = db.log_pipeline_start("segment_rooms",
        fmt::format(R"({{"grid_size":{},"resolution":{},"beta":{},"max_iter":{}}})",
                    opt.grid_size, opt.resolution, opt.beta, opt.max_iter));

    spdlog::trace("Loading point cloud and plane data from ProjectDB");
    auto cloud = db.point_cloud_xyzrgb("cloud");
    auto planes = db.point_cloud_label("planes");
    auto plane_centroids = db.point_cloud_xyz("plane_centroids");
    auto plane_normals = db.point_cloud_normal("plane_normals");

    if (planes->size() != cloud->size()) {
      spdlog::error("Point cloud size mismatch: cloud={}, planes={}",
                    cloud->size(), planes->size());
      db.log_pipeline_end(logId, false, "Size validation failed");
      return RuxError::INVALID_ARGUMENT;
    }

    std::set<int> unique_labels{};
    for (const auto &point : planes->points)
      unique_labels.insert(point.label);
    spdlog::debug("Unique plane labels found: {}",
                  fmt::join(unique_labels, ", "));

    // FIXME: Label encoding offset may cause indexing errors for unlabeled points
    // category=Geometry estimate=2h
    // Current code uses label-1 as index into plane_normals vector (line below).
    // If unlabeled points (label < 1) exist and are not skipped properly, this
    // could cause off-by-one indexing errors. Need to verify that:
    // 1. All unlabeled points have label < 1 (currently checking this)
    // 2. Plane labels start at 1 (not 0) consistently throughout pipeline
    // 3. plane_normals vector size matches max(label) not unique label count
    // See MEMORY.md for label encoding conventions: -1 for background in API
    spdlog::trace("Creating per-point normals from plane labels");
    CloudNPtr normals(new CloudN);
    normals->resize(planes->size());
    normals->width = planes->width;
    normals->height = planes->height;
    for (size_t i = 0; i < planes->points.size(); ++i) {
      if (planes->points[i].label < 1)
        continue;
      normals->points[i] = plane_normals->points[planes->points[i].label - 1];
    }

    spdlog::trace("Running room segmentation algorithm");
    using namespace reusex::geometry;

    // Build options struct
    reusex::geometry::SegmentRoomsOptions options;
    options.grid_size = opt.grid_size;
    options.resolution = opt.resolution;
    options.beta = opt.beta;
    options.max_iter = opt.max_iter;

    // Evaluate filter if provided
    if (!opt.filter_expr.empty()) {
      try {
        options.filter = rux::filters::evaluate_filter(opt.filter_expr, db, cloud->size());
      } catch (const std::exception &e) {
        spdlog::error("Filter evaluation failed: {}", e.what());
        return RuxError::INVALID_ARGUMENT;
      }
    }

    auto labels = reusex::geometry::segment_rooms(cloud, normals, planes, options);

    spdlog::trace("Saving room labels to ProjectDB");
    db.save_point_cloud("rooms", *labels, "segment_rooms");

    spdlog::info("Room segmentation complete");

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Room segmentation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
