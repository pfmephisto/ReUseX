// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segment/rooms.hpp"
#include "pcl/markov_clustering.hpp"
#include "spdmon.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/utils/fmt_formatter.hpp>

#include <reusex/geometry/segment_rooms.hpp>

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

  sub->add_option("project", opt->project,
                  "Path to the .rux project file.")
      ->required()
      ->check(CLI::ExistingFile);

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
 * Loads point cloud, plane labels, and plane data from ProjectDB, then
 * performs room segmentation using MCL algorithm based on visual relations.
 *
 * @param opt Options containing project path and MCL parameters.
 * @return Exit code (RuxError::SUCCESS on success).
 */
int run_subcommand_segment_rooms(SubcommandSegRoomsOptions const &opt) {
  spdlog::info("Segmenting rooms in project: {}", opt.project.string());

  try {
    ReUseX::ProjectDB db(opt.project);

    int logId = db.log_pipeline_start("segment_rooms",
        fmt::format(R"({{"grid_size":{},"inflation":{},"expansion":{},"pruning_threshold":{},"convergence_threshold":{},"max_iter":{}}})",
                    opt.grid_size, opt.inflation, opt.expansion,
                    opt.pruning_threshold, opt.convergence_threshold, opt.max_iter));

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
