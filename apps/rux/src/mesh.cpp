// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mesh.hpp"
#include "processing_observer.hpp"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

#include <fmt/format.h>
#include <fmt/std.h>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/mesh.hpp>
#include <reusex/io/reusex.hpp>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <range/v3/to_container.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>

#include <algorithm>
#include <filesystem>
#include <latch>
#include <mutex>
#include <queue>
#include <thread>

namespace fs = std::filesystem;

void setup_subcommand_mesh(CLI::App &app) {

  auto opt = std::make_shared<SubcommandMeshOptions>();
  auto *sub = app.add_subcommand(
      "mesh",
      "Generate a mesh by computing best-fit volumes from segmented planes.");

  sub->add_option("project", opt->project,
                  "Path to the .rux project file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("--output-name", opt->output_mesh_name,
                  "Name for the output mesh in ProjectDB")
      ->default_val(opt->output_mesh_name);

  sub->add_option("-a, --angle", opt->angle_threshold,
                  "Angle threshold for plane pairing in degrees.")
      ->default_val(opt->angle_threshold)
      ->check(CLI::Range(1.0, 180.0));

  sub->add_option("-l, --distance", opt->distance_threshold,
                  "Maximum distance threshold for plane pairing.")
      ->default_val(opt->distance_threshold)
      ->check(CLI::Range(0.001, 3.0));

  sub->add_option("-t, --threshold", opt->search_threshold,
                  "Search distance threshold for finding nearby planes.")
      ->default_val(opt->search_threshold)
      ->check(CLI::Range(0.01, 10.0));

  sub->add_option("-o, --offset", opt->new_plane_offset,
                  "Offset distance for new plane creation.")
      ->default_val(opt->new_plane_offset)
      ->check(CLI::Range(0.01, 1.0));

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_mesh");
    return run_subcommand_mesh(*opt);
  });
};

int run_subcommand_mesh(SubcommandMeshOptions const &opt) {
  spdlog::info("Generating mesh from project: {}", opt.project.string());

  // rux::VizualizationObserver
  auto &observer = rux::get_processing_observer();
  observer.viewer_request_viewports(4);

  try {
    ReUseX::ProjectDB db(opt.project);

    int logId = db.log_pipeline_start("mesh_generation",
        fmt::format(R"({{"angle_threshold":{},"distance_threshold":{},"search_threshold":{},"new_plane_offset":{}}})",
                    opt.angle_threshold, opt.distance_threshold,
                    opt.search_threshold, opt.new_plane_offset));

    spdlog::trace("Loading point clouds from ProjectDB");
    auto cloud = db.point_cloud_xyzrgb("cloud");
    auto normals = db.point_cloud_normal("normals");
    auto rooms = db.point_cloud_label("rooms");
    auto plane_labels = db.point_cloud_label("planes");
    auto plane_centroids = db.point_cloud_xyz("plane_centroids");
    auto plane_normals = db.point_cloud_normal("plane_normals");

    // TODO: Add comprehensive input size validation with detailed error messages
    // category=CLI estimate=30m
    // Current validation only checks a subset of input files. Should validate all:
    // 1. Check cloud, rooms, normals, plane_labels all have same size
    // 2. Verify plane_normals and plane_centroids have expected dimensions
    // 3. Provide specific error message showing actual vs expected sizes
    // 4. Add early validation before heavy processing to fail fast
    if (rooms->size() != cloud->size() || normals->size() != cloud->size()) {
      spdlog::error("Point cloud size mismatch: cloud={}, normals={}, rooms={}",
                    cloud->size(), normals->size(), rooms->size());
      db.log_pipeline_end(logId, false, "Size validation failed");
      return RuxError::INVALID_ARGUMENT;
    }

    spdlog::trace("Processing plane data");
    auto [planes, centroids, inliers] =
        ReUseX::io::getPlanes(plane_labels, plane_normals, plane_centroids);

    ReUseX::geometry::MeshOptions options;
    options.search_threshold = opt.search_threshold;
    options.new_plane_offset = opt.new_plane_offset;

    spdlog::trace("Generating mesh geometry");
    pcl::PolygonMeshPtr mesh = ReUseX::geometry::mesh(
        cloud, normals, planes, centroids, inliers, rooms, options);

    spdlog::trace("Saving mesh to ProjectDB");
    db.save_mesh(opt.output_mesh_name, *mesh, "mesh_generation");

    spdlog::info("Mesh '{}' saved to ProjectDB", opt.output_mesh_name);

    db.log_pipeline_end(logId, true);
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Mesh generation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
