// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mesh.hpp"
#include "processing_observer.hpp"

#include <CLI/CLI.hpp>
#include <spdlog/spdlog.h>

// #include <fmt/color.h>
#include <fmt/std.h>

#include <reusex/geometry/mesh.hpp>
#include <reusex/io/reusex.hpp>
// #include <reusex/geometry/CellComplex.hpp>
// #include <reusex/geometry/Solidifier.hpp>
// #include <reusex/geometry/regularization.hpp>
// #include <reusex/geometry/utils.hpp>
// #include <reusex/visualize/pcl.hpp>

#include <reusex/visualize/Visualizer.hpp>

#include <pcl/common/common.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
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

  sub->add_option("cloud", opt->cloud_path_in, "Path to the input cloud file.")
      //->required()
      //->check(CLI::ExistingFile)
      ->default_val(opt->cloud_path_in);

  sub->add_option("normals", opt->normals_path_in,
                  "Path to the input normals file.")
      //->required()
      //->check(CLI::ExistingFile)
      ->default_val(opt->normals_path_in);

  sub->add_option("planes", opt->planes_path_in,
                  "Path to the input planes file.")
      //->required()
      //->check(CLI::ExistingFile)
      ->default_val(opt->planes_path_in);

  sub->add_option("plane-centroids", opt->plane_centroids_path_in,
                  "Path to the input plane centroids file.")
      ->default_val(opt->plane_centroids_path_in);

  sub->add_option("plane-normals", opt->plane_normals_path_in,
                  "Path to the input plane normals file.")
      ->default_val(opt->plane_normals_path_in);

  sub->add_option("rooms", opt->rooms_path_in,
                  "Path to the input room labels file.")
      //->required()
      // ->check(CLI::ExistingFile)
      ->default_val(opt->rooms_path_in);

  sub->add_option("output", opt->output_out,
                  "Path to the output mesh file. "
                  "Supported formats: .ply, .obj, .stl, .vtk")
      ->default_val(opt->output_out);

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

  // rux::VizualizationObserver
  auto &observer = rux::get_processing_observer();
  observer.viewer_request_viewports(4);

  assert(fs::exists(opt.cloud_path_in) &&
         "Input point cloud file does not exist");
  assert(fs::exists(opt.normals_path_in) &&
         "Input normals file does not exist");
  assert(fs::exists(opt.planes_path_in) && "Input planes file does not exist");
  assert(fs::exists(opt.rooms_path_in) && "Input rooms file does not exist");

  CloudPtr cloud(new Cloud);
  spdlog::trace("Reading {:<8} file: {}", "input", opt.cloud_path_in);
  pcl::io::load<PointT>(opt.cloud_path_in.string(), *cloud);

  CloudNPtr normals(new CloudN);
  spdlog::trace("Reading {:<8} file: {}", "normals", opt.normals_path_in);
  pcl::io::load<NormalT>(opt.normals_path_in.string(), *normals);

  CloudLPtr rooms(new CloudL);
  spdlog::trace("Reading {:<8} file: {}", "rooms", opt.rooms_path_in);
  pcl::io::load<LabelT>(opt.rooms_path_in.string(), *rooms);

  CloudLPtr plane_labels(new CloudL);
  spdlog::trace("Reading {:<8} file: {}", "plane labels", opt.planes_path_in);
  pcl::io::load<LabelT>(opt.planes_path_in.string(), *plane_labels);

  CloudLocPtr plane_centroids(new CloudLoc);
  spdlog::trace("Reading {:<8} file: {}", "plane centroids",
                opt.plane_centroids_path_in);
  pcl::io::load<LocT>(opt.plane_centroids_path_in.string(), *plane_centroids);

  CloudNPtr plane_normals(new CloudN);
  spdlog::trace("Reading {:<8} file: {}", "plane normals",
                opt.plane_normals_path_in);
  pcl::io::load<NormalT>(opt.plane_normals_path_in.string(), *plane_normals);

  // TODO: Add comprehensive input size validation with detailed error messages
  // category=CLI estimate=30m
  // Current validation only checks a subset of input files. Should validate
  // all:
  // 1. Check cloud, rooms, normals, plane_labels all have same size
  // 2. Verify plane_normals and plane_centroids have expected dimensions
  // 3. Provide specific error message showing actual vs expected sizes
  // 4. Add early validation before heavy processing to fail fast
  if (rooms->size() != cloud->size() || /*planes->size() != cloud->size() ||*/
      normals->size() != cloud->size()) {
    spdlog::error("Input files have inconsistent sizes:");
    return EXIT_FAILURE;
  }

  auto [planes, centroids, inliers] =
      ReUseX::io::getPlanes(plane_labels, plane_normals, plane_centroids);

  ReUseX::geometry::MeshOptions options;
  options.search_threshold = opt.search_threshold;
  options.new_plane_offset = opt.new_plane_offset;

  pcl::PolygonMeshPtr mesh = ReUseX::geometry::mesh(
      cloud, normals, planes, centroids, inliers, rooms, options);

  pcl::io::save(opt.output_out.string(), *mesh);

  return RuxError::SUCCESS;
}
