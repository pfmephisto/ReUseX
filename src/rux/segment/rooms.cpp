// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/segment/rooms.hpp"
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
#include <pcl/visualization/pcl_visualizer.h>

namespace fs = std::filesystem;

void setup_subcommand_segment_rooms(CLI::App &app) {

  auto opt = std::make_shared<SubcommandSegRoomsOptions>();
  auto *sub = app.add_subcommand(
      "rooms",
      "This tool applies the Markov clustering algorithm to a point cloud "
      "based in the visual relation between the points.");

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

  sub->add_option("planes", opt->planes_path_in,
                  "Path to the input planes file.")
      //->required()
      // ->check(CLI::ExistingFile)
      ->default_val(opt->planes_path_in);

  sub->add_option("rooms", opt->rooms_path_out,
                  "Path to the output point cloud file")
      ->default_val(opt->rooms_path_out);

  sub->add_option("-i, --inflation", opt->inflation,
                  "The inflation factor for the MCL algorithm. "
                  "Higher values lead to more clusters. "
                  "Common values are: "
                  "1.4 1.6 2 3 4")
      ->default_val(opt->inflation)
      ->check(CLI::Range(0.0, 10.0));

  sub->add_option("-e, --expansion", opt->expansion,
                  "The expansion factor for the MCL algorithm.")
      ->default_val(opt->expansion)
      ->check(CLI::Range(1, 10));

  sub->add_option("-p, --pruning-threshold", opt->pruning_threshold,
                  "The pruning threshold for the MCL algorithm.")
      ->default_val(opt->pruning_threshold)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-c, --convergence-threshold", opt->convergence_threshold,
                  "The convergence threshold for the MCL algorithm.")
      ->default_val(opt->convergence_threshold)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-m, --max-iter", opt->max_iter,
                  "The maximum number of iterations for the MCL algorithm.")
      ->default_val(opt->max_iter)
      ->check(CLI::Range(1, 1000));

  sub->add_option("-g, --grid-size", opt->grid_size,
                  "The grid size for the MCL algorithm.")
      ->default_val(opt->grid_size)
      ->check(CLI::Range(0.01, 10.0));

  sub->add_flag("-d, --visualize", opt->visualize,
                "Visualize the clustering process.")
      ->default_val(opt->visualize);

  sub->callback([opt]() {
    spdlog::trace("calling seg-rooms subcommand");
    return run_subcommand_segment_rooms(*opt);
  });
}

int run_subcommand_segment_rooms(SubcommandSegRoomsOptions const &opt) {

  spdlog::trace("Load the point cloud from disk");
  CloudPtr cloud(new Cloud);
  pcl::io::load<PointT>(opt.cloud_path_in.string(), *cloud);

  spdlog::trace("Load the normals from disk");
  CloudNPtr normals(new CloudN);
  pcl::io::load<NormalT>(opt.normals_path_in.string(), *normals);

  using namespace ReUseX::geometry;
  auto labels = ReUseX::geometry::segment_rooms(
      cloud, normals, _grid_size = opt.grid_size, _inflation = opt.inflation,
      _expansion = opt.expansion, _pruning_threshold = opt.pruning_threshold,
      _convergence_threshold = opt.convergence_threshold,
      _max_iter = opt.max_iter, _visualize = opt.visualize);

  spdlog::trace("Save the point cloud with planes to dist at: {}",
                opt.rooms_path_out.string());
  pcl::io::savePCDFileBinary(opt.rooms_path_out, *labels);

  return 0;
}
