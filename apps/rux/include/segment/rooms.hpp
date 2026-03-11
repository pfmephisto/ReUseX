// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options for room segmentation subcommand.
struct SubcommandSegRoomsOptions {
  fs::path cloud_path_in = GlobalParams::cloud;  ///< Input point cloud path
  // fs::path normals_path_in = GlobalParams::normals;
  fs::path planes_path_in = GlobalParams::planes;  ///< Input planes path
  fs::path plane_centroids_path_in = GlobalParams::plane_centroids;  ///< Input plane centroids path
  fs::path plane_normals_path_in = GlobalParams::plane_normals;  ///< Input plane normals path

  fs::path rooms_path_out = GlobalParams::rooms;  ///< Output rooms path

  int expansion = 2;  ///< Cell expansion factor
  double inflation = 2;  ///< Inflation factor for room boundaries
  double pruning_threshold = 0.0001;  ///< Threshold for pruning small regions
  double convergence_threshold = 1e-8;  ///< Convergence threshold for optimization
  int max_iter = 100;  ///< Maximum number of iterations

  float grid_size = GlobalParams::grid_size;  ///< Grid cell size

  bool visualize = GlobalParams::visualize;  ///< Enable visualization
};

/**
 * @brief Setup the segment rooms subcommand in the CLI application.
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_segment_rooms(CLI::App &app);

/**
 * @brief Run the segment rooms subcommand with given options.
 * @param opt Options for room segmentation.
 * @return Exit code (0 for success).
 */
int run_subcommand_segment_rooms(SubcommandSegRoomsOptions const &opt);
