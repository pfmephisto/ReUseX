// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandSegRoomsOptions {
  fs::path cloud_path_in = GlobalParams::cloud;
  // fs::path normals_path_in = GlobalParams::normals;
  fs::path planes_path_in = GlobalParams::planes;
  fs::path plane_centroids_path_in = GlobalParams::plane_centroids;
  fs::path plane_normals_path_in = GlobalParams::plane_normals;

  fs::path rooms_path_out = GlobalParams::rooms;

  int expansion = 2;
  double inflation = 2;
  double pruning_threshold = 0.0001;
  double convergence_threshold = 1e-8;
  int max_iter = 100;

  float grid_size = GlobalParams::grid_size;

  bool visualize = GlobalParams::visualize;
};

// Function declarations.
void setup_subcommand_segment_rooms(CLI::App &app);
int run_subcommand_segment_rooms(SubcommandSegRoomsOptions const &opt);
