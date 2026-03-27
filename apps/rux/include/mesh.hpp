// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"
#include <CLI/CLI.hpp>
#include <string>

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandMeshOptions {
  fs::path cloud_path_in = GlobalParams::cloud;
  fs::path normals_path_in = GlobalParams::normals;
  fs::path planes_path_in = GlobalParams::planes;
  fs::path plane_centroids_path_in = GlobalParams::plane_centroids;
  fs::path plane_normals_path_in = GlobalParams::plane_normals;
  fs::path rooms_path_in = GlobalParams::rooms;
  fs::path output_out = fs::current_path() / "mesh.ply";

  float grid_size = GlobalParams::grid_size;

  double angle_threshold = 25.0;
  double distance_threshold = 0.2;

  double search_threshold = 0.60;
  double new_plane_offset = 0.25;
};

// Function declarations.
void setup_subcommand_mesh(CLI::App &app);
int run_subcommand_mesh(SubcommandMeshOptions const &opt);
