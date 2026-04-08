// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <string>

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandMeshOptions {
  fs::path project;  ///< Path to .rux project file (required, positional)
  std::string output_mesh_name = "mesh";  ///< Mesh name in ProjectDB

  float grid_size = GlobalParams::grid_size;

  double angle_threshold = 25.0;
  double distance_threshold = 0.2;

  double search_threshold = 0.60;
  double new_plane_offset = 0.25;

  std::string filter_expr;  ///< Filter expression to limit processing
};

// Function declarations.
void setup_subcommand_create_mesh(CLI::App &app);
int run_subcommand_mesh(SubcommandMeshOptions const &opt);
