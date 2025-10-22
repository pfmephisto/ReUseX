// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"
#include <CLI/CLI.hpp>
#include <string>

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandCellcomplexOptions {
  fs::path cloud_path_in = GlobalParams::cloud;
  fs::path normals_path_in = GlobalParams::normals;
  fs::path planes_path_in = GlobalParams::planes;
  fs::path rooms_path_in = GlobalParams::rooms;
  fs::path output_out = fs::current_path() / "output.vtk";

  float grid_size = GlobalParams::grid_size;

  bool display = GlobalParams::visualize;
};

// Function declarations.
void setup_subcommand_cellcomplex(CLI::App &app);
int run_subcommand_cellcomplex(SubcommandCellcomplexOptions const &opt);
