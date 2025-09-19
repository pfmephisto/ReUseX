// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
#include <string>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandSurfaceReconstructionOptions {
  fs::path path_in;
  fs::path path_in_labels;
  fs::path path_out = fs::current_path() / "mesh.ply";

  double fitting = 0.20;
  double coverage = 0.10;
  double complexity = 0.70;
};

// Function declarations.
void setup_subcommand_surface_reconstruction(CLI::App &app);
int run_subcommand_surface_reconstruction(
    SubcommandSurfaceReconstructionOptions const &opt);
