// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandSegRoomsOptions {
  fs::path path_in;
  fs::path path_out = fs::current_path() / "clusters.pcd";

  int expansion = 2;
  double inflation = 2;
  double pruning_threshold = 0.0001;
  double convergence_threshold = 1e-8;
  int max_iter = 100;

  float grid_size = 0.2f;

  bool visualize = false;
};

// Function declarations.
void setup_subcommand_seg_rooms(CLI::App &app);
int run_subcommand_seg_rooms(SubcommandSegRoomsOptions const &opt);
