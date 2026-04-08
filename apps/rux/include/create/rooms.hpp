// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options for room segmentation subcommand.
struct SubcommandSegRoomsOptions {
  fs::path project;  ///< Path to .rux project file (required, positional)

  int expansion = 2;  ///< Cell expansion factor
  float inflation = 2.0F;  ///< Inflation factor for room boundaries
  float pruning_threshold = 0.0001F;  ///< Threshold for pruning small regions
  float convergence_threshold = 1e-8F;  ///< Convergence threshold for optimization
  int max_iter = 100;  ///< Maximum number of iterations

  float grid_size = GlobalParams::grid_size;  ///< Grid cell size

  std::string filter_expr;  ///< Filter expression to limit processing

};

/**
 * @brief Setup the segment rooms subcommand in the CLI application.
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_create_rooms(CLI::App &app);

/**
 * @brief Run the segment rooms subcommand with given options.
 * @param opt Options for room segmentation.
 * @return Exit code (0 for success).
 */
int run_subcommand_segment_rooms(SubcommandSegRoomsOptions const &opt);
