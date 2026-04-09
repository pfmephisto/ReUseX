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

  float resolution = 1.0F;  ///< Leiden resolution parameter (cluster granularity)
  float beta = 0.01F;  ///< Leiden beta (refinement randomness)
  int max_iter = -1;  ///< Maximum iterations (-1 = until convergence)

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
