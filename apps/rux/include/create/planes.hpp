// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options for plane segmentation subcommand.
struct SubcommandSegPlanesOptions {
  fs::path project;  ///< Path to .rux project file (required, positional)

  float angle_threshold = 25.0f;  ///< Angular threshold for plane detection (degrees)
  float plane_dist_threshold = 0.07;  ///< Distance threshold for plane detection
  int minInliers = 1000;  ///< Minimum number of inliers for a valid plane
  // 2 * (1 / 0.02) * (1 / 0.02); // ca 2sqm in 2cm resolution of point cloud
  float radius = 0.5;  ///< Search radius for region growing
  float interval_0 = 16;  ///< Initial interval for multi-scale processing
  float interval_factor = 1.5;  ///< Factor for interval scaling

};

/**
 * @brief Setup the segment planes subcommand in the CLI application.
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_create_planes(CLI::App &app);

/**
 * @brief Run the segment planes subcommand with given options.
 * @param opt Options for plane segmentation.
 * @return Exit code (0 for success).
 */
int run_subcommand_segment_planes(SubcommandSegPlanesOptions const &opt);
