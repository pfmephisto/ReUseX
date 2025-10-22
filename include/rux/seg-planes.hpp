// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"
#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandSegPlanesOptions {
  fs::path cloud_path_in = GlobalParams::cloud;
  fs::path normals_path_in = GlobalParams::normals;
  fs::path planes_path_out = GlobalParams::planes;

  float angle_threshold = 25.0f;
  float plane_dist_threshold = 0.07;
  int minInliers = 1000;
  // 2 * (1 / 0.02) * (1 / 0.02); // ca 2sqm in 2cm resolution of point cloud
  float radius = 0.5;
  float interval_0 = 16;
  float interval_factor = 1.5;

  bool visualize = GlobalParams::visualize;
};

// Function declarations.
void setup_subcommand_seg_planes(CLI::App &app);
int run_subcommand_seg_planes(SubcommandSegPlanesOptions const &opt);
