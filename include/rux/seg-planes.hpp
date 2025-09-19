// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandSegPlanesOptions {
  fs::path path_in;
  fs::path path_out = fs::current_path() / "planes.pcd";

  float angle_threshold = 25.0f;
  float plane_dist_threshold = 0.07;
  int minInliers = 1000;
  // 2 * (1 / 0.02) * (1 / 0.02); // ca 2sqm in 2cm resolution of point cloud
  float radius = 0.5;
  float interval_0 = 16;
  float interval_factor = 1.5;

  bool visualize = false;
};

// Function declarations.
void setup_subcommand_seg_planes(CLI::App &app);
int run_subcommand_seg_planes(SubcommandSegPlanesOptions const &opt);
