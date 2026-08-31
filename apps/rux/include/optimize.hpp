// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

/// Options for the `rux optimize` subcommand (global plane-landmark pose-graph
/// optimization). Defaults mirror reusex::geometry::PlaneGraphOptions so the
/// library remains the single source of truth (docs/STANDARDS.md §4).
struct SubcommandOptimizeOptions {
  // Per-frame plane detection.
  int max_planes_per_frame = 4;
  int min_plane_inliers = 200;
  float ransac_distance = 0.02f;
  float ransac_normal_angle = 20.0f;
  int ransac_iterations = 200;

  // Cross-frame association.
  float assoc_normal_angle = 10.0f;
  float assoc_distance = 0.10f;
  int min_observations = 5;

  // Factor-graph noise.
  float odometry_sigma_rot = 0.05f;
  float odometry_sigma_trans = 0.10f;
  float plane_sigma_normal = 0.05f;
  float plane_sigma_distance = 0.03f;
  float prior_sigma_rot = 0.001f;
  float prior_sigma_trans = 0.001f;

  // Solver.
  bool no_gnc = false; ///< disable Graduated Non-Convexity (plain LM)
  float gnc_inlier_cost = 5.67f;
  int iterations = 100;
  unsigned seed = 42;

  // Surfel extraction (shared with `rux register`).
  float surfel_voxel = 0.03f;
  float min_distance = 0.0f;
  float max_distance = 4.0f;
  int sampling_factor = 8;
  int confidence_threshold = 2;

  bool dry_run = false;
};

void setup_subcommand_optimize(CLI::App &app,
                               std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_optimize(SubcommandOptimizeOptions const &opt,
                            const RuxOptions &global_opt);
