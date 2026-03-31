// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

/**
 * @brief CLI options for instance segmentation subcommand
 *
 * Usage: rux create instances project.rux [OPTIONS]
 *
 * Applies Euclidean clustering to semantic labels to generate instance-level
 * segmentation. For example, separates multiple walls with the same semantic
 * label into wall_1, wall_2, etc.
 */
struct SubcommandSegInstancesOptions {
  /// Path to .rux project file (required)
  fs::path project;

  /// Euclidean distance threshold for clustering (meters)
  float cluster_tolerance = 0.5F;

  /// Minimum points per instance cluster
  int min_cluster_size = 50;

  /// Maximum points per instance cluster
  int max_cluster_size = 1000000;

  /// Name of input semantic labels cloud in ProjectDB
  std::string semantic_cloud_name = "labels";

  /// Name of output instance labels cloud in ProjectDB
  std::string output_cloud_name = "instances";

  /// Optional: only process these semantic labels (comma-separated list)
  std::vector<uint32_t> labels_to_process;
};

/**
 * @brief Setup CLI11 subcommand for instance segmentation
 */
void setup_subcommand_create_instances(CLI::App &app);

/**
 * @brief Execute instance segmentation subcommand
 */
int run_subcommand_segment_instances(const SubcommandSegInstancesOptions &opt);
