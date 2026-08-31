// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>

#include <memory>
#include <string>

/// Options for the ground-truth accuracy analysis subcommand.
struct SubcommandAnalyzeAccuracyOptions {
  std::string gt_path;              ///< Ground-truth PLY point cloud (required)
  std::string cloud_name = "cloud"; ///< Reconstruction cloud to score
  float threshold = 0.05f;          ///< F-score inlier threshold [m]
  float gt_voxel = 0.01f;           ///< GT downsample leaf [m], <=0 disables
  std::string output_path;          ///< JSON output file ("" = stdout)
};

/**
 * @brief Setup the analyze accuracy subcommand in the CLI application.
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_analyze_accuracy(CLI::App &app,
                                       std::shared_ptr<RuxOptions> global_opt);

/**
 * @brief Run the analyze accuracy subcommand with given options.
 * @return Exit code (0 for success).
 */
int run_subcommand_analyze_accuracy(SubcommandAnalyzeAccuracyOptions const &opt,
                                    const RuxOptions &global_opt);
