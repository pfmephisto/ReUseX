// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>

#include <memory>
#include <string>

/// Options for the reconstruction quality analysis subcommand.
struct SubcommandAnalyzeQualityOptions {
  std::string cloud_name = "cloud";   ///< Point cloud to analyze
  std::string planes_name = "planes"; ///< Plane label cloud
  std::size_t min_points = 100;       ///< Minimum points per reported plane
  std::string output_path;            ///< JSON output file ("" = stdout)
};

/**
 * @brief Setup the analyze quality subcommand in the CLI application.
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_analyze_quality(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt);

/**
 * @brief Run the analyze quality subcommand with given options.
 * @return Exit code (0 for success).
 */
int run_subcommand_analyze_quality(SubcommandAnalyzeQualityOptions const &opt,
                                   const RuxOptions &global_opt);
