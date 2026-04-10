// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

/**
 * @brief Options for unified database get command
 */
struct DatabaseGetOptions {
  std::string path;
  std::filesystem::path output_file;
  bool pretty = false;
};

/**
 * @brief Setup unified database get subcommand
 *
 * Usage:
 *   rux get <project_file> <path> [--output=file] [--pretty]
 *
 * Examples:
 *   rux get project.rux clouds
 *   rux get project.rux clouds.scan1 > output.pcd
 *   rux get project.rux clouds.scan1.metadata --pretty
 *   rux get project.rux clouds[0].point_count
 */
void setup_subcommand_get(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);

/**
 * @brief Execute unified database get command
 */
int run_subcommand_get(const DatabaseGetOptions &opt, const RuxOptions &global_opt);
