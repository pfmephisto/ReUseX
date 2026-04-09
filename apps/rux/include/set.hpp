// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
#include <filesystem>
#include <optional>
#include <string>

/**
 * @brief Options for unified database set command
 */
struct DatabaseSetOptions {
  std::filesystem::path project_file;
  std::string path;
  std::optional<std::string> value; // Inline value (optional)
};

/**
 * @brief Setup unified database set subcommand
 *
 * Usage:
 *   rux set <project_file> <path> [value]
 *   rux set <project_file> <path> < file
 *   cat file | rux set <project_file> <path>
 *
 * Examples:
 *   rux set project.rux clouds.newscan < scan.pcd
 *   cat scan.pcd | rux set project.rux clouds.newscan
 *   rux set project.rux project.name "Historic Building"
 */
void setup_subcommand_set(CLI::App &app);

/**
 * @brief Execute unified database set command
 */
int run_subcommand_set(const DatabaseSetOptions &opt);
