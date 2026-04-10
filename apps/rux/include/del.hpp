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
 * @brief Options for unified database del command
 */
struct DatabaseDelOptions {
  std::string path;
  bool force = false; // Required for wildcard deletions
  bool yes = false;   // Skip confirmation prompt
};

/**
 * @brief Setup unified database del subcommand
 *
 * Usage:
 *   rux del <project_file> <path> [--force] [--yes]
 *
 * Examples:
 *   rux del project.rux clouds.oldcloud
 *   rux del project.rux clouds.old_* --force
 *   rux del project.rux clouds.test_* --force --yes
 */
void setup_subcommand_del(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);

/**
 * @brief Execute unified database del command
 */
int run_subcommand_del(const DatabaseDelOptions &opt, const RuxOptions &global_opt);
