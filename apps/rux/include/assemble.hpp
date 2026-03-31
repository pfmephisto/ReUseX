// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <vector>
namespace fs = std::filesystem;

/// Collection of all options for the assemble subcommand.
struct SubcommandAssembleOptions {
  std::vector<fs::path> paths_in;  ///< Input file paths to assemble
  fs::path db_path_out = fs::current_path() / "assembled.db";  ///< Output database path
};

/**
 * @brief Setup the assemble subcommand in the CLI application.
 * @param app CLI application to add the subcommand to.
 */
void setup_subcommand_assemble(CLI::App &app);

/**
 * @brief Run the assemble subcommand with given options.
 * @param opt Options for the assemble operation.
 * @return Exit code (0 for success).
 */
int run_subcommand_assemble(SubcommandAssembleOptions const &opt);
