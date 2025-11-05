// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <vector>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandAssembleOptions {
  std::vector<fs::path> paths_in;
  fs::path db_path_out = GlobalParams::db;
};

// Function declarations.
void setup_subcommand_assemble(CLI::App &app);
int run_subcommand_assemble(SubcommandAssembleOptions const &opt);
