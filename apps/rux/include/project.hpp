// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"
#include <CLI/CLI.hpp>
#include <string>

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandProjectOptions {
  fs::path cloud_path_in = GlobalParams::cloud;
  fs::path database_path_in = GlobalParams::db;
  fs::path labels_path_out = GlobalParams::labels;
};

// Function declarations.
void setup_subcommand_project(CLI::App &app);
int run_subcommand_project(SubcommandProjectOptions const &opt);
