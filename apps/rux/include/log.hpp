// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>

namespace fs = std::filesystem;

/// Options for the log subcommand
struct SubcommandLogOptions {
  fs::path project = GlobalParams::project_db;
  bool json_output = false;
  int limit = 0;  // 0 = no limit
};

// Function declarations
void setup_subcommand_log(CLI::App &app);
int run_subcommand_log(SubcommandLogOptions const &opt);
