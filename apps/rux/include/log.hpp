// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>

namespace fs = std::filesystem;

/// Options for the log subcommand
struct SubcommandLogOptions {
  bool json_output = false;
  int limit = 0; // 0 = no limit
};

// Function declarations
void setup_subcommand_log(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_log(SubcommandLogOptions const &opt, const RuxOptions &global_opt);
