// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>

namespace fs = std::filesystem;

/// Options for the info subcommand
struct SubcommandInfoOptions {
  bool json_output = false;
};

// Function declarations
void setup_subcommand_info(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_info(SubcommandInfoOptions const &opt, const RuxOptions &global_opt);
