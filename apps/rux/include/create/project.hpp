// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandProjectOptions {};

// Function declarations.
void setup_subcommand_create_project(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_project(SubcommandProjectOptions const &opt, const RuxOptions &global_opt);
