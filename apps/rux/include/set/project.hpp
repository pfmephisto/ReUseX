// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

struct SubcommandSetProjectOptions {
  fs::path project = GlobalParams::project_db;
  std::string property;
  std::string value;
};

// Function declarations.
void setup_subcommand_set_project(CLI::App &app);
int run_subcommand_set_project(SubcommandSetProjectOptions const &opt);
