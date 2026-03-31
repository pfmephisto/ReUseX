// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

struct SubcommandAddCloudOptions {
  fs::path project = GlobalParams::project_db;
  std::string name;
  fs::path input_file;
  std::string stage;
};

// Function declarations.
void setup_subcommand_add_cloud(CLI::App &app);
int run_subcommand_add_cloud(SubcommandAddCloudOptions const &opt);
