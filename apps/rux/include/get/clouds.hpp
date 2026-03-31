// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>

namespace fs = std::filesystem;

struct SubcommandGetCloudsOptions {
  fs::path project = GlobalParams::project_db;
  bool json_output = false;
  bool verbose = false;
};

// Function declarations.
void setup_subcommand_get_clouds(CLI::App &app);
int run_subcommand_get_clouds(SubcommandGetCloudsOptions const &opt);
