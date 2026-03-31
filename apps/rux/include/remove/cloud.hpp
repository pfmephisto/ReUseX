// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

struct SubcommandRemoveCloudOptions {
  fs::path project = GlobalParams::project_db;
  std::string name;
  bool force = false;
};

// Function declarations.
void setup_subcommand_remove_cloud(CLI::App &app);
int run_subcommand_remove_cloud(SubcommandRemoveCloudOptions const &opt);
