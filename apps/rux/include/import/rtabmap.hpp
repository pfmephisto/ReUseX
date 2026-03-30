// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>

namespace fs = std::filesystem;

struct SubcommandImportRTABMapOptions {
  fs::path database_path_in;
  fs::path project_path_out = GlobalParams::project_db;
};

// Function declarations.
void setup_subcommand_import_rtabmap(CLI::App &app);
int run_subcommand_import_rtabmap(SubcommandImportRTABMapOptions const &opt);
