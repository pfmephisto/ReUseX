// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include "../import.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <optional>
#include <string>

namespace fs = std::filesystem;

struct SubcommandImportRTABMapOptions {

  fs::path database_path_in;
  fs::path trajectory_path_out = fs::current_path() / "trajectory.txt";

  float min_distance = 0.00f;
  float max_distance = 4.00f;
  size_t sampling_factor = 4;
  float resulution = GlobalParams::resulution;

  std::optional<fs::path> project;
};

// Function declarations.
void setup_subcommand_import_rtabmap(CLI::App &app, ImportContext &ctx);
int run_subcommand_import_rtabmap(SubcommandImportRTABMapOptions const &opt,
                                  ImportContext &ctx);
