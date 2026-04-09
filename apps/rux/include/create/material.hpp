// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

struct SubcommandCreateMaterialOptions {
  std::string guid;            // Optional custom GUID
  fs::path project;            // Project database (empty = output to stdout only)
  fs::path output_file;        // Optional output file
  bool stdout_output = false;  // Output JSON to stdout
};

void setup_subcommand_create_material(CLI::App &parent);
int run_subcommand_create_material(SubcommandCreateMaterialOptions const &opt);
