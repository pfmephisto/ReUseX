// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
#include <filesystem>

namespace fs = std::filesystem;

struct SubcommandExportMaterialepasOptions {
  fs::path input_path;
  fs::path output_path = fs::current_path() / "materialepas.json";
  bool exact = false; // If true, export exact values (no defaults)
};

void setup_subcommand_export_materialepas(CLI::App &parent);
int run_subcommand_export_materialepas(
    SubcommandExportMaterialepasOptions const &opt);
