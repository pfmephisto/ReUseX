// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
#include <filesystem>

namespace fs = std::filesystem;

struct SubcommandImportMaterialepasOptions {
  fs::path input_path;
  fs::path db_path = fs::current_path() / "project.db";
  std::string project_id;
};

void setup_subcommand_import_materialepas(CLI::App &parent);
int run_subcommand_import_materialepas(
    SubcommandImportMaterialepasOptions const &opt);
