// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
#include <filesystem>

namespace fs = std::filesystem;

struct SubcommandExportRhinoOptions {
  fs::path project;  ///< Path to .rux project file (required, positional)
  std::string cloud_name = "cloud";  ///< Cloud name in ProjectDB
  std::string labels_name = "";  ///< Optional label name in ProjectDB
  fs::path path_out = fs::current_path() / "cloud.3dm";  ///< Output .3dm file
};

void setup_subcommand_export_rhino(CLI::App &parent);
int run_subcommand_export_rhino(SubcommandExportRhinoOptions const &opt);
