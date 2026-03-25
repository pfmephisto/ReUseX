// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
#include <filesystem>

namespace fs = std::filesystem;

struct SubcommandExportRhinoOptions {
  fs::path cloud_path_in;
  fs::path labels_path_in;
  fs::path path_out = fs::current_path() / "cloud.3dm";
};

void setup_subcommand_export_rhino(CLI::App &parent);
int run_subcommand_export_rhino(SubcommandExportRhinoOptions const &opt);
