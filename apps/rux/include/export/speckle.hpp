// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

struct SubcommandExportSpeckleOptions {
  fs::path project;  ///< Path to .rux project file (required, positional)
  std::string data_name = "cloud";  ///< Cloud or mesh name in ProjectDB
  bool is_mesh = false;  ///< Load as mesh instead of cloud

  std::string server_url;
  std::string project_id;
  std::string model_name = "main";
  std::string commit_message = "ReUseX export";
  std::size_t max_batch_bytes = 25 * 1024 * 1024; // 25 MB
};

void setup_subcommand_export_speckle(CLI::App &parent);
int run_subcommand_export_speckle(SubcommandExportSpeckleOptions const &opt);
