// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

struct SubcommandExportRhinoOptions {
  std::string cloud_name = "cloud"; ///< Cloud name in ProjectDB
  std::string labels_name = "";     ///< Optional label name in ProjectDB
  fs::path path_out = fs::current_path() / "cloud.3dm"; ///< Output .3dm file
};

void setup_subcommand_export_rhino(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_export_rhino(SubcommandExportRhinoOptions const &opt, const RuxOptions &global_opt);
