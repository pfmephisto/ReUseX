// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

struct SubcommandExportPLYOptions {
  std::string cloud_name  = "cloud"; ///< Cloud name in ProjectDB (--name)
  fs::path    output_path;           ///< Output .ply path (-o); default: {cloud_name}.ply
  std::string filter_expr;           ///< Point filter expression (-f)
};

void setup_subcommand_export_ply(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
int  run_subcommand_export_ply(SubcommandExportPLYOptions const &opt, const RuxOptions &global_opt);
