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

struct SubcommandExportE57Options {
  std::string cloud_name  = "cloud"; ///< Cloud name in ProjectDB (--name)
  fs::path    output_path;           ///< Output .e57 path (-o); default: {cloud_name}.e57
  std::string filter_expr;           ///< Point filter expression (-f)
};

void setup_subcommand_export_e57(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
int  run_subcommand_export_e57(SubcommandExportE57Options const &opt, const RuxOptions &global_opt);
