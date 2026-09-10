// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

/// Options for `rux import gsplat`.
struct SubcommandImportGsplatOptions {
  std::filesystem::path input_path;
  std::string splat_name = "splat";
};

void setup_subcommand_import_gsplat(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_gsplat(SubcommandImportGsplatOptions const &opt,
                                 const RuxOptions &global_opt);
