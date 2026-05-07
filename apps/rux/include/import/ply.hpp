// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

struct SubcommandImportPLYOptions {
  fs::path input_path;
};

void setup_subcommand_import_ply(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_ply(SubcommandImportPLYOptions const &opt, const RuxOptions &global_opt);
