// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

struct SubcommandImportMaterialepasOptions {
  fs::path input_path;
  std::string project_id;
};

void setup_subcommand_import_materialepas(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_materialepas(
    SubcommandImportMaterialepasOptions const &opt, const RuxOptions &global_opt);
