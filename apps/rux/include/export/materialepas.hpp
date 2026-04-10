// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

struct SubcommandExportMaterialepasOptions {
  fs::path output_path = fs::current_path() / "materialepas.json";
};

void setup_subcommand_export_materialepas(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_export_materialepas(
    SubcommandExportMaterialepasOptions const &opt, const RuxOptions &global_opt);
