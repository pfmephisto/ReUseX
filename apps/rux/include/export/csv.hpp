// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

struct SubcommandExportCSVOptions {
  fs::path output_path = fs::current_path() / "elements.csv";
};

void setup_subcommand_export_csv(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_export_csv(SubcommandExportCSVOptions const &opt,
                              const RuxOptions &global_opt);
