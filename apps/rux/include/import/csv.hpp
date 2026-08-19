// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

struct SubcommandImportCSVOptions {
  fs::path input_path;
  std::string project_id; // used when a CSV row creates a new passport
};

void setup_subcommand_import_csv(CLI::App &parent,
                                 std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_csv(SubcommandImportCSVOptions const &opt,
                              const RuxOptions &global_opt);
