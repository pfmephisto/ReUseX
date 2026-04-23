// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct SubcommandImportPhotosOptions {
  std::vector<fs::path> input_paths; // directories or individual files
  std::string project_id;
};

void setup_subcommand_import_photos(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_photos(SubcommandImportPhotosOptions const &opt,
                                 const RuxOptions &global_opt);
