// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <vector>

namespace fs = std::filesystem;

struct SubcommandImportPanoramaOptions {
  std::vector<fs::path> input_paths; // directories or individual files
};

void setup_subcommand_import_panorama(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_panorama(SubcommandImportPanoramaOptions const &opt,
                                   const RuxOptions &global_opt);
