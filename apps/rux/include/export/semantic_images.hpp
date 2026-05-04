// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

struct SubcommandExportSemanticImagesOptions {
  fs::path output_dir = fs::current_path(); ///< Directory to write PNG files
  std::string prefix = "semantic";          ///< Filename prefix: <prefix>_<id>.png
};

void setup_subcommand_export_semantic_images(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_export_semantic_images(SubcommandExportSemanticImagesOptions const &opt,
                                          const RuxOptions &global_opt);
