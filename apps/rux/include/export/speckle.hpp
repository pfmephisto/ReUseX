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

struct SubcommandExportSpeckleOptions {
  std::string server_url;
  std::string project_id;
  std::string commit_message = "ReUseX export";
  std::size_t max_batch_bytes = 25 * 1024 * 1024; // 25 MB
};

void setup_subcommand_export_speckle(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_export_speckle(SubcommandExportSpeckleOptions const &opt, const RuxOptions &global_opt);
