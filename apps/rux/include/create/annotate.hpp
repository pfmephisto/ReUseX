// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

struct SubcommandAnnotateOptions {

  fs::path net_path = fs::current_path() / "yolov8x-seg.torchscript";

  bool isCuda{false};
};

// Function declarations.
void setup_subcommand_create_annotate(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_annotate(SubcommandAnnotateOptions const &opt, const RuxOptions &global_opt);
