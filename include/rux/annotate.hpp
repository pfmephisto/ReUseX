// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

struct SubcommandAnnotateOptions {

  fs::path database_path_in;
  fs::path net_path = fs::current_path() / "yolov8x-seg.torchscript";

  bool isCuda{false};
};

// Function declarations.
void setup_subcommand_annotate(CLI::App &app);
int run_subcommand_annotate(SubcommandAnnotateOptions const &opt);
