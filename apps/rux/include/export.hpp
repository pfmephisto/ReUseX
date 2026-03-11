// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandExportOptions {
  fs::path cloud_path_in;
  fs::path labels_path_in;
  fs::path path_out = fs::current_path() / "cloud.3dm";
};

// We could manually make a few variables and use shared pointers for each; this
// is just done this way to be nicely organized

// Function declarations.
void setup_subcommand_export(CLI::App &app);
int run_subcommand_export(SubcommandExportOptions const &opt);
