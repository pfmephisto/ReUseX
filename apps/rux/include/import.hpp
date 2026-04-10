// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
// struct SubcommandImportOptions {};

// Function declarations.
void setup_subcommand_import(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import(/*SubcommandImportOptions const &opt*/);
