// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"
#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandViewOptions {};

// Function declarations.
void setup_subcommand_view(CLI::App &app);
int run_subcommand_view(SubcommandViewOptions const &opt);
