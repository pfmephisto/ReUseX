// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandSegmentOptions {};

// We could manually make a few variables and use shared pointers for each; this
// is just done this way to be nicely organized

// Function declarations.
void setup_subcommand_segment(CLI::App &app);
int run_subcommand_segment(SubcommandSegmentOptions const &opt);
