// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <CLI/CLI.hpp>
#include <string>

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandCellcomplexOptions {
  fs::path input;
  fs::path output = fs::current_path() / "output.vtk";

  bool display = false;
};

// Function declarations.
void setup_subcommand_cellcomplex(CLI::App &app);
int run_subcommand_cellcomplex(SubcommandCellcomplexOptions const &opt);
