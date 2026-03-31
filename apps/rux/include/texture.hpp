// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"
#include <CLI/CLI.hpp>
#include <string>

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandTextureOptions {
  fs::path project;  ///< Path to .rux project file (required, positional)
  std::string mesh_name = "mesh";  ///< Mesh name in ProjectDB
  std::string output_name = "textured_mesh";  ///< Output textured mesh name in ProjectDB
};

// Function declarations.
void setup_subcommand_texture(CLI::App &app);
int run_subcommand_texture(SubcommandTextureOptions const &opt);
