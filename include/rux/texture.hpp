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
  fs::path mesh_path_in = fs::current_path() / "mesh.ply";
  fs::path db_path_in = GlobalParams::db;
  fs::path mesh_path_out = fs::current_path() / "mesh.obj";
};

// Function declarations.
void setup_subcommand_texture(CLI::App &app);
int run_subcommand_texture(SubcommandTextureOptions const &opt);
