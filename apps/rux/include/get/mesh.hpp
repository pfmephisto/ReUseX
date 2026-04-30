// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

/**
 * @brief Options for the 'get mesh' subcommand.
 *
 * Exports a specific mesh from ProjectDB to a file in various formats.
 */
struct SubcommandGetMeshOptions {
  std::string mesh_name;  ///< Name of mesh in ProjectDB
  fs::path output_path;   ///< Output file path (optional)
  std::string format;     ///< Output format (optional, defaults to database format)
};

/**
 * @brief Run the 'get mesh' subcommand.
 *
 * Exports a specific mesh from ProjectDB to a standard file format.
 * Supports OBJ (with textures) and PLY formats.
 *
 * @param opt Options for the subcommand
 * @param global_opt Global options (contains project DB path)
 * @return Exit code (0 for success)
 */
int run_subcommand_get_mesh(SubcommandGetMeshOptions const &opt, const RuxOptions &global_opt);

/**
 * @brief Setup the 'get mesh' subcommand.
 *
 * @param app CLI11 app to add the subcommand to
 * @param global_opt Global options shared pointer
 */
void setup_subcommand_get_mesh(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
