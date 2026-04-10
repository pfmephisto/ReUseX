// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandWindowOptions {
  // std::string mesh_name = "mesh"; ///< Mesh name in ProjectDB
  // std::string output_name =
  //     "textured_mesh"; ///< Output textured mesh name in ProjectDB
};

// Function declarations.
void setup_subcommand_create_windows(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_create_windows(SubcommandWindowOptions const &opt, const RuxOptions &global_opt);
