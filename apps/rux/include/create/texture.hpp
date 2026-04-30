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
struct SubcommandTextureOptions {
  std::string mesh_name = "mesh"; ///< Mesh name in ProjectDB
  std::string output_name =
      "textured_mesh"; ///< Output textured mesh name in ProjectDB
  std::string cloud_name = "cloud"; ///< Point cloud name in ProjectDB
  bool debug_colors = false; ///< Use distinct colors for UV mapping verification
  float texels_per_meter = 400.0f; ///< Target texture resolution
  int max_resolution = 4096; ///< Maximum texture size
  int atlas_tile_size = 2048; ///< Atlas tile size for PCL visualization
  float distance_threshold = 0.02f; ///< Max distance from point to surface
};

// Function declarations.
void setup_subcommand_create_texture(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_texture(SubcommandTextureOptions const &opt, const RuxOptions &global_opt);
