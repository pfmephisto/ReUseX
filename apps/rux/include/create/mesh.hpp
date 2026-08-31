// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>

#include <reusex/geometry/mesh.hpp>

namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
///
/// search_threshold / new_plane_offset defaults are derived from
/// reusex::geometry::MeshOptions (docs/STANDARDS.md §4). angle_threshold,
/// distance_threshold and grid_size have no library counterpart in MeshOptions
/// (they are not forwarded to reusex::geometry::mesh()), so they keep local
/// literals until the library grows matching fields.
struct SubcommandMeshOptions {
  std::string output_mesh_name = "mesh"; ///< Mesh name in ProjectDB

  float grid_size = 0.5;

  double angle_threshold = 25.0;
  double distance_threshold = 0.2;

  double search_threshold = reusex::geometry::MeshOptions{}.search_threshold;
  double new_plane_offset = reusex::geometry::MeshOptions{}.new_plane_offset;

  double time_limit_seconds = 120.0; ///< MIP solver time limit (s), issue #212
  double alpha = 0.04;               ///< MIP objective wall-weight, issue #212
  size_t max_cells = 5000;           ///< Cell complex guard, issue #213

  /// Sectioned MIP solve controls (issue #226). Defaults mirror MeshOptions.
  bool sectioned = reusex::geometry::MeshOptions{}.sectioned;
  size_t sectioned_threshold =
      reusex::geometry::MeshOptions{}.sectioned_threshold;
  std::string solver = "auto"; ///< MIP backend: auto | cuopt | highs.

  std::string filter_expr; ///< Filter expression to limit processing
};

// Function declarations.
void setup_subcommand_create_mesh(CLI::App &app,
                                  std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_mesh(SubcommandMeshOptions const &opt,
                        const RuxOptions &global_opt);
