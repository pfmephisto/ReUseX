// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <reusex/reconstruction/create_windows.hpp>

#include <CLI/CLI.hpp>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

/// Options for `rux create windows`.
///
/// Defaults are derived from reusex::geometry::CreateWindowsOptions so the CLI
/// and library never disagree on a parameter's default (docs/STANDARDS.md §4).
struct SubcommandWindowOptions {
  std::string mesh_name = "mesh";
  std::string instance_cloud_name = "instances";
  std::string semantic_cloud_name = "labels";
  std::string mode = "rect"; ///< "rect" or "poly"
  /// Offset along the outward wall normal (meters).
  float wall_offset = reusex::geometry::CreateWindowsOptions{}.wall_offset;
  /// ConcaveHull alpha for polyline mode.
  float alpha = reusex::geometry::CreateWindowsOptions{}.alpha;
  /// Verticality gate on wall candidates (#326).
  float wall_normal_z_threshold =
      reusex::geometry::CreateWindowsOptions{}.wall_normal_z_threshold;
  std::vector<uint32_t>
      labels_to_process; ///< Semantic labels to treat as windows
  bool clear_existing =
      false; ///< Delete all existing windows before creating new ones
  /// Include windows inside the mesh volume.
  bool include_internal =
      reusex::geometry::CreateWindowsOptions{}.include_internal;
};

// Function declarations.
void setup_subcommand_create_windows(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_create_windows(SubcommandWindowOptions const &opt,
                                  const RuxOptions &global_opt);
