// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

/// Options for `rux create windows`.
struct SubcommandWindowOptions {
  std::string mesh_name = "mesh";
  std::string instance_cloud_name = "instances";
  std::string semantic_cloud_name = "labels";
  std::string mode = "rect";  ///< "rect" or "poly"
  float wall_offset = 0.5f;   ///< Offset along outward wall normal (meters)
  float alpha = 0.5f;         ///< ConcaveHull alpha for polyline mode
  std::vector<uint32_t> labels_to_process; ///< Semantic labels to treat as windows
  bool clear_existing = false; ///< Delete all existing windows before creating new ones
  bool include_internal = false; ///< Include windows inside mesh volume (default: false)
};

// Function declarations.
void setup_subcommand_create_windows(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_create_windows(SubcommandWindowOptions const &opt, const RuxOptions &global_opt);
