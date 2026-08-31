// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>
#include <string>
#include <vector>

struct SubcommandEditDownsampleOptions {
  /// Source cloud name in the project DB. Defaults to "cloud" (the
  /// canonical name produced by import / reconstruct).
  std::string input = "cloud";

  /// Output cloud name. Empty string means "same as input" (overwrite).
  std::string output;

  /// Voxel leaf size in meters. Required; non-positive values are rejected.
  float resolution = 0.0f;

  /// When set, only the primary cloud is downsampled. Because this
  /// desynchronizes every index-aligned sibling (normals, labels, planes,
  /// rooms, instances), it is refused unless `force_desync` is also set.
  bool only_primary = false;

  /// Required together with `only_primary` to actually perform a
  /// desynchronizing downsample (docs/STANDARDS.md §5, loud failure).
  bool force_desync = false;
};

void setup_subcommand_edit_downsample(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_edit_downsample(SubcommandEditDownsampleOptions const &opt,
                                   const RuxOptions &global_opt);
