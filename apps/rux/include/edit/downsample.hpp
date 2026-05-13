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

  /// Parallel clouds to downsample with the same voxel assignment.
  /// Each entry is of the form "NAME" (overwrite NAME) or
  /// "NAME:OUT_NAME" (write to OUT_NAME).
  std::vector<std::string> with;
};

void setup_subcommand_edit_downsample(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_edit_downsample(
    SubcommandEditDownsampleOptions const &opt, const RuxOptions &global_opt);
