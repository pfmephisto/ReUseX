// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

#include <reusex/geometry/reconstruct.hpp>

namespace fs = std::filesystem;

/// Defaults derived from reusex::geometry::ReconstructionParams so the CLI and
/// library never disagree on a parameter's default (docs/STANDARDS.md §4).
struct SubcommandCreateCloudsOptions {
  float resolution = reusex::geometry::ReconstructionParams{}.resolution;
  float min_distance = reusex::geometry::ReconstructionParams{}.min_distance;
  float max_distance = reusex::geometry::ReconstructionParams{}.max_distance;
  int sampling_factor =
      reusex::geometry::ReconstructionParams{}.sampling_factor;
  int confidence_threshold =
      reusex::geometry::ReconstructionParams{}.confidence_threshold;
};

// Function declarations.
void setup_subcommand_create_clouds(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_create_clouds(SubcommandCreateCloudsOptions const &opt,
                                 const RuxOptions &global_opt);
