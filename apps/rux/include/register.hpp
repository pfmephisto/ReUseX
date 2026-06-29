// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

/// Options for the `rux register` subcommand (within-scan pose refinement).
struct SubcommandRegisterOptions {
  int iterations = 20;
  int neighbor_window = 5;
  float max_corr_distance = 0.10f;
  float normal_angle = 45.0f;
  float robust_width = 0.05f;
  std::string kernel = "welsch"; // "welsch" or "huber"
  float prior_weight = 1.0f;
  int anchor_frame = -1;
  float surfel_voxel = 0.03f;
  float min_distance = 0.0f;
  float max_distance = 4.0f;
  int sampling_factor = 8;
  int confidence_threshold = 2;
  bool dry_run = false;
};

void setup_subcommand_register(CLI::App &app,
                               std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_register(SubcommandRegisterOptions const &opt,
                            const RuxOptions &global_opt);
