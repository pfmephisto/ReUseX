// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

struct SubcommandAnnotatePanoramaOptions {

  fs::path net_path; // SAM3 engine directory (required)

  int n_yaw = 8;           // equator tiles around the sphere
  double fov_deg = 90.0;   // per-tile horizontal FOV (deg)
  int tile = 1008;         // tile size in px (SAM3 native input)
  float confidence = 0.5f; // detection confidence threshold

  bool skip_annotated = false; // skip panoramas already segmented (resume mode)

  std::string figures_dir; // if set, dump colorized segmentation figures here
  int figures_limit = 6;   // cap number of panoramas figured
};

// Function declarations.
void setup_subcommand_create_annotate_panorama(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_create_annotate_panorama(
    SubcommandAnnotatePanoramaOptions const &opt, const RuxOptions &global_opt);
