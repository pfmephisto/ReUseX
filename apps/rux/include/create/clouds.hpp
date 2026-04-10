// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

struct SubcommandCreateCloudsOptions {
  float resolution = 0.05f;
  float min_distance = 0.0f;
  float max_distance = 4.0f;
  int sampling_factor = 4;
  int confidence_threshold = 2;
};

// Function declarations.
void setup_subcommand_create_clouds(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_create_clouds(SubcommandCreateCloudsOptions const &opt, const RuxOptions &global_opt);
