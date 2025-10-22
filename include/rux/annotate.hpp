// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

struct SubcommandAnnotateOptions {

  fs::path database_path_in;
  fs::path cloud_path_out = GlobalParams::cloud;
  fs::path labels_path_out = GlobalParams::labels;
  fs::path normals_path_out = GlobalParams::normals;
  fs::path trajectory_path_out = fs::current_path() / "trajectory.txt";
  fs::path net_path = fs::current_path() / "yolov8x-seg.onnx";

  bool isCuda{false};
  bool skipInference{false};
  bool ascii{false};
  float min_distance = 0.00f;
  float max_distance = 4.00f;
  size_t sampling_factor = 4;
  float resulution = GlobalParams::resulution;
};

// Function declarations.
void setup_subcommand_annotate(CLI::App &app);
int run_subcommand_annotate(SubcommandAnnotateOptions const &opt);
