// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <CLI/CLI.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

struct SubcommandAnnotateOptions {

  fs::path database_path;
  fs::path out_cloud_path = fs::current_path() / "cloud.pcd";
  fs::path out_normals_path = fs::current_path() / "normals.pcd";
  fs::path out_trajectory_path = fs::current_path() / "trajectory.txt";
  fs::path net_path = fs::current_path() / "yolov8x-seg.onnx";

  bool isCuda{false};
  bool skipInference{false};
  bool ascii{false};
  float min_distance = 0.00f;
  float max_distance = 4.00f;
  size_t sampling_factor = 4;
  float grid_size = 0.01f;
};

// Function declarations.
void setup_subcommand_annotate(CLI::App &app);
int run_subcommand_annotate(SubcommandAnnotateOptions const &opt);
