// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

struct SubcommandExportColmapOptions {
  fs::path output_dir;
  bool no_lidar_seed = false;
  std::size_t max_lidar_points = 100'000;
  std::string lidar_cloud_name = "cloud";
  int jpeg_quality = 95;
};

void setup_subcommand_export_colmap(CLI::App &parent,
                                    std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_export_colmap(SubcommandExportColmapOptions const &opt,
                                 const RuxOptions &global_opt);
