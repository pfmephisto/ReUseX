// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>
#include <string>

struct SubcommandCreateDenseOptions {
  std::string output_name = "dense";
  std::string seed_cloud_name = "cloud";
  int seed_max_points = 100'000;
  // 0 = full resolution (no downsample). With the spdmon RAM leak fixed
  // the depth-map and fusion stages are bounded by upstream OPTDENSE
  // defaults, so full-res is the right default — bump --resolution-level
  // only if you actually want faster/coarser output.
  int resolution_level = 0;
  // 0 = no upper bound on max(width,height). Override if input frames
  // are unusually large and a single depth-map worker would OOM.
  int max_resolution = 0;
  int min_resolution = 320;
  int num_views = 4;
  int frame_stride = 1;
  int gpu_index = -2; // -2 = CPU, -1 = best GPU, >=0 = specific device
  int max_threads = 0; // 0 = all CPU threads; lower to bound RAM use
  int chunk_size = 0; // 0 = whole-scene fusion; >0 = N frames per chunk
  int max_fuse_points = 0; // 0 = unlimited
  bool no_geom_consistency = false;
  bool no_lidar_seed = false;
  bool no_outlier_filter = false;
  bool no_bbox_clip = false;
};

void setup_subcommand_create_dense(CLI::App &app,
                                   std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_create_dense(SubcommandCreateDenseOptions const &opt,
                                const RuxOptions &global_opt);
