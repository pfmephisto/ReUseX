// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <CLI/CLI.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/// Options for `rux create gsplat`.
///
/// Defaults deliberately mirror the library option structs
/// (reusex::gsplat::{GaussianInitOptions, TrainingViewOptions, TrainOptions})
/// rather than restating numbers, so the CLI and the library can never disagree
/// on a default (docs/STANDARDS.md §4). They are read through the accessors in
/// the .cpp, which is the only translation unit that may include the library
/// headers — those pull in nothing torch-shaped, but keeping the include here
/// would still couple every CLI TU to the optional module.
struct SubcommandCreateGsplatOptions {
  std::string seed_cloud = "cloud";
  std::size_t max_points = 0;
  int sh_degree = 0;

  int iterations = 2000;
  float lambda_dssim = 0.2f;
  unsigned seed = 42;
  bool no_prune = false;

  // Held-out evaluation (TrainOptions::holdout_every / eval_interval /
  // eval_max_views).
  int holdout_every = 8;
  int eval_interval = 1000;
  int eval_max_views = 32;

  // MCMC density control (TrainOptions::mcmc).
  bool mcmc = false;
  double mcmc_cap_factor = 2.0;
  std::int64_t mcmc_cap = 0;
  int mcmc_refine_every = 100;
  float mcmc_noise_lr = 5e5f;
  float mcmc_opacity_reg = 0.0f;
  float mcmc_scale_reg = 0.0f;

  int frame_stride = 1;
  int first_frame = -1;
  int last_frame = -1;
  int max_image_size = 0;
  std::size_t max_views = 0;

  bool use_panoramas = false;
  int pano_n_yaw = 8;
  double pano_fov_deg = 90.0;
  int pano_tile = 1024;

  std::string splat_name = "splat";
  std::string out_ply;
  std::string render_dir;
  std::vector<int> render_iterations;
  std::size_t render_view_index = 0;

  // Periodic checkpointing (TrainOptions::checkpoint_every / checkpoint_keep /
  // checkpoint_dir).
  int checkpoint_every = 0;
  int checkpoint_keep = 3;
  std::string checkpoint_dir;
};

void setup_subcommand_create_gsplat(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);

int run_subcommand_create_gsplat(SubcommandCreateGsplatOptions const &opt,
                                 const RuxOptions &global_opt);
