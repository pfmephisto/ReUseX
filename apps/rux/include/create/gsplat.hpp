// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"
#include <reusex/gsplat/GaussianCloud.hpp>
#include <reusex/gsplat/TrainingViews.hpp>
#include <reusex/gsplat/train.hpp>

#include <CLI/CLI.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/// Options for `rux create gsplat`.
///
/// Defaults are derived from the library option structs
/// (reusex::gsplat::{GaussianInitOptions, TrainingViewOptions, TrainOptions,
/// MCMCOptions, GsplatStageOptions}) so the CLI and library never disagree on a
/// parameter's default (docs/STANDARDS.md §4).
struct SubcommandCreateGsplatOptions {
  std::string seed_cloud = reusex::gsplat::GsplatStageOptions{}.seed_cloud;
  std::size_t max_points = reusex::gsplat::GaussianInitOptions{}.max_points;
  int sh_degree = reusex::gsplat::GaussianInitOptions{}.sh_degree;
  /// TrainOptions::sh_degree_interval / lr_sh_rest — the view-dependent-colour
  /// warm-up. Both are inert at the default `sh_degree = 0`.
  int sh_degree_interval = reusex::gsplat::TrainOptions{}.sh_degree_interval;
  float lr_sh_rest = reusex::gsplat::TrainOptions{}.lr_sh_rest;

  int iterations = reusex::gsplat::TrainOptions{}.iterations;
  float lambda_dssim = reusex::gsplat::TrainOptions{}.lambda_dssim;
  unsigned seed = reusex::gsplat::TrainOptions{}.seed;
  bool no_prune = !reusex::gsplat::TrainOptions{}.prune_enabled;

  // Held-out evaluation (TrainOptions::holdout_every / eval_interval /
  // eval_max_views).
  int holdout_every = reusex::gsplat::TrainOptions{}.holdout_every;
  int eval_interval = reusex::gsplat::TrainOptions{}.eval_interval;
  int eval_max_views = reusex::gsplat::TrainOptions{}.eval_max_views;

  // MCMC density control (TrainOptions::mcmc).
  bool mcmc = reusex::gsplat::MCMCOptions{}.enabled;
  double mcmc_cap_factor = reusex::gsplat::MCMCOptions{}.cap_factor;
  std::int64_t mcmc_cap = reusex::gsplat::MCMCOptions{}.cap_absolute;
  int mcmc_refine_every = reusex::gsplat::MCMCOptions{}.refine_every;
  float mcmc_noise_lr = reusex::gsplat::MCMCOptions{}.noise_lr;
  float mcmc_opacity_reg = reusex::gsplat::MCMCOptions{}.opacity_reg;
  float mcmc_scale_reg = reusex::gsplat::MCMCOptions{}.scale_reg;

  int frame_stride = reusex::gsplat::TrainingViewOptions{}.frame_stride;
  int first_frame = reusex::gsplat::TrainingViewOptions{}.first_frame;
  int last_frame = reusex::gsplat::TrainingViewOptions{}.last_frame;
  int max_image_size = reusex::gsplat::TrainingViewOptions{}.max_image_size;
  std::size_t max_views = reusex::gsplat::TrainingViewOptions{}.max_views;

  bool use_panoramas =
      reusex::gsplat::TrainingViewOptions{}.include_panorama_slices;
  int pano_n_yaw = reusex::gsplat::TrainingViewOptions{}.pano_n_yaw;
  double pano_fov_deg = reusex::gsplat::TrainingViewOptions{}.pano_fov_deg;
  int pano_tile = reusex::gsplat::TrainingViewOptions{}.pano_tile;

  std::string splat_name = reusex::gsplat::GsplatStageOptions{}.splat_name;
  std::string out_ply;
  std::string render_dir;
  std::vector<int> render_iterations;
  std::size_t render_view_index =
      reusex::gsplat::TrainOptions{}.render_view_index;

  // Periodic checkpointing (TrainOptions::checkpoint_every / checkpoint_keep /
  // checkpoint_dir).
  int checkpoint_every = reusex::gsplat::TrainOptions{}.checkpoint_every;
  int checkpoint_keep = reusex::gsplat::TrainOptions{}.checkpoint_keep;
  std::string checkpoint_dir;
};

void setup_subcommand_create_gsplat(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt);

int run_subcommand_create_gsplat(SubcommandCreateGsplatOptions const &opt,
                                 const RuxOptions &global_opt);
