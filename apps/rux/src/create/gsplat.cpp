// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/gsplat.hpp"

#include <spdlog/spdlog.h>

#ifdef REUSEX_HAVE_GSPLAT
#include <filesystem>
#include <reusex/core/ProjectDB.hpp>
#include <reusex/gsplat/train.hpp>
#endif

void setup_subcommand_create_gsplat(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateGsplatOptions>();
  auto *sub = app.add_subcommand(
      "gsplat", "Train a 3D Gaussian Splatting model from the scan");

  sub->footer(R"(
DESCRIPTION:
  Trains a 3D Gaussian Splatting (3DGS) radiance field natively on the GPU.
  The Gaussians are SEEDED from a point cloud already in the project (so the
  scene starts at LiDAR density rather than from a sparse SfM cloud) and
  optimized against the project's posed sensor frames, optionally including
  perspective slices of content-aligned 360 panoramas.

  The differentiable rasterizer is gsplat's Apache-2.0 CUDA backend; the
  trainer around it is part of ReUseX (GPL-3.0-or-later).

EXAMPLES:
  rux create gsplat --iterations 2000 --out splat.ply
  rux create gsplat --mcmc --iterations 30000 --out splat.ply
  rux create gsplat --first-frame 1995 --last-frame 2400 --max-image-size 512
  rux create gsplat --use-panoramas --render-dir figs --render-at 0,500,2000

DENSITY CONTROL:
  By default the trainer only prunes: it removes collapsed Gaussians but
  cannot add any, so it never resolves detail finer than the seed cloud.
  --mcmc switches to 3DGS-MCMC, which relocates dead Gaussians onto dense
  regions and samples new ones up to --mcmc-cap-factor x the seed count. Use
  it for scans whose cloud is coarse relative to the image resolution.

REPORTED QUALITY:
  Every Nth view (--holdout-every, default 8) is excluded from training and
  used only for evaluation. The held-out PSNR is the honest number; the
  training-view PSNR will always be higher.

REGION SELECTION MATTERS:
  3DGS needs dense multi-view overlap. Training on a sparse sample spread over
  a whole building reconstructs nothing; a contiguous capture segment
  reconstructs cleanly. Prefer --first-frame/--last-frame over --frame-stride.

NOTES:
  - Requires a CUDA GPU and a build with the gsplat backend compiled in
  - Requires sensor frames and a point cloud (run 'rux create clouds' first)
  - --max-image-size is the most effective speed knob: cost is per-pixel
  - Output .ply follows the reference 3DGS layout and opens in splat viewers
)");

  sub->get_formatter()->column_width(40);

  sub->add_option("--seed-cloud", opt->seed_cloud,
                  "Point cloud in the project used to seed the Gaussians")
      ->default_val(opt->seed_cloud);
  sub->add_option("--max-points", opt->max_points,
                  "Cap on seed Gaussians (0 = use the whole cloud)")
      ->default_val(opt->max_points);
  sub->add_option("--sh-degree", opt->sh_degree,
                  "Spherical-harmonic degree (0 = view-independent colour)")
      ->default_val(opt->sh_degree)
      ->check(CLI::Range(0, 3));

  sub->add_option("-i, --iterations", opt->iterations, "Training iterations")
      ->default_val(opt->iterations)
      ->check(CLI::Range(1, 1000000));
  sub->add_option("--lambda-dssim", opt->lambda_dssim,
                  "Weight of the D-SSIM term in the photometric loss")
      ->default_val(opt->lambda_dssim)
      ->check(CLI::Range(0.0, 1.0));
  sub->add_option("--seed", opt->seed, "Deterministic view-shuffling seed")
      ->default_val(opt->seed);
  sub->add_flag("--no-prune", opt->no_prune,
                "Disable opacity/size pruning during training");

  sub->add_option("--holdout-every", opt->holdout_every,
                  "Hold every Nth view out of training and report PSNR/SSIM "
                  "on it (0 = train on every view, no held-out metric)")
      ->default_val(opt->holdout_every)
      ->check(CLI::Range(0, 1000));
  sub->add_option("--eval-interval", opt->eval_interval,
                  "Iterations between held-out evaluation passes")
      ->default_val(opt->eval_interval)
      ->check(CLI::Range(1, 1000000));
  sub->add_option("--eval-views", opt->eval_max_views,
                  "Views per side of an evaluation pass")
      ->default_val(opt->eval_max_views)
      ->check(CLI::Range(1, 10000));

  sub->add_flag("--mcmc", opt->mcmc,
                "Enable 3DGS-MCMC density control (relocation + growth + "
                "Langevin noise). Replaces pruning; this is the only mode "
                "that can add detail beyond the seed cloud");
  sub->add_option("--mcmc-cap-factor", opt->mcmc_cap_factor,
                  "Gaussian budget as a multiple of the seed count")
      ->default_val(opt->mcmc_cap_factor)
      ->check(CLI::Range(1.0, 100.0));
  sub->add_option("--mcmc-cap", opt->mcmc_cap,
                  "Absolute Gaussian budget (0 = use --mcmc-cap-factor)")
      ->default_val(opt->mcmc_cap);
  sub->add_option("--mcmc-refine-every", opt->mcmc_refine_every,
                  "Iterations between relocate+grow passes")
      ->default_val(opt->mcmc_refine_every)
      ->check(CLI::Range(1, 100000));
  sub->add_option("--mcmc-noise-lr", opt->mcmc_noise_lr, "Langevin noise scale")
      ->default_val(opt->mcmc_noise_lr);
  sub->add_option("--mcmc-opacity-reg", opt->mcmc_opacity_reg,
                  "L1 weight on activated opacity (0 by default; the "
                  "paper's 0.01 collapses interior-traverse scans)")
      ->default_val(opt->mcmc_opacity_reg);
  sub->add_option("--mcmc-scale-reg", opt->mcmc_scale_reg,
                  "L1 weight on activated scale (0 by default, see above)")
      ->default_val(opt->mcmc_scale_reg);

  sub->add_option("--frame-stride", opt->frame_stride,
                  "Use every Nth sensor frame")
      ->default_val(opt->frame_stride)
      ->check(CLI::Range(1, 10000));
  sub->add_option("--first-frame", opt->first_frame,
                  "First sensor-frame node id to train on (-1 = unbounded)")
      ->default_val(opt->first_frame);
  sub->add_option("--last-frame", opt->last_frame,
                  "Last sensor-frame node id to train on (-1 = unbounded)")
      ->default_val(opt->last_frame);
  sub->add_option("--max-image-size", opt->max_image_size,
                  "Downscale training images so the long edge is at most this "
                  "many pixels (0 = keep native resolution)")
      ->default_val(opt->max_image_size);
  sub->add_option("--max-views", opt->max_views,
                  "Hard cap on the number of training views (0 = unlimited)")
      ->default_val(opt->max_views);

  sub->add_flag("--use-panoramas", opt->use_panoramas,
                "Also train on perspective slices of content-aligned 360 "
                "panoramas (requires 'rux align 360')");
  sub->add_option("--pano-n-yaw", opt->pano_n_yaw,
                  "Equator slices per panorama")
      ->default_val(opt->pano_n_yaw)
      ->check(CLI::Range(1, 64));
  sub->add_option("--pano-fov", opt->pano_fov_deg,
                  "Per-slice horizontal field of view [deg]")
      ->default_val(opt->pano_fov_deg)
      ->check(CLI::Range(10.0, 170.0));
  sub->add_option("--pano-tile", opt->pano_tile, "Slice size [px, square]")
      ->default_val(opt->pano_tile)
      ->check(CLI::Range(64, 8192));

  sub->add_option("-o, --out", opt->out_ply,
                  "Write the trained splat to this .ply");
  sub->add_option("--render-dir", opt->render_dir,
                  "Directory for checkpoint renders");
  sub->add_option("--render-at", opt->render_iterations,
                  "Iterations at which to dump a rendered PNG (0 = the "
                  "untrained seed), e.g. --render-at 0 500 2000")
      ->delimiter(',');
  sub->add_option("--render-view", opt->render_view_index,
                  "Index of the training view used for checkpoint renders")
      ->default_val(opt->render_view_index);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling create gsplat subcommand");
    return run_subcommand_create_gsplat(*opt, *global_opt);
  });
}

#ifndef REUSEX_HAVE_GSPLAT

int run_subcommand_create_gsplat(SubcommandCreateGsplatOptions const &,
                                 const RuxOptions &) {
  spdlog::error(
      "This build has no Gaussian-splatting support. It requires WITH_CUDA=ON "
      "and the gsplat-cuda package (nix build .#gsplat-cuda); see "
      "docs/research/gaussian-splatting.md.");
  return RuxError::INVALID_ARGUMENT;
}

#else

int run_subcommand_create_gsplat(SubcommandCreateGsplatOptions const &opt,
                                 const RuxOptions &global_opt) {
  namespace fs = std::filesystem;
  namespace gs = reusex::gsplat;

  const fs::path project_path = global_opt.project_db;
  spdlog::info("Training Gaussian splats for project: {}",
               project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    gs::GsplatStageOptions o;
    o.seed_cloud = opt.seed_cloud;
    o.init.max_points = opt.max_points;
    o.init.sh_degree = opt.sh_degree;
    o.init.seed = opt.seed;

    o.views.frame_stride = opt.frame_stride;
    o.views.first_frame = opt.first_frame;
    o.views.last_frame = opt.last_frame;
    o.views.max_image_size = opt.max_image_size;
    o.views.max_views = opt.max_views;
    o.views.include_panorama_slices = opt.use_panoramas;
    o.views.pano_n_yaw = opt.pano_n_yaw;
    o.views.pano_fov_deg = opt.pano_fov_deg;
    o.views.pano_tile = opt.pano_tile;

    o.train.iterations = opt.iterations;
    o.train.lambda_dssim = opt.lambda_dssim;
    o.train.seed = opt.seed;
    o.train.prune_enabled = !opt.no_prune;
    o.train.holdout_every = opt.holdout_every;
    o.train.eval_interval = opt.eval_interval;
    o.train.eval_max_views = opt.eval_max_views;
    o.train.mcmc.enabled = opt.mcmc;
    o.train.mcmc.cap_factor = opt.mcmc_cap_factor;
    o.train.mcmc.cap_absolute = opt.mcmc_cap;
    o.train.mcmc.refine_every = opt.mcmc_refine_every;
    o.train.mcmc.noise_lr = opt.mcmc_noise_lr;
    o.train.mcmc.opacity_reg = opt.mcmc_opacity_reg;
    o.train.mcmc.scale_reg = opt.mcmc_scale_reg;
    o.train.render_iterations = opt.render_iterations;
    o.train.render_view_index = opt.render_view_index;
    if (!opt.render_dir.empty())
      o.train.render_dir = opt.render_dir;
    if (!opt.out_ply.empty())
      o.out_ply = opt.out_ply;

    const gs::TrainResult r = gs::run_gsplat_stage(db, o);

    spdlog::info("Gaussian splatting complete: {} Gaussians, {:.2f} dB "
                 "(training views), {:.1f} s",
                 r.final_count, r.final_psnr, r.seconds);
    if (!r.evals.empty() && r.evals.back().holdout_views > 0)
      spdlog::info("Held-out: {:.2f} dB / SSIM {:.4f} over {} views excluded "
                   "from training",
                   r.final_holdout_psnr, r.final_holdout_ssim,
                   r.evals.back().holdout_views);
    if (opt.mcmc)
      spdlog::info("MCMC: relocated {}, added {} Gaussians", r.relocated,
                   r.added);
    if (!opt.out_ply.empty())
      spdlog::info("Wrote {}", opt.out_ply);
    for (const auto &p : r.renders)
      spdlog::info("Render: {}", p.string());

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Gaussian splatting failed: {}", e.what());
    return RuxError::GENERIC;
  }
}

#endif // REUSEX_HAVE_GSPLAT
