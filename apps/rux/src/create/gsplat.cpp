// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/gsplat.hpp"

#include <spdlog/spdlog.h>

#ifdef REUSEX_HAVE_GSPLAT
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/stage_contract.hpp>
#include <reusex/gsplat/train.hpp>

#include <atomic>
#include <csignal>
#include <filesystem>
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
  rux create gsplat --iterations 2000
  rux create gsplat --mcmc --iterations 30000 --name detailed
  rux create gsplat --first-frame 1995 --last-frame 2400 --max-image-size 512
  rux create gsplat --iterations 2000 --out splat.ply   # also export a file
  rux create gsplat --use-panoramas --render-dir figs --render-at 0,500,2000

OUTPUT:
  The trained splat is stored IN THE PROJECT (schema v12) under --name,
  defaulting to 'splat', and is what `rux gui` renders as its splat layer.
  -o/--out additionally writes the same bytes to a .ply for viewers outside
  ReUseX; it is an export, not the output, and is optional.

DENSITY CONTROL:
  By default the trainer only prunes: it removes collapsed Gaussians but
  cannot add any, so it never resolves detail finer than the seed cloud.
  --mcmc switches to 3DGS-MCMC, which relocates dead Gaussians onto dense
  regions and samples new ones up to --mcmc-cap-factor x the seed count. Use
  it for scans whose cloud is coarse relative to the image resolution.

VIEW-DEPENDENT COLOUR:
  --sh-degree 0 (the default) gives every Gaussian one colour from every
  direction, so glossy floors, screens and windows cannot be represented and
  their view-dependent energy is absorbed as blur. --sh-degree 1..3 adds
  spherical-harmonic bands that model it. They are unlocked one at a time,
  every --sh-degree-interval iterations, and trained at --lr-sh-rest (a
  twentieth of the DC rate, as in the reference implementation) — both
  because the higher bands start at zero and, let loose early or fast, fit
  per-view residuals that belong to geometry. Budget at least
  sh-degree x sh-degree-interval iterations or the top bands never unlock;
  the trainer warns when they will not.

REPORTED QUALITY:
  Every Nth view (--holdout-every, default 8) is excluded from training and
  used only for evaluation. The held-out PSNR is the honest number; the
  training-view PSNR will always be higher.

CHECKPOINTING:
  Ctrl-C no longer throws the run away. The first SIGINT asks the trainer to
  stop at the end of the current iteration; it then runs a final evaluation
  and stores the splat exactly as a completed run would, logging which
  iteration it reached. Press Ctrl-C a SECOND time to abort immediately the
  usual way, with nothing stored.
  --checkpoint-every N additionally writes an intermediate .ply every N
  iterations (atomically, via a .tmp + rename), keeping the last
  --checkpoint-keep of them under --checkpoint-dir (default: beside the
  project). Checkpoints stay files, not project rows: they are insurance
  against a crash, not project state.

REGION SELECTION MATTERS:
  3DGS needs dense multi-view overlap. Training on a sparse sample spread over
  a whole building reconstructs nothing; a contiguous capture segment
  reconstructs cleanly. Prefer --first-frame/--last-frame over --frame-stride.

NOTES:
  - Requires a CUDA GPU and a build with the gsplat backend compiled in
  - Requires sensor frames and a point cloud (run 'rux create clouds' first)
  - --max-image-size is the most effective speed knob: cost is per-pixel
  - Output .ply follows the reference 3DGS layout and opens in splat viewers
  - An existing .ply trained before schema v12 can be brought into a project
    with 'rux import gsplat <file.ply>'
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
  sub->add_option("--sh-degree-interval", opt->sh_degree_interval,
                  "Iterations between unlocking one more SH band "
                  "(0 = all bands from the start)")
      ->default_val(opt->sh_degree_interval)
      ->check(CLI::NonNegativeNumber);
  sub->add_option("--lr-sh-rest", opt->lr_sh_rest,
                  "Learning rate for SH degrees 1..n "
                  "(reference 3DGS uses 1/20 of the DC rate)")
      ->default_val(opt->lr_sh_rest)
      ->check(CLI::NonNegativeNumber);

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

  sub->add_option("--name", opt->splat_name,
                  "Name the trained splat is stored under in the project")
      ->default_val(opt->splat_name);
  // Optional since #322: the splat is stored in the project, so a run can no
  // longer end with nothing to show for itself. This is an export for viewers
  // outside ReUseX, not the output.
  sub->add_option("-o, --out", opt->out_ply,
                  "Additionally write the trained splat to this .ply");
  sub->add_option("--render-dir", opt->render_dir,
                  "Directory for checkpoint renders");
  sub->add_option("--render-at", opt->render_iterations,
                  "Iterations at which to dump a rendered PNG (0 = the "
                  "untrained seed), e.g. --render-at 0 500 2000")
      ->delimiter(',');
  sub->add_option("--render-view", opt->render_view_index,
                  "Index of the training view used for checkpoint renders")
      ->default_val(opt->render_view_index);

  sub->add_option("--checkpoint-every", opt->checkpoint_every,
                  "Write an intermediate .ply every N iterations (0 = off)")
      ->default_val(opt->checkpoint_every)
      ->check(CLI::Range(0, 1000000));
  sub->add_option("--checkpoint-keep", opt->checkpoint_keep,
                  "How many checkpoint .ply files to keep (0 = keep all)")
      ->default_val(opt->checkpoint_keep)
      ->check(CLI::Range(0, 10000));
  sub->add_option(
      "--checkpoint-dir", opt->checkpoint_dir,
      "Directory for checkpoint .ply files (default: beside --out)");

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

namespace {

/// Set by the SIGINT handler, polled by the training loop. Namespace scope
/// because a signal handler may only touch objects of static storage duration,
/// and lock-free because a handler may not block.
std::atomic_bool g_gsplat_cancel{false};
static_assert(std::atomic_bool::is_always_lock_free,
              "the SIGINT handler stores to this flag and must not block");

extern "C" void gsplat_sigint_handler(int sig) {
  // Async-signal-safe by construction: one relaxed store, and on the second
  // signal two calls that POSIX lists as safe. No logging, no allocation — the
  // training loop's own core::warn reports the cancel once it observes it.
  if (g_gsplat_cancel.load(std::memory_order_relaxed)) {
    // Second Ctrl-C: the user wants out NOW. Offering only the graceful path
    // would make the feature a trap on a run whose next iteration is minutes
    // away. Hand the signal back to the default disposition and re-raise, so
    // the process dies exactly as it would have with no handler installed.
    std::signal(sig, SIG_DFL);
    std::raise(sig);
    return;
  }
  g_gsplat_cancel.store(true, std::memory_order_relaxed);
}

/// Installs the SIGINT handler for the lifetime of one `create gsplat` run and
/// puts back whatever was there before. `rux` is one process running many
/// subcommands: leaving a handler pointing at this TU's flag after the
/// subcommand returned would silently swallow a later Ctrl-C.
class SigintGuard {
    public:
  SigintGuard() {
    // A previous run in the same process may have left the flag set.
    g_gsplat_cancel.store(false, std::memory_order_relaxed);
    previous_ = std::signal(SIGINT, gsplat_sigint_handler);
  }
  ~SigintGuard() {
    if (previous_ != SIG_ERR)
      std::signal(SIGINT, previous_);
  }
  SigintGuard(const SigintGuard &) = delete;
  SigintGuard &operator=(const SigintGuard &) = delete;

    private:
  void (*previous_)(int) = SIG_ERR;
};

} // namespace

int run_subcommand_create_gsplat(SubcommandCreateGsplatOptions const &opt,
                                 const RuxOptions &global_opt) {
  namespace fs = std::filesystem;
  namespace gs = reusex::gsplat;

  const fs::path project_path = global_opt.project_db;
  spdlog::info("Training Gaussian splats for project: {}",
               project_path.string());

  try {
    reusex::ProjectDB db(project_path);

    // Gate on the documented stage contract like every sibling `create`
    // subcommand (#331). The seed cloud is passed as an override so that
    // `--seed-cloud foo` is checked for `foo` rather than for the contract's
    // default name — without it the check would refuse for the wrong reason.
    //
    // This is also why `run_gsplat_stage`'s own "project has no point cloud
    // named 'X'" refusal never doubles up with this one: the CLI returns here
    // before the library is called, so exactly one message is printed. The
    // library keeps its check for the non-CLI callers (pipeline, ruxd) that do
    // not pass through this gate.
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::gsplat,
            {{"cloud", opt.seed_cloud}});
        rc != RuxError::SUCCESS)
      return rc;

    gs::GsplatStageOptions o;
    o.seed_cloud = opt.seed_cloud;
    o.init.max_points = opt.max_points;
    o.init.sh_degree = opt.sh_degree;

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
    o.train.sh_degree_interval = opt.sh_degree_interval;
    o.train.lr_sh_rest = opt.lr_sh_rest;
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
    o.splat_name = opt.splat_name;
    if (!opt.out_ply.empty())
      o.out_ply = opt.out_ply;

    o.train.checkpoint_every = opt.checkpoint_every;
    o.train.checkpoint_keep = opt.checkpoint_keep;
    if (!opt.checkpoint_dir.empty())
      o.train.checkpoint_dir = opt.checkpoint_dir;

    // Signal handling belongs to the application, never the library: the
    // library's half of the contract is the plain flag below. The guard is
    // scoped to the training call so the previous disposition is back in place
    // the moment the stage returns, however it returns.
    const SigintGuard sigint_guard;
    o.train.cancel_token = &g_gsplat_cancel;

    const gs::TrainResult r = gs::run_gsplat_stage(db, o);

    if (r.cancelled)
      spdlog::warn("Cancelled at iteration {} of {} — the splat was still "
                   "stored and holds the model as trained so far. Its metrics "
                   "are from a shortened run.",
                   r.iterations_run, opt.iterations);

    spdlog::info("Gaussian splatting {}: {} Gaussians, {:.2f} dB "
                 "(training views), {:.1f} s over {} iterations",
                 r.cancelled ? "stopped early" : "complete", r.final_count,
                 r.final_psnr, r.seconds, r.iterations_run);
    if (!r.evals.empty() && r.evals.back().holdout_views > 0)
      spdlog::info("Held-out: {:.2f} dB / SSIM {:.4f} over {} views excluded "
                   "from training",
                   r.final_holdout_psnr, r.final_holdout_ssim,
                   r.evals.back().holdout_views);
    if (opt.mcmc)
      spdlog::info("MCMC: relocated {}, added {} Gaussians", r.relocated,
                   r.added);
    spdlog::info("Stored in the project as '{}' — view it with `rux gui`",
                 opt.splat_name);
    if (!opt.out_ply.empty())
      spdlog::info("Wrote {}", opt.out_ply);
    for (const auto &p : r.renders)
      spdlog::info("Render: {}", p.string());
    for (const auto &p : r.checkpoints)
      spdlog::info("Checkpoint: {}", p.string());

    // SUCCESS even when cancelled. The user asked for the run to stop and it
    // stopped where they asked, having written the artifact they asked for; a
    // non-zero status would make `rux create gsplat && rux ...` and every CI
    // script treat a deliberate, successful salvage as a failure. The shortened
    // run is reported through the warning above and through the `pipeline_log`
    // note, which is where a reader looks for what happened.
    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Gaussian splatting failed: {}", e.what());
    return RuxError::GENERIC;
  }
}

#endif // REUSEX_HAVE_GSPLAT
