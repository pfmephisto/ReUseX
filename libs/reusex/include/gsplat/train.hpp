// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <reusex/gsplat/GaussianCloud.hpp>
#include <reusex/gsplat/TrainingViews.hpp>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace cv {
class Mat;
}

namespace reusex {
class ProjectDB;
}

namespace reusex::gsplat {

/// 3DGS-MCMC density control (Kheradmand et al. 2024).
///
/// The reference 3DGS densifier keys on the screen-space position gradient,
/// which gsplat's world-space rasterizer does not produce — see the note on
/// `TrainOptions::prune_enabled`. MCMC needs no screen-space quantity: it
/// relocates collapsed Gaussians onto high-opacity ones, samples new ones from
/// the same distribution up to a capped budget, and injects Langevin noise so
/// the whole thing is a sampler rather than a greedy heuristic.
///
/// Defaults follow gsplat's `MCMCStrategy` except for `cap`, which is relative
/// here (see below).
struct MCMCOptions {
  bool enabled = false;

  /// Hard ceiling on the Gaussian count, as a multiple of the seed count.
  /// Relative rather than absolute because this trainer seeds from a LiDAR
  /// cloud whose size is a property of the scan, not of the algorithm: a
  /// building-scale capture and a 1 m corridor differ by an order of
  /// magnitude, and one absolute cap cannot serve both. `cap_absolute`
  /// overrides it when non-zero.
  double cap_factor = 2.0;
  std::int64_t cap_absolute = 0;

  /// Multiplicative growth per refine pass, capped by the budget.
  double growth = 1.05;

  int refine_every = 100; ///< iterations between relocate+grow passes
  int refine_start = 500; ///< first eligible iteration
  /// Last eligible iteration, as a fraction of the run. Refinement has to stop
  /// well before the end so the final Gaussians get optimized rather than
  /// merely placed.
  double refine_stop_fraction = 0.85;

  /// Opacity at or below which a Gaussian counts as dead and is relocated.
  float min_opacity = 0.005f;

  float noise_lr = 5e5f;          ///< scales the Langevin noise (paper's value)
  float noise_opacity_t = 0.005f; ///< noise gate transition point
  float noise_opacity_k = 100.0f; ///< noise gate sharpness

  /// L1 regularizers in activated space. In the paper these are what let
  /// Gaussians die, so relocation has dead samples to recycle.
  ///
  /// **Default 0, against the paper's 0.01, on measured evidence.** They are
  /// unusable for a building-scale interior traverse, and the reason is
  /// scene topology rather than a tuning problem. 3DGS-MCMC was developed on
  /// object-centric captures where nearly every Gaussian projects into nearly
  /// every view, so the constant regularizer pull is balanced by a
  /// photometric gradient on almost every step. A corridor traverse is the
  /// opposite: each of our 344 views sees a small fraction of 1.2 M
  /// Gaussians, so most Gaussians receive *only* the regularizer gradient on
  /// most steps — and Adam, being scale-invariant, converts that into a step
  /// of the full learning rate. At `lr_opacities = 5e-2` an off-screen
  /// Gaussian falls from the seed opacity to `min_opacity` in ~60 iterations.
  ///
  /// Measured on NewOffice (394 views, 30 k iterations): with the paper's
  /// 0.01 the model collapses to 12.73 dB held-out and MCMC relocates 415 M
  /// Gaussians — ~68 % of the model on every refine pass. Lowering the weight
  /// does not help, exactly as Adam's scale invariance predicts: at 1e-4 it
  /// still collapses (15.20 dB, 22 M relocations). Only 0 is stable.
  ///
  /// Re-enable them for object-centric capture, where the assumption holds.
  float opacity_reg = 0.0f;
  float scale_reg = 0.0f;
};

/// Optimizer / schedule parameters for the 3DGS training loop.
///
/// Learning rates follow the reference 3DGS implementation. `lr_means` is
/// additionally scaled by the scene extent inside the trainer, because a
/// position learning rate in metres is only meaningful relative to how large
/// the scene is.
struct TrainOptions {
  int iterations = 2000;

  float lr_means = 1.6e-4f;   ///< scaled by scene extent (see above)
  float lr_scales = 5e-3f;    ///< on log-scales
  float lr_quats = 1e-3f;     ///< on raw (unnormalised) quaternions
  float lr_opacities = 5e-2f; ///< on logit-opacity
  float lr_sh_dc = 2.5e-3f;   ///< on the degree-0 SH coefficients

  /// Weight of the D-SSIM term: `loss = (1-l)*L1 + l*(1 - SSIM)`.
  float lambda_dssim = 0.2f;

  /// Periodically drop Gaussians whose opacity has collapsed. This is the only
  /// density control in this version — see the note on densification in
  /// docs/research/gaussian-splatting.md: the world-space rasterizer this
  /// trainer uses does not expose the screen-space `absgrad` that the
  /// reference clone/split heuristic keys on. Seeding from a LiDAR cloud
  /// supplies the density the reference has to grow.
  bool prune_enabled = true;
  int prune_interval = 500;      ///< iterations between prune passes
  int prune_start = 500;         ///< first iteration eligible for pruning
  float prune_opacity = 0.005f;  ///< alpha below which a Gaussian is dropped
  float prune_max_scale = 10.0f; ///< drop Gaussians larger than this [m]

  /// MCMC density control. When enabled it *replaces* pruning: MCMC relocates
  /// collapsed Gaussians instead of deleting them, and deleting them out from
  /// under it would just starve the relocation step.
  MCMCOptions mcmc;

  /// Held-out split: every `holdout_every`-th view (indices 0, N, 2N, …) is
  /// excluded from training and used only for evaluation. 0 trains on every
  /// view and reports no held-out number.
  ///
  /// The split is a pure function of the view count — it does not consume the
  /// RNG — so it is identical across runs and across the prune-only/MCMC
  /// comparison (STANDARDS §6).
  int holdout_every = 8;

  /// Iterations between held-out evaluation passes. Separate from
  /// `log_interval` because an evaluation renders many views and the training
  /// log line renders none.
  int eval_interval = 1000;

  /// Upper bound on the views used per evaluation pass, per side. Both sides
  /// are sub-sampled by a deterministic stride to the same count, so the train
  /// and held-out numbers stay comparable and the pass stays cheap.
  int eval_max_views = 32;

  /// Seeds the per-iteration view draw, which makes the *training schedule*
  /// reproducible.
  ///
  /// It does **not** make a run bit-reproducible, and nothing here can:
  /// gsplat's backward kernels accumulate per-Gaussian gradients with
  /// `atomicAdd`, so the summation order is decided by the GPU scheduler.
  /// Measured run-to-run spread is ~2.5e-6 relative on the loss after 20
  /// iterations. This is a documented departure from STANDARDS §6 — the
  /// alternative is a deterministic-but-far-slower gradient reduction that
  /// gsplat does not provide.
  unsigned seed = 42;

  int log_interval = 100; ///< iterations between progress log lines

  /// Dump a rendered PNG of `render_view_index` at each of these iterations
  /// (0 = the untrained seed). Ignored when `render_dir` is empty.
  std::vector<int> render_iterations;
  std::filesystem::path render_dir;
  std::size_t render_view_index = 0;

  /// Cooperative cancellation, owned by the caller and polled once per
  /// iteration. When it flips, the loop stops at the end of the current
  /// iteration, runs a final evaluation, and returns a complete TrainResult —
  /// the run is cut short, not thrown away. May be null.
  ///
  /// A raw pointer rather than a pipeline type on purpose: `reusex_gsplat` is
  /// a Layer-3 peer and cannot see `reusex_pipeline` (STANDARDS §1), so the
  /// contract is the narrowest thing both layers can name.
  const std::atomic_bool *cancel_token = nullptr;

  /// Write an intermediate `.ply` every N iterations (0 = off, the default).
  /// A long run that dies at iteration 29 000 of 30 000 otherwise produces
  /// nothing at all.
  int checkpoint_every = 0;
  /// How many checkpoint files to keep; older ones are deleted as new ones
  /// land. 0 keeps every checkpoint.
  int checkpoint_keep = 3;
  /// Directory for checkpoint `.ply` files. When empty, they go next to
  /// `GsplatStageOptions::out_ply`.
  std::filesystem::path checkpoint_dir;
};

/// One row of the loss history.
struct TrainMetrics {
  int iteration = 0;
  double loss = 0; ///< the optimized objective
  double l1 = 0;   ///< mean absolute error against the target view
  double psnr = 0; ///< dB, from the MSE of the same view
  std::size_t gaussians = 0;
};

/// One held-out evaluation pass.
///
/// Both sides are measured the same way on the same iteration, over
/// equally-sized view sets, so the gap between them is the generalisation gap
/// and not an artefact of how each was sampled.
struct EvalMetrics {
  int iteration = 0;
  double train_psnr = 0; ///< dB over views the optimizer does see
  double train_ssim = 0;
  double holdout_psnr = 0; ///< dB over views it never sees
  double holdout_ssim = 0;
  std::size_t train_views = 0;
  std::size_t holdout_views = 0;
  std::size_t gaussians = 0;
};

struct TrainResult {
  GaussianCloud gaussians;
  std::vector<TrainMetrics> history;
  /// Held-out evaluations, empty when `TrainOptions::holdout_every` is 0.
  std::vector<EvalMetrics> evals;
  double seconds = 0;
  std::size_t final_count = 0;
  double final_psnr = 0; ///< mean PSNR over the last logged window
  /// Held-out PSNR at the last evaluation, 0 when there was no split. This is
  /// the honest quality number; `final_psnr` is a training-view number.
  double final_holdout_psnr = 0;
  double final_holdout_ssim = 0;
  std::size_t relocated = 0; ///< cumulative MCMC relocations
  std::size_t added = 0;     ///< cumulative MCMC growth

  /// The view indices the optimizer actually drew during the run, sorted and
  /// deduplicated, and the ones reserved for evaluation.
  ///
  /// These are recorded rather than merely intended: "held out" is only a real
  /// claim if nothing ever backpropagated through those views, and that is a
  /// property of the draw loop, not of the split. Keeping the observed draws
  /// makes the separation checkable by a caller (and by a test) instead of
  /// having to be taken on trust.
  std::vector<std::size_t> trained_views;
  std::vector<std::size_t> holdout_view_indices;
  /// PNGs written by the `render_iterations` schedule, in write order. A
  /// scheduled render whose write failed is absent — the list is what is on
  /// disk, not what was attempted.
  std::vector<std::filesystem::path> renders;

  /// Intermediate `.ply` files from the `checkpoint_every` schedule that are
  /// still on disk, oldest first. Files already deleted by `checkpoint_keep`
  /// retention are dropped from this list as they go, so every entry names a
  /// file a caller can actually open.
  std::vector<std::filesystem::path> checkpoints;

  /// True when the run stopped early on a cancel request rather than by
  /// reaching `TrainOptions::iterations`. The model and metrics are real, they
  /// are just from fewer iterations — check this before comparing two runs.
  bool cancelled = false;
  /// The last iteration actually executed. Equals `TrainOptions::iterations`
  /// for a run that finished.
  int iterations_run = 0;
};

/// True when this build actually contains the CUDA trainer. Always true in a
/// translation unit that can link `reusex_gsplat`; provided so callers can
/// report the capability without an #ifdef of their own.
bool is_available();

/// True when this process can actually reach a CUDA device right now.
/// `is_available()` answers "was the trainer compiled in"; this answers "is
/// there a GPU to run it on", which is what a [gpu]-tagged test needs in order
/// to SKIP honestly instead of failing.
bool has_cuda_device();

/// Train Gaussians against posed views.
///
/// Requires a CUDA device. All parameters (means, log-scales, quaternions,
/// logit-opacity, SH DC) are optimized jointly with Adam; the photometric loss
/// is L1 + D-SSIM against one randomly drawn view per iteration.
///
/// @throws std::runtime_error if no CUDA device is available, if @p views is
///         empty, or if @p init contains no Gaussians.
TrainResult train_gaussians(const GaussianCloud &init,
                            const std::vector<TrainingView> &views,
                            const TrainOptions &opt = {});

/// Rasterize @p gaussians from @p view. Returns a BGR8 image the same size as
/// the view, suitable for cv::imwrite. Requires CUDA.
cv::Mat render_view(const GaussianCloud &gaussians, const TrainingView &view);

/// End-to-end options for the `rux create gsplat` stage.
struct GsplatStageOptions {
  std::string seed_cloud = "cloud"; ///< ProjectDB cloud used to seed Gaussians
  GaussianInitOptions init;
  TrainingViewOptions views;
  TrainOptions train;

  /// Name the trained splat is stored under in the project (#322).
  ///
  /// The stage's real output. An empty name is refused rather than treated as
  /// "do not store": training for hours and keeping nothing is the failure
  /// mode this stage was built to prevent.
  std::string splat_name = "splat";

  /// Optional extra copy on disk, for a viewer outside ReUseX. The project
  /// copy is written either way, so this is an export, not the output.
  std::filesystem::path out_ply;
};

/// Load the seed cloud and views from @p db, train, and store the splat.
///
/// The trained model is written into @p db under `GsplatStageOptions::
/// splat_name` (schema v12), and *additionally* to `out_ply` when one is
/// given. Both get the identical bytes, from one call to gaussian_ply_bytes().
///
/// @p db is therefore non-const for two reasons: the splat itself, and the
/// `pipeline_log` start and finish rows every other `create` stage writes —
/// `rux log` is how a user reconstructs what produced a project, and a stage
/// that runs for hours without appearing there is invisible. All project
/// *data* is read-only.
///
/// A run cancelled through `TrainOptions::cancel_token` still stores its splat
/// — salvaging the model is the entire point of cancelling rather than killing
/// the process — and closes its `pipeline_log` row as a success carrying a
/// "CANCELLED" note (see the comment at the call site for why not a failure).
///
/// @throws std::runtime_error if `splat_name` is empty, if checkpointing is
///         enabled with nowhere to put the files, or if the seed cloud is
///         missing — all before any training happens, because a long run whose
///         output is silently discarded is worse than a refusal.
TrainResult run_gsplat_stage(ProjectDB &db, const GsplatStageOptions &opt);

} // namespace reusex::gsplat
