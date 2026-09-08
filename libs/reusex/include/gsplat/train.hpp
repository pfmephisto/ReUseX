// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <reusex/gsplat/GaussianCloud.hpp>
#include <reusex/gsplat/TrainingViews.hpp>

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
};

/// One row of the loss history.
struct TrainMetrics {
  int iteration = 0;
  double loss = 0; ///< the optimized objective
  double l1 = 0;   ///< mean absolute error against the target view
  double psnr = 0; ///< dB, from the MSE of the same view
  std::size_t gaussians = 0;
};

struct TrainResult {
  GaussianCloud gaussians;
  std::vector<TrainMetrics> history;
  double seconds = 0;
  std::size_t final_count = 0;
  double final_psnr = 0; ///< mean PSNR over the last logged window
  /// PNGs written by the `render_iterations` schedule, in write order.
  std::vector<std::filesystem::path> renders;
};

/// True when this build actually contains the CUDA trainer. Always true in a
/// translation unit that can link `reusex_gsplat`; provided so callers can
/// report the capability without an #ifdef of their own.
bool is_available();

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
  std::filesystem::path out_ply; ///< where to write the trained splat
};

/// Load the seed cloud and views from @p db, train, and write the `.ply`.
TrainResult run_gsplat_stage(const ProjectDB &db,
                             const GsplatStageOptions &opt);

} // namespace reusex::gsplat
