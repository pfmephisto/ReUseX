// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/gsplat/train.hpp>

#include "mcmc.hpp"
#include "optimizer.hpp"
#include "rasterize.hpp"
#include "ssim.hpp"
#include "view_sampling.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>

#include <ATen/cuda/CUDAContext.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>

namespace reusex::gsplat {

namespace {

using detail::GaussianTensors;
using detail::sh_bands;

/// Compute capabilities the vendored gsplat kernels were compiled for, as a
/// comma-separated list of `major*10+minor` values. Set by reusexLibrary.cmake
/// from REUSEX_GSPLAT_CUDA_ARCHS, which mirrors `cudaCapabilities` in
/// pkgs/gsplat-cuda/package.nix.
#ifndef REUSEX_GSPLAT_CUDA_ARCHS
#define REUSEX_GSPLAT_CUDA_ARCHS "89"
#endif

/// Parse "89" or "80,86,89" into (major, minor) pairs. Tolerates the trailing
/// letter of nvcc's `90a`-style variants by ignoring non-digits.
std::vector<std::pair<int, int>> compiled_archs() {
  std::vector<std::pair<int, int>> out;
  const std::string spec = REUSEX_GSPLAT_CUDA_ARCHS;
  std::string tok;
  auto flush = [&] {
    if (tok.empty())
      return;
    const int v = std::atoi(tok.c_str());
    if (v > 0)
      out.emplace_back(v / 10, v % 10);
    tok.clear();
  };
  for (const char c : spec) {
    if (c >= '0' && c <= '9')
      tok.push_back(c);
    else if (c == ',' || c == ';' || c == ' ')
      flush();
  }
  flush();
  return out;
}

void require_cuda() {
  if (!torch::cuda::is_available())
    throw std::runtime_error(
        "gsplat: no CUDA device available. The Gaussian-splatting trainer is "
        "GPU-only (the rasterizer has no CPU path).");

  // The vendored rasterizer is built for a *single* architecture by default
  // and ships no PTX, so there is no JIT fallback: on any other GPU the first
  // kernel launch fails with a bare "no kernel image is available for
  // execution on the device", minutes into a run, after the cloud and every
  // training view have already been loaded. Check up front instead, and say
  // what to do about it.
  const cudaDeviceProp *props = at::cuda::getCurrentDeviceProperties();
  const auto archs = compiled_archs();
  // CUDA cubins are forward-compatible within a major version only: a kernel
  // built for sm_80 runs on sm_86/sm_89, but nothing built for sm_89 runs on
  // sm_86, and nothing crosses a major version.
  const bool ok = archs.empty() ||
                  std::any_of(archs.begin(), archs.end(), [&](const auto &a) {
                    return a.first == props->major && a.second <= props->minor;
                  });
  if (!ok) {
    std::string have;
    for (const auto &a : archs)
      have += (have.empty() ? "" : ", ") +
              fmt::format("sm_{}{}", a.first, a.second);
    throw std::runtime_error(fmt::format(
        "gsplat: this build's CUDA kernels target {} but '{}' is sm_{}{}, and "
        "the vendored rasterizer contains no PTX to JIT from — every kernel "
        "launch would fail with 'no kernel image is available'. Rebuild the "
        "rasterizer for this GPU: gsplat-cuda.override {{ cudaCapabilities = "
        "[\"{}.{}\"]; }} (see pkgs/gsplat-cuda/package.nix; budget ~20 min of "
        "nvcc per architecture), then reconfigure with "
        "-DREUSEX_GSPLAT_CUDA_ARCHS={}{}.",
        have, props->name, props->major, props->minor, props->major,
        props->minor, props->major, props->minor));
  }
}

// ---------------------------------------------------------------------------
// GaussianCloud <-> CUDA tensors
// ---------------------------------------------------------------------------

/// Flatten a vector of fixed-size float arrays into a [N,D] CUDA tensor.
template <std::size_t D>
torch::Tensor upload(const std::vector<std::array<float, D>> &src) {
  const int64_t n = static_cast<int64_t>(src.size());
  std::vector<float> flat;
  flat.reserve(src.size() * D);
  for (const auto &e : src)
    flat.insert(flat.end(), e.begin(), e.end());
  return torch::from_blob(flat.data(), {n, static_cast<int64_t>(D)},
                          torch::kFloat32)
      .clone()
      .to(torch::kCUDA);
}

GaussianTensors to_tensors(const GaussianCloud &g) {
  const int64_t n = static_cast<int64_t>(g.size());
  const int64_t K = sh_bands(g.sh_degree);

  GaussianTensors t;
  t.sh_degree = g.sh_degree;
  t.means = upload(g.means);
  t.log_scales = upload(g.scales);
  t.quats = upload(g.quats);
  t.logit_opacity = torch::from_blob(const_cast<float *>(g.opacities.data()),
                                     {n}, torch::kFloat32)
                        .clone()
                        .to(torch::kCUDA);

  // SH layout [N, K, 3]. Band 0 is the DC triple; the higher bands come from
  // sh_rest, which is stored channel-major (all of channel 0's coefficients,
  // then channel 1's, then channel 2's) to match the reference .ply.
  std::vector<float> sh(static_cast<std::size_t>(n * K * 3), 0.0f);
  const int64_t rest = K - 1;
  for (int64_t i = 0; i < n; ++i) {
    for (int64_t c = 0; c < 3; ++c)
      sh[static_cast<std::size_t>(i * K * 3 + 0 * 3 + c)] =
          g.sh_dc[static_cast<std::size_t>(i)][static_cast<std::size_t>(c)];
    if (rest > 0 && !g.sh_rest.empty()) {
      const auto &r = g.sh_rest[static_cast<std::size_t>(i)];
      for (int64_t c = 0; c < 3; ++c)
        for (int64_t j = 0; j < rest; ++j)
          sh[static_cast<std::size_t>(i * K * 3 + (j + 1) * 3 + c)] =
              r[static_cast<std::size_t>(c * rest + j)];
    }
  }
  t.sh = torch::from_blob(sh.data(), {n, K, 3}, torch::kFloat32)
             .clone()
             .to(torch::kCUDA);
  return t;
}

GaussianCloud to_cloud(const GaussianTensors &t) {
  auto means = t.means.detach().to(torch::kCPU).contiguous();
  auto scales = t.log_scales.detach().to(torch::kCPU).contiguous();
  auto quats = t.quats.detach().to(torch::kCPU).contiguous();
  auto opac = t.logit_opacity.detach().to(torch::kCPU).contiguous();
  auto sh = t.sh.detach().to(torch::kCPU).contiguous();

  const int64_t n = means.size(0);
  const int64_t K = sh.size(1);
  const int64_t rest = K - 1;

  GaussianCloud g;
  g.sh_degree = t.sh_degree;
  g.means.resize(static_cast<std::size_t>(n));
  g.scales.resize(static_cast<std::size_t>(n));
  g.quats.resize(static_cast<std::size_t>(n));
  g.opacities.resize(static_cast<std::size_t>(n));
  g.sh_dc.resize(static_cast<std::size_t>(n));
  if (rest > 0)
    g.sh_rest.assign(static_cast<std::size_t>(n),
                     std::vector<float>(static_cast<std::size_t>(rest * 3)));

  const float *pm = means.data_ptr<float>();
  const float *ps = scales.data_ptr<float>();
  const float *pq = quats.data_ptr<float>();
  const float *po = opac.data_ptr<float>();
  const float *ph = sh.data_ptr<float>();

  for (int64_t i = 0; i < n; ++i) {
    const auto u = static_cast<std::size_t>(i);
    g.means[u] = {pm[i * 3], pm[i * 3 + 1], pm[i * 3 + 2]};
    g.scales[u] = {ps[i * 3], ps[i * 3 + 1], ps[i * 3 + 2]};
    g.quats[u] = {pq[i * 4], pq[i * 4 + 1], pq[i * 4 + 2], pq[i * 4 + 3]};
    g.opacities[u] = po[i];
    g.sh_dc[u] = {ph[i * K * 3 + 0], ph[i * K * 3 + 1], ph[i * K * 3 + 2]};
    for (int64_t c = 0; c < 3; ++c)
      for (int64_t j = 0; j < rest; ++j)
        g.sh_rest[u][static_cast<std::size_t>(c * rest + j)] =
            ph[i * K * 3 + (j + 1) * 3 + c];
  }
  g.validate();
  return g;
}

// ---------------------------------------------------------------------------
// Images
// ---------------------------------------------------------------------------

/// BGR8 cv::Mat -> [H,W,3] float CUDA tensor in [0,1], RGB order.
///
/// The channel swap matters: Gaussians are seeded from PointT's r/g/b, so the
/// SH DC term is RGB. Training against a BGR target would silently learn the
/// colours swapped.
torch::Tensor image_to_tensor(const cv::Mat &bgr) {
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  cv::Mat f;
  rgb.convertTo(f, CV_32FC3, 1.0 / 255.0);
  return torch::from_blob(f.data, {f.rows, f.cols, 3}, torch::kFloat32)
      .clone()
      .to(torch::kCUDA);
}

/// [H,W,3] float RGB tensor -> BGR8 cv::Mat, clamped to [0,1].
cv::Mat tensor_to_image(const torch::Tensor &hwc) {
  auto c = hwc.detach()
               .clamp(0.0f, 1.0f)
               .mul(255.0f)
               .to(torch::kU8)
               .to(torch::kCPU)
               .contiguous();
  cv::Mat rgb(static_cast<int>(c.size(0)), static_cast<int>(c.size(1)), CV_8UC3,
              c.data_ptr<uint8_t>());
  cv::Mat bgr;
  cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
  return bgr; // cvtColor allocated fresh storage, so this outlives `c`
}

} // namespace

bool is_available() { return true; }

bool has_cuda_device() { return torch::cuda::is_available(); }

// ---------------------------------------------------------------------------

TrainResult train_gaussians(const GaussianCloud &init,
                            const std::vector<TrainingView> &views,
                            const TrainOptions &opt) {
  require_cuda();
  if (init.empty())
    throw std::runtime_error("gsplat: cannot train from zero Gaussians");
  if (views.empty())
    throw std::runtime_error("gsplat: cannot train without training views");
  if (opt.iterations < 1)
    throw std::runtime_error("gsplat: iterations must be >= 1");

  init.validate();

  GaussianTensors g = to_tensors(init);
  g.means.set_requires_grad(true);
  g.log_scales.set_requires_grad(true);
  g.quats.set_requires_grad(true);
  g.logit_opacity.set_requires_grad(true);
  g.sh.set_requires_grad(true);

  const double extent = detail::scene_extent(views);
  const detail::ViewSplit split =
      detail::split_views(views.size(), opt.holdout_every);

  core::info(
      "gsplat: {} views ({} train / {} held out), scene extent {:.2f} m, "
      "{} seed Gaussians, SH degree {}",
      views.size(), split.train.size(), split.holdout.size(), extent,
      init.size(), init.sh_degree);

  detail::AdamLrs lrs;
  lrs.means = opt.lr_means * extent;
  lrs.log_scales = opt.lr_scales;
  lrs.quats = opt.lr_quats;
  lrs.logit_opacity = opt.lr_opacities;
  lrs.sh = opt.lr_sh_dc;
  auto optimizer = detail::make_adam(g, lrs);

  // Seeds torch's own generators. MCMC draws from them (multinomial for
  // relocation targets, randn for the Langevin noise), so without this the
  // density control would differ run to run even though the view schedule
  // would not.
  torch::manual_seed(opt.seed);

  // Pre-upload the targets once. A short run revisits every view many times, so
  // paying the host->device copy per iteration would dominate the loop.
  std::vector<torch::Tensor> targets;
  std::vector<torch::Tensor> viewmats;
  std::vector<torch::Tensor> intrinsics;
  targets.reserve(views.size());
  for (const auto &v : views) {
    targets.push_back(image_to_tensor(v.image));
    viewmats.push_back(
        torch::from_blob(const_cast<double *>(v.T_cw.data()), {4, 4},
                         torch::kFloat64)
            .t() // Eigen is column-major, torch expects row-major
            .contiguous()
            .to(torch::kFloat32)
            .to(torch::kCUDA));
    intrinsics.push_back(torch::from_blob(const_cast<double *>(v.K.data()),
                                          {3, 3}, torch::kFloat64)
                             .t()
                             .contiguous()
                             .to(torch::kFloat32)
                             .to(torch::kCUDA));
  }

  // A destination the caller named but that cannot be created is a mistake to
  // report NOW, before the hours of training that would discover it. This is
  // the opposite call from a *write* failure mid-run (see dump_render and
  // write_checkpoint): the destination is an input, the individual file is a
  // by-product.
  auto require_directory = [](const std::filesystem::path &dir,
                              std::string_view what, std::string_view flag) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    // create_directories reports no error for a directory that already exists,
    // so the existence question has to be asked separately.
    if (!std::filesystem::is_directory(dir))
      throw std::runtime_error(fmt::format(
          "gsplat: cannot create the {} directory '{}'{} — refusing "
          "to train with an unusable {}.",
          what, dir.string(),
          ec ? fmt::format(": {}", ec.message()) : std::string{}, flag));
  };

  std::filesystem::path render_dir = opt.render_dir;
  const std::size_t render_idx =
      std::min(opt.render_view_index, views.size() - 1);
  if (!render_dir.empty())
    require_directory(render_dir, "render", "--render-dir");

  const bool checkpointing = opt.checkpoint_every > 0;
  if (checkpointing) {
    if (opt.checkpoint_dir.empty())
      throw std::runtime_error(
          "gsplat: checkpointing is enabled but no checkpoint directory was "
          "resolved — pass --checkpoint-dir (or -o/--out, whose parent "
          "directory is used by default).");
    require_directory(opt.checkpoint_dir, "checkpoint", "--checkpoint-dir");
    core::info("gsplat: checkpointing every {} iterations to '{}', keeping {}",
               opt.checkpoint_every, opt.checkpoint_dir.string(),
               opt.checkpoint_keep > 0 ? fmt::format("{}", opt.checkpoint_keep)
                                       : std::string("all"));
  }

  TrainResult result;

  // The last iteration a render / evaluation was attempted at, so the
  // post-cancel wrap-up can top the run up without duplicating work the loop
  // already did on its final pass.
  int last_render_iter = -1;
  int last_eval_iter = -1;

  auto dump_render = [&](int iteration) {
    if (render_dir.empty())
      return;
    last_render_iter = iteration;
    torch::NoGradGuard no_grad;
    const auto &v = views[render_idx];
    auto out = detail::render(g, viewmats[render_idx], intrinsics[render_idx],
                              v.width(), v.height(), g.sh_degree);
    const auto path =
        render_dir / fmt::format("render_{}_iter{:06d}.png", v.name, iteration);

    // A checkpoint render is a diagnostic, not the product: a full disk or a
    // missing codec at iteration 500 of 30 000 must not throw away the run.
    // cv::imwrite signals both ways — `false` for an unwritable path, a
    // cv::Exception for an unknown extension or an absent codec — so both are
    // caught here and reported the same way.
    std::string reason;
    try {
      if (!cv::imwrite(path.string(), tensor_to_image(out.image)))
        reason = "cv::imwrite returned false (unwritable path, full disk, or "
                 "no codec for this extension)";
    } catch (const std::exception &e) {
      reason = e.what();
    }
    if (!reason.empty()) {
      core::warn("gsplat: could not write checkpoint render '{}': {}. Training "
                 "continues — the render is a diagnostic, not the run's "
                 "output. TrainResult::renders will not list this file.",
                 path.string(), reason);
      return;
    }

    result.renders.push_back(path);
    core::info("gsplat: wrote checkpoint render {}", path.string());
  };

  // --- periodic checkpoints ------------------------------------------------
  // Insurance against a run that dies late: a 30 k-iteration job that crashes
  // at 29 k otherwise leaves nothing at all behind.
  auto write_checkpoint = [&](int iteration) {
    if (!checkpointing)
      return;
    const auto final_path = opt.checkpoint_dir /
                            fmt::format("checkpoint_iter{:06d}.ply", iteration);
    // Write to a sibling `.tmp` and rename onto the final name. rename() is
    // atomic within a filesystem, so a crash or a Ctrl-C part-way through the
    // (large, slow) .ply write leaves either the previous checkpoint or the
    // new one — never a truncated file whose header still parses as valid and
    // whose Gaussian list stops halfway.
    const std::filesystem::path tmp_path = final_path.string() + ".tmp";

    try {
      const GaussianCloud snapshot = to_cloud(g);
      save_gaussian_ply(snapshot, tmp_path);
      std::filesystem::rename(tmp_path, final_path);
      result.checkpoints.push_back(final_path);
      core::info("gsplat: checkpoint at iter {} -> {} ({} Gaussians)",
                 iteration, final_path.string(), snapshot.size());
    } catch (const std::exception &e) {
      // Same reasoning as a failed render: the checkpoint is insurance, not
      // the product. Killing an hours-long run because the insurance could not
      // be filed is the worse outcome.
      core::warn("gsplat: could not write checkpoint '{}': {}. Training "
                 "continues without it.",
                 final_path.string(), e.what());
      std::error_code ec;
      std::filesystem::remove(tmp_path, ec); // best-effort; a stray .tmp is
                                             // noise, not corruption
      return;
    }

    // Retention. Driven by the paths this run wrote, in order, rather than by
    // globbing the directory — the trainer must never delete a file it did not
    // create, and a user's own `.ply` sitting in the same folder is not ours
    // to remove.
    if (opt.checkpoint_keep > 0) {
      while (result.checkpoints.size() >
             static_cast<std::size_t>(opt.checkpoint_keep)) {
        const auto oldest = result.checkpoints.front();
        result.checkpoints.erase(result.checkpoints.begin());
        std::error_code ec;
        std::filesystem::remove(oldest, ec);
        if (ec)
          core::warn("gsplat: could not delete superseded checkpoint '{}': {}",
                     oldest.string(), ec.message());
      }
    }
  };

  auto wants_render = [&](int it) {
    return std::find(opt.render_iterations.begin(), opt.render_iterations.end(),
                     it) != opt.render_iterations.end();
  };

  if (wants_render(0)) {
    // The seed render is the honest "before" picture: pure point-cloud colour,
    // no optimization at all.
    dump_render(0);
  }

  // --- held-out evaluation -------------------------------------------------
  // Both sides are strided down to the same budget so the pair is comparable
  // and the pass stays cheap relative to training.
  const auto eval_train_views =
      detail::stride_sample(split.train, opt.eval_max_views);
  const auto eval_holdout_views =
      detail::stride_sample(split.holdout, opt.eval_max_views);

  auto eval_set = [&](const std::vector<std::size_t> &idx) {
    torch::NoGradGuard no_grad;
    double psnr_sum = 0.0;
    double ssim_sum = 0.0;
    for (const std::size_t vi : idx) {
      const auto &v = views[vi];
      auto out = detail::render(g, viewmats[vi], intrinsics[vi], v.width(),
                                v.height(), g.sh_degree);
      auto pred = out.image.clamp(0.0f, 1.0f);
      const auto &target = targets[vi];
      auto mse = torch::mse_loss(pred, target);
      psnr_sum += 10.0 * std::log10(1.0 / std::max(mse.item<double>(), 1e-12));
      ssim_sum += detail::ssim(pred.permute({2, 0, 1}).unsqueeze(0),
                               target.permute({2, 0, 1}).unsqueeze(0))
                      .item<double>();
    }
    const auto n = static_cast<double>(std::max<std::size_t>(idx.size(), 1));
    return std::pair<double, double>{psnr_sum / n, ssim_sum / n};
  };

  auto run_eval = [&](int it) {
    if (eval_train_views.empty())
      return;
    last_eval_iter = it;
    EvalMetrics m;
    m.iteration = it;
    m.gaussians = static_cast<std::size_t>(g.count());
    std::tie(m.train_psnr, m.train_ssim) = eval_set(eval_train_views);
    m.train_views = eval_train_views.size();
    if (!eval_holdout_views.empty()) {
      std::tie(m.holdout_psnr, m.holdout_ssim) = eval_set(eval_holdout_views);
      m.holdout_views = eval_holdout_views.size();
      core::info("gsplat: eval iter {:6d}  train {:.2f} dB / SSIM {:.4f} ({} "
                 "views)  held-out {:.2f} dB / SSIM {:.4f} ({} views)  N {}",
                 m.iteration, m.train_psnr, m.train_ssim, m.train_views,
                 m.holdout_psnr, m.holdout_ssim, m.holdout_views, m.gaussians);
    } else {
      core::info("gsplat: eval iter {:6d}  train {:.2f} dB / SSIM {:.4f} ({} "
                 "views)  no held-out split  N {}",
                 m.iteration, m.train_psnr, m.train_ssim, m.train_views,
                 m.gaussians);
    }
    result.evals.push_back(m);
  };

  // --- MCMC density control ------------------------------------------------
  const bool mcmc = opt.mcmc.enabled;
  detail::MCMCState mcmc_state;
  int64_t mcmc_cap = 0;
  int mcmc_stop = 0;
  if (mcmc) {
    mcmc_state = detail::make_mcmc_state(g.means.device());
    mcmc_cap = opt.mcmc.cap_absolute > 0
                   ? opt.mcmc.cap_absolute
                   : static_cast<int64_t>(static_cast<double>(init.size()) *
                                          opt.mcmc.cap_factor);
    mcmc_cap = std::max<int64_t>(mcmc_cap, g.count());
    mcmc_stop = static_cast<int>(static_cast<double>(opt.iterations) *
                                 opt.mcmc.refine_stop_fraction);
    core::info("gsplat: MCMC density control on — budget {} Gaussians "
               "(x{:.2f} of seed), refine every {} from iter {} to {}, "
               "noise_lr {:.3g}, opacity_reg {:.3g}, scale_reg {:.3g}",
               mcmc_cap, opt.mcmc.cap_factor, opt.mcmc.refine_every,
               opt.mcmc.refine_start, mcmc_stop, opt.mcmc.noise_lr,
               opt.mcmc.opacity_reg, opt.mcmc.scale_reg);
    if (opt.prune_enabled)
      core::info("gsplat: pruning is disabled while MCMC is active — MCMC "
                 "relocates collapsed Gaussians rather than deleting them");
  }

  // Baseline at iteration 0: the seed cloud rendered as Gaussians, before any
  // optimization. Without it the quality curves start mid-air and there is
  // nothing to attribute the first thousand iterations' gain to.
  run_eval(0);

  std::mt19937 rng(opt.seed);
  std::uniform_int_distribution<std::size_t> pick(0, split.train.size() - 1);

  const auto t0 = std::chrono::steady_clock::now();
  double window_psnr = 0.0;
  int window_n = 0;
  // Kept so a cancel between two log lines can still flush an honest history
  // row rather than reporting the last complete window as if it were the end.
  double last_loss_value = 0.0;
  double last_l1_value = 0.0;

  // Which views the loop actually touched, recorded as it goes so the held-out
  // claim rests on the observed draws rather than on the split alone.
  std::vector<bool> drawn(views.size(), false);

  // The last iteration that ran to completion. Assigned at the bottom of the
  // loop body so a cancel can never claim credit for a half-applied step.
  int completed = 0;

  for (int it = 1; it <= opt.iterations; ++it) {
    // Poll at the TOP of the body, before anything is rendered or stepped. A
    // cancel arriving mid-iteration therefore takes effect at the next clean
    // boundary rather than leaving a backward() without its optimizer step, or
    // an MCMC relocation without the noise injection that belongs with it.
    if (opt.cancel_token != nullptr &&
        opt.cancel_token->load(std::memory_order_relaxed)) {
      result.cancelled = true;
      break;
    }

    const std::size_t vi = split.train[pick(rng)];
    drawn[vi] = true;
    const auto &v = views[vi];

    auto out = detail::render(g, viewmats[vi], intrinsics[vi], v.width(),
                              v.height(), g.sh_degree);

    auto pred = out.image;            // [H,W,3]
    const auto &target = targets[vi]; // [H,W,3]

    auto l1 = torch::abs(pred - target).mean();
    torch::Tensor loss;
    if (opt.lambda_dssim > 0.0f) {
      auto p = pred.permute({2, 0, 1}).unsqueeze(0); // [1,3,H,W]
      auto t = target.permute({2, 0, 1}).unsqueeze(0);
      auto s = detail::ssim(p, t);
      loss = (1.0f - opt.lambda_dssim) * l1 + opt.lambda_dssim * (1.0 - s);
    } else {
      loss = l1;
    }

    if (mcmc) {
      // The MCMC objective is the photometric loss plus L1 on activated
      // opacity and scale. These are not cosmetic: they are what drives
      // redundant Gaussians down to `min_opacity` so relocation has dead
      // samples to recycle. `TrainMetrics::loss` therefore includes them,
      // while `l1` and `psnr` stay purely photometric and comparable across
      // the prune-only/MCMC pair.
      auto reg = detail::mcmc_regularizer(g, opt.mcmc);
      if (reg.defined())
        loss = loss + reg;
    }

    // Fail loud on divergence (STANDARDS §5). Without this a single non-finite
    // loss NaNs every gradient in backward(), Adam NaNs all five parameters in
    // one step, and the run continues to completion: the log shows `PSNR nan`,
    // validate() checks lengths and passes, a full-size garbage .ply is written
    // and `rux` exits 0. There is no recovery once the parameters are NaN, so
    // the only honest outcome is to stop at the iteration that produced it.
    // The sync this costs is already paid below for the PSNR line.
    const double loss_value = loss.item<double>();
    // Read out alongside the loss rather than after the optimizer step: the
    // stream is already drained by the line above, so the second copy costs
    // almost nothing, and having the number in hand lets a cancelled run flush
    // a final history row instead of reporting the last full log window.
    const double l1_value = l1.item<double>();
    if (!std::isfinite(loss_value))
      throw std::runtime_error(fmt::format(
          "gsplat: training diverged at iteration {} — loss is {} on view "
          "'{}' ({} Gaussians). Aborting rather than writing a NaN model. "
          "Usual causes: a learning rate too high for this scene, a seed cloud "
          "with degenerate scales, or MCMC noise_lr too large for the scene "
          "extent.",
          it, loss_value, v.name, g.count()));

    optimizer->zero_grad();
    loss.backward();
    optimizer->step();

    double psnr = 0.0;
    {
      torch::NoGradGuard no_grad;
      auto mse = torch::mse_loss(pred.clamp(0.0f, 1.0f), target);
      psnr = 10.0 * std::log10(1.0 / std::max(mse.item<double>(), 1e-12));
    }
    window_psnr += psnr;
    ++window_n;
    last_loss_value = loss_value;
    last_l1_value = l1_value;

    if (it % opt.log_interval == 0 || it == opt.iterations) {
      TrainMetrics m;
      m.iteration = it;
      m.loss = loss_value;
      m.l1 = l1_value;
      m.psnr = window_psnr / std::max(window_n, 1);
      m.gaussians = static_cast<std::size_t>(g.count());
      result.history.push_back(m);
      core::info("gsplat: iter {:6d}  loss {:.5f}  L1 {:.5f}  PSNR {:.2f} dB  "
                 "N {}",
                 m.iteration, m.loss, m.l1, m.psnr, m.gaussians);
      window_psnr = 0.0;
      window_n = 0;
    }

    // ---- density control ---------------------------------------------------
    // Two mutually exclusive regimes. MCMC (relocate + grow + Langevin noise)
    // is the one that can add detail beyond the seed cloud; pruning alone is
    // the fallback, and it can only remove. The reference clone/split
    // heuristic is available in neither, because gsplat's world-space
    // rasterizer never forms the screen-space gradient it keys on (mcmc.hpp).
    if (mcmc) {
      if (it >= opt.mcmc.refine_start && it <= mcmc_stop &&
          it % opt.mcmc.refine_every == 0) {
        const int64_t before = g.count();
        const auto counts = detail::mcmc_refine(g, optimizer, lrs, mcmc_state,
                                                opt.mcmc, mcmc_cap);
        if (counts.relocated > 0 || counts.added > 0)
          core::debug("gsplat: MCMC refine at iter {}: relocated {}, added {}, "
                      "{} -> {} Gaussians (budget {})",
                      it, counts.relocated, counts.added, before, g.count(),
                      mcmc_cap);
      }
      // The Langevin term runs every iteration, not just on refine passes:
      // it is part of the sampler, not part of density control.
      detail::mcmc_inject_noise(g, opt.mcmc, lrs.means);
    } else if (opt.prune_enabled && it >= opt.prune_start &&
               it % opt.prune_interval == 0 && it < opt.iterations) {
      torch::NoGradGuard no_grad;
      auto alpha = torch::sigmoid(g.logit_opacity);
      auto max_scale = torch::exp(g.log_scales).amax(1);
      auto keep = (alpha > opt.prune_opacity)
                      .logical_and(max_scale < opt.prune_max_scale);
      const int64_t kept = keep.sum().item<int64_t>();
      const int64_t before = g.count();
      if (kept == 0) {
        core::warn("gsplat: prune at iter {} would remove every Gaussian "
                   "(all {} below opacity {}); skipping the pass",
                   it, before, opt.prune_opacity);
      } else if (kept < before) {
        auto idx = torch::nonzero(keep).squeeze(1);
        // Carries Adam's moments through the row remap — surviving Gaussians
        // keep their momentum instead of restarting it every prune pass.
        detail::remap_parameters(g, optimizer, lrs, idx, /*reset=*/{});
        core::info("gsplat: pruned {} -> {} Gaussians at iter {} "
                   "(alpha <= {} or scale >= {} m)",
                   before, kept, it, opt.prune_opacity, opt.prune_max_scale);
      }
    }

    if (it % std::max(opt.eval_interval, 1) == 0 || it == opt.iterations)
      run_eval(it);

    if (wants_render(it))
      dump_render(it);

    // No checkpoint on the final iteration: `result.gaussians` already carries
    // that exact model back to the caller, which writes it to `out_ply`. A
    // checkpoint there would be a byte-identical duplicate.
    if (checkpointing && it % opt.checkpoint_every == 0 && it < opt.iterations)
      write_checkpoint(it);

    completed = it;
  }

  result.iterations_run = completed;

  if (result.cancelled) {
    core::warn("gsplat: CANCELLED at iteration {} of {} — stopping here and "
               "keeping the model as trained so far. Metrics below are from {} "
               "iterations, not {}; do not compare them against a completed "
               "run.",
               completed, opt.iterations, completed, opt.iterations);

    if (completed >= 1) {
      // Finish the run the way a completed one would, so the numbers that come
      // back describe the model that is about to be written rather than
      // whatever the last scheduled pass happened to catch.
      if (window_n > 0) {
        TrainMetrics m;
        m.iteration = completed;
        m.loss = last_loss_value;
        m.l1 = last_l1_value;
        m.psnr = window_psnr / window_n;
        m.gaussians = static_cast<std::size_t>(g.count());
        result.history.push_back(m);
      }
      if (last_eval_iter != completed)
        run_eval(completed);
      // The scheduled render for the end of the run will now never fire, so
      // dump one here instead — but only if the caller asked for checkpoint
      // renders at all.
      if (!render_dir.empty() && !opt.render_iterations.empty() &&
          last_render_iter != completed)
        dump_render(completed);
    }
    // Deliberately NO final checkpoint here. `result.gaussians` below is the
    // complete cancelled model, and `run_gsplat_stage` writes it to `out_ply`
    // — a checkpoint would duplicate that file byte for byte. The checkpoint
    // schedule exists for the case where nobody gets to write anything at all,
    // which a cooperative cancel is not.
  }

  result.seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();
  result.gaussians = to_cloud(g);
  result.final_count = result.gaussians.size();
  result.final_psnr = result.history.empty() ? 0.0 : result.history.back().psnr;
  result.relocated = mcmc_state.relocated;
  result.added = mcmc_state.added;
  for (std::size_t i = 0; i < drawn.size(); ++i)
    if (drawn[i])
      result.trained_views.push_back(i);
  result.holdout_view_indices = split.holdout;
  if (!result.evals.empty()) {
    result.final_holdout_psnr = result.evals.back().holdout_psnr;
    result.final_holdout_ssim = result.evals.back().holdout_ssim;
  }

  // Reports what actually ran, not what was requested: after a cancel those
  // are different numbers, and quoting the request would make a 1 200-iteration
  // model look like a 30 000-iteration one in the log.
  core::info("gsplat: trained {} of {} iterations in {:.1f} s "
             "({:.1f} it/s), final PSNR {:.2f} dB over {} Gaussians",
             result.iterations_run, opt.iterations, result.seconds,
             result.iterations_run / std::max(result.seconds, 1e-9),
             result.final_psnr, result.final_count);
  if (!result.evals.empty() && result.evals.back().holdout_views > 0)
    core::info("gsplat: held-out PSNR {:.2f} dB / SSIM {:.4f} over {} views "
               "the optimizer never saw",
               result.final_holdout_psnr, result.final_holdout_ssim,
               result.evals.back().holdout_views);
  if (mcmc)
    core::info("gsplat: MCMC relocated {} and added {} Gaussians in total",
               result.relocated, result.added);
  return result;
}

cv::Mat render_view(const GaussianCloud &gaussians, const TrainingView &view) {
  require_cuda();
  if (gaussians.empty())
    throw std::runtime_error("gsplat: cannot render an empty Gaussian cloud");
  torch::NoGradGuard no_grad;

  GaussianTensors g = to_tensors(gaussians);
  auto viewmat = torch::from_blob(const_cast<double *>(view.T_cw.data()),
                                  {4, 4}, torch::kFloat64)
                     .t()
                     .contiguous()
                     .to(torch::kFloat32)
                     .to(torch::kCUDA);
  auto K = torch::from_blob(const_cast<double *>(view.K.data()), {3, 3},
                            torch::kFloat64)
               .t()
               .contiguous()
               .to(torch::kFloat32)
               .to(torch::kCUDA);
  auto out =
      detail::render(g, viewmat, K, view.width(), view.height(), g.sh_degree);
  return tensor_to_image(out.image);
}

TrainResult run_gsplat_stage(ProjectDB &db, const GsplatStageOptions &opt) {
  // Refuse before anything expensive rather than after. Training is a
  // minutes-to-hours operation; with no `.ply` destination and no checkpoint
  // renders it produces nothing at all, and the old behaviour was to run the
  // whole thing and log success (STANDARDS §5).
  if (opt.out_ply.empty() && opt.train.render_iterations.empty())
    throw std::runtime_error(
        "gsplat: nothing would be written — pass -o/--out to save the trained "
        "splat as a .ply (and/or --render-dir with --render-at for checkpoint "
        "renders). Refusing to train and discard the result.");

  if (!db.has_point_cloud(opt.seed_cloud))
    throw std::runtime_error(fmt::format(
        "gsplat: project has no point cloud named '{}'. Run `rux create "
        "clouds` first, or pass --seed-cloud with an existing name.",
        opt.seed_cloud));

  // `train_gaussians` never sees `out_ply`, so the "next to the output"
  // default has to be resolved here. Only when checkpointing is actually on:
  // filling the field unconditionally would turn a stray --checkpoint-dir
  // typo into a silently ignored setting.
  TrainOptions train = opt.train;
  if (train.checkpoint_every > 0 && train.checkpoint_dir.empty()) {
    if (opt.out_ply.empty())
      throw std::runtime_error(
          "gsplat: --checkpoint-every was given but there is nowhere to write "
          "the checkpoints — pass --checkpoint-dir, or -o/--out so they can go "
          "beside the output .ply.");
    const auto parent = opt.out_ply.parent_path();
    train.checkpoint_dir = parent.empty() ? std::filesystem::path(".") : parent;
  }

  // Same start/finish contract as the sibling create stages, so `rux log`
  // shows the run, its parameters, and whether it succeeded.
  const int log_id = db.log_pipeline_start(
      "gsplat",
      fmt::format(
          R"({{"seed_cloud":"{}","max_points":{},"sh_degree":{},"iterations":{},"mcmc":{},"holdout_every":{},"checkpoint_every":{},"out":"{}"}})",
          opt.seed_cloud, opt.init.max_points, opt.init.sh_degree,
          train.iterations, train.mcmc.enabled ? "true" : "false",
          train.holdout_every, train.checkpoint_every, opt.out_ply.string()));

  try {
    CloudPtr seed = db.point_cloud_xyzrgb(opt.seed_cloud);
    GaussianCloud init = init_from_point_cloud(seed, opt.init);
    auto views = load_training_views(db, opt.views);

    TrainResult result = train_gaussians(init, views, train);

    // Unconditionally, cancelled or not: salvaging the partly-trained model is
    // the whole reason a cooperative cancel exists rather than a SIGKILL.
    if (!opt.out_ply.empty()) {
      if (opt.out_ply.has_parent_path() && !opt.out_ply.parent_path().empty())
        std::filesystem::create_directories(opt.out_ply.parent_path());
      save_gaussian_ply(result.gaussians, opt.out_ply);
    }

    // A cancelled run closes as `success`, not `failed`. It did what it was
    // asked to do and its artifact is on disk; marking it failed would make
    // `rux log --json | jq 'select(.status=="failed")'` — the way a user finds
    // runs that produced nothing — report a run that produced a usable model.
    // The distinction is not dropped, it moves into the message, which
    // `rux log` prints verbatim and which leads with the literal token
    // CANCELLED so it is greppable.
    db.log_pipeline_end(
        log_id, true,
        result.cancelled
            ? fmt::format("CANCELLED at iteration {} of {} on user request; "
                          "'{}' holds the model as trained so far",
                          result.iterations_run, train.iterations,
                          opt.out_ply.string())
            : std::string{});
    return result;

  } catch (const std::exception &e) {
    // Close the row rather than leaving it "running" forever — a crashed run
    // that still reads as in-progress is indistinguishable from a live one.
    db.log_pipeline_end(log_id, false, e.what());
    throw;
  }
}

} // namespace reusex::gsplat
