// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/gsplat/train.hpp>

#include "mcmc.hpp"
#include "optimizer.hpp"
#include "rasterize.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace reusex::gsplat {

namespace {

using detail::GaussianTensors;
using detail::sh_bands;

void require_cuda() {
  if (!torch::cuda::is_available())
    throw std::runtime_error(
        "gsplat: no CUDA device available. The Gaussian-splatting trainer is "
        "GPU-only (the rasterizer has no CPU path).");
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

// ---------------------------------------------------------------------------
// Losses
// ---------------------------------------------------------------------------

torch::Tensor gaussian_kernel1d(int window, double sigma,
                                const torch::TensorOptions &o) {
  auto x = torch::arange(window, o) - (window - 1) / 2.0;
  auto g = torch::exp(-x.pow(2) / (2.0 * sigma * sigma));
  return g / g.sum();
}

/// Structural similarity over a [1,3,H,W] pair, the standard 11x11 sigma=1.5
/// Gaussian-windowed formulation used by every 3DGS implementation.
torch::Tensor ssim(const torch::Tensor &a, const torch::Tensor &b) {
  constexpr int window = 11;
  constexpr double sigma = 1.5;
  const int64_t channels = a.size(1);

  auto k1 = gaussian_kernel1d(window, sigma, a.options());
  auto k2 = k1.unsqueeze(1).mm(k1.unsqueeze(0)); // [w,w]
  auto kernel = k2.expand({channels, 1, window, window}).contiguous();

  auto conv = [&](const torch::Tensor &t) {
    return torch::conv2d(t, kernel, /*bias=*/{}, /*stride=*/1,
                         /*padding=*/window / 2, /*dilation=*/1,
                         /*groups=*/channels);
  };

  auto mu_a = conv(a);
  auto mu_b = conv(b);
  auto mu_a2 = mu_a * mu_a;
  auto mu_b2 = mu_b * mu_b;
  auto mu_ab = mu_a * mu_b;

  auto sigma_a2 = conv(a * a) - mu_a2;
  auto sigma_b2 = conv(b * b) - mu_b2;
  auto sigma_ab = conv(a * b) - mu_ab;

  constexpr double c1 = 0.01 * 0.01;
  constexpr double c2 = 0.03 * 0.03;
  auto num = (2 * mu_ab + c1) * (2 * sigma_ab + c2);
  auto den = (mu_a2 + mu_b2 + c1) * (sigma_a2 + sigma_b2 + c2);
  return (num / den).mean();
}

/// Radius of the camera cloud — the reference scales the position learning
/// rate by it, because a step in metres only means something relative to how
/// big the scene is.
double scene_extent(const std::vector<TrainingView> &views) {
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(views.size());
  for (const auto &v : views) {
    // camera centre in world = -R^T t  for T_cw = [R|t]
    const Eigen::Matrix3d R = v.T_cw.block<3, 3>(0, 0);
    const Eigen::Vector3d t = v.T_cw.block<3, 1>(0, 3);
    centers.push_back(-R.transpose() * t);
    centroid += centers.back();
  }
  centroid /= static_cast<double>(centers.size());
  double radius = 0.0;
  for (const auto &c : centers)
    radius = std::max(radius, (c - centroid).norm());
  // A single-viewpoint capture would otherwise scale the LR to zero.
  return std::max(radius, 1e-3);
}

// ---------------------------------------------------------------------------
// Train / held-out split
// ---------------------------------------------------------------------------

/// The two view index sets. Deterministic by construction: membership depends
/// only on a view's position in the list, never on the RNG, so two runs — or a
/// prune-only run and an MCMC run — evaluate on exactly the same images
/// (STANDARDS §6).
struct ViewSplit {
  std::vector<std::size_t> train;
  std::vector<std::size_t> holdout;
};

ViewSplit split_views(std::size_t n_views, int holdout_every) {
  ViewSplit s;
  for (std::size_t i = 0; i < n_views; ++i) {
    const bool held =
        holdout_every > 0 && i % static_cast<std::size_t>(holdout_every) == 0;
    (held ? s.holdout : s.train).push_back(i);
  }

  // A split that leaves nothing to train on is worse than no split. This only
  // triggers for tiny view sets (holdout_every == 1, or a single view).
  if (s.train.empty()) {
    core::warn("gsplat: a held-out split of every {}th view would leave {} "
               "training views out of {} — training on all views instead, and "
               "reporting no held-out metric",
               holdout_every, s.train.size(), n_views);
    s.train.clear();
    s.holdout.clear();
    for (std::size_t i = 0; i < n_views; ++i)
      s.train.push_back(i);
  }
  return s;
}

/// Sub-sample @p src to at most @p max_n entries by a uniform stride, keeping
/// the first entry. Deterministic, and it spreads the sample over the whole
/// trajectory rather than taking a contiguous prefix.
std::vector<std::size_t> stride_sample(const std::vector<std::size_t> &src,
                                       int max_n) {
  if (max_n <= 0 || src.size() <= static_cast<std::size_t>(max_n))
    return src;
  const std::size_t step = src.size() / static_cast<std::size_t>(max_n);
  std::vector<std::size_t> out;
  out.reserve(static_cast<std::size_t>(max_n));
  for (std::size_t i = 0; out.size() < static_cast<std::size_t>(max_n);
       i += step)
    out.push_back(src[i]);
  return out;
}

} // namespace

bool is_available() { return true; }

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

  const double extent = scene_extent(views);
  const ViewSplit split = split_views(views.size(), opt.holdout_every);

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

  std::filesystem::path render_dir = opt.render_dir;
  const std::size_t render_idx =
      std::min(opt.render_view_index, views.size() - 1);
  if (!render_dir.empty())
    std::filesystem::create_directories(render_dir);

  TrainResult result;

  auto dump_render = [&](int iteration) {
    if (render_dir.empty())
      return;
    torch::NoGradGuard no_grad;
    const auto &v = views[render_idx];
    auto out = detail::render(g, viewmats[render_idx], intrinsics[render_idx],
                              v.width(), v.height(), g.sh_degree);
    const auto path =
        render_dir / fmt::format("render_{}_iter{:06d}.png", v.name, iteration);
    if (!cv::imwrite(path.string(), tensor_to_image(out.image)))
      throw std::runtime_error(
          fmt::format("gsplat: failed to write render '{}'", path.string()));
    result.renders.push_back(path);
    core::info("gsplat: wrote checkpoint render {}", path.string());
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
  const auto eval_train_views = stride_sample(split.train, opt.eval_max_views);
  const auto eval_holdout_views =
      stride_sample(split.holdout, opt.eval_max_views);

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
      ssim_sum += ssim(pred.permute({2, 0, 1}).unsqueeze(0),
                       target.permute({2, 0, 1}).unsqueeze(0))
                      .item<double>();
    }
    const auto n = static_cast<double>(std::max<std::size_t>(idx.size(), 1));
    return std::pair<double, double>{psnr_sum / n, ssim_sum / n};
  };

  auto run_eval = [&](int it) {
    if (eval_train_views.empty())
      return;
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

  for (int it = 1; it <= opt.iterations; ++it) {
    const std::size_t vi = split.train[pick(rng)];
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
      auto s = ssim(p, t);
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

    if (it % opt.log_interval == 0 || it == opt.iterations) {
      TrainMetrics m;
      m.iteration = it;
      m.loss = loss.item<double>();
      m.l1 = l1.item<double>();
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
  }

  result.seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();
  result.gaussians = to_cloud(g);
  result.final_count = result.gaussians.size();
  result.final_psnr = result.history.empty() ? 0.0 : result.history.back().psnr;
  result.relocated = mcmc_state.relocated;
  result.added = mcmc_state.added;
  if (!result.evals.empty()) {
    result.final_holdout_psnr = result.evals.back().holdout_psnr;
    result.final_holdout_ssim = result.evals.back().holdout_ssim;
  }

  core::info("gsplat: trained {} iterations in {:.1f} s "
             "({:.1f} it/s), final PSNR {:.2f} dB over {} Gaussians",
             opt.iterations, result.seconds,
             opt.iterations / std::max(result.seconds, 1e-9), result.final_psnr,
             result.final_count);
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

TrainResult run_gsplat_stage(const ProjectDB &db,
                             const GsplatStageOptions &opt) {
  if (!db.has_point_cloud(opt.seed_cloud))
    throw std::runtime_error(fmt::format(
        "gsplat: project has no point cloud named '{}'. Run `rux create "
        "clouds` first, or pass --seed-cloud with an existing name.",
        opt.seed_cloud));

  CloudPtr seed = db.point_cloud_xyzrgb(opt.seed_cloud);
  GaussianCloud init = init_from_point_cloud(seed, opt.init);
  auto views = load_training_views(db, opt.views);

  TrainResult result = train_gaussians(init, views, opt.train);

  if (!opt.out_ply.empty()) {
    if (opt.out_ply.has_parent_path() && !opt.out_ply.parent_path().empty())
      std::filesystem::create_directories(opt.out_ply.parent_path());
    save_gaussian_ply(result.gaussians, opt.out_ply);
  }
  return result;
}

} // namespace reusex::gsplat
