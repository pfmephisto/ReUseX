// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/gsplat/train.hpp>

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

/// Adam parameter group with its own learning rate. eps follows the reference
/// 3DGS value (1e-15), which matters because SH gradients are tiny.
torch::optim::OptimizerParamGroup param_group(torch::Tensor p, double lr) {
  auto opts = std::make_unique<torch::optim::AdamOptions>(lr);
  opts->eps(1e-15);
  return torch::optim::OptimizerParamGroup({std::move(p)}, std::move(opts));
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
  core::info("gsplat: {} views, scene extent {:.2f} m, {} seed Gaussians, "
             "SH degree {}",
             views.size(), extent, init.size(), init.sh_degree);

  auto make_optimizer = [&](GaussianTensors &t) {
    std::vector<torch::optim::OptimizerParamGroup> groups;
    groups.push_back(param_group(t.means, opt.lr_means * extent));
    groups.push_back(param_group(t.log_scales, opt.lr_scales));
    groups.push_back(param_group(t.quats, opt.lr_quats));
    groups.push_back(param_group(t.logit_opacity, opt.lr_opacities));
    groups.push_back(param_group(t.sh, opt.lr_sh_dc));
    return std::make_unique<torch::optim::Adam>(
        groups, torch::optim::AdamOptions(1e-3));
  };
  auto optimizer = make_optimizer(g);

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

  std::mt19937 rng(opt.seed);
  std::uniform_int_distribution<std::size_t> pick(0, views.size() - 1);

  const auto t0 = std::chrono::steady_clock::now();
  double window_psnr = 0.0;
  int window_n = 0;

  for (int it = 1; it <= opt.iterations; ++it) {
    const std::size_t vi = pick(rng);
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

    // ---- density control: opacity/size pruning only ------------------------
    // The reference clone/split heuristic needs the screen-space position
    // gradient, which gsplat's world-space rasterizer does not expose. Seeding
    // from a LiDAR cloud supplies the density that heuristic exists to grow, so
    // this version prunes but does not densify (see train.hpp).
    if (opt.prune_enabled && it >= opt.prune_start &&
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
        GaussianTensors ng;
        ng.sh_degree = g.sh_degree;
        ng.means = g.means.detach().index_select(0, idx).clone();
        ng.log_scales = g.log_scales.detach().index_select(0, idx).clone();
        ng.quats = g.quats.detach().index_select(0, idx).clone();
        ng.logit_opacity =
            g.logit_opacity.detach().index_select(0, idx).clone();
        ng.sh = g.sh.detach().index_select(0, idx).clone();
        ng.means.set_requires_grad(true);
        ng.log_scales.set_requires_grad(true);
        ng.quats.set_requires_grad(true);
        ng.logit_opacity.set_requires_grad(true);
        ng.sh.set_requires_grad(true);
        g = std::move(ng);
        // Adam's per-parameter moments are indexed by tensor identity, so a
        // resize means a fresh optimizer. Restarting the moments costs a few
        // iterations of momentum; carrying them would require re-indexing
        // torch's optimizer state by hand.
        optimizer = make_optimizer(g);
        core::info("gsplat: pruned {} -> {} Gaussians at iter {} "
                   "(alpha <= {} or scale >= {} m)",
                   before, kept, it, opt.prune_opacity, opt.prune_max_scale);
      }
    }

    if (wants_render(it))
      dump_render(it);
  }

  result.seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();
  result.gaussians = to_cloud(g);
  result.final_count = result.gaussians.size();
  result.final_psnr = result.history.empty() ? 0.0 : result.history.back().psnr;

  core::info("gsplat: trained {} iterations in {:.1f} s "
             "({:.1f} it/s), final PSNR {:.2f} dB over {} Gaussians",
             opt.iterations, result.seconds,
             opt.iterations / std::max(result.seconds, 1e-9), result.final_psnr,
             result.final_count);
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
