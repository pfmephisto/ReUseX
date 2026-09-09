// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mcmc.hpp"

#include <reusex/core/logging.hpp>

#include <MCMCPerturb.h>
#include <Relocation.h>

#include <algorithm>
#include <stdexcept>

namespace reusex::gsplat::detail {

namespace {

torch::Tensor int64_options_arange(int64_t n, const torch::Device &device) {
  return torch::arange(n, torch::TensorOptions(torch::kInt64).device(device));
}

/// Sample @p n rows with replacement, with probability proportional to
/// @p probs. Returns an undefined tensor when the distribution is degenerate
/// (every weight zero), which the callers treat as "nothing to do".
torch::Tensor sample_by_weight(const torch::Tensor &probs, int64_t n) {
  if (n <= 0 || probs.numel() == 0)
    return {};
  if (probs.sum().item<double>() <= 0.0)
    return {};
  return torch::multinomial(probs, n, /*replacement=*/true);
}

/// The paper's Eq. 9, evaluated by gsplat's CUDA kernel.
///
/// For each row in @p sampled (which may contain repeats), returns the
/// activated opacity and scale a Gaussian must take so that `ratio` co-located
/// copies of it are radiometrically equivalent to the single Gaussian it
/// replaces — where `ratio` is how many copies that row is being split into.
struct Relocated {
  torch::Tensor opacity; ///< [M] activated
  torch::Tensor scale;   ///< [M,3] activated (metres)
};

Relocated relocation_for(const GaussianTensors &g, const torch::Tensor &sampled,
                         const MCMCState &state, float min_opacity) {
  auto alpha = torch::sigmoid(g.logit_opacity.detach())
                   .index_select(0, sampled)
                   .contiguous();
  auto scale =
      torch::exp(g.log_scales.detach()).index_select(0, sampled).contiguous();

  // How many times each sampled row was drawn, plus the original.
  auto counts = torch::bincount(sampled);
  auto ratios = (counts.index_select(0, sampled) + 1)
                    // Eq. 9's binomial table is [n_max, n_max]; gsplat's kernel
                    // indexes it with the ratio directly and does not bound
                    // check, so clamp rather than read past the end when a very
                    // dominant Gaussian is drawn more than n_max times.
                    .clamp(1, state.n_max)
                    .to(torch::kInt32)
                    .contiguous();

  Relocated out;
  out.opacity = torch::empty_like(alpha);
  out.scale = torch::empty_like(scale);
  ::gsplat::launch_relocation_kernel(alpha, scale, ratios, state.binoms,
                                     state.n_max, min_opacity, out.opacity,
                                     out.scale);
  return out;
}

/// Write Eq. 9's corrected opacity/scale onto the sampled source rows.
void apply_relocation(GaussianTensors &g, const torch::Tensor &sampled,
                      const Relocated &rel) {
  g.logit_opacity.detach().index_put_({sampled}, torch::logit(rel.opacity));
  g.log_scales.detach().index_put_({sampled}, torch::log(rel.scale));
}

/// Move dead Gaussians onto live ones. Count is unchanged.
int64_t relocate(GaussianTensors &g, AdamPtr &optimizer, const AdamLrs &lrs,
                 const MCMCState &state, const MCMCOptions &opt) {
  torch::NoGradGuard no_grad;
  const int64_t n = g.count();
  const auto device = g.means.device();

  auto alpha = torch::sigmoid(g.logit_opacity.detach());
  auto dead = alpha <= opt.min_opacity;
  const int64_t n_dead = dead.sum().item<int64_t>();
  if (n_dead == 0)
    return 0;
  if (n_dead == n) {
    core::warn("gsplat/mcmc: every one of {} Gaussians is below the minimum "
               "opacity {} — no live Gaussian to relocate onto, skipping",
               n, opt.min_opacity);
    return 0;
  }

  auto dead_idx = torch::nonzero(dead).squeeze(1);
  auto alive_idx = torch::nonzero(torch::logical_not(dead)).squeeze(1);

  auto drawn = sample_by_weight(alpha.index_select(0, alive_idx), n_dead);
  if (!drawn.defined())
    return 0;
  auto sampled = alive_idx.index_select(0, drawn);

  apply_relocation(g, sampled,
                   relocation_for(g, sampled, state, opt.min_opacity));

  // Every dead row now takes a copy of its (already corrected) source row.
  auto src = int64_options_arange(n, device);
  src.index_put_({dead_idx}, sampled);

  auto reset =
      torch::zeros({n}, torch::TensorOptions(torch::kBool).device(device));
  reset.index_put_({dead_idx}, true);
  reset.index_put_({sampled}, true);

  // Deliberate deviation from gsplat's ops.py, which zeroes the Adam moments
  // at `sampled` only and leaves the dead rows' moments in place. A dead row
  // now holds a copy of a different Gaussian, so its inherited momentum
  // describes something that no longer exists there. In practice the two
  // agree closely — a collapsed Gaussian's moments are near zero anyway —
  // but "reset the rows whose contents changed" is the rule that stays
  // correct if the death criterion is ever loosened.

  remap_parameters(g, optimizer, lrs, src, reset);
  return n_dead;
}

/// Grow toward the budget by sampling from the opacity distribution.
int64_t grow(GaussianTensors &g, AdamPtr &optimizer, const AdamLrs &lrs,
             const MCMCState &state, const MCMCOptions &opt, int64_t cap) {
  torch::NoGradGuard no_grad;
  const int64_t n = g.count();
  const auto device = g.means.device();

  const auto target =
      std::min(cap, static_cast<int64_t>(static_cast<double>(n) * opt.growth));
  const int64_t n_add = std::max<int64_t>(0, target - n);
  if (n_add == 0)
    return 0;

  auto alpha = torch::sigmoid(g.logit_opacity.detach());
  auto sampled = sample_by_weight(alpha, n_add);
  if (!sampled.defined()) {
    core::warn("gsplat/mcmc: opacity distribution is degenerate (total {:.3e}) "
               "— cannot sample {} new Gaussians, skipping growth",
               alpha.sum().item<double>(), n_add);
    return 0;
  }

  apply_relocation(g, sampled,
                   relocation_for(g, sampled, state, opt.min_opacity));

  // Old rows keep their place; the new rows are copies of the sampled sources.
  auto src = torch::cat({int64_options_arange(n, device), sampled});
  auto reset = torch::zeros({n + n_add},
                            torch::TensorOptions(torch::kBool).device(device));
  reset.index_put_({sampled}, true);        // sources were rescaled
  reset.slice(0, n, n + n_add).fill_(true); // new rows have no history

  remap_parameters(g, optimizer, lrs, src, reset);
  return n_add;
}

} // namespace

MCMCState make_mcmc_state(const torch::Device &device, int n_max) {
  if (n_max < 2)
    throw std::runtime_error("gsplat/mcmc: n_max must be >= 2");

  MCMCState state;
  state.n_max = n_max;

  auto binoms = torch::zeros({n_max, n_max}, torch::kFloat32);
  auto a = binoms.accessor<float, 2>();
  for (int n = 0; n < n_max; ++n) {
    double c = 1.0; // C(n,0)
    for (int k = 0; k <= n; ++k) {
      a[n][k] = static_cast<float>(c);
      c = c * static_cast<double>(n - k) / static_cast<double>(k + 1);
    }
  }
  state.binoms = binoms.to(device).contiguous();
  return state;
}

RefineCounts mcmc_refine(GaussianTensors &g, AdamPtr &optimizer,
                         const AdamLrs &lrs, MCMCState &state,
                         const MCMCOptions &opt, int64_t cap) {
  RefineCounts counts;
  counts.relocated = relocate(g, optimizer, lrs, state, opt);
  counts.added = grow(g, optimizer, lrs, state, opt, cap);
  state.relocated += static_cast<std::size_t>(counts.relocated);
  state.added += static_cast<std::size_t>(counts.added);
  return counts;
}

void mcmc_inject_noise(GaussianTensors &g, const MCMCOptions &opt,
                       double lr_means) {
  if (opt.noise_lr <= 0.0f)
    return;
  torch::NoGradGuard no_grad;

  // The kernel mutates `positions` in place, so it must be the parameter's own
  // storage — `.contiguous()` on a non-contiguous tensor would hand it a copy
  // and silently discard the perturbation.
  auto means = g.means.detach();
  if (!means.is_contiguous())
    throw std::runtime_error(
        "gsplat/mcmc: means must be contiguous for in-place perturbation");

  auto noise = torch::randn_like(means);
  ::gsplat::launch_mcmc_perturb_positions_kernel(
      means, g.quats.detach().contiguous(), g.log_scales.detach().contiguous(),
      g.logit_opacity.detach().contiguous(), noise,
      static_cast<float>(lr_means * static_cast<double>(opt.noise_lr)),
      opt.noise_opacity_t, opt.noise_opacity_k);
}

torch::Tensor mcmc_regularizer(const GaussianTensors &g,
                               const MCMCOptions &opt) {
  torch::Tensor reg;
  if (opt.opacity_reg > 0.0f)
    reg = opt.opacity_reg * torch::sigmoid(g.logit_opacity).mean();
  if (opt.scale_reg > 0.0f) {
    auto s = opt.scale_reg * torch::exp(g.log_scales).mean();
    reg = reg.defined() ? reg + s : s;
  }
  return reg;
}

} // namespace reusex::gsplat::detail
