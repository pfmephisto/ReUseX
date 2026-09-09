// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Module-private: 3DGS-MCMC density control.
//
// WHY MCMC AND NOT CLONE/SPLIT
// ----------------------------
// The reference 3DGS densifier clones or splits a Gaussian when the *screen
// space* position gradient exceeds a threshold. gsplat's 3DGUT world-space
// rasterizer — the only differentiable path reachable from C++ in this build,
// see rasterize.hpp — never forms that gradient, so the reference heuristic is
// not merely inconvenient here, it has no input.
//
// "3D Gaussian Splatting as Markov Chain Monte Carlo" (Kheradmand et al. 2024)
// reframes training as SGLD sampling over Gaussian configurations, and its
// density control needs no screen-space quantity at all:
//
//   * relocate — Gaussians whose opacity has collapsed are dead samples. Move
//     each onto a live Gaussian chosen with probability proportional to
//     opacity, and rescale the pair so the pile of n co-located Gaussians is
//     radiometrically equivalent to the one it replaced (the paper's Eq. 9).
//   * grow — sample additional Gaussians from the same opacity distribution
//     until a capped budget is reached. This is the part that resolves detail
//     finer than the seed cloud.
//   * perturb — inject covariance-shaped noise into the positions each step,
//     gated so it only moves low-opacity Gaussians. This is the Langevin term;
//     without it the relocation is a greedy heuristic rather than a sampler.
//   * regularize — L1 on activated opacity and scale, which is what makes
//     Gaussians die (and so become available for relocation) instead of
//     lingering at low opacity forever.
//
// gsplat ships Eq. 9 and the perturbation as CUDA kernels with plain external
// linkage (`Relocation.h`, `MCMCPerturb.h`) — no dispatcher, no schema from
// ext.cpp — so both are callable directly. The sampling, budgeting and Adam
// bookkeeping around them are implemented here, mirroring
// gsplat/strategy/mcmc.py + ops.py.
#pragma once

#include "optimizer.hpp"
#include "rasterize.hpp"

#include <reusex/gsplat/train.hpp>

namespace reusex::gsplat::detail {

/// Running state for the MCMC strategy.
struct MCMCState {
  /// Pascal's triangle, [n_max, n_max] f32 on the GPU. Eq. 9's scale
  /// correction sums binomial terms; gsplat's kernel reads them from this
  /// table rather than recomputing factorials per thread.
  torch::Tensor binoms;
  int n_max = 51; ///< matches gsplat's MCMCStrategy.initialize_state()

  std::size_t relocated = 0; ///< cumulative, for the end-of-run log
  std::size_t added = 0;
};

/// Build the binomial table on @p device.
MCMCState make_mcmc_state(const torch::Device &device, int n_max = 51);

/// One refine pass: relocate dead Gaussians, then grow toward the budget.
///
/// Rewrites @p g and @p optimizer. Returns {relocated, added} for logging.
/// Both steps zero the Adam moments of every row they touch — an inherited
/// moment would describe the Gaussian that used to live there.
struct RefineCounts {
  int64_t relocated = 0;
  int64_t added = 0;
};
RefineCounts mcmc_refine(GaussianTensors &g, AdamPtr &optimizer,
                         const AdamLrs &lrs, MCMCState &state,
                         const MCMCOptions &opt, int64_t cap);

/// Langevin noise injection, in place on `g.means`. @p lr_means is the current
/// means learning rate; the paper couples the noise magnitude to it.
void mcmc_inject_noise(GaussianTensors &g, const MCMCOptions &opt,
                       double lr_means);

/// L1 opacity + scale regularizers, in activated space. Returns an undefined
/// tensor when both weights are zero, so the caller can skip the add.
torch::Tensor mcmc_regularizer(const GaussianTensors &g,
                               const MCMCOptions &opt);

} // namespace reusex::gsplat::detail
