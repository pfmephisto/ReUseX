// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Module-private: Adam over a GaussianTensors set, and the row remap that
// density control needs.
//
// Every density-control operation — pruning, MCMC relocation, MCMC growth —
// rewrites the parameter tensors row-wise, and torch's Adam keys its moment
// buffers by tensor identity. Replacing a parameter therefore orphans its
// moments unless they are carried across the same remap, which is what
// `remap_parameters` exists to do. Dropping them instead (the first version of
// the trainer rebuilt the optimizer from scratch on every prune) costs a few
// hundred iterations of momentum each time; at MCMC's refine cadence of once
// per 100 iterations that would be most of the run.
#pragma once

#include "rasterize.hpp"

#include <memory>

namespace reusex::gsplat::detail {

/// Per-parameter learning rates, already scaled where scaling applies.
struct AdamLrs {
  double means = 1.6e-4; ///< caller multiplies by the scene extent
  double log_scales = 5e-3;
  double quats = 1e-3;
  double logit_opacity = 5e-2;
  double sh = 2.5e-3;
};

using AdamPtr = std::unique_ptr<torch::optim::Adam>;

/// One Adam over @p g's five parameter tensors, one param group each so the
/// learning rates stay independent.
AdamPtr make_adam(GaussianTensors &g, const AdamLrs &lrs);

/// Rebuild @p g and @p optimizer so that new row `i` takes its values from old
/// row `src_rows[i]`.
///
/// @param src_rows [M] int64 CUDA tensor indexing the current rows. May repeat
///        (growth), omit (pruning) or reorder them.
/// @param reset    [M] bool CUDA tensor, or an undefined tensor for "reset
///        nothing". Rows marked true get zeroed Adam moments — used where a row
///        now holds a *different* Gaussian, so the inherited momentum describes
///        something that no longer exists there.
///
/// Adam's step counter is preserved, so the bias correction does not restart.
void remap_parameters(GaussianTensors &g, AdamPtr &optimizer,
                      const AdamLrs &lrs, const torch::Tensor &src_rows,
                      const torch::Tensor &reset);

} // namespace reusex::gsplat::detail
