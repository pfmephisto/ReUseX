// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "optimizer.hpp"

#include <array>
#include <vector>

namespace reusex::gsplat::detail {

namespace {

/// How many leaf tensors the trainer optimizes. Named because three separate
/// fixed-size arrays have to agree with `parameters()` or moments get
/// reattached to the wrong parameter.
constexpr std::size_t kNumParams = 6;

/// Adam parameter group with its own learning rate. eps follows the reference
/// 3DGS value (1e-15), which matters because SH gradients are tiny.
torch::optim::OptimizerParamGroup param_group(torch::Tensor p, double lr) {
  auto opts = std::make_unique<torch::optim::AdamOptions>(lr);
  opts->eps(1e-15);
  return torch::optim::OptimizerParamGroup({std::move(p)}, std::move(opts));
}

/// The six optimizable tensors, in a fixed order shared by make_adam and
/// remap_parameters so moments cannot be reattached to the wrong parameter.
///
/// The SH split (sh_dc / sh_rest) is what makes the two SH learning rates
/// expressible at all — see the note on GaussianTensors in rasterize.hpp.
std::array<torch::Tensor *, kNumParams> parameters(GaussianTensors &g) {
  return {&g.means,         &g.log_scales, &g.quats,
          &g.logit_opacity, &g.sh_dc,      &g.sh_rest};
}

std::array<double, kNumParams> learning_rates(const AdamLrs &lrs) {
  return {lrs.means,         lrs.log_scales, lrs.quats,
          lrs.logit_opacity, lrs.sh_dc,      lrs.sh_rest};
}

/// Broadcast a [M] row mask over a [M, ...] moment buffer.
torch::Tensor row_mask_like(const torch::Tensor &keep,
                            const torch::Tensor &moment) {
  std::vector<int64_t> shape(static_cast<std::size_t>(moment.dim()), 1);
  shape[0] = keep.size(0);
  return keep.to(moment.dtype()).view(shape);
}

/// One Adam over six already-materialised tensors, in the fixed order of
/// `parameters()`. Split out of make_adam so remap_parameters can build the
/// replacement optimizer *before* it commits the new tensors into `g` — see
/// the atomicity note there.
AdamPtr make_adam_from(const std::array<torch::Tensor, kNumParams> &params,
                       const AdamLrs &lrs) {
  const auto lr = learning_rates(lrs);

  std::vector<torch::optim::OptimizerParamGroup> groups;
  groups.reserve(params.size());
  for (std::size_t i = 0; i < params.size(); ++i)
    groups.push_back(param_group(params[i], lr[i]));

  return std::make_unique<torch::optim::Adam>(groups,
                                              torch::optim::AdamOptions(1e-3));
}

} // namespace

AdamPtr make_adam(GaussianTensors &g, const AdamLrs &lrs) {
  const auto p = parameters(g);
  return make_adam_from({*p[0], *p[1], *p[2], *p[3], *p[4], *p[5]}, lrs);
}

void remap_parameters(GaussianTensors &g, AdamPtr &optimizer,
                      const AdamLrs &lrs, const torch::Tensor &src_rows,
                      const torch::Tensor &reset) {
  torch::NoGradGuard no_grad;

  const auto old_params = parameters(g);

  // Snapshot the moments *before* the parameter tensors are replaced: the
  // state map is keyed by the old tensors' TensorImpl pointers.
  struct Moments {
    bool present = false;
    int64_t step = 0;
    torch::Tensor exp_avg;
    torch::Tensor exp_avg_sq;
  };
  std::array<Moments, kNumParams> saved;
  for (std::size_t i = 0; i < old_params.size(); ++i) {
    auto &state = optimizer->state();
    const auto it =
        state.find(static_cast<void *>(old_params[i]->unsafeGetTensorImpl()));
    if (it == state.end())
      continue; // no step taken yet — nothing to carry
    const auto *s =
        static_cast<const torch::optim::AdamParamState *>(it->second.get());
    saved[i].present = true;
    saved[i].step = s->step();
    // index_select gathers into fresh storage, so these already own their
    // data — an extra .clone() would only duplicate it (at 2.4 M Gaussians
    // the six parameters plus twelve moment buffers cost ~408 MB per pass).
    saved[i].exp_avg = s->exp_avg().index_select(0, src_rows);
    saved[i].exp_avg_sq = s->exp_avg_sq().index_select(0, src_rows);
    if (reset.defined()) {
      // `reset` marks rows holding a different Gaussian than before.
      const auto keep = torch::logical_not(reset);
      saved[i].exp_avg =
          saved[i].exp_avg * row_mask_like(keep, saved[i].exp_avg);
      saved[i].exp_avg_sq =
          saved[i].exp_avg_sq * row_mask_like(keep, saved[i].exp_avg_sq);
    }
  }

  // Allocate every replacement tensor into locals FIRST, and only commit once
  // all six exist. Reallocating them one at a time through the pointers into
  // `g` is not exception-safe: an OOM at allocation 5 of 6 would leave `g`
  // holding a mix of M-row and N-row tensors with the old Adam still keyed on
  // the old storage. gsplat derives the Gaussian count from `means.size(0)`,
  // so a caller that caught that OOM and carried on — say by disabling MCMC
  // and continuing to train — would rasterize out of bounds on the very next
  // iteration. Failing with `g` untouched is recoverable; failing halfway is
  // not.
  std::array<torch::Tensor, kNumParams> next;
  for (std::size_t i = 0; i < old_params.size(); ++i) {
    // Under NoGradGuard this is a leaf, so it can take requires_grad directly.
    next[i] = old_params[i]->detach().index_select(0, src_rows);
    next[i].set_requires_grad(true);
  }

  // Built before the commit for the same reason: make_adam_from only allocates
  // host-side, but if it did throw, `g` would already be the new model with an
  // optimizer still pointing at the old one.
  auto next_optimizer = make_adam_from(next, lrs);

  // ---- commit: nothing below this line allocates device memory -------------
  for (std::size_t i = 0; i < old_params.size(); ++i)
    *old_params[i] = next[i];
  optimizer = std::move(next_optimizer);

  const auto new_params = parameters(g);
  for (std::size_t i = 0; i < new_params.size(); ++i) {
    if (!saved[i].present)
      continue;
    auto s = std::make_unique<torch::optim::AdamParamState>();
    s->step(saved[i].step);
    s->exp_avg(saved[i].exp_avg);
    s->exp_avg_sq(saved[i].exp_avg_sq);
    optimizer
        ->state()[static_cast<void *>(new_params[i]->unsafeGetTensorImpl())] =
        std::move(s);
  }
}

} // namespace reusex::gsplat::detail
