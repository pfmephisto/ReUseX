// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "optimizer.hpp"

#include <array>
#include <vector>

namespace reusex::gsplat::detail {

namespace {

/// Adam parameter group with its own learning rate. eps follows the reference
/// 3DGS value (1e-15), which matters because SH gradients are tiny.
torch::optim::OptimizerParamGroup param_group(torch::Tensor p, double lr) {
  auto opts = std::make_unique<torch::optim::AdamOptions>(lr);
  opts->eps(1e-15);
  return torch::optim::OptimizerParamGroup({std::move(p)}, std::move(opts));
}

/// The five optimizable tensors, in a fixed order shared by make_adam and
/// remap_parameters so moments cannot be reattached to the wrong parameter.
std::array<torch::Tensor *, 5> parameters(GaussianTensors &g) {
  return {&g.means, &g.log_scales, &g.quats, &g.logit_opacity, &g.sh};
}

std::array<double, 5> learning_rates(const AdamLrs &lrs) {
  return {lrs.means, lrs.log_scales, lrs.quats, lrs.logit_opacity, lrs.sh};
}

/// Broadcast a [M] row mask over a [M, ...] moment buffer.
torch::Tensor row_mask_like(const torch::Tensor &keep,
                            const torch::Tensor &moment) {
  std::vector<int64_t> shape(static_cast<std::size_t>(moment.dim()), 1);
  shape[0] = keep.size(0);
  return keep.to(moment.dtype()).view(shape);
}

} // namespace

AdamPtr make_adam(GaussianTensors &g, const AdamLrs &lrs) {
  const auto params = parameters(g);
  const auto lr = learning_rates(lrs);

  std::vector<torch::optim::OptimizerParamGroup> groups;
  groups.reserve(params.size());
  for (std::size_t i = 0; i < params.size(); ++i)
    groups.push_back(param_group(*params[i], lr[i]));

  return std::make_unique<torch::optim::Adam>(groups,
                                              torch::optim::AdamOptions(1e-3));
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
  std::array<Moments, 5> saved;
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
    saved[i].exp_avg = s->exp_avg().index_select(0, src_rows).clone();
    saved[i].exp_avg_sq = s->exp_avg_sq().index_select(0, src_rows).clone();
    if (reset.defined()) {
      // `reset` marks rows holding a different Gaussian than before.
      const auto keep = torch::logical_not(reset);
      saved[i].exp_avg =
          saved[i].exp_avg * row_mask_like(keep, saved[i].exp_avg);
      saved[i].exp_avg_sq =
          saved[i].exp_avg_sq * row_mask_like(keep, saved[i].exp_avg_sq);
    }
  }

  for (auto *p : old_params) {
    auto next = p->detach().index_select(0, src_rows).clone();
    next.set_requires_grad(true);
    *p = std::move(next);
  }

  optimizer = make_adam(g, lrs);

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
