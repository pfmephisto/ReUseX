// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Direct tests for the density-control row remap (src/gsplat/optimizer.cpp).
//
// This is the one piece of the trainer that every density-control path funnels
// through — pruning, MCMC relocation and MCMC growth all rewrite the parameter
// tensors by calling remap_parameters — and it is also the piece whose failure
// mode is silent: torch's Adam keys its moment buffers by tensor identity, so
// replacing a parameter without carrying its moments does not error, it just
// quietly restarts momentum on every pass. Testing it through train_gaussians
// can only observe that pruning happened, not that the moments came with it.
//
// Deliberately NOT tagged [gpu]: remap_parameters is pure torch (index_select
// plus an Adam rebuild) with no rasterizer call in it, so it runs on CPU
// tensors and is testable on a machine with no CUDA device. The file still
// only compiles when the gsplat module is enabled, because it reaches into the
// module-private header.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "gsplat/optimizer.hpp"

#include <cstdint>

using namespace reusex::gsplat::detail;
using Catch::Approx;

namespace {

/// Five CPU parameter tensors with distinct, position-dependent values, so a
/// row that ends up in the wrong place is visible rather than plausible.
GaussianTensors make_tensors(int64_t n) {
  const auto f32 = torch::TensorOptions(torch::kFloat32);
  auto ramp = [&](std::vector<int64_t> shape) {
    int64_t total = 1;
    for (const int64_t s : shape)
      total *= s;
    return torch::arange(total, f32).view(shape).clone();
  };

  GaussianTensors g;
  g.sh_degree = 0;
  g.means = ramp({n, 3});
  g.log_scales = ramp({n, 3});
  g.quats = ramp({n, 4});
  g.logit_opacity = ramp({n});
  g.sh = ramp({n, 1, 3});
  for (torch::Tensor *p :
       {&g.means, &g.log_scales, &g.quats, &g.logit_opacity, &g.sh})
    p->set_requires_grad(true);
  return g;
}

/// Give every parameter a gradient and take one Adam step, so the moment
/// buffers exist and are non-zero. Without this there is nothing to carry and
/// the remap's interesting branch never runs.
void one_step(GaussianTensors &g, AdamPtr &opt) {
  for (torch::Tensor *p :
       {&g.means, &g.log_scales, &g.quats, &g.logit_opacity, &g.sh})
    p->mutable_grad() = torch::ones_like(*p) * 0.5f;
  opt->step();
}

/// The Adam state torch stored for @p p, or nullptr if it has none.
const torch::optim::AdamParamState *adam_state(const AdamPtr &opt,
                                               const torch::Tensor &p) {
  auto &state = const_cast<AdamPtr &>(opt)->state();
  const auto it = state.find(static_cast<void *>(p.unsafeGetTensorImpl()));
  if (it == state.end())
    return nullptr;
  return static_cast<const torch::optim::AdamParamState *>(it->second.get());
}

bool all_close(const torch::Tensor &a, const torch::Tensor &b) {
  return a.sizes() == b.sizes() && torch::allclose(a, b);
}

} // namespace

TEST_CASE("RemapParameters_RowSubsetKeep_CarriesAdamMoments", "[gsplat]") {
  auto g = make_tensors(8);
  AdamLrs lrs;
  auto opt = make_adam(g, lrs);
  one_step(g, opt);

  // Snapshot what must survive.
  const auto *before = adam_state(opt, g.means);
  REQUIRE(before != nullptr);
  const auto step_before = before->step();
  REQUIRE(step_before == 1);
  const auto exp_avg_before = before->exp_avg().clone();
  const auto exp_avg_sq_before = before->exp_avg_sq().clone();
  REQUIRE(exp_avg_before.abs().sum().item<double>() > 0.0);
  const auto means_before = g.means.detach().clone();

  const auto keep = torch::tensor({0, 2, 4, 6}, torch::kInt64);
  remap_parameters(g, opt, lrs, keep, /*reset=*/{});

  SECTION("every parameter is remapped to the same row count") {
    REQUIRE(g.count() == 4);
    REQUIRE(g.means.size(0) == 4);
    REQUIRE(g.log_scales.size(0) == 4);
    REQUIRE(g.quats.size(0) == 4);
    REQUIRE(g.logit_opacity.size(0) == 4);
    REQUIRE(g.sh.size(0) == 4);
    // Shapes past dim 0 are untouched.
    REQUIRE(g.quats.size(1) == 4);
    REQUIRE(g.sh.size(1) == 1);
  }

  SECTION("values come from the selected source rows") {
    REQUIRE(all_close(g.means.detach(), means_before.index_select(0, keep)));
  }

  SECTION("the moments are carried, not restarted") {
    const auto *after = adam_state(opt, g.means);
    REQUIRE(after != nullptr);
    // The whole point: a rebuilt-from-scratch optimizer would have step 0 and
    // zero moments here.
    REQUIRE(after->step() == step_before);
    REQUIRE(all_close(after->exp_avg(), exp_avg_before.index_select(0, keep)));
    REQUIRE(all_close(after->exp_avg_sq(),
                      exp_avg_sq_before.index_select(0, keep)));
    REQUIRE(after->exp_avg().abs().sum().item<double>() > 0.0);
  }

  SECTION("every parameter keeps its own moments, not another's") {
    for (const torch::Tensor *p :
         {&g.means, &g.log_scales, &g.quats, &g.logit_opacity, &g.sh}) {
      const auto *s = adam_state(opt, *p);
      REQUIRE(s != nullptr);
      REQUIRE(s->exp_avg().sizes() == p->sizes());
      REQUIRE(s->step() == step_before);
    }
  }

  SECTION("the remapped parameters still train") {
    REQUIRE_NOTHROW(one_step(g, opt));
    REQUIRE(adam_state(opt, g.means)->step() == step_before + 1);
    REQUIRE(torch::isfinite(g.means).all().item<bool>());
  }
}

TEST_CASE("RemapParameters_ResetRows_ZeroesAdamMoments", "[gsplat]") {
  auto g = make_tensors(6);
  AdamLrs lrs;
  auto opt = make_adam(g, lrs);
  one_step(g, opt);

  // Identity remap, but rows 1 and 3 now hold a different Gaussian.
  const auto src = torch::arange(6, torch::kInt64);
  auto reset = torch::zeros({6}, torch::kBool);
  reset.index_put_({torch::tensor({1, 3}, torch::kInt64)}, true);

  remap_parameters(g, opt, lrs, src, reset);

  const auto *after = adam_state(opt, g.means);
  REQUIRE(after != nullptr);
  const auto row_sums = after->exp_avg().abs().sum(1);
  REQUIRE(row_sums[0].item<double>() > 0.0);
  REQUIRE(row_sums[1].item<double>() == Approx(0.0));
  REQUIRE(row_sums[2].item<double>() > 0.0);
  REQUIRE(row_sums[3].item<double>() == Approx(0.0));
  REQUIRE(row_sums[4].item<double>() > 0.0);
  // The step counter is global, not per row: bias correction must not restart.
  REQUIRE(after->step() == 1);
}

TEST_CASE("RemapParameters_RowDuplicationGrowth_ExpandsTensorsCorrectly",
          "[gsplat]") {
  auto g = make_tensors(4);
  AdamLrs lrs;
  auto opt = make_adam(g, lrs);
  one_step(g, opt);

  const auto means_before = g.means.detach().clone();
  // Old rows keep their place; rows 4 and 5 are copies of rows 0 and 2, the
  // shape MCMC growth produces.
  const auto src = torch::tensor({0, 1, 2, 3, 0, 2}, torch::kInt64);
  auto reset = torch::zeros({6}, torch::kBool);
  reset.slice(0, 4, 6).fill_(true);

  remap_parameters(g, opt, lrs, src, reset);

  REQUIRE(g.count() == 6);
  REQUIRE(g.sh.size(0) == 6);
  REQUIRE(all_close(g.means.detach(), means_before.index_select(0, src)));

  const auto *after = adam_state(opt, g.means);
  REQUIRE(after != nullptr);
  REQUIRE(after->exp_avg().size(0) == 6);
  const auto row_sums = after->exp_avg().abs().sum(1);
  REQUIRE(row_sums[0].item<double>() > 0.0); // surviving row keeps momentum
  REQUIRE(row_sums[4].item<double>() == Approx(0.0)); // new row starts clean
  REQUIRE(row_sums[5].item<double>() == Approx(0.0));
}

TEST_CASE("RemapParameters_OptimizerNotYetStepped_RemapsWithoutMomentState",
          "[gsplat]") {
  // No step taken means no moment buffers to carry. The remap must still
  // rewrite the parameters rather than trip over the missing state — this is
  // the path a prune at iteration 1 would take.
  auto g = make_tensors(5);
  AdamLrs lrs;
  auto opt = make_adam(g, lrs);

  const auto keep = torch::tensor({1, 3}, torch::kInt64);
  REQUIRE_NOTHROW(remap_parameters(g, opt, lrs, keep, /*reset=*/{}));

  REQUIRE(g.count() == 2);
  REQUIRE(adam_state(opt, g.means) == nullptr);
  REQUIRE_NOTHROW(one_step(g, opt));
  REQUIRE(adam_state(opt, g.means)->step() == 1);
}
