// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Internal (module-private) wrapper around gsplat's differentiable rasterizer.
// Not installed: it exposes torch types, which reusex_gsplat's public headers
// deliberately do not (docs/STANDARDS.md §2).
//
// WHICH gsplat ENTRY POINT, AND WHY
// ---------------------------------
// gsplat exposes two render paths. The familiar one — `rasterization()` in
// Python, backed by `projection_ewa_3dgs_fused` + `rasterize_to_pixels_3dgs` —
// is **not usable from C++ in this build**: those C++ wrappers dispatch through
// `c10::Dispatcher::findSchemaOrThrow("gsplat::…")`, and the schemas are
// registered in `gsplat/cuda/ext.cpp`, which lives outside `csrc/` and is
// therefore not part of libgsplat.a. Calling them throws
// "Could not find schema for gsplat::rasterize_to_pixels_3dgs" at run time.
// Worse, their autograd is attached from Python
// (`torch.library.register_autograd`), so even registering the schemas would
// only restore a forward pass.
//
// The path used here — `rasterize_to_pixels_from_world_3dgs` (gsplat's 3DGUT
// "eval3d" renderer) — calls `torch::autograd::Function` directly, with no
// dispatcher and no Python, so gradients flow to means / quats / scales /
// colors / opacities natively from C++. `assemble_proj_features` is likewise
// the only autograd-aware SH evaluator reachable from C++.
//
// Consequence for density control: the world path does not produce the
// screen-space `absgrad` (`means2d_absgrad`) that the reference clone/split
// heuristic keys on. See TrainOptions::prune_enabled.
#pragma once

#include <torch/torch.h>

namespace reusex::gsplat::detail {

/// The optimizable state, as CUDA tensors.
///
/// Stored in the same parameterisation as GaussianCloud — log-scales, logit
/// opacity, unnormalised quaternions — because that is what Adam should step
/// in; the activations are applied inside render().
struct GaussianTensors {
  torch::Tensor means;         ///< [N,3] f32
  torch::Tensor log_scales;    ///< [N,3] f32
  torch::Tensor quats;         ///< [N,4] f32, (w,x,y,z)
  torch::Tensor logit_opacity; ///< [N]   f32
  torch::Tensor sh;            ///< [N,K,3] f32, raw SH coefficients
  int sh_degree = 0;

  int64_t count() const { return means.size(0); }
};

/// One rendered image plus its alpha.
struct RenderOutput {
  torch::Tensor image; ///< [H,W,3] f32, channel order matches the SH input
  torch::Tensor alpha; ///< [H,W,1] f32
};

/// Differentiably render @p g from a single camera.
///
/// @param viewmat [4,4] world -> camera, float32 CUDA
/// @param K       [3,3] pinhole intrinsics, float32 CUDA
RenderOutput render(const GaussianTensors &g, const torch::Tensor &viewmat,
                    const torch::Tensor &K, int64_t width, int64_t height,
                    int active_sh_degree);

/// Number of SH coefficients per channel for a degree, i.e. (d+1)^2.
inline int64_t sh_bands(int degree) {
  return static_cast<int64_t>((degree + 1) * (degree + 1));
}

} // namespace reusex::gsplat::detail
