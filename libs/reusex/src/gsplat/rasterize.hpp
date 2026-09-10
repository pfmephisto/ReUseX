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
///
/// The spherical harmonics are **two** tensors rather than one [N,K,3] block,
/// and that is a deliberate optimizer decision rather than a layout
/// preference. Adam needs the degree-0 term and the higher bands in separate
/// parameter groups, because the reference 3DGS trains the higher bands at
/// `lr / 20` — and a shared group cannot express that. Nor can a gradient
/// hook: Adam normalises by its own second moment, so uniformly scaling a
/// parameter's gradient leaves its step length unchanged. Separate leaves are
/// the only way. (The reference implementation splits them for the same
/// reason, as `_features_dc` / `_features_rest`.)
struct GaussianTensors {
  torch::Tensor means;         ///< [N,3] f32
  torch::Tensor log_scales;    ///< [N,3] f32
  torch::Tensor quats;         ///< [N,4] f32, (w,x,y,z)
  torch::Tensor logit_opacity; ///< [N]   f32
  torch::Tensor sh_dc;         ///< [N,1,3] f32, degree-0 coefficients
  /// [N,K-1,3] f32, degrees 1..sh_degree. Always defined and always [N,·,3],
  /// with K-1 == 0 for a degree-0 model — an always-present zero-width tensor
  /// keeps the optimizer's parameter list one fixed shape instead of two.
  torch::Tensor sh_rest;
  int sh_degree = 0;

  int64_t count() const { return means.size(0); }

  /// The coefficients the rasterizer wants: [N, (d+1)^2, 3] for
  /// @p active_degree, assembled from the two parameter tensors.
  ///
  /// Only the bands `active_degree` actually uses are concatenated, so the
  /// SH warm-up costs nothing while the higher bands are still locked — at a
  /// million Gaussians a full degree-3 block is ~190 MB, and copying it on
  /// every iteration to feed zeros to the kernel would be pure waste.
  /// Concatenation is differentiable, so gradients land back on whichever of
  /// the two leaves contributed.
  torch::Tensor sh_coeffs(int active_degree) const;
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
