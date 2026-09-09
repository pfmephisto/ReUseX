// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rasterize.hpp"

#include <reusex/core/logging.hpp>

#include <Intersect.h>
#include <Projection.h>
#include <Rasterization.h>
#include <SphericalHarmonics.h>

#include <stdexcept>

// gsplat's torch custom-class registrations live in `gsplat/cuda/ext.cpp`,
// which our derivation does not compile (it is the pybind entry point and sits
// outside csrc/). `UnscentedTransformParameters` and
// `FThetaCameraDistortionParameters` derive from torch::CustomClassHolder and
// are stashed as IValues in the rasterizer's autograd context, so without a
// registration the *backward* pass dies with
//   "Trying to instantiate a class that isn't a registered custom class".
// Registering them here is the minimal replacement. TORCH_LIBRARY (not
// TORCH_LIBRARY_FRAGMENT) is required because class_ registration is not
// permitted in a fragment; this claims the `gsplat` library name, so if
// ext.cpp is ever compiled into libgsplat.a this block must go.
TORCH_LIBRARY(gsplat, m) {
  m.class_<UnscentedTransformParameters>("UnscentedTransformParameters")
      .def(torch::init<>());
  m.class_<FThetaCameraDistortionParameters>(
      "FThetaCameraDistortionParameters");
}

namespace reusex::gsplat::detail {

namespace {

/// EWA projection defaults, mirroring gsplat/rendering.py.
constexpr double kEps2d = 0.3;
constexpr double kNearPlane = 0.01;
constexpr double kFarPlane = 1e10;
constexpr double kRadiusClip = 0.0;

/// gsplat's own heuristic (`_resolve_tile_size`): the wide tile only pays off
/// on large images. The eval3d kernel is instantiated for both 8 and 16.
int64_t resolve_tile_size(int64_t width, int64_t height) {
  return std::min(width, height) >= 1080 ? 16 : 8;
}

/// PINHOLE, no distortion, no rolling shutter — but gsplat dereferences the
/// unscented-transform and f-theta parameter blocks unconditionally, so both
/// must be non-null even though a pinhole camera uses neither.
c10::intrusive_ptr<UnscentedTransformParameters> default_ut_params() {
  static const auto p = c10::make_intrusive<UnscentedTransformParameters>();
  return p;
}

c10::intrusive_ptr<FThetaCameraDistortionParameters> default_ftheta_params() {
  static const auto p = c10::make_intrusive<FThetaCameraDistortionParameters>(
      FThetaCameraDistortionParameters::PolynomialType::PIXELDIST_TO_ANGLE,
      std::array<float, 6>{1, 0, 0, 0, 0, 0},
      std::array<float, 6>{1, 0, 0, 0, 0, 0}, 3.14159265f,
      std::array<float, 3>{1, 0, 0});
  return p;
}

} // namespace

RenderOutput render(const GaussianTensors &g, const torch::Tensor &viewmat,
                    const torch::Tensor &K, int64_t width, int64_t height,
                    int active_sh_degree) {
  const int64_t N = g.count();
  if (N == 0)
    throw std::runtime_error("gsplat: cannot render zero Gaussians");
  if (active_sh_degree < 0 || active_sh_degree > g.sh_degree)
    throw std::runtime_error(
        fmt::format("gsplat: active SH degree {} outside the model's 0..{}",
                    active_sh_degree, g.sh_degree));

  const int64_t C = 1;
  auto viewmats = viewmat.reshape({C, 4, 4}).contiguous();
  auto Ks = K.reshape({C, 3, 3}).contiguous();

  // ---- activations ---------------------------------------------------------
  // Adam steps the raw parameters; the rasterizer wants metric scales, an alpha
  // in [0,1] and a unit quaternion. Doing it here keeps the optimizer's view of
  // the parameters unconstrained (no projection step needed).
  auto scales = torch::exp(g.log_scales);
  auto opacities = torch::sigmoid(g.logit_opacity);
  auto quats = torch::nn::functional::normalize(
      g.quats, torch::nn::functional::NormalizeFuncOptions().dim(-1));

  // ---- SH -> per-camera colours (differentiable) ---------------------------
  // color_post = 2 is gsplat's SH_POST_SHIFT_RELU: RGB = max(SH(v) + 0.5, 0),
  // the standard 3DGS convention. Coefficients are passed raw; the kernel
  // applies the SH_C0 factor itself.
  at::Tensor colors = ::gsplat::assemble_proj_features(
      /*degrees_to_use=*/static_cast<int64_t>(active_sh_degree),
      /*B=*/1, /*C=*/C, /*N=*/N, /*Dc=*/3, /*E=*/0,
      /*color_post=*/2, /*extra_post=*/0, /*has_depth=*/false,
      /*depth_is_zero=*/false, /*extra_has_c=*/false,
      g.means.reshape({1, N, 3}), viewmats.reshape({1, C, 4, 4}), c10::nullopt,
      g.sh, c10::nullopt, c10::nullopt, c10::nullopt);
  colors = colors.reshape({C, N, 3}).contiguous();

  // ---- projection: purely to decide which Gaussian touches which tile ------
  // The world-space rasterizer re-derives its own geometry, so nothing here
  // needs a gradient; InferenceMode keeps autograd from taping it.
  ::gsplat::ProjectionEWA3DGSFusedFwdResult proj;
  {
    c10::InferenceMode guard;
    proj = ::gsplat::projection_ewa_3dgs_fused_fwd(
        g.means, c10::nullopt, quats, scales, opacities, viewmats, Ks, width,
        height, kEps2d, kNearPlane, kFarPlane, kRadiusClip,
        /*calc_compensations=*/false, ::gsplat::CameraModelType::PINHOLE);
  }

  const int64_t tile = resolve_tile_size(width, height);
  const int64_t tile_w = (width + tile - 1) / tile;
  const int64_t tile_h = (height + tile - 1) / tile;

  ::gsplat::TileIntersectResult isect = ::gsplat::intersect_tile(
      proj.means2d.contiguous(), proj.radii.contiguous(),
      proj.depths.contiguous(), c10::nullopt, c10::nullopt, c10::nullopt,
      c10::nullopt, std::optional<int64_t>(C), tile, tile_w, tile_h,
      /*sort=*/true, /*segmented=*/false);
  at::Tensor tile_offsets =
      ::gsplat::intersect_offset(isect.isect_ids, C, tile_w, tile_h)
          .reshape({C, tile_h, tile_w})
          .contiguous();

  // ---- differentiable rasterization ---------------------------------------
  auto out = ::gsplat::rasterize_to_pixels_from_world_3dgs(
      g.means, quats, scales, colors,
      opacities.unsqueeze(0).expand({C, N}).contiguous(),
      /*backgrounds=*/c10::nullopt, /*masks=*/c10::nullopt, width, height, tile,
      viewmats, /*viewmats1=*/c10::nullopt, Ks,
      static_cast<int64_t>(::gsplat::CameraModelType::PINHOLE),
      default_ut_params(), static_cast<int64_t>(ShutterType::GLOBAL),
      /*rays=*/c10::nullopt, /*radial_coeffs=*/c10::nullopt,
      /*tangential_coeffs=*/c10::nullopt, /*thin_prism_coeffs=*/c10::nullopt,
      default_ftheta_params(), /*lidar_coeffs=*/c10::nullopt,
      /*external_distortion_params=*/c10::nullopt, tile_offsets,
      isect.flatten_ids.contiguous(),
      /*return_sample_counts=*/false, /*use_hit_distance=*/false,
      /*return_normals=*/false,
      /*renderer_config=*/static_cast<int64_t>(::gsplat::MIXED_BATCH),
      /*return_last_ids=*/false, /*unsafe_masked_tile_outputs=*/false);

  RenderOutput r;
  r.image = out.renders.reshape({height, width, 3});
  r.alpha = out.alphas.reshape({height, width, 1});
  return r;
}

} // namespace reusex::gsplat::detail
