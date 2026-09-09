// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <vector>

#include <reusex/types/point_types.hpp>

namespace reusex::gsplat {

/// The 3D Gaussian Splatting DC spherical-harmonic basis function value,
/// `0.5 * sqrt(1/pi)`. Converting between a linear RGB colour in [0,1] and the
/// degree-0 SH coefficient stored per Gaussian goes through it in both
/// directions (see rgb_to_sh_dc / sh_dc_to_rgb).
inline constexpr float kShC0 = 0.28209479177387814f;

/// RGB in [0,1] -> degree-0 SH coefficient.
inline float rgb_to_sh_dc(float c) { return (c - 0.5f) / kShC0; }

/// Degree-0 SH coefficient -> RGB in [0,1] (not clamped).
inline float sh_dc_to_rgb(float f) { return f * kShC0 + 0.5f; }

/// Numerically safe logit, the inverse of the sigmoid used for opacity.
float inverse_sigmoid(float x);

/// A set of 3D Gaussians in plain host memory.
///
/// Deliberately POD-ish and torch-free: this is the module's public exchange
/// type, so `reusex_gsplat`'s public headers stay clear of LibTorch (the same
/// reason ProjectDB hides sqlite3 — docs/STANDARDS.md §2). The trainer uploads
/// these arrays into CUDA tensors internally and writes them back on save.
///
/// Storage conventions follow the reference 3DGS `.ply` format so a trained
/// model opens in standard splat viewers:
/// * `scales` are **log**-scales (exp() gives metres),
/// * `opacities` are **logits** (sigmoid() gives alpha in [0,1]),
/// * `quats` are `(w, x, y, z)`, not necessarily normalised,
/// * `sh_dc` is the degree-0 coefficient triple, *not* RGB.
///
/// All arrays are index-aligned and must be filtered or reordered together.
struct GaussianCloud {
  std::vector<std::array<float, 3>> means;  ///< world-space centres [m]
  std::vector<std::array<float, 3>> scales; ///< log-scales, per axis
  std::vector<std::array<float, 4>> quats;  ///< rotation (w, x, y, z)
  std::vector<float> opacities;             ///< logit-opacity
  std::vector<std::array<float, 3>> sh_dc;  ///< degree-0 SH per channel
  /// Higher-order SH, `[N][(sh_bands-1)*3]` flattened; empty when degree 0.
  std::vector<std::vector<float>> sh_rest;
  int sh_degree = 0; ///< active SH degree (0 = DC only)

  std::size_t size() const { return means.size(); }
  bool empty() const { return means.empty(); }

  /// Throws std::runtime_error naming the first array whose length disagrees.
  void validate() const;
};

/// Parameters for seeding Gaussians from a ReUseX point cloud.
struct GaussianInitOptions {
  /// Neighbours used for the nearest-neighbour spacing that sets the initial
  /// (isotropic) scale of each Gaussian. The reference implementation uses the
  /// mean squared distance to the 3 nearest points.
  int knn = 3;

  /// Initial alpha before the logit transform.
  float initial_opacity = 0.1f;

  /// Clamp on the initial scale [m]. A LiDAR cloud can contain near-duplicate
  /// points whose neighbour distance is ~0; without a floor those Gaussians
  /// start infinitely small and never receive gradient.
  float min_scale = 1e-4f;
  float max_scale = 0.5f;

  /// Active SH degree. 0 keeps only the DC (view-independent) colour, which is
  /// what a short training run can actually fit.
  int sh_degree = 0;

  /// Uniform stride cap on the number of seeds (0 = use every point).
  ///
  /// There is deliberately no RNG seed here: seeding picks every `stride`-th
  /// point, which is reproducible without consuming randomness at all
  /// (STANDARDS §6). `TrainOptions::seed` covers the parts that are random.
  std::size_t max_points = 0;
};

/// Seed Gaussians from an XYZRGB cloud: means = point positions, DC colour =
/// point colour, scale = log of the k-NN mean spacing, rotation = identity,
/// opacity = logit(initial_opacity).
///
/// @throws std::runtime_error if @p cloud is null or empty.
GaussianCloud init_from_point_cloud(const CloudPtr &cloud,
                                    const GaussianInitOptions &opt = {});

/// Write a 3DGS-format binary-little-endian `.ply` (the property names the
/// reference implementation and every common viewer expect: x/y/z, nx/ny/nz,
/// f_dc_*, f_rest_*, opacity, scale_*, rot_*).
void save_gaussian_ply(const GaussianCloud &gaussians,
                       const std::filesystem::path &path);

/// Read back a `.ply` written by save_gaussian_ply.
GaussianCloud load_gaussian_ply(const std::filesystem::path &path);

} // namespace reusex::gsplat
