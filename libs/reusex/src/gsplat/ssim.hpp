// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Module-private: structural similarity, the perceptual half of the training
// loss and the metric the held-out evaluation reports alongside PSNR.
//
// Not installed and not public, for the same reason as rasterize.hpp /
// optimizer.hpp: the signature is torch-typed, and reusex_gsplat's public
// headers deliberately expose no torch type (docs/STANDARDS.md §2).
//
// It lives in the CUDA half of the module (`reusex_gsplat`) purely because it
// is torch — the computation itself is device-agnostic and runs perfectly well
// on CPU tensors, which is what makes it testable without a GPU (#332).
#pragma once

#include <torch/torch.h>

namespace reusex::gsplat::detail {

/// Normalised 1-D Gaussian window of @p window taps with standard deviation
/// @p sigma, on the device/dtype described by @p o.
torch::Tensor gaussian_kernel1d(int window, double sigma,
                                const torch::TensorOptions &o);

/// Structural similarity over a [1,C,H,W] pair, the standard 11x11 sigma=1.5
/// Gaussian-windowed formulation used by every 3DGS implementation.
///
/// Returns the mean over all pixels and channels as a 0-d tensor: 1.0 for
/// identical inputs, falling toward 0 as structure diverges. Symmetric in its
/// arguments.
torch::Tensor ssim(const torch::Tensor &a, const torch::Tensor &b);

} // namespace reusex::gsplat::detail
