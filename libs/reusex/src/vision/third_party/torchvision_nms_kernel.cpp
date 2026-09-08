// SPDX-FileCopyrightText: Soumith Chintala 2016
//
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file torchvision_nms_kernel.cpp
 * @brief Vendored copy of torchvision's reference CPU NMS kernel.
 *
 * Upstream: https://github.com/pytorch/vision
 * Path:     torchvision/csrc/ops/cpu/nms_kernel.cpp
 * Tag:      v0.24.0 (the torchvision release paired with torch 2.9.0, which is
 *           the LibTorch version built by `pkgs/libtorch/package.nix`)
 * License:  BSD-3-Clause — see `LICENSES/BSD-3-Clause.txt`
 *
 * Why vendored rather than depended upon (see #141): nixpkgs ships no C++
 * torchvision. There is no `torchvision`/`libtorchvision` attribute at all, and
 * `python3Packages.torchvision` is a `buildPythonPackage` that installs only
 * the Python extension module — no `libtorchvision.so`, no headers and no
 * `TorchVisionConfig.cmake`, so `find_package(TorchVision)` cannot succeed. It
 * is also built against nixpkgs' own `torch` rather than our from-source
 * LibTorch, so it would not be ABI-compatible even if the artifacts existed.
 * Building torchvision's whole ops library from source to obtain one function
 * is not a trade worth making.
 *
 * Note that upstream `main` has since been rewritten onto torchvision's new
 * stable-ABI headers and includes a torchvision-private `StableABICompat.h`,
 * which makes it un-vendorable standalone. v0.24.0 is the last plain-`at::`-API
 * revision and is the correct pairing for our LibTorch regardless.
 *
 * ## Adaptations from upstream
 *
 * The kernel body is carried verbatim. Only the surrounding scaffolding
 * changed, so that a future re-sync stays a trivial diff:
 *
 * 1. Namespace `vision::ops` -> `reusex::vision::torchvision`, and the kernel
 *    lifted out of the anonymous namespace so it has external linkage and can
 *    be called directly.
 * 2. The `TORCH_LIBRARY_IMPL(torchvision, CPU, m)` registration block was
 *    dropped. We do not build torchvision's dispatcher library, and
 *    registering an implementation for a `torchvision::nms` schema that was
 *    never declared would abort during static initialisation.
 *
 * Only the CPU kernel is vendored. Every NMS call site in ReUseX moves its
 * tensors to the CPU first (`vision/libtorch/Yolo.cpp` calls `.cpu()` on the
 * model outputs before post-processing), so the CUDA kernel would be dead code
 * plus an extra translation unit in every CUDA build.
 */

#include "vision/third_party/torchvision_nms_kernel.hpp"

#include <ATen/ATen.h>

#include <algorithm>
#include <cstdint>

namespace reusex::vision::torchvision {

namespace {

template <typename scalar_t>
at::Tensor nms_kernel_impl(const at::Tensor &dets, const at::Tensor &scores,
                           double iou_threshold) {
  TORCH_CHECK(dets.is_cpu(), "dets must be a CPU tensor");
  TORCH_CHECK(scores.is_cpu(), "scores must be a CPU tensor");
  TORCH_CHECK(dets.scalar_type() == scores.scalar_type(),
              "dets should have the same type as scores");

  if (dets.numel() == 0) {
    return at::empty({0}, dets.options().dtype(at::kLong));
  }

  auto x1_t = dets.select(1, 0).contiguous();
  auto y1_t = dets.select(1, 1).contiguous();
  auto x2_t = dets.select(1, 2).contiguous();
  auto y2_t = dets.select(1, 3).contiguous();

  at::Tensor areas_t = (x2_t - x1_t) * (y2_t - y1_t);

  auto order_t = std::get<1>(
      scores.sort(/*stable=*/true, /*dim=*/0, /* descending=*/true));

  auto ndets = dets.size(0);
  at::Tensor suppressed_t = at::zeros({ndets}, dets.options().dtype(at::kByte));
  at::Tensor keep_t = at::zeros({ndets}, dets.options().dtype(at::kLong));

  auto suppressed = suppressed_t.data_ptr<uint8_t>();
  auto keep = keep_t.data_ptr<int64_t>();
  auto order = order_t.data_ptr<int64_t>();
  auto x1 = x1_t.data_ptr<scalar_t>();
  auto y1 = y1_t.data_ptr<scalar_t>();
  auto x2 = x2_t.data_ptr<scalar_t>();
  auto y2 = y2_t.data_ptr<scalar_t>();
  auto areas = areas_t.data_ptr<scalar_t>();

  int64_t num_to_keep = 0;

  for (int64_t _i = 0; _i < ndets; _i++) {
    auto i = order[_i];
    if (suppressed[i] == 1) {
      continue;
    }
    keep[num_to_keep++] = i;
    auto ix1 = x1[i];
    auto iy1 = y1[i];
    auto ix2 = x2[i];
    auto iy2 = y2[i];
    auto iarea = areas[i];

    for (int64_t _j = _i + 1; _j < ndets; _j++) {
      auto j = order[_j];
      if (suppressed[j] == 1) {
        continue;
      }
      auto xx1 = std::max(ix1, x1[j]);
      auto yy1 = std::max(iy1, y1[j]);
      auto xx2 = std::min(ix2, x2[j]);
      auto yy2 = std::min(iy2, y2[j]);

      auto w = std::max(static_cast<scalar_t>(0), xx2 - xx1);
      auto h = std::max(static_cast<scalar_t>(0), yy2 - yy1);
      auto inter = w * h;
      auto ovr = inter / (iarea + areas[j] - inter);
      if (ovr > iou_threshold) {
        suppressed[j] = 1;
      }
    }
  }
  return keep_t.narrow(/*dim=*/0, /*start=*/0, /*length=*/num_to_keep);
}

} // namespace

at::Tensor nms_kernel(const at::Tensor &dets, const at::Tensor &scores,
                      double iou_threshold) {
  TORCH_CHECK(dets.dim() == 2, "boxes should be a 2d tensor, got ", dets.dim(),
              "D");
  TORCH_CHECK(dets.size(1) == 4,
              "boxes should have 4 elements in dimension 1, got ",
              dets.size(1));
  TORCH_CHECK(scores.dim() == 1, "scores should be a 1d tensor, got ",
              scores.dim(), "D");
  TORCH_CHECK(dets.size(0) == scores.size(0),
              "boxes and scores should have same number of elements in ",
              "dimension 0, got ", dets.size(0), " and ", scores.size(0));

  auto result = at::empty({0}, dets.options());

  AT_DISPATCH_FLOATING_TYPES(dets.scalar_type(), "nms_kernel", [&] {
    result = nms_kernel_impl<scalar_t>(dets, scores, iou_threshold);
  });
  return result;
}

} // namespace reusex::vision::torchvision
