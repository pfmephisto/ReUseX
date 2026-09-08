// SPDX-FileCopyrightText: Soumith Chintala 2016
//
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file torchvision_nms_kernel.hpp
 * @brief Declaration for the vendored torchvision CPU NMS kernel.
 *
 * See `src/vision/third_party/torchvision_nms_kernel.cpp` for provenance and
 * the list of adaptations made to the upstream source.
 *
 * Callers inside ReUseX should not use this directly — go through
 * `reusex::vision::nms()` in `vision/nms.hpp`, which owns the ReUseX-facing
 * preconditions and API contract.
 */

#pragma once

#include <ATen/ATen.h>

namespace reusex::vision::torchvision {

/**
 * @brief torchvision's reference CPU non-maximum-suppression kernel.
 *
 * @param dets Boxes `[N, 4]` in `(x1, y1, x2, y2)` order, CPU, floating point.
 * @param scores Confidence scores `[N]`, same dtype and device as @p dets.
 * @param iou_threshold Boxes are suppressed when IoU is *strictly greater*
 *        than this value.
 * @return `int64` tensor of kept indices, in descending score order.
 *
 * Throws `c10::Error` (via `TORCH_CHECK`) if the tensors are not on the CPU,
 * have mismatched dtypes, or have the wrong rank/extent.
 */
at::Tensor nms_kernel(const at::Tensor &dets, const at::Tensor &scores,
                      double iou_threshold);

} // namespace reusex::vision::torchvision
