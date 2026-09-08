// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace reusex::vision::tensor_rt {

// Rearrange a device buffer from spatial [C,H,W] (index c*H*W + h*W + w) to
// seq-major [H*W,C] (index (h*W+w)*C + c), out-of-place. `src` and `dst` must
// be distinct device allocations of at least C*H*W floats.
//
// Enqueued on `stream` (a cudaStream_t, taken as void* so consumers do not need
// the CUDA headers) and never synchronises: the caller keeps ordering through
// the stream. No-op for a degenerate (non-positive) extent.
void chw_to_hwc(const float *src, float *dst, int c, int h, int w,
                void *stream);

// Verification / benchmarking entry point: upload `h_src` ([C,H,W], C*H*W
// floats), run the transpose kernel `iters` times on a private stream, and
// download the result into `h_dst` ([H*W,C]). Returns the mean per-iteration
// kernel time in milliseconds (measured with CUDA events), or a negative value
// if no CUDA device is usable. Not used on the inference path — the per-frame
// path calls chw_to_hwc() on device buffers it already owns.
double chw_to_hwc_device_roundtrip(const float *h_src, float *h_dst, int c,
                                   int h, int w, int iters = 1);

} // namespace reusex::vision::tensor_rt
