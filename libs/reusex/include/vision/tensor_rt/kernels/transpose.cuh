// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace cuda {

// Tile geometry for the shared-memory transpose. A 32x32 tile is staged by a
// 32x8 thread block in four strided steps, so both the read and the write are
// fully coalesced 128-byte transactions. The tile row is padded to 33 floats so
// that the column-wise read on the write side hits 32 distinct shared-memory
// banks (no bank conflicts).
inline constexpr int kTransposeTile = 32;
inline constexpr int kTransposeBlockRows = 8;

// Transpose a row-major [rows, cols] float matrix into [cols, rows].
//
// The CHW -> HWC rearrange is exactly this with rows = C and cols = H*W:
// src[c*HW + p] -> dst[p*C + c]. Pure data movement, so the output is
// bit-for-bit identical to a host-side transpose of the same input.
__global__ void transpose_2d_tiled_kernel(const float *__restrict__ src,
                                          float *__restrict__ dst, int rows,
                                          int cols);

} // namespace cuda
