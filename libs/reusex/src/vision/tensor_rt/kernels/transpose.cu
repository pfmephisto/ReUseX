// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/tensor_rt/common/check.hpp"
#include "vision/tensor_rt/kernels/transpose.cuh"
#include "vision/tensor_rt/kernels/transpose.hpp"

#include <cstddef>

namespace cuda {

__global__ void transpose_2d_tiled_kernel(const float *__restrict__ src,
                                          float *__restrict__ dst, int rows,
                                          int cols) {
  // +1 padding: consecutive tile columns land in consecutive banks, so the
  // column-major read below is conflict-free.
  __shared__ float tile[kTransposeTile][kTransposeTile + 1];

  // --- Read a 32x32 tile of src, row-major (coalesced along `cols`). --------
  int x = blockIdx.x * kTransposeTile + threadIdx.x; // column in src
  int y = blockIdx.y * kTransposeTile + threadIdx.y; // row in src

  for (int j = 0; j < kTransposeTile; j += kTransposeBlockRows) {
    if (x < cols && (y + j) < rows)
      tile[threadIdx.y + j][threadIdx.x] =
          src[static_cast<size_t>(y + j) * cols + x];
  }

  __syncthreads();

  // --- Write it back transposed, again coalesced (now along `rows`). -------
  x = blockIdx.y * kTransposeTile + threadIdx.x; // column in dst (== src row)
  y = blockIdx.x * kTransposeTile + threadIdx.y; // row in dst    (== src col)

  for (int j = 0; j < kTransposeTile; j += kTransposeBlockRows) {
    if (x < rows && (y + j) < cols)
      dst[static_cast<size_t>(y + j) * rows + x] =
          tile[threadIdx.x][threadIdx.y + j];
  }
}

} // namespace cuda

namespace reusex::vision::tensor_rt {

void chw_to_hwc(const float *src, float *dst, int c, int h, int w,
                void *stream) {
  const int rows = c;
  const int cols = h * w;
  if (rows <= 0 || cols <= 0 || src == nullptr || dst == nullptr)
    return;

  dim3 grid((cols + cuda::kTransposeTile - 1) / cuda::kTransposeTile,
            (rows + cuda::kTransposeTile - 1) / cuda::kTransposeTile);
  dim3 block(cuda::kTransposeTile, cuda::kTransposeBlockRows);

  checkKernel(
      cuda::transpose_2d_tiled_kernel<<<grid, block, 0, (cudaStream_t)stream>>>(
          src, dst, rows, cols));
}

double chw_to_hwc_device_roundtrip(const float *h_src, float *h_dst, int c,
                                   int h, int w, int iters) {
  const size_t n = static_cast<size_t>(c) * h * w;
  if (n == 0 || iters <= 0)
    return -1.0;

  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0)
    return -1.0;

  float *d_src = nullptr;
  float *d_dst = nullptr;
  cudaStream_t stream = nullptr;
  cudaEvent_t start = nullptr;
  cudaEvent_t stop = nullptr;
  double ms = -1.0;

  if (cudaMalloc(&d_src, n * sizeof(float)) == cudaSuccess &&
      cudaMalloc(&d_dst, n * sizeof(float)) == cudaSuccess &&
      cudaStreamCreate(&stream) == cudaSuccess &&
      cudaEventCreate(&start) == cudaSuccess &&
      cudaEventCreate(&stop) == cudaSuccess) {

    checkRuntime(cudaMemcpyAsync(d_src, h_src, n * sizeof(float),
                                 cudaMemcpyHostToDevice, stream));
    // Poison the destination so a partially-written output cannot silently
    // pass a comparison against a previous run's contents.
    checkRuntime(cudaMemsetAsync(d_dst, 0xff, n * sizeof(float), stream));

    checkRuntime(cudaEventRecord(start, stream));
    for (int i = 0; i < iters; ++i)
      chw_to_hwc(d_src, d_dst, c, h, w, stream);
    checkRuntime(cudaEventRecord(stop, stream));

    checkRuntime(cudaMemcpyAsync(h_dst, d_dst, n * sizeof(float),
                                 cudaMemcpyDeviceToHost, stream));
    checkRuntime(cudaStreamSynchronize(stream));

    float elapsed_ms = 0.0f;
    if (cudaEventElapsedTime(&elapsed_ms, start, stop) == cudaSuccess)
      ms = static_cast<double>(elapsed_ms) / iters;
  }

  if (start)
    cudaEventDestroy(start);
  if (stop)
    cudaEventDestroy(stop);
  if (stream)
    cudaStreamDestroy(stream);
  if (d_src)
    cudaFree(d_src);
  if (d_dst)
    cudaFree(d_dst);

  return ms;
}

} // namespace reusex::vision::tensor_rt
