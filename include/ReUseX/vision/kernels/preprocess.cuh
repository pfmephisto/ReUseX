#pragma once
#include <ReUseX/vision/common/norm.hpp>
#include <cuda_runtime.h>
#include <memory>

namespace cuda {

// Kernel function: Warpaffin operation on three-channel images and
// normalization.
__global__ void warp_affine_bilinear_and_normalize_plane_kernel(
    uint8_t *src, int src_line_size, int src_width, int src_height, float *dst,
    int dst_width, int dst_height, uint8_t const_value_st,
    float *warp_affine_matrix_2_3, ReUseX::vision::norm_image::Norm norm);

// Kernel function for single-channel image warpaffin operation
__global__ void warp_affine_bilinear_single_channel_kernel(
    float *src, int src_line_size, int src_width, int src_height, float *dst,
    int dst_width, int dst_height, float const_value_st,
    float *warp_affine_matrix_2_3);

__global__ void warp_affine_bilinear_single_channel_mask_kernel(
    float *src, int src_line_size, int src_width, int src_height, uint8_t *dst,
    int dst_width, int dst_height, float const_value_st,
    float *warp_affine_matrix_2_3);

} // namespace cuda
