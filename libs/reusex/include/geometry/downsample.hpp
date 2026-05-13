// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/types.hpp"

#include <cstdint>
#include <vector>

namespace reusex::geometry {

/// Per-input-point assignment to an output voxel bucket.
///
/// Built once from a cloud of positions, then reused to downsample any
/// parallel cloud (e.g. `normals`) with the same row count so output rows
/// stay aligned position-for-position with the primary downsampled cloud.
///
/// Voxel coordinates are computed as int64 and packed into a uint64 hash
/// key, which avoids the int32 overflow that pcl::VoxelGrid hits on
/// multi-billion-point inputs when (n_x * n_y * n_z) exceeds INT_MAX.
struct VoxelAssignment {
  /// Voxel edge length used to bucket points.
  float leaf_size = 0.0f;

  /// Number of unique occupied voxels (== output cloud size).
  uint32_t bucket_count = 0;

  /// For each input point: the bucket id it falls into, or
  /// `kSkippedPoint` if the point was non-finite (and therefore dropped).
  std::vector<uint32_t> point_to_bucket;

  /// Sentinel marking input points that were skipped during assignment.
  static constexpr uint32_t kSkippedPoint =
      static_cast<uint32_t>(-1);
};

/// Build a voxel assignment for `cloud` at the given leaf size.
///
/// Non-finite points are recorded with `kSkippedPoint` in `point_to_bucket`
/// and contribute neither a bucket nor a centroid.
///
/// Throws std::invalid_argument if `leaf_size <= 0` or `cloud` is empty.
/// Throws std::out_of_range if the cloud spans more than ~2 million voxels
/// per axis at the chosen leaf size (raise the leaf size).
/// Throws std::overflow_error if the number of occupied voxels exceeds
/// 2^32 - 1 (raise the leaf size).
VoxelAssignment voxel_assignment(const Cloud &cloud, float leaf_size);

/// Downsample `cloud` using the precomputed assignment. The output point
/// in bucket `b` is the centroid (mean position and mean RGB) of every
/// input point that fell into bucket `b`.
///
/// `cloud.size()` must equal `a.point_to_bucket.size()`.
CloudPtr downsample(const Cloud &cloud, const VoxelAssignment &a);

/// Downsample a parallel normals cloud using the same assignment that was
/// built from a sibling XYZRGB cloud. Per-bucket normals are averaged and
/// renormalized. Curvature is averaged.
///
/// `cloud.size()` must equal `a.point_to_bucket.size()`.
CloudNPtr downsample(const CloudN &cloud, const VoxelAssignment &a);

} // namespace reusex::geometry
