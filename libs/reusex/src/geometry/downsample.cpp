// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/downsample.hpp"
#include "core/logging.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace reusex::geometry {

namespace {

// Pack signed 21-bit (ix, iy, iz) into a uint64_t. With 21 bits per axis
// the per-axis index range is ±2^20 = ±1,048,576 voxels, i.e. ±52 km at a
// 5 cm leaf — far beyond any realistic single-building scan extent.
constexpr int kVoxelBits = 21;
constexpr int64_t kVoxelMax = (int64_t{1} << (kVoxelBits - 1)) - 1;
constexpr int64_t kVoxelMin = -(int64_t{1} << (kVoxelBits - 1));
constexpr uint64_t kVoxelMask = (uint64_t{1} << kVoxelBits) - 1;

inline uint64_t pack_voxel(int64_t ix, int64_t iy, int64_t iz) {
  if (ix < kVoxelMin || ix > kVoxelMax || iy < kVoxelMin || iy > kVoxelMax ||
      iz < kVoxelMin || iz > kVoxelMax)
    throw std::out_of_range(
        "Voxel index exceeds packed range — increase leaf size");
  uint64_t kx = static_cast<uint64_t>(ix - kVoxelMin) & kVoxelMask;
  uint64_t ky = static_cast<uint64_t>(iy - kVoxelMin) & kVoxelMask;
  uint64_t kz = static_cast<uint64_t>(iz - kVoxelMin) & kVoxelMask;
  return kx | (ky << kVoxelBits) | (kz << (2 * kVoxelBits));
}

} // namespace

VoxelAssignment voxel_assignment(const Cloud &cloud, float leaf_size) {
  if (!(leaf_size > 0.0f))
    throw std::invalid_argument("Leaf size must be positive");
  if (cloud.empty())
    throw std::invalid_argument("Cannot voxelize an empty cloud");

  VoxelAssignment a;
  a.leaf_size = leaf_size;
  a.point_to_bucket.resize(cloud.size());

  const float inv_leaf = 1.0f / leaf_size;

  // Reserve based on a rough density heuristic; saves a lot of rehashing
  // on large inputs. Most occupancy ratios for indoor scans land in the
  // 1–10% range, so 1/16 is a conservative starting point.
  std::unordered_map<uint64_t, uint32_t> key_to_bucket;
  key_to_bucket.reserve(cloud.size() / 16 + 1);

  uint32_t next_bucket = 0;
  size_t skipped_nan = 0;

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto &p = cloud[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      a.point_to_bucket[i] = VoxelAssignment::kSkippedPoint;
      ++skipped_nan;
      continue;
    }
    int64_t ix = static_cast<int64_t>(std::floor(p.x * inv_leaf));
    int64_t iy = static_cast<int64_t>(std::floor(p.y * inv_leaf));
    int64_t iz = static_cast<int64_t>(std::floor(p.z * inv_leaf));
    uint64_t key = pack_voxel(ix, iy, iz);

    auto [it, inserted] = key_to_bucket.emplace(key, next_bucket);
    if (inserted) {
      if (next_bucket == std::numeric_limits<uint32_t>::max() - 1)
        throw std::overflow_error(
            "Too many occupied voxels (>= 2^32 - 1) — increase leaf size");
      ++next_bucket;
    }
    a.point_to_bucket[i] = it->second;
  }

  a.bucket_count = next_bucket;

  if (skipped_nan > 0)
    core::warn("voxel_assignment: skipped {} non-finite point(s)", skipped_nan);
  core::info("voxel_assignment: {} buckets from {} points (leaf={} m)",
             a.bucket_count, cloud.size(), leaf_size);

  return a;
}

CloudPtr downsample(const Cloud &cloud, const VoxelAssignment &a) {
  if (cloud.size() != a.point_to_bucket.size())
    throw std::invalid_argument(
        "Cloud size does not match voxel assignment size");

  struct Accumulator {
    double sx = 0.0, sy = 0.0, sz = 0.0;
    double sr = 0.0, sg = 0.0, sb = 0.0;
    uint32_t count = 0;
  };

  std::vector<Accumulator> buckets(a.bucket_count);

  for (size_t i = 0; i < cloud.size(); ++i) {
    uint32_t b = a.point_to_bucket[i];
    if (b == VoxelAssignment::kSkippedPoint)
      continue;
    const auto &p = cloud[i];
    auto &acc = buckets[b];
    acc.sx += p.x;
    acc.sy += p.y;
    acc.sz += p.z;
    acc.sr += static_cast<double>(p.r);
    acc.sg += static_cast<double>(p.g);
    acc.sb += static_cast<double>(p.b);
    ++acc.count;
  }

  auto out = std::make_shared<Cloud>();
  out->reserve(a.bucket_count);
  for (const auto &acc : buckets) {
    if (acc.count == 0)
      continue; // defensive — shouldn't happen for the primary cloud
    const double inv = 1.0 / acc.count;
    PointT p;
    p.x = static_cast<float>(acc.sx * inv);
    p.y = static_cast<float>(acc.sy * inv);
    p.z = static_cast<float>(acc.sz * inv);
    p.r = static_cast<uint8_t>(std::lround(acc.sr * inv));
    p.g = static_cast<uint8_t>(std::lround(acc.sg * inv));
    p.b = static_cast<uint8_t>(std::lround(acc.sb * inv));
    out->push_back(p);
  }
  out->width = static_cast<uint32_t>(out->size());
  out->height = 1;
  out->is_dense = true;
  return out;
}

CloudNPtr downsample(const CloudN &cloud, const VoxelAssignment &a) {
  if (cloud.size() != a.point_to_bucket.size())
    throw std::invalid_argument(
        "Normals size does not match voxel assignment size");

  struct Accumulator {
    double sx = 0.0, sy = 0.0, sz = 0.0;
    double sc = 0.0; // sum of curvature
    uint32_t count = 0;
  };

  std::vector<Accumulator> buckets(a.bucket_count);

  for (size_t i = 0; i < cloud.size(); ++i) {
    uint32_t b = a.point_to_bucket[i];
    if (b == VoxelAssignment::kSkippedPoint)
      continue;
    const auto &n = cloud[i];
    if (!std::isfinite(n.normal_x) || !std::isfinite(n.normal_y) ||
        !std::isfinite(n.normal_z))
      continue;
    auto &acc = buckets[b];
    acc.sx += n.normal_x;
    acc.sy += n.normal_y;
    acc.sz += n.normal_z;
    if (std::isfinite(n.curvature))
      acc.sc += n.curvature;
    ++acc.count;
  }

  auto out = std::make_shared<CloudN>();
  out->reserve(a.bucket_count);
  // Walk every bucket so output rows stay aligned with the primary cloud,
  // emitting a NaN placeholder for buckets with no finite normals.
  for (const auto &acc : buckets) {
    NormalT n;
    if (acc.count == 0) {
      n.normal_x = std::numeric_limits<float>::quiet_NaN();
      n.normal_y = std::numeric_limits<float>::quiet_NaN();
      n.normal_z = std::numeric_limits<float>::quiet_NaN();
      n.curvature = 0.0f;
      out->push_back(n);
      continue;
    }
    const double inv = 1.0 / acc.count;
    const double nx = acc.sx * inv;
    const double ny = acc.sy * inv;
    const double nz = acc.sz * inv;
    const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (norm > 1e-12) {
      const double s = 1.0 / norm;
      n.normal_x = static_cast<float>(nx * s);
      n.normal_y = static_cast<float>(ny * s);
      n.normal_z = static_cast<float>(nz * s);
    } else {
      // Opposing normals cancelled out — fall back to the raw (zero-ish)
      // average rather than guessing a direction.
      n.normal_x = static_cast<float>(nx);
      n.normal_y = static_cast<float>(ny);
      n.normal_z = static_cast<float>(nz);
    }
    n.curvature = static_cast<float>(acc.sc * inv);
    out->push_back(n);
  }
  out->width = static_cast<uint32_t>(out->size());
  out->height = 1;
  out->is_dense = true;
  return out;
}

} // namespace reusex::geometry
