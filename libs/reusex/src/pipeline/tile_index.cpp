// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Moved from ruxd's api/point_lod.cpp so the clouds stage can build the index
// (and `rux` need not link the web API to do it).

#include "pipeline/tile_index.hpp"

#include <reusex/core/logging.hpp>
#include <reusex/geometry/morton.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace reusex::pipeline {
namespace {

/// Widest stored record handled (`PointXYZRGB` is 16 bytes).
constexpr size_t kMaxRecord = 16;
/// Points read per window while scanning a cloud.
constexpr uint64_t kLodStreamChunk = 262144;

float read_f32(const uint8_t *at) {
  float value = 0.0F;
  std::memcpy(&value, at, sizeof(value));
  return value;
}

bool finite_position(const uint8_t *record) {
  return std::isfinite(read_f32(record)) &&
         std::isfinite(read_f32(record + 4)) &&
         std::isfinite(read_f32(record + 8));
}

} // namespace

// 30-bit bit-reversed Morton sort key for (x,y,z) quantised in [lo, hi].
// Uses the shared helpers from <reusex/geometry/morton.hpp> so the computation
// stays identical to reconstruct.cpp's sort and can never silently diverge.
uint32_t morton_tile_sort_key(float x, float y, float z, const float lo[3],
                              const float range[3]) {
  namespace gm = reusex::geometry;
  constexpr uint32_t kMax10 = 1023u;
  const auto clamp01 = [](float v) {
    return v < 0.f ? 0.f : (v > 1.f ? 1.f : v);
  };
  const uint32_t xi =
      static_cast<uint32_t>(clamp01((x - lo[0]) / range[0]) * kMax10);
  const uint32_t yi =
      static_cast<uint32_t>(clamp01((y - lo[1]) / range[1]) * kMax10);
  const uint32_t zi =
      static_cast<uint32_t>(clamp01((z - lo[2]) / range[2]) * kMax10);
  return gm::morton_reverse_bits30(gm::morton_expand3(xi) |
                                   (gm::morton_expand3(yi) << 1u) |
                                   (gm::morton_expand3(zi) << 2u));
}

std::vector<uint8_t> compute_tile_index(const ProjectDB &db,
                                        std::string_view name,
                                        uint32_t tile_bits) {
  const auto probe = db.point_cloud_page(name, 0, 0);
  const uint64_t total = probe.total;
  const size_t step = probe.point_step;

  if (step == 0 || step > kMaxRecord || total == 0)
    return {};

  const uint32_t K = 1u << tile_bits;
  if (total < K)
    return {}; // cloud too small for K distinct tiles

  // Pass 1: compute cloud bbox (needed for sort_key normalization).
  float lo[3] = {std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity()};
  float hi[3] = {-std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity()};
  for (uint64_t start = 0; start < total; start += kLodStreamChunk) {
    const auto window = db.point_cloud_page(name, start, kLodStreamChunk);
    if (window.count == 0)
      break;
    for (uint64_t i = 0; i < window.count; ++i) {
      const uint8_t *rec = window.data.data() + i * step;
      if (!finite_position(rec))
        continue;
      for (int ax = 0; ax < 3; ++ax) {
        const float v = read_f32(rec + 4 * ax);
        if (v < lo[ax])
          lo[ax] = v;
        if (v > hi[ax])
          hi[ax] = v;
      }
    }
  }
  // Avoid division by zero for degenerate clouds.
  float rng[3];
  for (int ax = 0; ax < 3; ++ax)
    rng[ax] = (hi[ax] - lo[ax]) > 1e-9f ? (hi[ax] - lo[ax]) : 1e-9f;

  // Pass 2: assign each point to tile (sort_key & (K-1)), update tight AABB.
  // The bottom tile_bits bits of the 30-bit bit-reversed Morton sort_key are
  // the TOP tile_bits bits of the plain Morton code — the coarsest spatial
  // octant bits. Points sharing these bits occupy one spatial cell.
  std::vector<TileInfo> tiles(K);
  for (auto &t : tiles) {
    t.min[0] = t.min[1] = t.min[2] = std::numeric_limits<float>::infinity();
    t.max[0] = t.max[1] = t.max[2] = -std::numeric_limits<float>::infinity();
    t.count = 0;
  }

  const uint32_t mask = K - 1;
  for (uint64_t start = 0; start < total; start += kLodStreamChunk) {
    const auto window = db.point_cloud_page(name, start, kLodStreamChunk);
    if (window.count == 0)
      break;

    for (uint64_t i = 0; i < window.count; ++i) {
      const uint8_t *rec = window.data.data() + i * step;
      if (!finite_position(rec))
        continue;

      const float x = read_f32(rec);
      const float y = read_f32(rec + 4);
      const float z = read_f32(rec + 8);
      const uint32_t sk = morton_tile_sort_key(x, y, z, lo, rng);
      const uint32_t tile = sk & mask;
      TileInfo &t = tiles[tile];
      t.count += 1;
      if (x < t.min[0])
        t.min[0] = x;
      if (y < t.min[1])
        t.min[1] = y;
      if (z < t.min[2])
        t.min[2] = z;
      if (x > t.max[0])
        t.max[0] = x;
      if (y > t.max[1])
        t.max[1] = y;
      if (z > t.max[2])
        t.max[2] = z;
    }
  }

  // Empty tiles: degenerate bbox at origin.
  for (auto &t : tiles) {
    if (t.count == 0) {
      t.min[0] = t.min[1] = t.min[2] = 0.0f;
      t.max[0] = t.max[1] = t.max[2] = 0.0f;
    }
  }

  // Serialize: fixed-size header (v2 includes bbox) + per-tile data.
  TileIndexHeader hdr{};
  hdr.magic = kTileIndexMagic;
  hdr.version = 2u;
  hdr.tile_bits = tile_bits;
  hdr.point_count =
      static_cast<uint32_t>(total > UINT32_MAX ? UINT32_MAX : total);
  for (int ax = 0; ax < 3; ++ax) {
    hdr.lo[ax] = lo[ax];
    hdr.hi[ax] = hi[ax];
  }
  const size_t blob_size = sizeof(TileIndexHeader) + K * sizeof(TileInfo);
  std::vector<uint8_t> blob(blob_size);
  std::memcpy(blob.data(), &hdr, sizeof(hdr));
  std::memcpy(blob.data() + sizeof(hdr), tiles.data(), K * sizeof(TileInfo));
  return blob;
}

std::pair<TileIndexHeader, std::vector<TileInfo>>
parse_tile_index(const std::vector<uint8_t> &blob) {
  if (blob.size() < sizeof(TileIndexHeader))
    throw std::runtime_error("tile index blob too short");

  TileIndexHeader hdr;
  std::memcpy(&hdr, blob.data(), sizeof(hdr));
  if (hdr.magic != kTileIndexMagic)
    throw std::runtime_error("tile index blob has wrong magic");
  if (hdr.version != 2u)
    throw std::runtime_error(
        "unsupported tile index version " + std::to_string(hdr.version) +
        " (expected 2; re-run rux create clouds to rebuild)");
  if (hdr.tile_bits == 0 || hdr.tile_bits > 20)
    throw std::runtime_error("tile_bits out of range");

  const uint32_t K = 1u << hdr.tile_bits;
  const size_t expected = sizeof(TileIndexHeader) + K * sizeof(TileInfo);
  if (blob.size() != expected)
    throw std::runtime_error("tile index blob size mismatch");

  std::vector<TileInfo> tiles(K);
  std::memcpy(tiles.data(), blob.data() + sizeof(hdr), K * sizeof(TileInfo));
  return {hdr, std::move(tiles)};
}

std::size_t build_tile_index(ProjectDB &db, std::string_view name) {
  if (!db.has_point_cloud(name) ||
      db.point_cloud_storage_order(name) != kMortonStorageOrder)
    return 0;
  const auto blob = compute_tile_index(db, name);
  if (blob.empty())
    return 0;
  db.save_tile_index(name, blob);
  return blob.size();
}

} // namespace reusex::pipeline
