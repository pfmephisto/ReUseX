// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Spatial tile index for Morton-ordered clouds (#395, #396).
//
// The index is a blob stored beside a `morton_10bit_bitrev` cloud with
// ProjectDB::save_tile_index(). The web API (ruxd's api/point_lod) reads it to
// stream a cloud tile by tile. It is BUILT here, by the clouds stage, so every
// path that runs that stage — `rux create clouds`, the GUI's POST /jobs, any
// pipeline::run_stage caller — leaves the same index behind.

#include <reusex/core/ProjectDB.hpp>

#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

namespace reusex::pipeline {

/// Magic number identifying a tile index blob.
inline constexpr uint32_t kTileIndexMagic = 0x52555854u; // 'RUXT'

/// The storage order a cloud must have for a tile index to make sense.
inline constexpr std::string_view kMortonStorageOrder = "morton_10bit_bitrev";

/// Header of a serialized tile index blob (version 2).
///
/// Tile assignment uses the BOTTOM tile_bits bits of the 30-bit bit-reversed
/// Morton sort key, which equal the TOP tile_bits bits of the plain Morton
/// code. These bits identify the coarsest spatial octant, so each tile covers
/// one spatial cell of a (2^(tile_bits/3))³ grid over the cloud's bbox.
/// The cloud bbox (lo, hi) stored here is required to recompute sort keys at
/// serve time without re-reading all point data.
struct TileIndexHeader {
  uint32_t magic;
  uint32_t version;   ///< 2.
  uint32_t tile_bits; ///< log2(K), so K = 2^tile_bits tiles.
  uint32_t point_count;
  float lo[3]; ///< Cloud bbox minimum (world coords).
  float hi[3]; ///< Cloud bbox maximum (world coords).
};

/// Per-tile AABB and point count within the tile.
struct TileInfo {
  float min[3];
  float max[3];
  uint32_t count;
};

/// 30-bit bit-reversed Morton sort key of (x, y, z) quantised to 10 bits per
/// axis over the box starting at @p lo with extent @p range (each > 0).
/// Identical to the key reconstruct.cpp sorts a cloud by.
uint32_t morton_tile_sort_key(float x, float y, float z, const float lo[3],
                              const float range[3]);

/// Compute the spatial tile index for a morton_10bit_bitrev cloud.
///
/// Two O(N) forward scans over the stored cloud (bbox, then assignment).
/// Assigns each point to tile `sort_key & (K-1)` where `K = 2^tile_bits`.
///
/// @param tile_bits log2 of the tile count; 6 gives K=64 (a 4×4×4 grid).
/// @returns serialized binary blob suitable for ProjectDB::save_tile_index().
///          Empty when the cloud has fewer than K points or is not found.
std::vector<uint8_t> compute_tile_index(const ProjectDB &db,
                                        std::string_view name,
                                        uint32_t tile_bits = 6);

/// Deserialize the header + tile array from a tile index blob.
/// @throws std::runtime_error on a corrupt or unknown-version blob.
std::pair<TileIndexHeader, std::vector<TileInfo>>
parse_tile_index(const std::vector<uint8_t> &blob);

/// Build and store the tile index of cloud @p name when it is Morton-ordered.
/// @returns the stored blob's size in bytes; 0 when the cloud is absent, not
///          Morton-ordered, or too small for an index (nothing is stored).
std::size_t build_tile_index(ProjectDB &db, std::string_view name);

} // namespace reusex::pipeline
