// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Voxel LOD for `GET /api/v1/clouds/{name}/points?max_points=N` (#320).
//
// Paging answers "the first N points of this cloud". A viewport wants "all of
// this cloud, coarsely" — those are different questions, and this file answers
// the second one. The rationale, the alternatives that were rejected, and the
// forward path to a precomputed progressive ordering are in
// docs/gui/binary-points.md § "Level of detail".
//
// Deliberately framework-free, like gui/binary_points.hpp: a ProjectDB and a
// budget in, a page of storage records out. The HTTP layer (gui/api.cpp) turns
// that page into RUXP or JSON with the same encoders a paged read uses.

// ProjectDB is included rather than forward-declared: this header returns the
// nested ProjectDB::CloudPage by value, which needs the complete class.
#include <reusex/core/ProjectDB.hpp>

#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

namespace rux::gui {

/// Points read from storage per streaming step while selecting.
///
/// The cloud is walked in windows of this size rather than read whole, so the
/// scan costs a fixed ~4 MB of buffer (at the 16-byte `PointXYZRGB` stride)
/// regardless of how large the cloud is.
inline constexpr uint64_t kLodStreamChunk = 262144;

/// The outcome of a voxel selection over one whole cloud.
struct LodSelection {
  /// The selected records, in storage layout, ascending by point index.
  ///
  /// `offset` is 0 and `total` is the size of the *whole* cloud, so the page
  /// says exactly what it is: `count` points drawn from all `total` of them.
  /// The points are **not** contiguous in storage, which is why a page built
  /// from this must advertise itself (RUXP `flags` bit 0, JSON `lod: true`).
  reusex::ProjectDB::CloudPage page;

  /// Storage indices of the selected points, ascending and unique.
  ///
  /// Exported so a sibling cloud of the same scan can be sampled at the *same*
  /// points — the only way a `Label` cloud, which carries no positions of its
  /// own, can be zipped against a subsampled geometry cloud (`gather_points`).
  std::vector<uint64_t> indices;

  /// False when the cloud already fit the budget and every point was returned.
  /// A caller must not claim LOD on the wire in that case: the page is then an
  /// ordinary complete, storage-ordered read.
  bool subsampled = false;

  /// Edge length of the final voxel, in cloud units (metres for a scan). 0
  /// when @c subsampled is false.
  double voxel_size = 0.0;

  /// Points skipped because a coordinate was NaN or infinite. A PCL cloud may
  /// legitimately carry them; binning them would put the whole grid at a
  /// nonsense scale, so they are dropped and counted (STANDARDS §5).
  uint64_t skipped_non_finite = 0;
};

/// True when @p point_type carries the xyz positions the grid bins.
///
/// `Normal` and `Label` clouds do not: a LOD of them is only meaningful
/// relative to the geometry cloud they are index-aligned with.
bool lod_supports(std::string_view point_type);

/**
 * @brief Select a spatially representative subset of a whole cloud.
 *
 * One streaming pass over the cloud's stored records, keeping the
 * lowest-indexed point of each occupied voxel. When the kept set would exceed
 * @p max_points the grid is coarsened — the voxel edge doubles, which maps
 * every key onto its parent by an arithmetic shift, so the already-kept set is
 * merged in place rather than re-read. The scan therefore never rewinds, and
 * peak memory is `O(max_points)`, not `O(cloud)`.
 *
 * **Guarantees.** `page.count <= max_points` always. The result is
 * deterministic: the representative of a voxel is its lowest storage index,
 * the output is sorted by that index, and nothing depends on hash order
 * (STANDARDS §6).
 *
 * **Not guaranteed.** The count is not close to @p max_points from below. The
 * grid is dyadic, so one coarsening step drops the count of a surface-like
 * cloud by roughly 4×; a result may legitimately land anywhere in
 * `(max_points/4, max_points]`. Hitting a budget exactly is what a precomputed
 * progressive ordering would buy — see the design note in
 * docs/gui/binary-points.md.
 *
 * @param max_points Budget, >= 1. The caller clamps it to the server maximum.
 * @throws std::runtime_error when @p name is not a stored cloud, when its type
 *         has no positions (`lod_supports`), or when @p max_points is 0.
 */
LodSelection voxel_lod(const reusex::ProjectDB &db, std::string_view name,
                       uint64_t max_points);

/**
 * @brief Read the records of @p name at @p indices, as one page.
 *
 * The sibling half of a LOD: `indices` comes from `voxel_lod()` over a
 * geometry cloud, and this pulls the same point positions out of an
 * index-aligned cloud (`docs/CONTRACTS.md`, STANDARDS §3.2) so the two pages
 * can still be zipped positionally.
 *
 * Streams the cloud in `kLodStreamChunk` windows and picks the wanted records
 * out as they go by, so this is one sequential pass and `O(indices)` memory —
 * not one blob open per index.
 *
 * @param indices Ascending, unique, all `< total` of @p name.
 * @throws std::runtime_error when @p name is not a stored cloud, or when an
 *         index is out of range (which means the two clouds are not
 *         index-aligned and the caller asked for something meaningless).
 */
reusex::ProjectDB::CloudPage
gather_points(const reusex::ProjectDB &db, std::string_view name,
              const std::vector<uint64_t> &indices);

/// Magic number identifying a tile index blob.
inline constexpr uint32_t kTileIndexMagic = 0x52555854u; // 'RUXT'

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

/// Compute the spatial tile index for a morton_10bit_bitrev cloud.
///
/// Single O(N) forward scan over the stored cloud. Assigns each point to tile
/// `sort_key & (K-1)` where `sort_key = reverse_bits30(morton(x,y,z))` and
/// `K = 2^tile_bits`. The bottom tile_bits bits of the 30-bit bit-reversed
/// Morton code are the TOP tile_bits bits of the plain code — the coarsest
/// spatial octant bits — so each tile covers one spatial cell of a
/// (2^(tile_bits/3))^3 grid over the cloud bbox.
///
/// @param tile_bits log2 of the tile count; 6 gives K=64 (a 4×4×4 grid).
/// @returns serialized binary blob suitable for ProjectDB::save_tile_index().
///          Empty when the cloud has fewer than K points or is not found.
std::vector<uint8_t> compute_tile_index(const reusex::ProjectDB &db,
                                        std::string_view name,
                                        uint32_t tile_bits = 6);

/// Deserialize the header + tile array from a tile index blob.
/// @throws std::runtime_error on a corrupt or unknown-version blob.
std::pair<TileIndexHeader, std::vector<TileInfo>>
parse_tile_index(const std::vector<uint8_t> &blob);

/// Return points in spatial tile @p tile_id, optionally as a slice.
///
/// O(N) scan: recomputes each point's sort key from its coordinates and the
/// bbox stored in @p hdr, then collects points whose `sort_key & (K-1) == k`.
/// Within a tile the points appear in ascending sort_key order, which is itself
/// a bit-reversed Morton ordering of the sub-octant — so any prefix of a tile's
/// points is a spatially stratified sample of that tile.
///
/// @param skip  Skip this many matching points before collecting. 0 = start
///              from the first point of the tile.
/// @param limit Collect at most this many points. 0 = no limit (full tile).
/// @throws std::runtime_error when @p tile_id >= K.
reusex::ProjectDB::CloudPage
gather_tile_points(const reusex::ProjectDB &db, std::string_view name,
                   const TileIndexHeader &hdr, uint32_t tile_id,
                   uint64_t skip = 0, uint64_t limit = 0);

/// Return sorted storage indices of points in spatial tile @p tile_id.
///
/// Same O(N) scan as gather_tile_points, but yields only the indices — not the
/// records — so a position-free sibling cloud (e.g. a Label cloud) can be
/// gathered at the same points via gather_points(). The skip/limit contract is
/// identical to gather_tile_points, so the two always produce index-aligned
/// results when called with the same arguments.
///
/// @param skip  Skip this many matching points before collecting.
/// @param limit Collect at most this many indices. 0 = no limit.
/// @throws std::runtime_error when @p tile_id >= K or the cloud has an
///         unusable point_step.
std::vector<uint64_t> gather_tile_indices(const reusex::ProjectDB &db,
                                          std::string_view name,
                                          const TileIndexHeader &hdr,
                                          uint32_t tile_id, uint64_t skip = 0,
                                          uint64_t limit = 0);

} // namespace rux::gui
