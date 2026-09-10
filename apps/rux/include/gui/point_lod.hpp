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

} // namespace rux::gui
