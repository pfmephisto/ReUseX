// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/core/ProjectDB.hpp"

#include <string>
#include <vector>

namespace reusex::geometry {

/// Options controlling a synchronized project-cloud downsample.
///
/// The default behaviour keeps every derived product in sync: all point
/// clouds that share the primary cloud's point count (its index-aligned
/// siblings — normals, labels, planes, rooms, instances, …) are downsampled
/// with the same voxel assignment (docs/STANDARDS.md §3.2, "parallel clouds
/// move together"). Desynchronizing a project is only possible by explicitly
/// opting out.
struct SyncDownsampleOptions {
  /// Voxel leaf size in meters. Must be positive.
  float leaf_size = 0.0f;

  /// Output name for the primary cloud. Empty means overwrite `primary`.
  std::string primary_output;

  /// When true, only the primary cloud is downsampled. This desynchronizes
  /// any index-aligned siblings, so it is refused (throws) unless
  /// `force_desync` is also set. See `only_primary` handling below.
  bool only_primary = false;

  /// Required alongside `only_primary` to actually perform a desynchronizing
  /// downsample. Without it, `sync_downsample` throws listing the siblings
  /// that would be invalidated (docs/STANDARDS.md §5, loud failure).
  bool force_desync = false;
};

/// Outcome of a synchronized downsample, for logging / reporting.
struct SyncDownsampleResult {
  /// Number of output points in the downsampled primary cloud.
  size_t output_points = 0;

  /// Names of every cloud written (primary first, then siblings). When an
  /// output name differs from the input, the output name is recorded.
  std::vector<std::string> written_clouds;

  /// Names of sibling clouds that were intentionally left untouched because
  /// `only_primary`/`force_desync` was requested (now stale).
  std::vector<std::string> desynced_clouds;
};

/// Downsample `primary` and — unless `only_primary` is set — all of its
/// index-aligned sibling clouds in `db`, using one shared voxel assignment so
/// the outputs stay row-aligned.
///
/// Sibling discovery: every point cloud whose point count equals the primary
/// cloud's point count (excluding the primary itself) is treated as a
/// sibling. Supported sibling types are downsampled with the matching
/// overload:
///   - PointXYZRGB → centroid
///   - Normal      → averaged + renormalized
///   - Label       → per-bucket majority vote (deterministic tie-break)
/// Label definitions attached to a Label sibling are carried over to its
/// output. A sibling of an unsupported type causes the whole operation to
/// throw (no partial, silently-desynchronized write).
///
/// When `only_primary` is true and index-aligned siblings exist, the call
/// throws unless `force_desync` is also set; the exception message names
/// every sibling that would be invalidated.
///
/// The operation is recorded in the pipeline log (start/end), naming the
/// affected clouds, so staleness is auditable.
///
/// Throws std::invalid_argument if the primary is missing, not PointXYZRGB,
/// empty, or the leaf size is non-positive; std::runtime_error on unsupported
/// sibling types, size mismatches, or a refused desync.
SyncDownsampleResult sync_downsample(ProjectDB &db, const std::string &primary,
                                     const SyncDownsampleOptions &opts);

} // namespace reusex::geometry
