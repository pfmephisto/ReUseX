// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Evidence for a scanned instance: where it is (centroid), which sensor
// frames see it (photo counts for every survey part in one pass), and which
// 360° panoramas were taken near it (with the equirect pixel it lands on).
//
// Lives in `core` next to `frame_visibility` for the same reason that does:
// the GUI read endpoints (`rux_gui_lib`) link `reusex_core` only. Core does
// not link `reusex_geometry_common`, so the equirect bearing convention of
// `geometry/EquirectProjection.hpp` is restated here (`equirect_uv`) and
// pinned by a test against the frontend's `bearingToUv`.

#include "reusex/core/frame_visibility.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// Mean position of one instance's points.
struct InstanceCentroid {
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  std::size_t point_count = 0; ///< Finite points averaged.
};

/**
 * @brief Centroid of every instance id in an instance-label cloud.
 *
 * The label cloud stores labels only; positions come from @p positions_cloud,
 * index-aligned point for point (the `instances` stage's contract). Label `0`
 * (unlabeled, STANDARDS §3) is skipped, as are non-finite points. One pass
 * over the clouds, whatever the number of instances.
 *
 * @throws std::invalid_argument when either cloud is not in the project.
 * @throws std::runtime_error    when the clouds are not index-aligned.
 */
std::map<std::uint32_t, InstanceCentroid>
instance_centroids(const ProjectDB &db, std::string_view label_cloud,
                   std::string_view positions_cloud = "cloud");

/// The photo evidence of one survey part.
struct PartPhotos {
  std::size_t count = 0;            ///< Sensor frames that see the centroid.
  std::optional<int> best_frame_id; ///< Most central of them, if any.
};

/**
 * @brief Photo count and best frame for every instance-backed survey part.
 *
 * Loads each instance cloud the parts refer to (and the positions cloud)
 * once, takes every instance's centroid and ranks the posed sensor frames for
 * all centroids together with `visible_frames_batch`. The result for a part
 * is exactly what `visible_frames(db, centroid, query)` would give for it —
 * the count is its size and the best frame its first entry.
 *
 * Parts without an instance link, or whose instance has no points, are
 * absent from the map. A cloud that cannot be used (missing positions,
 * misaligned) is skipped with a `warn` naming it (STANDARDS §5); the call
 * does not throw for it, because one stale cloud should not blank the table.
 *
 * @return Keyed on the part code.
 */
std::map<std::string, PartPhotos>
survey_part_photos(const ProjectDB &db, const VisibilityQuery &query = {},
                   std::string_view positions_cloud = "cloud");

/// Tunables for `panoramas_for_point()`.
struct PanoramaQuery {
  /// Drop panoramas whose centre is farther than this from the point, in
  /// metres. `0` keeps every placeable panorama.
  double max_distance = 15.0;
};

/// Where a panorama's orientation comes from (mirrors the frontend's
/// `PanoramaHeading` in `viewport/panorama.ts`).
enum class PanoramaHeading {
  resected, ///< `rux align 360` pose: position and heading measured.
  levelled, ///< Matched frame's position, heading unknown (LEVELLED basis).
};

/// One panorama near a world point.
struct PanoramaSighting {
  int panorama_id = 0;
  int node_id = -1;    ///< Matched sensor frame, -1 if none.
  double distance = 0; ///< Panorama centre to the point, metres.
  double u = 0;        ///< Equirect column of the point, 0..1 (left to right).
  double v = 0;        ///< Equirect row of the point, 0..1 (0 = north pole).
  PanoramaHeading heading = PanoramaHeading::resected;
};

/**
 * @brief Normalised equirect coordinates of a bearing in the panorama frame.
 *
 * The panorama frame is the pipeline's optical convention (+x right, +y down,
 * +z forward); `theta = atan2(x, z)`, `phi = asin(-y)`,
 * `u = (theta + pi) / 2pi`, `v = (pi/2 - phi) / pi`. Identical to
 * `geometry::bearing_to_pixel` divided by the image size and to the frontend's
 * `bearingToUv`. The bearing need not be normalised; a zero bearing maps to
 * the image centre.
 */
Eigen::Vector2d equirect_uv(const Eigen::Vector3d &bearing);

/**
 * @brief Panoramas that can be placed in the world, nearest first, with where
 * @p world_point appears in each.
 *
 * Placement follows the frontend's `resolvePlacement`: a resected pose
 * (`has_pose`) wins; otherwise the matched sensor frame's position with the
 * levelled, heading-free basis (+z forward on world +X, +y down on world -Z).
 * Panoramas with neither are skipped. For a levelled panorama `u` is relative
 * to that arbitrary heading, so it is only as good as the heading (the
 * `heading` field says which). Sorted by distance, then id (STANDARDS §6).
 *
 * When the project has panoramas but none can be placed, a `warn` with the
 * counts is logged (STANDARDS §5).
 */
std::vector<PanoramaSighting>
panoramas_for_point(const ProjectDB &db, const Eigen::Vector3d &world_point,
                    const PanoramaQuery &query = {});

} // namespace reusex::core
