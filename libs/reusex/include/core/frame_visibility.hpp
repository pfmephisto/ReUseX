// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Eigen/Core>

#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// One frame in which a queried world point is visible, with the geometry of
/// its projection. Produced by `visible_frames()`.
struct FrameVisibility {
  int frame_id = 0; ///< `node_id` of the sensor frame.
  double u = 0.0;   ///< Projected pixel column (0 = left edge).
  double v = 0.0;   ///< Projected pixel row (0 = top edge).
  double depth =
      0.0; ///< Point's depth in the camera optical frame, metres (> 0).
  /// Normalised distance of the projection from the principal point:
  /// `0` at the principal point (most central), `~1` at an image corner.
  /// Smaller is more central, hence a better source image. This is the sort
  /// key: results come back ascending in `centrality`.
  double centrality = 0.0;
};

/// Tunables for `visible_frames()`. Defaults keep the frustum + in-front test
/// alone; a positive `max_depth` additionally rejects points beyond a sensor's
/// useful range.
struct VisibilityQuery {
  /// Reject frames where the point is farther than this many metres from the
  /// camera. `0` (the default) means no far limit — every in-front,
  /// in-bounds frame is kept regardless of range.
  double max_depth = 0.0;
  /// Shrink the accepted image region by this many pixels on every edge, so a
  /// point grazing the border is not counted as "visible". `0` uses the full
  /// image bounds.
  double margin_px = 0.0;
};

/**
 * @brief Rank the posed sensor frames by how centrally a world point projects.
 *
 * For every sensor frame that carries a usable stored pose (gated on
 * `ProjectDB::has_sensor_frame_pose`, per #336) and valid pinhole intrinsics,
 * the point is transformed into the camera optical frame using
 * `world -> camera = (pose * local_transform)^-1` — the same convention
 * `reusex::vision::project()` uses. A frame keeps the point when it is
 * **in front of the camera** (`z > 0`) and its pinhole projection **lands
 * within the image bounds** (a frustum test). The centrality score is the
 * projection's normalised distance from the principal point.
 *
 * The result is sorted best-first: ascending `centrality`, ties broken by
 * nearer `depth`, then by `frame_id` — so the order is fully deterministic
 * (STANDARDS §6). Work is bounded by the number of sensor frames.
 *
 * Frames without a usable pose or with degenerate intrinsics are skipped, not
 * fatal: a project may be partway through import. When *no* frame carries a
 * pose the returned vector is empty and a `warn` is logged with the counts
 * (STANDARDS §5), rather than failing outright.
 *
 * Lives in `core` (not `vision`) deliberately: it depends only on `ProjectDB`
 * and `SensorIntrinsics` plus Eigen, and the GUI read endpoint that drives it
 * (`rux_gui_lib`) links `reusex_core` but NOT `reusex_vision`, which stays out
 * of the light test binary (#268). There is no PCL, CGAL, RTABMap or ML here,
 * so core's header-hygiene rule (STANDARDS §1) is respected.
 *
 * @param db          Project to read sensor frames from.
 * @param world_point The 3D point in world coordinates.
 * @param query       Optional visibility tunables.
 * @return Visible frames, most central first. Empty when the point is seen by
 *         no posed frame.
 */
std::vector<FrameVisibility> visible_frames(const ProjectDB &db,
                                            const Eigen::Vector3d &world_point,
                                            const VisibilityQuery &query = {});

} // namespace reusex::core
