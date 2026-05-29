// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <filesystem>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

/// Options for COLMAP sparse model export.
struct ColmapExportOptions {
  /// Sub-sample the LiDAR cloud into points3D to seed PatchMatch + 3DGS init.
  /// Set to false to write an empty points3D file (valid COLMAP input).
  bool include_lidar_points = true;

  /// Approximate cap on the number of seed points (uniform stride).
  std::size_t max_lidar_points = 100'000;

  /// Name of the point cloud in ProjectDB to use as seed (typically "cloud").
  std::string lidar_cloud_name = "cloud";

  /// JPEG quality for the on-disk image dump (1..100). Used only when the
  /// stored frame is re-encoded; if the JPEG blob can be written verbatim we
  /// prefer that and ignore this knob.
  int jpeg_quality = 95;
};

/// Export a COLMAP sparse model directory from a ProjectDB.
///
/// Produces this layout under @p out_dir :
///
///   <out_dir>/
///     images/<frame_id>.jpg          # one per sensor_frame
///     sparse/0/cameras.txt           # PINHOLE camera per unique intrinsics
///     sparse/0/images.txt            # one entry per sensor_frame (T_cw)
///     sparse/0/points3D.txt          # optionally seeded from LiDAR cloud
///
/// Notes
/// -----
/// * COLMAP requires camera-frame poses (`T_cw`, world -> camera). ProjectDB
///   stores `pose = T_wb` (sensor base in world) and intrinsics
///   `local_transform = T_bc` (camera optical in sensor base). This function
///   composes `T_wc = pose * local_transform` and inverts to produce
///   `T_cw = (T_wc)^{-1}`, then writes `(qw, qx, qy, qz, tx, ty, tz)` as
///   COLMAP expects.
/// * Intrinsics are stored as JSON in ProjectDB with no distortion model, so
///   the PINHOLE camera model (fx, fy, cx, cy) is used directly. Images do
///   not need to be undistorted by `colmap image_undistorter` — but the dense
///   workspace step still requires running it once to set up the layout.
/// * Sensor frames missing color/depth are silently skipped (mirroring
///   `reconstruct_point_clouds`).
///
/// @param db      Source project database (read-only access).
/// @param out_dir Output directory; created if missing. Existing files are
///                overwritten.
/// @param opt     Export options.
/// @throws std::runtime_error on I/O failure or if @p db has no sensor
///                  frames.
void export_colmap_scene(const ProjectDB &db,
                         const std::filesystem::path &out_dir,
                         const ColmapExportOptions &opt = {});

} // namespace reusex::io
