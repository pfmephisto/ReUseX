// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <filesystem>
#include <string>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

/// Parsed contents of an ARKitScenes `.pincam` intrinsics line.
struct ArkitPincam {
  double width = 0, height = 0;
  double fx = 0, fy = 0, cx = 0, cy = 0;
};

/// Parse a single ARKitScenes `.pincam` line: `width height fx fy cx cy`
/// (six whitespace-separated doubles).
/// @throws std::runtime_error if fewer than six numbers can be parsed.
ArkitPincam parse_pincam(const std::string &line);

/// Convert one ARKitScenes trajectory pose to an optical-to-world 4x4 matrix
/// (row-major, 16 doubles).
///
/// (rx,ry,rz) is a Rodrigues rotation vector, (tx,ty,tz) a translation. Per
/// Apple's `traj_string_to_matrix`, the assembled extrinsic E=[R|t] is
/// *world->camera*, so camera->world = E.inverse(). ARKitScenes' camera frame
/// is already the OpenCV optical convention (x-right, y-down, z-forward) — the
/// same frame reconstruct back-projects into — so c2w is returned directly as
/// the optical-to-world pose, with no axis flip (an inserted flip empirically
/// fans the reconstructed walls into a spiral).
std::array<double, 16> arkit_traj_to_optical_world(double rx, double ry,
                                                   double rz, double tx,
                                                   double ty, double tz);

/// Import an ARKitScenes scene (lowres iPad-LiDAR streams) into a ReUseX
/// project.
///
/// ARKitScenes (Apple/Dehghan et al., NeurIPS 2021, research-only license)
/// provides room-scale RGB-D sequences captured with the same iPad-LiDAR
/// sensor class as ReUseX's own scans — an external benchmark for
/// reconstruction quality (issue #224).
///
/// Expects the scene *frames* directory (the one directly containing
/// `lowres_wide.traj`); if @p scene_dir does not contain it, one level of
/// nesting is searched (e.g. `<video_id>/` or `<video_id>_frames/`). The
/// frames dir holds the per-stream folders: `lowres_wide/` (RGB PNG),
/// `lowres_depth/` (CV_16UC1 mm PNG), `confidence/` (CV_8UC1 {0,1,2}) and
/// `lowres_wide_intrinsics/` (`<ts>.pincam`), plus `lowres_wide.traj`.
///
/// Poses in `lowres_wide.traj` are world->camera in ARKit axes and are
/// converted to optical-to-world before storage (see
/// arkit_traj_to_optical_world). Depth is already CV_16UC1 millimeters and is
/// passed through unchanged.
///
/// @param db         Open project database (frames are written to it).
/// @param scene_dir  Scene directory (frames dir or its parent).
/// @returns          Number of sensor frames imported.
/// @throws std::runtime_error on missing/malformed inputs or zero frames.
std::size_t import_arkitscenes(ProjectDB &db,
                               const std::filesystem::path &scene_dir);

} // namespace reusex::io
