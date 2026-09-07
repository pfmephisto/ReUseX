// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

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

/// One trajectory sample: a device-relative timestamp and the optical-to-world
/// (camera->world) pose at that instant, row-major 4x4.
struct ArkitPoseSample {
  double ts = 0;
  std::array<double, 16> pose{};
};

/// Trajectory as a timestamp-ascending sequence of camera->world samples.
/// (`load_arkit_trajectory` guarantees the ordering; `interpolate_pose`
/// requires it.)
using ArkitTrajectory = std::vector<ArkitPoseSample>;

/// Largest bracketing interval `interpolate_pose` will interpolate across.
/// The `lowres_wide.traj` stream is ~10 Hz (0.1 s spacing); a gap an order of
/// magnitude larger means ARKit lost tracking, and interpolating across it
/// would invent a pose, so such frames are rejected instead.
constexpr double kArkitMaxPoseGap = 0.5; // seconds

/// Parse the trajectory file into timestamp-sorted camera->world poses.
///
/// Each line is `ts rx ry rz tx ty tz` (world->camera; see
/// arkit_traj_to_optical_world); unparseable and blank lines are skipped.
/// @throws std::runtime_error if the file yields no poses.
ArkitTrajectory load_arkit_trajectory(const std::filesystem::path &traj_path);

/// Interpolate the camera->world pose at @p ts from the bracketing trajectory
/// samples: SLERP on the rotation, linear interpolation on the camera position.
///
/// Interpolation is done on the *camera->world* pose (rather than the raw
/// world->camera extrinsic) so the lerped translation is the camera's world
/// position, which moves smoothly; lerping the world->camera translation
/// `-R·p` would mix in the rotation and bend the camera path.
///
/// The depth stream (~60 Hz) is six times denser than the trajectory (~10 Hz),
/// so nearest-pose matching would hand most frames a pose up to ~17 ms stale
/// (tens of millimetres of lateral error at typical hand-held speeds);
/// interpolating removes that temporal error.
///
/// @param traj     Timestamp-ascending trajectory (see load_arkit_trajectory).
/// @param ts       Query timestamp, in the trajectory's own time base.
/// @param max_gap  Reject if the bracketing samples are farther apart than
///                 this (tracking loss).
/// @returns The interpolated row-major 4x4 pose, or `std::nullopt` if @p ts
///          lies outside the trajectory's span or the bracketing gap exceeds
///          @p max_gap. No extrapolation is performed.
std::optional<std::array<double, 16>>
interpolate_pose(const ArkitTrajectory &traj, double ts,
                 double max_gap = kArkitMaxPoseGap);

/// Extract the timestamp encoded in an ARKitScenes stream filename such as
/// `41069021_452.395.png`: drop the extension, then take everything after the
/// last underscore (a bare `std::stod` would stop at the video-id prefix).
/// @returns false if that substring is not a complete number.
bool parse_frame_timestamp(const std::filesystem::path &file, double &ts);

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
/// arkit_traj_to_optical_world). The trajectory is ~10 Hz while depth is
/// ~60 Hz, so one frame in six lands on a trajectory sample: each frame's pose
/// is *interpolated* to its own timestamp (see interpolate_pose), and frames
/// outside the trajectory's time span are skipped. Depth is already CV_16UC1
/// millimeters and is passed through unchanged.
///
/// @note Timestamps (in the stream filenames, the `.traj`, and the ones stored
/// on each sensor frame) are ARKit **device-relative** seconds — uptime since
/// boot, not a Unix epoch — so they are only comparable within one scene.
///
/// @param db         Open project database (frames are written to it).
/// @param scene_dir  Scene directory (frames dir or its parent).
/// @returns          Number of sensor frames imported.
/// @throws std::runtime_error on missing/malformed inputs or zero frames.
std::size_t import_arkitscenes(ProjectDB &db,
                               const std::filesystem::path &scene_dir);

} // namespace reusex::io
