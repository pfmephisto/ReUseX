// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Equirectangular (360 panorama) <-> perspective (pinhole) reprojection.
//
// The codebase has no spherical-image geometry outside the VTK viewer, which
// only *samples* an equirect texture onto a sphere for display
// (apps/rux/src/view/panorama_handler.cpp). This utility provides the CPU
// geometry shared by two features:
//   * SAM3-on-360  : slice an equirect into pinhole tiles the segmenter can
//                    consume at its native resolution, then stitch the per-tile
//                    label maps back into one equirect label image.
//   * 360 alignment: render a virtual pinhole view of the panorama toward a
//                    nearby sensor frame so it can be ORB-matched, and map
//                    slice pixels back to panorama bearings for pose resection.
//
// Panorama frame convention (right-handed, camera-style y-down):
//   longitude theta in [-pi, pi]  -> column u in [0, W)
//   latitude  phi   in [-pi/2,pi/2] -> row    v in [0, H)   (v=0 = north pole)
//   bearing d(theta,phi) = ( sin(theta) cos(phi),
//                           -sin(phi),
//                            cos(theta) cos(phi) )
// so d(0,0) = +Z (forward), +X points right (theta>0), +Y points down. This is
// the same optical convention (x right, y down, z forward) the rest of the
// pipeline uses for sensor-frame cameras, so a resected panorama pose composes
// directly with sensor-frame poses.

#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <vector>

namespace reusex::geometry {

/// A virtual pinhole view rendered out of an equirectangular panorama.
struct PerspectiveView {
  cv::Mat image;     ///< rendered pinhole image (same type as source)
  Eigen::Matrix3d K; ///< intrinsics of the virtual pinhole camera
  Eigen::Matrix3d
      R_pano_from_view;   ///< maps view-camera rays into panorama frame
  double fov_deg = 90.0;  ///< horizontal field of view used to build K
  double yaw_deg = 0.0;   ///< view centre longitude (deg)
  double pitch_deg = 0.0; ///< view centre latitude (deg)
};

/// Equirect pixel (u,v) -> unit bearing in the panorama frame.
Eigen::Vector3d pixel_to_bearing(const cv::Size &equirect, double u, double v);

/// Unit bearing (need not be normalised) in panorama frame -> equirect pixel
/// (u,v). u wraps into [0,W); v is clamped to [0,H).
Eigen::Vector2d bearing_to_pixel(const cv::Size &equirect,
                                 const Eigen::Vector3d &bearing);

/// Render a virtual pinhole view centred at (yaw,pitch) with the given
/// horizontal FOV. Longitude wraps (BORDER_WRAP); the horizon stays level (no
/// roll). @p interp is an OpenCV interpolation flag (default INTER_LINEAR).
PerspectiveView extract_perspective(const cv::Mat &equirect, double yaw_deg,
                                    double pitch_deg, double fov_deg, int out_w,
                                    int out_h, int interp = 1 /*INTER_LINEAR*/);

/// Render a virtual pinhole view with an explicit view->panorama rotation and
/// intrinsics. Used to reproject a specific camera's view out of the panorama
/// (e.g. render what a sensor frame should see, given the aligned pose).
PerspectiveView extract_perspective(const cv::Mat &equirect,
                                    const Eigen::Matrix3d &R_pano_from_view,
                                    const Eigen::Matrix3d &K, int out_w,
                                    int out_h, int interp = 1);

/// Tile the whole sphere as the six faces of a cube (90 deg FOV each). Poles
/// are covered by the up/down faces, avoiding the equirect's polar stretch.
std::vector<PerspectiveView> cube_faces(const cv::Mat &equirect, int face_size,
                                        int interp = 1);

/// Tile the sphere as @p n_yaw evenly spaced views around the equator plus one
/// up and one down view, each with @p fov_deg horizontal FOV and @p tile pixels
/// square. Overlap between neighbours (fov_deg > 360/n_yaw) hides seams; the
/// larger overlap (vs cube faces) is what the ORB matcher wants so a feature is
/// never split across a tile boundary.
std::vector<PerspectiveView> overlapping_views(const cv::Mat &equirect,
                                               int n_yaw, double fov_deg,
                                               int tile, int interp = 1);

/// Stitch per-tile label maps (CV_32S, -1 = background) back into a single
/// equirect label map of size @p out. For each equirect pixel the covering tile
/// whose optical axis is *closest* to that bearing wins (most central sample,
/// least perspective distortion). Tiles must align 1:1 with @p views.
/// Background (-1) samples never override a real label from another tile.
cv::Mat stitch_labels_to_equirect(const cv::Size &out,
                                  const std::vector<PerspectiveView> &views,
                                  const std::vector<cv::Mat> &tile_labels);

} // namespace reusex::geometry
