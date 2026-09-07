// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/EquirectProjection.hpp"

#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace reusex::geometry {

namespace {

constexpr double kPi = 3.14159265358979323846;

// View-camera basis expressed in the panorama frame, for a view whose optical
// axis points at (yaw,pitch). Columns are the view x(right), y(down),
// z(forward) axes. Horizon kept level (roll-free) by referencing panorama up =
// (0,-1,0).
Eigen::Matrix3d view_basis(double yaw_rad, double pitch_rad) {
  const Eigen::Vector3d f(std::sin(yaw_rad) * std::cos(pitch_rad),
                          -std::sin(pitch_rad),
                          std::cos(yaw_rad) * std::cos(pitch_rad));
  const Eigen::Vector3d up_ref(0.0, -1.0, 0.0); // panorama "up" (y is down)
  Eigen::Vector3d r = f.cross(up_ref);
  if (r.norm() < 1e-6)
    r = Eigen::Vector3d(1.0, 0.0, 0.0); // looking straight up/down: pick x
  r.normalize();
  const Eigen::Vector3d d = f.cross(r); // camera down axis
  Eigen::Matrix3d R;
  R.col(0) = r;
  R.col(1) = d;
  R.col(2) = f;
  return R;
}

Eigen::Matrix3d intrinsics_from_fov(double fov_deg, int w, int h) {
  const double f = (0.5 * w) / std::tan(0.5 * fov_deg * kPi / 180.0);
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = f;
  K(1, 1) = f;
  K(0, 2) = 0.5 * w;
  K(1, 2) = 0.5 * h;
  return K;
}

} // namespace

Eigen::Vector3d pixel_to_bearing(const cv::Size &equirect, double u, double v) {
  const double theta = (u / equirect.width) * 2.0 * kPi - kPi; // [-pi, pi]
  const double phi = 0.5 * kPi - (v / equirect.height) * kPi;  // [pi/2,-pi/2]
  const double cp = std::cos(phi);
  return {std::sin(theta) * cp, -std::sin(phi), std::cos(theta) * cp};
}

Eigen::Vector2d bearing_to_pixel(const cv::Size &equirect,
                                 const Eigen::Vector3d &bearing) {
  const Eigen::Vector3d d = bearing.normalized();
  const double theta = std::atan2(d.x(), d.z());               // [-pi, pi]
  const double phi = std::asin(std::clamp(-d.y(), -1.0, 1.0)); // [-pi/2, pi/2]
  double u = (theta + kPi) / (2.0 * kPi) * equirect.width;
  double v = (0.5 * kPi - phi) / kPi * equirect.height;
  // wrap longitude, clamp latitude
  u = std::fmod(u, static_cast<double>(equirect.width));
  if (u < 0)
    u += equirect.width;
  v = std::clamp(v, 0.0, equirect.height - 1.0);
  return {u, v};
}

namespace {

// Shared remap: sample the equirect through a pinhole (K, R_pano_from_view).
void render_into(const cv::Mat &equirect, PerspectiveView &view, int out_w,
                 int out_h, int interp) {
  const double fx = view.K(0, 0), fy = view.K(1, 1);
  const double cx = view.K(0, 2), cy = view.K(1, 2);
  const cv::Size esz = equirect.size();

  cv::Mat map_x(out_h, out_w, CV_32FC1);
  cv::Mat map_y(out_h, out_w, CV_32FC1);
  for (int py = 0; py < out_h; ++py) {
    auto *mx = map_x.ptr<float>(py);
    auto *my = map_y.ptr<float>(py);
    for (int px = 0; px < out_w; ++px) {
      const Eigen::Vector3d ray_view((px - cx) / fx, (py - cy) / fy, 1.0);
      const Eigen::Vector3d d = view.R_pano_from_view * ray_view;
      const Eigen::Vector2d uv = bearing_to_pixel(esz, d);
      mx[px] = static_cast<float>(uv.x());
      my[px] = static_cast<float>(uv.y());
    }
  }
  cv::remap(equirect, view.image, map_x, map_y, interp, cv::BORDER_WRAP);
}

} // namespace

PerspectiveView extract_perspective(const cv::Mat &equirect, double yaw_deg,
                                    double pitch_deg, double fov_deg, int out_w,
                                    int out_h, int interp) {
  PerspectiveView view;
  view.fov_deg = fov_deg;
  view.yaw_deg = yaw_deg;
  view.pitch_deg = pitch_deg;
  view.K = intrinsics_from_fov(fov_deg, out_w, out_h);
  view.R_pano_from_view =
      view_basis(yaw_deg * kPi / 180.0, pitch_deg * kPi / 180.0);
  render_into(equirect, view, out_w, out_h, interp);
  return view;
}

PerspectiveView extract_perspective(const cv::Mat &equirect,
                                    const Eigen::Matrix3d &R_pano_from_view,
                                    const Eigen::Matrix3d &K, int out_w,
                                    int out_h, int interp) {
  PerspectiveView view;
  view.K = K;
  view.R_pano_from_view = R_pano_from_view;
  render_into(equirect, view, out_w, out_h, interp);
  return view;
}

std::vector<PerspectiveView> cube_faces(const cv::Mat &equirect, int face_size,
                                        int interp) {
  std::vector<PerspectiveView> faces;
  faces.reserve(6);
  // four around the equator + up + down, 90 deg FOV
  const double yaws[4] = {0.0, 90.0, 180.0, 270.0};
  for (double yaw : yaws)
    faces.push_back(extract_perspective(equirect, yaw, 0.0, 90.0, face_size,
                                        face_size, interp));
  faces.push_back(extract_perspective(equirect, 0.0, 90.0, 90.0, face_size,
                                      face_size,
                                      interp)); // up
  faces.push_back(extract_perspective(equirect, 0.0, -90.0, 90.0, face_size,
                                      face_size,
                                      interp)); // down
  return faces;
}

std::vector<PerspectiveView> overlapping_views(const cv::Mat &equirect,
                                               int n_yaw, double fov_deg,
                                               int tile, int interp) {
  std::vector<PerspectiveView> views;
  views.reserve(n_yaw + 2);
  for (int i = 0; i < n_yaw; ++i) {
    const double yaw = 360.0 * i / n_yaw;
    views.push_back(
        extract_perspective(equirect, yaw, 0.0, fov_deg, tile, tile, interp));
  }
  // caps: look up and down to cover the poles the equator band misses
  views.push_back(
      extract_perspective(equirect, 0.0, 90.0, fov_deg, tile, tile, interp));
  views.push_back(
      extract_perspective(equirect, 0.0, -90.0, fov_deg, tile, tile, interp));
  return views;
}

cv::Mat stitch_labels_to_equirect(const cv::Size &out,
                                  const std::vector<PerspectiveView> &views,
                                  const std::vector<cv::Mat> &tile_labels) {
  CV_Assert(views.size() == tile_labels.size());
  cv::Mat labels(out, CV_32S, cv::Scalar(-1));
  // Per-pixel "centrality" score of the winning tile (normalised optical-axis
  // alignment: view-frame z of the sample direction). Higher = more central.
  cv::Mat best_score(out, CV_32F, cv::Scalar(-1.0f));

  cv::parallel_for_(cv::Range(0, out.height), [&](const cv::Range &rows) {
    for (int v = rows.start; v < rows.end; ++v) {
      auto *lab = labels.ptr<int>(v);
      auto *scr = best_score.ptr<float>(v);
      for (int u = 0; u < out.width; ++u) {
        const Eigen::Vector3d d = pixel_to_bearing(out, u + 0.5, v + 0.5);
        for (size_t k = 0; k < views.size(); ++k) {
          const auto &view = views[k];
          const Eigen::Vector3d dv = view.R_pano_from_view.transpose() * d;
          if (dv.z() <= 1e-6)
            continue; // behind this view
          const double fx = view.K(0, 0), fy = view.K(1, 1);
          const double cx = view.K(0, 2), cy = view.K(1, 2);
          const double px = fx * dv.x() / dv.z() + cx;
          const double py = fy * dv.y() / dv.z() + cy;
          const cv::Mat &tl = tile_labels[k];
          const int ix = static_cast<int>(std::lround(px));
          const int iy = static_cast<int>(std::lround(py));
          if (ix < 0 || iy < 0 || ix >= tl.cols || iy >= tl.rows)
            continue;
          const float score = static_cast<float>(dv.z() / dv.norm());
          if (score <= scr[u])
            continue;
          const int lbl = tl.at<int>(iy, ix);
          if (lbl < 0)
            continue; // never let background override a real label
          lab[u] = lbl;
          scr[u] = score;
        }
      }
    }
  });
  return labels;
}

} // namespace reusex::geometry
