// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See panorama_features.hpp. Extracted verbatim from PanoramaAlignment.cpp when
// issue #236 added a second consumer; the only behavioural change is that
// keypoints are lifted to the frame's OPTICAL coordinates and composed to world
// on demand, instead of being baked into world during extraction.

#include "panorama_features.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace reusex::geometry::pano_detail {

Eigen::Matrix4d to_matrix4(const std::array<double, 16> &a) {
  Eigen::Matrix4d M;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      M(r, c) = a[r * 4 + c];
  return M;
}

std::array<double, 16> from_matrix4(const Eigen::Matrix4d &M) {
  std::array<double, 16> a{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      a[r * 4 + c] = M(r, c);
  return a;
}

Eigen::Matrix3d exp_so3(const Eigen::Vector3d &w) {
  const double th = w.norm();
  if (th < 1e-12)
    return Eigen::Matrix3d::Identity();
  const Eigen::Vector3d axis = w / th;
  Eigen::Matrix3d K;
  K << 0, -axis.z(), axis.y(), axis.z(), 0, -axis.x(), -axis.y(), axis.x(), 0;
  return Eigen::Matrix3d::Identity() + std::sin(th) * K +
         (1 - std::cos(th)) * K * K;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return S;
}

double bearing_angle(const Eigen::Matrix3d &Q, const Eigen::Vector3d &t,
                     const Eigen::Vector3d &X, const Eigen::Vector3d &b_obs) {
  const Eigen::Vector3d m = Q * X + t;
  const double n = m.norm();
  if (n < 1e-9)
    return M_PI;
  const double c = std::clamp((m / n).dot(b_obs), -1.0, 1.0);
  return std::acos(c);
}

FrameFeatures extract_frame_features(ProjectDB &db, int node_id,
                                     const cv::Ptr<cv::ORB> &orb,
                                     float min_depth, float max_depth) {
  FrameFeatures out;
  out.node_id = node_id;

  cv::Mat color = db.sensor_frame_image(node_id);
  cv::Mat depth16 = db.sensor_frame_depth(node_id);
  if (color.empty() || depth16.empty())
    return out;

  cv::Mat gray;
  if (color.channels() == 3)
    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
  else if (color.channels() == 4)
    cv::cvtColor(color, gray, cv::COLOR_BGRA2GRAY);
  else
    gray = color;

  cv::Mat depth_f;
  depth16.convertTo(depth_f, CV_32FC1, 1.0 / 1000.0); // mm -> m

  const core::SensorIntrinsics intr = db.sensor_frame_intrinsics(node_id);
  const double sx = static_cast<double>(depth_f.cols) / std::max(1, intr.width);
  const double sy =
      static_cast<double>(depth_f.rows) / std::max(1, intr.height);
  const double fx = intr.fx * sx, fy = intr.fy * sy;
  const double cx = intr.cx * sx, cy = intr.cy * sy;
  if (fx <= 0.0 || fy <= 0.0)
    return out;

  out.T_world_cam = to_matrix4(db.sensor_frame_pose(node_id));

  orb->detectAndCompute(gray, cv::noArray(), out.keypoints, out.descriptors);
  if (out.keypoints.empty()) {
    out.descriptors.release();
    return out;
  }

  out.local.resize(out.keypoints.size());
  out.valid.assign(out.keypoints.size(), 0);
  for (size_t k = 0; k < out.keypoints.size(); ++k) {
    const int ud = static_cast<int>(std::lround(
        out.keypoints[k].pt.x * static_cast<double>(depth_f.cols) / gray.cols));
    const int vd = static_cast<int>(std::lround(
        out.keypoints[k].pt.y * static_cast<double>(depth_f.rows) / gray.rows));
    if (ud < 0 || vd < 0 || ud >= depth_f.cols || vd >= depth_f.rows)
      continue;
    const float z = depth_f.at<float>(vd, ud);
    if (!std::isfinite(z) || z < min_depth || z > max_depth)
      continue;
    out.local[k] = Eigen::Vector3d((ud - cx) * z / fx, (vd - cy) * z / fy, z);
    out.valid[k] = 1;
  }
  return out;
}

std::vector<int> refine_bearing_pose(const std::vector<Eigen::Vector3d> &points,
                                     const std::vector<Eigen::Vector3d> &bearings,
                                     const BearingRefineOptions &opt,
                                     Eigen::Matrix3d &Q, Eigen::Vector3d &t,
                                     int *out_initial_inliers) {
  auto gate = [&](const Eigen::Matrix3d &Qc, const Eigen::Vector3d &tc) {
    std::vector<int> sel;
    for (size_t k = 0; k < points.size(); ++k)
      if (bearing_angle(Qc, tc, points[k], bearings[k]) < opt.ang_gate)
        sel.push_back(static_cast<int>(k));
    return sel;
  };

  std::vector<int> inl = gate(Q, t);
  if (out_initial_inliers)
    *out_initial_inliers = static_cast<int>(inl.size());
  if (static_cast<int>(inl.size()) < opt.min_inliers)
    return {};

  for (int it = 0; it < opt.iterations; ++it) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
    for (int k : inl) {
      const Eigen::Vector3d m = Q * points[k] + t;
      const double n = m.norm();
      if (n < 1e-9)
        continue;
      const Eigen::Vector3d bh = m / n;
      const Eigen::Vector3d e = bh - bearings[k];
      const Eigen::Matrix3d dbh_dm =
          (Eigen::Matrix3d::Identity() - bh * bh.transpose()) / n;
      Eigen::Matrix<double, 3, 6> Jm;
      Jm.block<3, 3>(0, 0) = -skew(m);                    // wrt rotation
      Jm.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(); // wrt translation
      const Eigen::Matrix<double, 3, 6> J = dbh_dm * Jm;
      H += J.transpose() * J;
      g += J.transpose() * e;
    }
    H += 1e-9 * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-g);
    if (!dx.allFinite())
      break;
    const Eigen::Matrix3d dR = exp_so3(dx.head<3>());
    Q = dR * Q;
    t = dR * t + dx.tail<3>();
    if (dx.norm() < 1e-8)
      break;
    // Re-gate so the inlier set follows the improving pose.
    if (it + 1 < opt.iterations)
      inl = gate(Q, t);
    if (static_cast<int>(inl.size()) < opt.min_inliers)
      break;
  }

  inl = gate(Q, t);
  if (static_cast<int>(inl.size()) < opt.min_inliers)
    return {};
  return inl;
}

} // namespace reusex::geometry::pano_detail
