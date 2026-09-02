// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// P2 loop-closure edge detection. See LoopClosure.hpp for the design.
//
// Front-end: OpenCV ORB (BSD, no model weights) detects and describes features;
// matches are lifted to metric 3D via the stored per-frame depth (exactly the
// pinhole convention used by surfel_extraction) and a robust relative pose is
// estimated with RANSAC over 3D-3D Umeyama fits. The matcher is intentionally
// simple and license-clean; a learned matcher can replace it behind the same
// 3D-correspondence interface without changing the graph wiring.

#include "slam/LoopClosure.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace reusex::geometry {

namespace {

/// Per-frame feature cache: ORB descriptors and the metric 3D point (optical
/// frame) each keypoint back-projects to. pts3d[k] is invalid (NaN) when the
/// keypoint has no usable depth.
struct FrameFeatures {
  cv::Mat descriptors;                ///< Nx32 CV_8U ORB descriptors
  std::vector<Eigen::Vector3d> pts3d; ///< optical-frame 3D point per keypoint
  std::vector<char> valid;            ///< depth was in range
};

FrameFeatures extract_features(ProjectDB &db, int node_id,
                               const LoopClosureOptions &opt,
                               const cv::Ptr<cv::ORB> &orb) {
  FrameFeatures out;
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
  // Intrinsics scaled to the DEPTH resolution (mirrors surfel_extraction).
  const double sx = static_cast<double>(depth_f.cols) / std::max(1, intr.width);
  const double sy =
      static_cast<double>(depth_f.rows) / std::max(1, intr.height);
  const double fx = intr.fx * sx, fy = intr.fy * sy;
  const double cx = intr.cx * sx, cy = intr.cy * sy;
  if (fx <= 0.0 || fy <= 0.0)
    return out;

  std::vector<cv::KeyPoint> kps;
  orb->detectAndCompute(gray, cv::noArray(), kps, out.descriptors);
  if (kps.empty()) {
    out.descriptors.release();
    return out;
  }

  out.pts3d.resize(kps.size());
  out.valid.assign(kps.size(), 0);
  for (size_t k = 0; k < kps.size(); ++k) {
    // Map the color-image keypoint to the depth grid and back-project.
    const int ud = static_cast<int>(std::lround(
        kps[k].pt.x * static_cast<double>(depth_f.cols) / gray.cols));
    const int vd = static_cast<int>(std::lround(
        kps[k].pt.y * static_cast<double>(depth_f.rows) / gray.rows));
    if (ud < 0 || vd < 0 || ud >= depth_f.cols || vd >= depth_f.rows)
      continue;
    const float z = depth_f.at<float>(vd, ud);
    if (!std::isfinite(z) || z < opt.min_depth || z > opt.max_depth)
      continue;
    out.pts3d[k] = Eigen::Vector3d((ud - cx) * z / fx, (vd - cy) * z / fy, z);
    out.valid[k] = 1;
  }
  return out;
}

/// 3D-3D rigid fit of src -> dst (both 3xN), returning the 4x4 transform T with
/// dst ~= T * src. Wraps Eigen::umeyama (Kabsch, no scaling).
Eigen::Matrix4d rigid_fit(const Eigen::Matrix3Xd &src,
                          const Eigen::Matrix3Xd &dst) {
  return Eigen::umeyama(src, dst, false);
}

/// RANSAC 3D-3D: estimate T with dst ~= T*src from putative correspondences.
/// Returns inlier indices (empty if the fit is too weak).
std::vector<int> ransac_rigid(const Eigen::Matrix3Xd &src,
                              const Eigen::Matrix3Xd &dst,
                              const LoopClosureOptions &opt, std::mt19937 &rng,
                              Eigen::Matrix4d &best_T) {
  const int n = static_cast<int>(src.cols());
  std::vector<int> best_inliers;
  if (n < 3)
    return best_inliers;
  const double thr2 = static_cast<double>(opt.ransac_inlier_dist) *
                      static_cast<double>(opt.ransac_inlier_dist);
  std::uniform_int_distribution<int> pick(0, n - 1);

  for (int it = 0; it < opt.ransac_iterations; ++it) {
    int a = pick(rng), b = pick(rng), c = pick(rng);
    if (a == b || b == c || a == c)
      continue;
    Eigen::Matrix3Xd s3(3, 3), d3(3, 3);
    s3.col(0) = src.col(a);
    s3.col(1) = src.col(b);
    s3.col(2) = src.col(c);
    d3.col(0) = dst.col(a);
    d3.col(1) = dst.col(b);
    d3.col(2) = dst.col(c);
    const Eigen::Matrix4d T = rigid_fit(s3, d3);
    if (!T.allFinite())
      continue;
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);
    std::vector<int> inliers;
    inliers.reserve(n);
    for (int k = 0; k < n; ++k) {
      const Eigen::Vector3d e = (R * src.col(k) + t) - dst.col(k);
      if (e.squaredNorm() <= thr2)
        inliers.push_back(k);
    }
    if (inliers.size() > best_inliers.size()) {
      best_inliers = std::move(inliers);
      best_T = T;
    }
  }

  // Refit on the full inlier set for a lower-variance estimate.
  if (static_cast<int>(best_inliers.size()) >= 3) {
    Eigen::Matrix3Xd s(3, best_inliers.size()), d(3, best_inliers.size());
    for (size_t k = 0; k < best_inliers.size(); ++k) {
      s.col(k) = src.col(best_inliers[k]);
      d.col(k) = dst.col(best_inliers[k]);
    }
    const Eigen::Matrix4d T = rigid_fit(s, d);
    if (T.allFinite())
      best_T = T;
  }
  return best_inliers;
}

} // namespace

std::vector<LoopEdge>
detect_loop_edges(ProjectDB &db, const std::vector<int> &node_ids,
                  const std::vector<Eigen::Matrix4d> &seed_poses,
                  const LoopClosureOptions &opt,
                  LoopClosureResult *out_result) {
  std::vector<LoopEdge> edges;
  const int N = static_cast<int>(node_ids.size());
  if (N < 2 || static_cast<int>(seed_poses.size()) != N)
    return edges;

  // --- 1. Per-frame features (cached) -------------------------------------
  auto orb = cv::ORB::create(opt.max_features);
  std::vector<FrameFeatures> feats(N);
  for (int i = 0; i < N; ++i)
    feats[i] = extract_features(db, node_ids[i], opt, orb);
  core::info("LoopClosure: extracted ORB features for {} frames", N);

  // --- 2. Spatial candidate proposal from the seed poses ------------------
  std::vector<Eigen::Vector3d> center(N), viewdir(N);
  for (int i = 0; i < N; ++i) {
    center[i] = seed_poses[i].block<3, 1>(0, 3);
    viewdir[i] = seed_poses[i].block<3, 3>(0, 0) * Eigen::Vector3d(0, 0, 1);
  }
  const double cos_view = std::cos(opt.max_view_angle * M_PI / 180.0);
  const double max_d2 = static_cast<double>(opt.max_candidate_distance) *
                        static_cast<double>(opt.max_candidate_distance);

  // Collect candidate (j, dist2) per i, keep the K nearest, dedup as i<j.
  std::vector<std::pair<int, int>> candidates;
  for (int i = 0; i < N; ++i) {
    std::vector<std::pair<double, int>> near;
    for (int j = i + opt.min_frame_gap; j < N; ++j) {
      const double d2 = (center[i] - center[j]).squaredNorm();
      if (d2 > max_d2)
        continue;
      if (viewdir[i].dot(viewdir[j]) < cos_view)
        continue;
      near.emplace_back(d2, j);
    }
    std::sort(near.begin(), near.end());
    const int keep =
        std::min<int>(near.size(), std::max(0, opt.max_candidates_per_frame));
    for (int k = 0; k < keep; ++k)
      candidates.emplace_back(i, near[k].second);
  }
  core::info("LoopClosure: {} spatial loop candidates proposed",
             candidates.size());

  // --- 3. Match + robust relative pose per candidate ----------------------
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::mt19937 rng(opt.seed);
  int matched = 0, total_inliers = 0;

  for (const auto &cand : candidates) {
    const int i = cand.first, j = cand.second;
    const auto &fi = feats[i];
    const auto &fj = feats[j];
    if (fi.descriptors.empty() || fj.descriptors.empty())
      continue;
    ++matched;

    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(fj.descriptors, fi.descriptors, knn,
                     2); // query=j, train=i

    // Lowe ratio test + require valid depth on both sides.
    std::vector<Eigen::Vector3d> ps, pd; // src = j, dst = i
    ps.reserve(knn.size());
    pd.reserve(knn.size());
    for (const auto &m : knn) {
      if (m.size() < 2)
        continue;
      if (m[0].distance > opt.ratio_test * m[1].distance)
        continue;
      const int qj = m[0].queryIdx; // index into frame j
      const int ti = m[0].trainIdx; // index into frame i
      if (!fj.valid[qj] || !fi.valid[ti])
        continue;
      ps.push_back(fj.pts3d[qj]);
      pd.push_back(fi.pts3d[ti]);
    }
    if (static_cast<int>(ps.size()) < opt.min_match_inliers)
      continue;

    Eigen::Matrix3Xd src(3, ps.size()), dst(3, pd.size());
    for (size_t k = 0; k < ps.size(); ++k) {
      src.col(k) = ps[k];
      dst.col(k) = pd[k];
    }
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const std::vector<int> inliers = ransac_rigid(src, dst, opt, rng, T);
    if (static_cast<int>(inliers.size()) < opt.min_match_inliers)
      continue;

    // Sanity gate against the seed relative pose (reject gross mismatches;
    // GNC handles the merely-imperfect ones).
    const Eigen::Matrix4d seed_rel = seed_poses[i].inverse() * seed_poses[j];
    if (opt.max_seed_disagreement > 0.0f) {
      const double dt =
          (T.block<3, 1>(0, 3) - seed_rel.block<3, 1>(0, 3)).norm();
      if (dt > opt.max_seed_disagreement)
        continue;
    }

    LoopEdge e;
    e.i = i;
    e.j = j;
    e.T_ij = T;
    e.inliers = static_cast<int>(inliers.size());
    const double s = std::sqrt(static_cast<double>(opt.min_match_inliers) /
                               std::max(1, e.inliers));
    e.sigma_trans = std::max(static_cast<double>(opt.min_sigma_trans),
                             opt.base_sigma_trans * s);
    e.sigma_rot = std::max(static_cast<double>(opt.min_sigma_rot),
                           opt.base_sigma_rot * s);
    edges.push_back(std::move(e));
    total_inliers += static_cast<int>(inliers.size());
  }

  core::info("LoopClosure: {} candidates matched, {} loop edges accepted "
             "({} total inliers)",
             matched, edges.size(), total_inliers);
  if (out_result) {
    out_result->candidates = matched;
    out_result->edges = static_cast<int>(edges.size());
    out_result->total_inliers = total_inliers;
  }
  return edges;
}

} // namespace reusex::geometry
