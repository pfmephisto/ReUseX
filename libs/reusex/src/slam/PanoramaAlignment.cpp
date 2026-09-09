// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See PanoramaAlignment.hpp for the design. Front-end (ORB + Lowe ratio + depth
// lift) mirrors slam/LoopClosure.cpp; the panorama pose is resected per slice
// with cv::solvePnPRansac and refined over all slices' inliers in
// panorama-bearing space with a small Gauss-Newton.

#include "slam/PanoramaAlignment.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "geometry/EquirectProjection.hpp"
#include "panorama_features.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace reusex::geometry {

namespace {

// Pose/rotation helpers, the frame feature extractor and the bearing-space
// Gauss-Newton refinement now live in panorama_features.hpp, shared with the
// panorama loop-edge front-end (issue #236).
using pano_detail::bearing_angle;
using pano_detail::from_matrix4;
using pano_detail::to_matrix4;

// ORB features of a frame with world-space 3D points. The extraction itself is
// shared (pano_detail::extract_frame_features, which lifts to the frame's OWN
// optical coordinates); alignment resects a single pooled pose in WORLD, so the
// points are composed with the frame's seed pose here.
struct FrameWorldFeatures {
  pano_detail::FrameFeatures f;
  std::vector<Eigen::Vector3d> world;

  const cv::Mat &descriptors() const { return f.descriptors; }
  bool valid(int k) const { return f.valid[k] != 0; }
};

FrameWorldFeatures extract_frame(ProjectDB &db, int node_id,
                                 const PanoramaAlignmentOptions &opt,
                                 const cv::Ptr<cv::ORB> &orb) {
  FrameWorldFeatures out;
  out.f = pano_detail::extract_frame_features(db, node_id, orb, opt.min_depth,
                                              opt.max_depth);
  out.world.resize(out.f.local.size());
  for (size_t k = 0; k < out.f.local.size(); ++k)
    if (out.f.valid[k])
      out.world[k] = out.f.world(k);
  return out;
}

// One 2D(slice pixel)-3D(world) + bearing correspondence.
struct Corr {
  cv::Point2f slice_px;
  Eigen::Vector3d world;
  Eigen::Vector3d bearing; ///< unit bearing of slice_px in the panorama frame
  int slice_idx = -1;      ///< which panorama slice
  int frame_node = -1;     ///< which sensor frame supplied the 3D point
  cv::Point2f frame_px;    ///< matched pixel in that sensor frame
};

// Draw an ORB-correspondence figure: the panorama slice (left) beside the
// best-matching sensor frame (right), inlier matches connected by lines. This
// is the literal "360 aligned with the scan image" visual.
void draw_correspondence_figure(ProjectDB &db, const cv::Mat &slice_img,
                                int frame_node,
                                const std::vector<const Corr *> &inliers,
                                const std::string &path) {
  cv::Mat frame = db.sensor_frame_image(frame_node);
  if (frame.empty() || slice_img.empty())
    return;
  cv::Mat lhs = slice_img.clone();
  cv::Mat rhs;
  if (frame.channels() == 1)
    cv::cvtColor(frame, rhs, cv::COLOR_GRAY2BGR);
  else
    rhs = frame.clone();
  // match heights for a clean side-by-side
  const int h = lhs.rows;
  const double s = static_cast<double>(h) / rhs.rows;
  cv::resize(rhs, rhs, cv::Size(), s, s);
  cv::Mat canvas;
  cv::hconcat(std::vector<cv::Mat>{lhs, rhs}, canvas);

  cv::RNG rng(12345);
  for (const Corr *c : inliers) {
    const cv::Point p0(cv::saturate_cast<int>(c->slice_px.x),
                       cv::saturate_cast<int>(c->slice_px.y));
    const cv::Point p1(cv::saturate_cast<int>(c->frame_px.x * s) + lhs.cols,
                       cv::saturate_cast<int>(c->frame_px.y * s));
    const cv::Scalar col(rng.uniform(0, 255), rng.uniform(0, 255),
                         rng.uniform(0, 255));
    cv::circle(canvas, p0, 3, col, 1, cv::LINE_AA);
    cv::circle(canvas, p1, 3, col, 1, cv::LINE_AA);
    cv::line(canvas, p0, p1, col, 1, cv::LINE_AA);
  }
  cv::putText(canvas,
              "panorama slice            |            scan frame  (" +
                  std::to_string(inliers.size()) + " inlier matches)",
              cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  std::filesystem::create_directories(
      std::filesystem::path(path).parent_path());
  cv::imwrite(path, canvas);
}

} // namespace

PanoramaAlignmentResult align_panorama(ProjectDB &db, int pano_id,
                                       int seed_node_id,
                                       const PanoramaAlignmentOptions &opt) {
  PanoramaAlignmentResult res;
  res.pano_id = pano_id;
  res.seed_node = seed_node_id;

  cv::setRNGSeed(static_cast<int>(opt.seed)); // deterministic solvePnPRansac

  cv::Mat equirect = db.panoramic_image(pano_id);
  if (equirect.empty()) {
    core::warn("PanoramaAlignment: panorama {} has no image", pano_id);
    return res;
  }

  // --- candidate sensor frames: window around the seed in id order ----------
  std::vector<int> all_ids = db.sensor_frame_ids();
  std::sort(all_ids.begin(), all_ids.end());
  auto seed_it = std::find(all_ids.begin(), all_ids.end(), seed_node_id);
  if (seed_it == all_ids.end()) {
    core::warn("PanoramaAlignment: seed node {} not found for panorama {}",
               seed_node_id, pano_id);
    return res;
  }
  const int seed_idx = static_cast<int>(seed_it - all_ids.begin());
  // order candidates by |offset| so the nearest frames are matched first
  std::vector<int> cand;
  for (int d = 0; d <= opt.candidate_window; ++d) {
    for (int s : (d == 0 ? std::vector<int>{0} : std::vector<int>{-d, +d})) {
      const int idx = seed_idx + s;
      if (idx >= 0 && idx < static_cast<int>(all_ids.size()))
        cand.push_back(all_ids[idx]);
      if (static_cast<int>(cand.size()) >= opt.max_candidates)
        break;
    }
    if (static_cast<int>(cand.size()) >= opt.max_candidates)
      break;
  }

  auto orb = cv::ORB::create(opt.max_features);

  // frame features (world 3D)
  std::vector<FrameWorldFeatures> frames;
  frames.reserve(cand.size());
  for (int nid : cand)
    frames.push_back(extract_frame(db, nid, opt, orb));

  // panorama slices
  const std::vector<PerspectiveView> slices =
      overlapping_views(equirect, opt.n_yaw, opt.fov_deg, opt.slice);

  cv::BFMatcher matcher(cv::NORM_HAMMING);

  // Per-slice correspondences (kept separate so each slice PnPs with its own
  // K), plus a global pool for the bearing-space refinement.
  std::vector<std::vector<Corr>> slice_corr(slices.size());
  std::vector<Corr> pool;

  for (size_t si = 0; si < slices.size(); ++si) {
    const auto &view = slices[si];
    cv::Mat sgray;
    cv::cvtColor(view.image, sgray, cv::COLOR_BGR2GRAY);
    std::vector<cv::KeyPoint> skps;
    cv::Mat sdesc;
    orb->detectAndCompute(sgray, cv::noArray(), skps, sdesc);
    if (sdesc.empty())
      continue;

    const double fx = view.K(0, 0), fy = view.K(1, 1);
    const double cx = view.K(0, 2), cy = view.K(1, 2);

    for (size_t fi = 0; fi < frames.size(); ++fi) {
      const auto &fr = frames[fi];
      if (fr.descriptors().empty())
        continue;
      std::vector<std::vector<cv::DMatch>> knn;
      matcher.knnMatch(sdesc, fr.f.descriptors, knn,
                       2); // query=slice, train=frame
      for (const auto &m : knn) {
        if (m.size() < 2)
          continue;
        if (m[0].distance > opt.ratio_test * m[1].distance)
          continue;
        const int qs = m[0].queryIdx; // slice keypoint
        const int tf = m[0].trainIdx; // frame keypoint
        if (!fr.valid(tf))
          continue;
        Corr corr;
        corr.slice_px = skps[qs].pt;
        corr.world = fr.world[tf];
        corr.slice_idx = static_cast<int>(si);
        corr.frame_node = cand[fi];
        corr.frame_px = fr.f.keypoints[tf].pt;
        const Eigen::Vector3d ray_view((corr.slice_px.x - cx) / fx,
                                       (corr.slice_px.y - cy) / fy, 1.0);
        corr.bearing = (view.R_pano_from_view * ray_view).normalized();
        slice_corr[si].push_back(corr);
        pool.push_back(corr);
      }
    }
  }

  // --- per-slice PnP; keep the strongest as the initial pose ----------------
  Eigen::Matrix3d Q_best = Eigen::Matrix3d::Identity(); // pano_from_world
  Eigen::Vector3d t_best = Eigen::Vector3d::Zero();
  int best_inliers = 0;

  for (size_t si = 0; si < slices.size(); ++si) {
    const auto &cs = slice_corr[si];
    if (static_cast<int>(cs.size()) < opt.min_slice_correspondences)
      continue;
    std::vector<cv::Point3f> obj;
    std::vector<cv::Point2f> img;
    obj.reserve(cs.size());
    img.reserve(cs.size());
    for (const auto &c : cs) {
      obj.emplace_back(static_cast<float>(c.world.x()),
                       static_cast<float>(c.world.y()),
                       static_cast<float>(c.world.z()));
      img.push_back(c.slice_px);
    }
    cv::Mat Kcv =
        (cv::Mat_<double>(3, 3) << slices[si].K(0, 0), 0, slices[si].K(0, 2), 0,
         slices[si].K(1, 1), slices[si].K(1, 2), 0, 0, 1);
    cv::Mat rvec, tvec, inliers;
    const bool ok = cv::solvePnPRansac(
        obj, img, Kcv, cv::noArray(), rvec, tvec, /*useExtrinsicGuess=*/false,
        opt.ransac_iterations, opt.ransac_reproj_px, 0.99, inliers,
        cv::SOLVEPNP_EPNP);
    if (!ok || inliers.rows < opt.min_slice_correspondences)
      continue;
    if (inliers.rows <= best_inliers)
      continue;

    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);
    Eigen::Matrix3d R_ws; // world -> slice-cam
    Eigen::Vector3d t_ws;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c)
        R_ws(r, c) = Rcv.at<double>(r, c);
      t_ws(r) = tvec.at<double>(r);
    }
    // pano_from_world = R_pano_from_view * (world -> slice-cam)
    const Eigen::Matrix3d R_pfv = slices[si].R_pano_from_view;
    Q_best = R_pfv * R_ws;
    t_best = R_pfv * t_ws;
    best_inliers = inliers.rows;
  }

  if (best_inliers < opt.min_slice_correspondences || pool.empty()) {
    core::debug("PanoramaAlignment: panorama {} — insufficient PnP support "
                "(best slice inliers {})",
                pano_id, best_inliers);
    return res;
  }

  // --- gate the pool by consistency with the initial pose -------------------
  // angular threshold from the slice reprojection tolerance (small-angle),
  // relaxed 2x to admit the honest inliers of the other slices.
  const double fx0 = slices.empty() ? 1.0 : slices[0].K(0, 0);
  const double ang_gate = 2.0 * std::atan(opt.ransac_reproj_px / fx0);

  // Parallel arrays for the shared bearing-space refinement: each pooled
  // correspondence's WORLD point against the panorama bearing it was seen
  // under.
  std::vector<Eigen::Vector3d> pool_pts, pool_bearings;
  pool_pts.reserve(pool.size());
  pool_bearings.reserve(pool.size());
  for (const auto &c : pool) {
    pool_pts.push_back(c.world);
    pool_bearings.push_back(c.bearing);
  }

  // --- Gauss-Newton refine pano_from_world (Q,t) over pooled inliers --------
  // Shared with the loop-edge front-end (pano_detail::refine_bearing_pose):
  // left perturbation T <- exp(xi) T, residual e = b_hat - b_obs, re-gating the
  // inlier set as the pose improves.
  Eigen::Matrix3d Q = Q_best;
  Eigen::Vector3d t = t_best;
  int initial_inliers = 0;
  const pano_detail::BearingRefineOptions ro{ang_gate, opt.refine_iterations,
                                             opt.min_inliers};
  const std::vector<int> inl_idx = pano_detail::refine_bearing_pose(
      pool_pts, pool_bearings, ro, Q, t, &initial_inliers);
  if (initial_inliers < opt.min_inliers) {
    core::debug("PanoramaAlignment: panorama {} — {} pooled inliers < {}",
                pano_id, initial_inliers, opt.min_inliers);
    return res;
  }
  if (inl_idx.empty()) {
    core::debug("PanoramaAlignment: panorama {} — refinement fell below {} "
                "inliers (started at {})",
                pano_id, opt.min_inliers, initial_inliers);
    return res;
  }

  std::vector<const Corr *> inl;
  inl.reserve(inl_idx.size());
  for (int k : inl_idx)
    inl.push_back(&pool[k]);

  double sq = 0.0;
  for (const Corr *c : inl) {
    const double a = bearing_angle(Q, t, c->world, c->bearing);
    sq += a * a;
  }
  const double rms_rad = std::sqrt(sq / inl.size());

  // world_from_pano: R = Q^T, c = -R t
  const Eigen::Matrix3d R = Q.transpose();
  const Eigen::Vector3d c = -R * t;
  Eigen::Matrix4d T_world_pano = Eigen::Matrix4d::Identity();
  T_world_pano.block<3, 3>(0, 0) = R;
  T_world_pano.block<3, 1>(0, 3) = c;

  // correction relative to the timestamp-seed placement
  const Eigen::Matrix4d T_seed = to_matrix4(db.sensor_frame_pose(seed_node_id));
  const double delta = (c - T_seed.block<3, 1>(0, 3)).norm();
  res.delta_from_seed_m = delta;

  // Plausibility gate: an implausibly large correction means the bearing
  // resection was ill-conditioned (narrow match cone) rather than a real fix.
  if (opt.max_correction_m > 0.0 && delta > opt.max_correction_m) {
    core::debug("PanoramaAlignment: panorama {} rejected — {:.1f} m correction "
                "exceeds {:.1f} m gate ({} inliers, {:.3f} deg RMS; likely "
                "ill-conditioned)",
                pano_id, delta, opt.max_correction_m,
                static_cast<int>(inl.size()), rms_rad * 180.0 / M_PI);
    return res; // aligned stays false -> keep timestamp placement
  }

  res.aligned = true;
  res.inliers = static_cast<int>(inl.size());
  res.rms_deg = rms_rad * 180.0 / M_PI;
  res.pose = from_matrix4(T_world_pano);

  core::info("PanoramaAlignment: panorama {} aligned — {} inliers, {:.3f} deg "
             "RMS, {:.3f} m from seed",
             pano_id, res.inliers, res.rms_deg, res.delta_from_seed_m);

  // Optional correspondence figure: pick the (slice, frame) pair contributing
  // the most inliers and draw the matches between the panorama slice and that
  // scan frame.
  if (!opt.debug_dir.empty()) {
    std::map<std::pair<int, int>, std::vector<const Corr *>> by_pair;
    for (const Corr *c : inl)
      by_pair[{c->slice_idx, c->frame_node}].push_back(c);
    const std::vector<const Corr *> *best = nullptr;
    std::pair<int, int> best_key;
    for (const auto &kv : by_pair)
      if (!best || kv.second.size() > best->size()) {
        best = &kv.second;
        best_key = kv.first;
      }
    if (best && best_key.first >= 0 &&
        best_key.first < static_cast<int>(slices.size())) {
      const std::string name =
          opt.debug_name.empty() ? std::to_string(pano_id) : opt.debug_name;
      draw_correspondence_figure(db, slices[best_key.first].image,
                                 best_key.second, *best,
                                 opt.debug_dir + "/" + name + "_matches.jpg");
    }
  }
  return res;
}

} // namespace reusex::geometry
