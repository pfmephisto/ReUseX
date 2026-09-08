// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See PanoramaLoopEdges.hpp for the design and, in particular, for WHY the
// per-frame resections must run in each frame's own optical coordinates.

#include "slam/PanoramaLoopEdges.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"
#include "geometry/EquirectProjection.hpp"
#include "panorama_features.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace reusex::geometry {

namespace {

using pano_detail::FrameFeatures;

/// Correspondences a panorama contributed to ONE sensor frame, split per slice
/// (each slice is a true pinhole with its own K, so PnP is per slice) and
/// pooled in bearing space (the refinement is per frame, across all slices).
struct FrameCorrespondences {
  /// slice index -> (slice pixel, 3D point in the FRAME's optical coordinates)
  std::map<int, std::vector<std::pair<cv::Point2f, Eigen::Vector3d>>> per_slice;
  std::vector<Eigen::Vector3d> points;   ///< pooled, frame-optical coordinates
  std::vector<Eigen::Vector3d> bearings; ///< pooled, panorama frame
};

/// Evenly subsample @p n frame indices down to at most @p budget, always
/// including the first and progressing by a fixed stride so the selection is
/// deterministic and spans the whole trajectory.
std::vector<int> subsample_indices(int n, int budget) {
  std::vector<int> out;
  if (n <= 0 || budget <= 0)
    return out;
  const int stride = std::max(1, (n + budget - 1) / budget);
  for (int i = 0; i < n; i += stride)
    out.push_back(i);
  return out;
}

} // namespace

namespace detail {

std::vector<LoopEdge>
edges_from_resections(std::vector<PanoResection> resections,
                      const std::vector<Eigen::Matrix4d> &seed_poses,
                      const PanoramaLoopOptions &opt,
                      const LoopClosureOptions &gates,
                      PanoramaLoopResult *stats) {
  // Deterministic pair enumeration regardless of the order resections were
  // produced in (docs/STANDARDS.md §6).
  std::sort(resections.begin(), resections.end(),
            [](const PanoResection &a, const PanoResection &b) {
              return std::tie(a.frame, a.inliers) <
                     std::tie(b.frame, b.inliers);
            });

  std::vector<LoopEdge> candidates;
  const int nseed = static_cast<int>(seed_poses.size());

  for (size_t a = 0; a < resections.size(); ++a) {
    for (size_t b = a + 1; b < resections.size(); ++b) {
      const PanoResection &ra = resections[a];
      const PanoResection &rb = resections[b];
      if (ra.frame < 0 || rb.frame < 0 || ra.frame >= nseed ||
          rb.frame >= nseed || ra.frame == rb.frame)
        continue;

      // T_AB = T_pano_A^-1 * T_pano_B  ==  pose(A)^-1 * pose(B).
      // Neither factor was built from a seed pose, so this is a measurement of
      // the relative pose, not a restatement of the drifted trajectory.
      const Eigen::Matrix4d T_ij = ra.T_pano_frame.inverse() * rb.T_pano_frame;
      if (!T_ij.allFinite()) {
        // A non-finite composition means a degenerate resection slipped through
        // the inlier gate — a bug, not a gating decision, so it is warned about
        // and excluded from the proposal accounting (STANDARDS §5).
        core::warn("PanoramaLoops: non-finite relative pose between frames {} "
                   "and {} ({} / {} inliers) — edge discarded",
                   ra.frame, rb.frame, ra.inliers, rb.inliers);
        continue;
      }

      if (stats)
        ++stats->proposed;

      if (std::abs(rb.frame - ra.frame) < gates.min_frame_gap) {
        if (stats)
          ++stats->dropped_gap;
        continue;
      }

      // Seed-disagreement gates, reused verbatim from the ORB front-end so one
      // set of CLI knobs governs every edge source. `min_seed_disagreement`
      // drops edges that merely restate poses already believed to be right (no
      // drift-correction information, only extra matcher noise);
      // `max_seed_disagreement` is a coarse gross-mismatch guard, off by
      // default because large disagreement is the SIGNAL under drift.
      const Eigen::Matrix4d rel_seed =
          seed_poses[ra.frame].inverse() * seed_poses[rb.frame];
      const double disagreement =
          (rel_seed.block<3, 1>(0, 3) - T_ij.block<3, 1>(0, 3)).norm();
      if ((gates.min_seed_disagreement > 0.0f &&
           disagreement < gates.min_seed_disagreement) ||
          (gates.max_seed_disagreement > 0.0f &&
           disagreement > gates.max_seed_disagreement)) {
        if (stats)
          ++stats->dropped_seed_gate;
        continue;
      }

      LoopEdge e;
      e.i = ra.frame;
      e.j = rb.frame;
      e.T_ij = T_ij;
      // Joint support is the WEAKER of the two resections: the edge chains both,
      // so the worse one bounds its trustworthiness.
      e.inliers = std::min(ra.inliers, rb.inliers);
      const double scale =
          std::sqrt(static_cast<double>(std::max(1, opt.min_frame_inliers)) /
                    std::max(opt.min_frame_inliers, e.inliers));
      e.sigma_trans =
          std::max(static_cast<double>(opt.min_sigma_trans),
                   static_cast<double>(opt.base_sigma_trans) * scale);
      e.sigma_rot = std::max(static_cast<double>(opt.min_sigma_rot),
                             static_cast<double>(opt.base_sigma_rot) * scale);
      candidates.push_back(e);
    }
  }

  // Per-panorama cap: keep the best-supported edges. One panorama matching many
  // frames produces O(F^2) edges that all share the same resection errors, so
  // they are not independent measurements and would over-weight this panorama.
  if (opt.max_edges_per_panorama > 0 &&
      static_cast<int>(candidates.size()) > opt.max_edges_per_panorama) {
    std::sort(candidates.begin(), candidates.end(),
              [](const LoopEdge &x, const LoopEdge &y) {
                // highest support first; (i, j) breaks ties deterministically
                return std::tie(y.inliers, x.i, x.j) <
                       std::tie(x.inliers, y.i, y.j);
              });
    if (stats)
      stats->dropped_cap +=
          static_cast<int>(candidates.size()) - opt.max_edges_per_panorama;
    candidates.resize(opt.max_edges_per_panorama);
  }

  // Stable, index-ordered output regardless of which cap path ran.
  std::sort(candidates.begin(), candidates.end(),
            [](const LoopEdge &x, const LoopEdge &y) {
              return std::tie(x.i, x.j) < std::tie(y.i, y.j);
            });

  if (stats) {
    stats->edges += static_cast<int>(candidates.size());
    for (const auto &e : candidates)
      stats->total_inliers += e.inliers;
  }
  return candidates;
}

} // namespace detail

std::vector<LoopEdge>
detect_panorama_loop_edges(ProjectDB &db, const std::vector<int> &node_ids,
                           const std::vector<Eigen::Matrix4d> &seed_poses,
                           const PanoramaLoopOptions &options,
                           const LoopClosureOptions &gates,
                           PanoramaLoopResult *out_result) {
  PanoramaLoopResult stats;
  std::vector<LoopEdge> edges;

  auto finish = [&]() {
    if (out_result)
      *out_result = stats;
    return edges;
  };

  std::vector<ProjectDB::PanoramicImage> panos = db.list_panoramic_images();
  std::sort(panos.begin(), panos.end(),
            [](const ProjectDB::PanoramicImage &a,
               const ProjectDB::PanoramicImage &b) { return a.id < b.id; });
  stats.panoramas = static_cast<int>(panos.size());

  // Not an error: a project simply may not have 360 imagery. Whether the USER
  // asked for panorama edges is the caller's knowledge, so the loud warning
  // lives there (STANDARDS §5).
  if (panos.empty())
    return finish();

  if (node_ids.size() != seed_poses.size())
    throw std::runtime_error(
        "detect_panorama_loop_edges: node_ids and seed_poses differ in size (" +
        std::to_string(node_ids.size()) + " vs " +
        std::to_string(seed_poses.size()) + ")");

  // --- candidate frames: an even sweep of the WHOLE trajectory --------------
  // (see PanoramaLoopOptions::max_frames for why this is not a temporal window)
  const std::vector<int> cand =
      subsample_indices(static_cast<int>(node_ids.size()), options.max_frames);

  auto orb = cv::ORB::create(options.max_features);

  // Extracted ONCE and reused for every panorama: the sweep is global, so the
  // per-frame ORB + depth lift (the second-largest cost after matching) must
  // not be repeated per panorama.
  std::vector<FrameFeatures> feats;
  std::vector<int> feat_frame; // frame index parallel to `feats`
  feats.reserve(cand.size());
  feat_frame.reserve(cand.size());
  for (int fi : cand) {
    FrameFeatures f = pano_detail::extract_frame_features(
        db, node_ids[fi], orb, options.min_depth, options.max_depth);
    if (f.descriptors.empty())
      continue;
    const int usable =
        static_cast<int>(std::count(f.valid.begin(), f.valid.end(), 1));
    if (usable < options.min_frame_correspondences)
      continue;
    feats.push_back(std::move(f));
    feat_frame.push_back(fi);
  }

  if (static_cast<int>(feats.size()) < options.min_candidate_frames) {
    core::warn("PanoramaLoops: only {} of {} swept frames carry usable "
               "colour+depth features (need {}) — no panorama loop edges",
               feats.size(), cand.size(), options.min_candidate_frames);
    return finish();
  }

  core::info("PanoramaLoops: {} panoramas against {} frames swept from {} "
             "(every {}th)",
             panos.size(), feats.size(), node_ids.size(),
             std::max<size_t>(1, node_ids.size() / std::max<size_t>(1, cand.size())));

  cv::BFMatcher matcher(cv::NORM_HAMMING);

  for (const auto &pano : panos) {
    // Re-seeded per panorama so a panorama's result never depends on how many
    // panoramas preceded it (docs/STANDARDS.md §6).
    cv::setRNGSeed(static_cast<int>(options.seed));

    cv::Mat equirect = db.panoramic_image(pano.id);
    if (equirect.empty()) {
      core::warn("PanoramaLoops: panorama {} ('{}') has no image — skipped",
                 pano.id, pano.filename);
      continue;
    }

    const std::vector<PerspectiveView> slices = overlapping_views(
        equirect, options.n_yaw, options.fov_deg, options.slice);
    if (slices.empty()) {
      core::warn("PanoramaLoops: panorama {} produced no perspective slices "
                 "(n_yaw={}, slice={}) — skipped",
                 pano.id, options.n_yaw, options.slice);
      continue;
    }

    // --- match every slice against every swept frame -----------------------
    std::vector<FrameCorrespondences> corr(feats.size());
    for (size_t si = 0; si < slices.size(); ++si) {
      const PerspectiveView &view = slices[si];
      cv::Mat sgray;
      if (view.image.channels() == 3)
        cv::cvtColor(view.image, sgray, cv::COLOR_BGR2GRAY);
      else
        sgray = view.image;
      std::vector<cv::KeyPoint> skps;
      cv::Mat sdesc;
      orb->detectAndCompute(sgray, cv::noArray(), skps, sdesc);
      if (sdesc.empty())
        continue;

      const double fx = view.K(0, 0), fy = view.K(1, 1);
      const double cx = view.K(0, 2), cy = view.K(1, 2);

      for (size_t fi = 0; fi < feats.size(); ++fi) {
        const FrameFeatures &fr = feats[fi];
        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(sdesc, fr.descriptors, knn, 2);
        for (const auto &m : knn) {
          if (m.size() < 2)
            continue;
          if (m[0].distance > options.ratio_test * m[1].distance)
            continue;
          const int qs = m[0].queryIdx; // slice keypoint
          const int tf = m[0].trainIdx; // frame keypoint
          if (!fr.valid[tf])
            continue;
          const cv::Point2f spx = skps[qs].pt;
          // THE point of this module: the frame's OWN optical coordinates.
          const Eigen::Vector3d &p_frame = fr.local[tf];
          const Eigen::Vector3d ray_view((spx.x - cx) / fx, (spx.y - cy) / fy,
                                         1.0);
          corr[fi].per_slice[static_cast<int>(si)].emplace_back(spx, p_frame);
          corr[fi].points.push_back(p_frame);
          corr[fi].bearings.push_back(
              (view.R_pano_from_view * ray_view).normalized());
        }
      }
    }

    // --- resect the panorama independently against each frame --------------
    const double fx0 = slices.front().K(0, 0);
    const double ang_gate = 2.0 * std::atan(options.ransac_reproj_px / fx0);

    std::vector<detail::PanoResection> resections;
    for (size_t fi = 0; fi < feats.size(); ++fi) {
      const FrameCorrespondences &fc = corr[fi];
      if (static_cast<int>(fc.points.size()) < options.min_frame_correspondences)
        continue;

      // Initialise from the single best (slice, frame) PnP: one slice is a true
      // pinhole with a known K and a known slice->panorama rotation.
      Eigen::Matrix3d Q_best = Eigen::Matrix3d::Identity(); // pano_from_frame
      Eigen::Vector3d t_best = Eigen::Vector3d::Zero();
      int best_inliers = 0;
      for (const auto &[si, pairs] : fc.per_slice) {
        if (static_cast<int>(pairs.size()) < options.min_slice_correspondences)
          continue;
        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> img;
        obj.reserve(pairs.size());
        img.reserve(pairs.size());
        for (const auto &[px, X] : pairs) {
          obj.emplace_back(static_cast<float>(X.x()), static_cast<float>(X.y()),
                           static_cast<float>(X.z()));
          img.push_back(px);
        }
        const Eigen::Matrix3d &K = slices[si].K;
        cv::Mat Kcv = (cv::Mat_<double>(3, 3) << K(0, 0), 0, K(0, 2), 0, K(1, 1),
                       K(1, 2), 0, 0, 1);
        cv::Mat rvec, tvec, inliers;
        const bool ok = cv::solvePnPRansac(
            obj, img, Kcv, cv::noArray(), rvec, tvec,
            /*useExtrinsicGuess=*/false, options.ransac_iterations,
            options.ransac_reproj_px, 0.99, inliers, cv::SOLVEPNP_EPNP);
        if (!ok || inliers.rows < options.min_slice_correspondences)
          continue;
        if (inliers.rows <= best_inliers)
          continue;

        cv::Mat Rcv;
        cv::Rodrigues(rvec, Rcv);
        Eigen::Matrix3d R_fs; // frame -> slice-cam
        Eigen::Vector3d t_fs;
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c)
            R_fs(r, c) = Rcv.at<double>(r, c);
          t_fs(r) = tvec.at<double>(r);
        }
        // pano_from_frame = R_pano_from_view * (frame -> slice-cam)
        const Eigen::Matrix3d &R_pfv = slices[si].R_pano_from_view;
        Q_best = R_pfv * R_fs;
        t_best = R_pfv * t_fs;
        best_inliers = inliers.rows;
      }

      if (best_inliers < options.min_slice_correspondences)
        continue;

      Eigen::Matrix3d Q = Q_best;
      Eigen::Vector3d t = t_best;
      int initial_inliers = 0;
      const pano_detail::BearingRefineOptions ro{ang_gate,
                                                 options.refine_iterations,
                                                 options.min_frame_inliers};
      const std::vector<int> inl = pano_detail::refine_bearing_pose(
          fc.points, fc.bearings, ro, Q, t, &initial_inliers);
      if (inl.empty()) {
        core::debug("PanoramaLoops: panorama {} vs frame {} — {} pooled "
                    "inliers < {}, resection rejected",
                    pano.id, feat_frame[fi], initial_inliers,
                    options.min_frame_inliers);
        continue;
      }

      // Plausibility: the panorama was physically captured near the frames it
      // matches. A far-away centre means a narrow match cone left translation
      // ill-conditioned, not that the panorama is really over there.
      const Eigen::Vector3d centre_in_frame = -Q.transpose() * t;
      if (options.max_pano_distance > 0.0 &&
          centre_in_frame.norm() > options.max_pano_distance) {
        core::debug("PanoramaLoops: panorama {} vs frame {} — resected centre "
                    "{:.2f} m away exceeds {:.2f} m gate ({} inliers); "
                    "ill-conditioned, rejected",
                    pano.id, feat_frame[fi], centre_in_frame.norm(),
                    options.max_pano_distance, inl.size());
        continue;
      }

      detail::PanoResection r;
      r.frame = feat_frame[fi];
      r.T_pano_frame = Eigen::Matrix4d::Identity();
      r.T_pano_frame.block<3, 3>(0, 0) = Q;
      r.T_pano_frame.block<3, 1>(0, 3) = t;
      r.inliers = static_cast<int>(inl.size());
      resections.push_back(std::move(r));
    }

    stats.frames_resected += static_cast<int>(resections.size());
    if (resections.size() < 2) {
      core::debug("PanoramaLoops: panorama {} ('{}') resected {} frame(s) — "
                  "needs 2 to form an edge",
                  pano.id, pano.filename, resections.size());
      continue;
    }
    ++stats.panoramas_matched;

    const int before = stats.edges;
    std::vector<LoopEdge> pano_edges = detail::edges_from_resections(
        std::move(resections), seed_poses, options, gates, &stats);
    core::info("PanoramaLoops: panorama {} ('{}') -> {} loop edges",
               pano.id, pano.filename, stats.edges - before);
    edges.insert(edges.end(), pano_edges.begin(), pano_edges.end());
  }

  // Deterministic global ordering, and a de-duplicating pass: two panoramas
  // that both see the same frame pair would otherwise contribute two
  // BetweenFactors on one pair, multiplying that pair's information. The
  // better-supported edge wins.
  std::sort(edges.begin(), edges.end(),
            [](const LoopEdge &x, const LoopEdge &y) {
              return std::tie(x.i, x.j, y.inliers) <
                     std::tie(y.i, y.j, x.inliers);
            });
  const size_t before_dedup = edges.size();
  edges.erase(std::unique(edges.begin(), edges.end(),
                          [](const LoopEdge &x, const LoopEdge &y) {
                            return x.i == y.i && x.j == y.j;
                          }),
              edges.end());
  if (edges.size() != before_dedup) {
    const int removed = static_cast<int>(before_dedup - edges.size());
    core::info("PanoramaLoops: merged {} duplicate frame pairs seen by more "
               "than one panorama (kept the best-supported edge each)",
               removed);
    stats.edges -= removed;
    stats.total_inliers = 0;
    for (const auto &e : edges)
      stats.total_inliers += e.inliers;
  }

  if (edges.empty())
    core::warn("PanoramaLoops: {} panoramas produced NO loop edges "
               "({} matched >=2 frames, {} resections, {} pairs proposed: "
               "{} below min_frame_gap={}, {} outside the seed-disagreement "
               "gate, {} over the per-panorama cap)",
               stats.panoramas, stats.panoramas_matched, stats.frames_resected,
               stats.proposed, stats.dropped_gap, gates.min_frame_gap,
               stats.dropped_seed_gate, stats.dropped_cap);
  else
    core::info("PanoramaLoops: {} edges from {}/{} panoramas ({} resections, "
               "{} pairs proposed; dropped {} gap / {} seed-gate / {} cap)",
               stats.edges, stats.panoramas_matched, stats.panoramas,
               stats.frames_resected, stats.proposed, stats.dropped_gap,
               stats.dropped_seed_gate, stats.dropped_cap);

  return finish();
}

} // namespace reusex::geometry
