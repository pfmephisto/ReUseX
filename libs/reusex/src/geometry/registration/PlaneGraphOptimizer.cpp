// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See PlaneGraphOptimizer.hpp for the method overview and attribution.

#include "geometry/registration/PlaneGraphOptimizer.hpp"
#include "core/logging.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

namespace reusex::geometry {

namespace {

/// gtsam's OrientedPlane3Factor does not implement clone(), which
/// GncOptimizer requires (it clones the graph to build its robustified
/// inner copy and throws NonlinearFactor::clone() otherwise). Thin subclass
/// adding the missing override.
class ClonableOrientedPlane3Factor : public gtsam::OrientedPlane3Factor {
    public:
  using gtsam::OrientedPlane3Factor::OrientedPlane3Factor;
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(
        new ClonableOrientedPlane3Factor(*this));
  }
};

/// A plane detected in one frame, expressed in that frame's OPTICAL frame as a
/// unit normal + signed offset d so that n.dot(p) + d = 0 for points on it.
struct FramePlane {
  int frame = -1;               ///< index into the frames vector
  Eigen::Vector3d normal;       ///< unit normal (optical frame)
  double d = 0.0;               ///< plane offset (optical frame)
  int inliers = 0;              ///< supporting surfel count
  Eigen::Vector3d world_normal; ///< normal in world (seed pose)
  double world_d = 0.0;         ///< offset in world (seed pose)
};

/// Transform an optical-frame plane (n, d) by the affine T (optical->world).
/// For a rigid transform p_w = R p + t, a plane n.p_o + d = 0 becomes
/// n_w = R n, d_w = d - n_w.dot(t).
void plane_to_world(const Eigen::Matrix3d &R, const Eigen::Vector3d &t,
                    const Eigen::Vector3d &n, double d, Eigen::Vector3d &n_w,
                    double &d_w) {
  n_w = (R * n).normalized();
  d_w = d - n_w.dot(t);
}

/// Sequential RANSAC plane extraction on one frame's surfels. Points and
/// normals are in the optical frame. Detected planes are oriented so their
/// normal agrees with the (camera-facing) surfel normals.
std::vector<FramePlane> detect_frame_planes(const FrameSurfels &f, int frame,
                                            const PlaneGraphOptions &opt,
                                            std::mt19937 &rng) {
  std::vector<FramePlane> planes;
  const auto &pts = *f.points;
  const auto &nrm = *f.normals;
  const size_t n = pts.size();
  if (n < static_cast<size_t>(opt.min_plane_inliers))
    return planes;

  std::vector<char> used(n, 0);
  const double cos_thresh = std::cos(opt.ransac_normal_angle * M_PI / 180.0);
  const double dist_thresh = opt.ransac_distance;

  std::vector<size_t> pool;
  pool.reserve(n);

  for (int p = 0; p < opt.max_planes_per_frame; ++p) {
    pool.clear();
    for (size_t i = 0; i < n; ++i)
      if (!used[i])
        pool.push_back(i);
    if (pool.size() < static_cast<size_t>(opt.min_plane_inliers))
      break;

    std::uniform_int_distribution<size_t> pick(0, pool.size() - 1);
    Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
    double best_d = 0.0;
    int best_inliers = 0;

    for (int it = 0; it < opt.ransac_iterations; ++it) {
      const size_t seed_i = pool[pick(rng)];
      const auto &sp = pts[seed_i];
      const auto &sn = nrm[seed_i];
      Eigen::Vector3d nrm_h(sn.normal_x, sn.normal_y, sn.normal_z);
      if (!nrm_h.allFinite() || nrm_h.squaredNorm() < 1e-9)
        continue;
      nrm_h.normalize();
      const Eigen::Vector3d ph(sp.x, sp.y, sp.z);
      const double dh = -nrm_h.dot(ph); // plane through the seed point

      int cnt = 0;
      for (size_t k : pool) {
        const auto &pk = pts[k];
        const Eigen::Vector3d pp(pk.x, pk.y, pk.z);
        if (std::abs(nrm_h.dot(pp) + dh) > dist_thresh)
          continue;
        const auto &nk = nrm[k];
        Eigen::Vector3d nn(nk.normal_x, nk.normal_y, nk.normal_z);
        if (!nn.allFinite() || nn.squaredNorm() < 1e-9)
          continue;
        if (std::abs(nrm_h.dot(nn.normalized())) < cos_thresh)
          continue;
        ++cnt;
      }
      if (cnt > best_inliers) {
        best_inliers = cnt;
        best_normal = nrm_h;
        best_d = dh;
      }
    }

    if (best_inliers < opt.min_plane_inliers)
      break;

    // Refit the plane to its inliers (least squares centroid + mean normal),
    // and mark them consumed.
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_normal = Eigen::Vector3d::Zero();
    std::vector<size_t> inlier_idx;
    inlier_idx.reserve(best_inliers);
    for (size_t k : pool) {
      const auto &pk = pts[k];
      const Eigen::Vector3d pp(pk.x, pk.y, pk.z);
      if (std::abs(best_normal.dot(pp) + best_d) > dist_thresh)
        continue;
      const auto &nk = nrm[k];
      Eigen::Vector3d nn(nk.normal_x, nk.normal_y, nk.normal_z);
      if (!nn.allFinite() || nn.squaredNorm() < 1e-9)
        continue;
      if (std::abs(best_normal.dot(nn.normalized())) < cos_thresh)
        continue;
      centroid += pp;
      mean_normal += nn.normalized();
      inlier_idx.push_back(k);
    }
    if (inlier_idx.size() < static_cast<size_t>(opt.min_plane_inliers))
      break;
    centroid /= static_cast<double>(inlier_idx.size());
    if (mean_normal.squaredNorm() < 1e-9)
      break;
    mean_normal.normalize();
    // Orient the plane normal to agree with the surfel normals (camera-facing).
    if (mean_normal.dot(best_normal) < 0.0)
      mean_normal = -mean_normal;
    const double d = -mean_normal.dot(centroid);

    for (size_t k : inlier_idx)
      used[k] = 1;

    FramePlane fp;
    fp.frame = frame;
    fp.normal = mean_normal;
    fp.d = d;
    fp.inliers = static_cast<int>(inlier_idx.size());
    planes.push_back(fp);
  }

  return planes;
}

} // namespace

PlaneGraphOptimizer::PlaneGraphOptimizer(PlaneGraphOptions options)
    : options_(std::move(options)) {}

PlaneGraphResult
PlaneGraphOptimizer::optimize(std::vector<FrameSurfels> &frames) const {
  using gtsam::symbol_shorthand::P; // plane landmarks
  using gtsam::symbol_shorthand::X; // poses

  PlaneGraphResult result;
  const int N = static_cast<int>(frames.size());
  result.frames = N;
  if (N < 2) {
    core::warn("PlaneGraph: need at least 2 frames, got {}", N);
    return result;
  }

  // --- Seed poses ----------------------------------------------------------
  std::vector<Eigen::Matrix4d> seed(N);
  for (int i = 0; i < N; ++i)
    seed[i] = frames[i].world_pose.matrix().cast<double>();

  // --- 1. Per-frame plane detection ---------------------------------------
  std::mt19937 rng(options_.seed);
  std::vector<FramePlane> all_planes;
  for (int i = 0; i < N; ++i) {
    auto fp = detect_frame_planes(frames[i], i, options_, rng);
    const Eigen::Matrix3d R = seed[i].block<3, 3>(0, 0);
    const Eigen::Vector3d t = seed[i].block<3, 1>(0, 3);
    for (auto &p : fp) {
      plane_to_world(R, t, p.normal, p.d, p.world_normal, p.world_d);
      all_planes.push_back(p);
    }
  }
  result.planes_detected = static_cast<int>(all_planes.size());
  core::info("PlaneGraph: {} frames, {} plane detections", N,
             all_planes.size());
  if (all_planes.empty()) {
    core::warn("PlaneGraph: no planes detected; poses unchanged");
    return result;
  }

  // --- 2. Greedy cross-frame association into landmarks -------------------
  // Each landmark keeps a running mean world plane; a detection joins the first
  // landmark it agrees with (normal angle + offset). Detections are processed
  // largest-first so dominant planes seed the clusters.
  std::vector<int> order(all_planes.size());
  for (size_t i = 0; i < order.size(); ++i)
    order[i] = static_cast<int>(i);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return all_planes[a].inliers > all_planes[b].inliers;
  });

  struct Landmark {
    Eigen::Vector3d normal;
    double d;
    std::vector<int> obs; ///< indices into all_planes
  };
  std::vector<Landmark> landmarks;
  const double assoc_cos = std::cos(options_.assoc_normal_angle * M_PI / 180.0);

  std::vector<int> plane_landmark(all_planes.size(), -1);
  for (int idx : order) {
    const auto &pl = all_planes[idx];
    int best = -1;
    double best_score = -1.0;
    for (size_t l = 0; l < landmarks.size(); ++l) {
      const double c = pl.world_normal.dot(landmarks[l].normal);
      if (c < assoc_cos)
        continue;
      if (std::abs(pl.world_d - landmarks[l].d) > options_.assoc_distance)
        continue;
      if (c > best_score) {
        best_score = c;
        best = static_cast<int>(l);
      }
    }
    if (best < 0) {
      Landmark lm;
      lm.normal = pl.world_normal;
      lm.d = pl.world_d;
      lm.obs.push_back(idx);
      plane_landmark[idx] = static_cast<int>(landmarks.size());
      landmarks.push_back(std::move(lm));
    } else {
      auto &lm = landmarks[best];
      const double m = static_cast<double>(lm.obs.size());
      lm.normal = (lm.normal * m + pl.world_normal).normalized();
      lm.d = (lm.d * m + pl.world_d) / (m + 1.0);
      lm.obs.push_back(idx);
      plane_landmark[idx] = best;
    }
  }

  // Keep only landmarks observed by enough *distinct* frames.
  std::vector<int> landmark_key(landmarks.size(), -1);
  int kept = 0;
  for (size_t l = 0; l < landmarks.size(); ++l) {
    std::vector<int> distinct;
    for (int idx : landmarks[l].obs)
      distinct.push_back(all_planes[idx].frame);
    std::sort(distinct.begin(), distinct.end());
    distinct.erase(std::unique(distinct.begin(), distinct.end()),
                   distinct.end());
    if (static_cast<int>(distinct.size()) >= options_.min_landmark_observations)
      landmark_key[l] = kept++;
  }
  result.landmarks = kept;
  core::info("PlaneGraph: {} landmarks ({} pass >= {} obs)", landmarks.size(),
             kept, options_.min_landmark_observations);

  if (kept == 0) {
    core::warn("PlaneGraph: no landmark reached {} observations; poses "
               "unchanged. Lower --min-observations or relax association "
               "thresholds.",
               options_.min_landmark_observations);
    return result;
  }

  // --- 3. Build the factor graph ------------------------------------------
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  // Factors that must never be down-weighted by GNC: the gauge prior and the
  // odometry chain are trusted by construction (they come from the seed poses,
  // not from the fragile cross-frame plane association). Only the
  // plane-landmark factors are candidates for robust re-weighting. We record
  // the trusted factor indices here and hand them to GNC as known inliers
  // below. GncParams stores these as a TBB-allocated vector, so use its exact
  // IndexVector type rather than a plain std::vector<size_t>.
  using GncParamsLM = gtsam::GncParams<gtsam::LevenbergMarquardtParams>;
  GncParamsLM::IndexVector trusted_factors;

  for (int i = 0; i < N; ++i) {
    const gtsam::Rot3 R(seed[i].block<3, 3>(0, 0));
    const gtsam::Point3 tt(seed[i].block<3, 1>(0, 3));
    initial.insert(X(i), gtsam::Pose3(R, tt));
  }

  // Gauge: soft prior on the first pose.
  {
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << options_.prior_sigma_rot, options_.prior_sigma_rot,
         options_.prior_sigma_rot, options_.prior_sigma_trans,
         options_.prior_sigma_trans, options_.prior_sigma_trans)
            .finished());
    const gtsam::Rot3 R0(seed[0].block<3, 3>(0, 0));
    const gtsam::Point3 t0(seed[0].block<3, 1>(0, 3));
    graph.addPrior(X(0), gtsam::Pose3(R0, t0), prior_noise);
    trusted_factors.push_back(graph.size() - 1);
  }

  // Odometry from consecutive seed poses (conservative noise).
  {
    auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << options_.odometry_sigma_rot,
         options_.odometry_sigma_rot, options_.odometry_sigma_rot,
         options_.odometry_sigma_trans, options_.odometry_sigma_trans,
         options_.odometry_sigma_trans)
            .finished());
    for (int i = 0; i + 1 < N; ++i) {
      const Eigen::Matrix4d rel = seed[i].inverse() * seed[i + 1];
      const gtsam::Pose3 odom(gtsam::Rot3(rel.block<3, 3>(0, 0)),
                              gtsam::Point3(rel.block<3, 1>(0, 3)));
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          X(i), X(i + 1), odom, odom_noise);
      trusted_factors.push_back(graph.size() - 1);
    }
  }

  // Plane landmark variables + one OrientedPlane3Factor per observation.
  auto plane_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << options_.plane_sigma_normal,
       options_.plane_sigma_normal, options_.plane_sigma_distance)
          .finished());

  int plane_factors = 0;
  for (size_t l = 0; l < landmarks.size(); ++l) {
    const int key = landmark_key[l];
    if (key < 0)
      continue;
    // Landmark initial value: world plane (normal + Hessian offset).
    // gtsam::OrientedPlane3 stores (normal, d) in Hessian normal form
    // n.dot(x) + d = 0 — the SAME convention as our FramePlane (n.dot(p) + d
    // = 0), so d is passed through directly (no negation). This is verified by
    // the "OrientedPlane3 conventions: consistent observation has zero
    // residual" regression test: negating d makes a perfectly consistent
    // observation produce a large residual, which mis-scaled the whole graph.
    const gtsam::OrientedPlane3 lm_plane(gtsam::Unit3(landmarks[l].normal),
                                         landmarks[l].d);
    initial.insert(P(key), lm_plane);

    for (int idx : landmarks[l].obs) {
      const auto &fp = all_planes[idx];
      // Measurement: the plane as seen in the frame's optical frame, encoded as
      // Vector4 (nx, ny, nz, d) in the same Hessian convention (n.x + d = 0).
      const gtsam::Vector4 measured(fp.normal.x(), fp.normal.y(), fp.normal.z(),
                                    fp.d);
      graph.emplace_shared<ClonableOrientedPlane3Factor>(measured, plane_noise,
                                                         X(fp.frame), P(key));
      ++plane_factors;
    }
  }
  result.plane_factors = plane_factors;

  // --- 4. Solve ------------------------------------------------------------
  result.initial_error = graph.error(initial);

  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setMaxIterations(options_.max_iterations);

  gtsam::Values solution;
  try {
    if (options_.use_gnc) {
      using GncLM = gtsam::GncOptimizer<GncParamsLM>;
      GncParamsLM gnc_params(lm_params);
      // Only the plane-landmark factors are robustified; the gauge prior and
      // the odometry chain are trusted seeds and must keep full weight
      // throughout the GNC schedule. This is the principled replacement for the
      // residual-gating that failed in the #221 sweep (see
      // registration-improvements.md §Q3).
      gnc_params.setKnownInliers(trusted_factors);
      GncLM optimizer(graph, initial, gnc_params);
      // Raise the TLS inlier threshold from gtsam's default (1.0) so the
      // informative-but-drifted plane observations survive the GNC schedule;
      // only genuinely wrong associations should be rejected.
      optimizer.setInlierCostThresholds(
          static_cast<double>(options_.gnc_inlier_cost));
      solution = optimizer.optimize();
    } else {
      gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, lm_params);
      solution = optimizer.optimize();
      result.iterations = static_cast<int>(optimizer.iterations());
    }
  } catch (const std::exception &e) {
    core::warn("PlaneGraph: optimizer failed ({}); poses unchanged", e.what());
    return result; // converged stays false
  }
  result.converged = true;

  result.final_error = graph.error(solution);

  // --- Write optimized poses back into the frames -------------------------
  double max_shift = 0.0;
  for (int i = 0; i < N; ++i) {
    const gtsam::Pose3 pose = solution.at<gtsam::Pose3>(X(i));
    Eigen::Matrix4d M = pose.matrix();
    const double shift =
        (M.block<3, 1>(0, 3) - seed[i].block<3, 1>(0, 3)).norm();
    max_shift = std::max(max_shift, shift);
    Eigen::Affine3f aff;
    aff.matrix() = M.cast<float>();
    frames[i].world_pose = aff;
  }
  result.max_pose_shift = max_shift;

  core::info("PlaneGraph: error {:.4f} -> {:.4f} ({} plane factors, "
             "max pose shift {:.4f} m)",
             result.initial_error, result.final_error, plane_factors,
             max_shift);
  return result;
}

// ---------------------------------------------------------------------------
// Convention check exposed for the gtsam-free regression test
// "OrientedPlane3 conventions: consistent observation has zero residual".
//
// Given an off-origin optical-frame plane (n_o.x + d_o = 0, d_o != 0) and a
// known non-trivial pose, it transforms the plane to world by hand, builds the
// world landmark and the optical measurement with the SAME Hessian convention
// the optimizer uses (distance/w = +d, no negation), constructs an
// OrientedPlane3Factor and returns the factor error norm. A correct convention
// must yield ~0; the previous (negated) encoding returned ~2.1 here, which is
// what mis-scaled the real-data graph. d_o != 0 is essential: for planes
// through the origin (d = 0) the sign is invisible.
double plane_factor_consistent_residual() {
  using gtsam::symbol_shorthand::P;
  using gtsam::symbol_shorthand::X;

  const gtsam::Rot3 R = gtsam::Rot3::RzRyRx(0.3, -0.2, 0.15);
  const gtsam::Point3 t(0.5, -0.3, 1.0);
  const gtsam::Pose3 pose(R, t);

  const Eigen::Vector3d n_o = Eigen::Vector3d(0.2, -0.3, 1.0).normalized();
  const double d_o = -0.8; // optical Hessian offset (n_o.x + d_o = 0)

  // Transform optical plane -> world (rigid): n_w = R n_o, d_w = d_o - n_w.t.
  const Eigen::Vector3d n_w = R.matrix() * n_o;
  const double d_w = d_o - n_w.dot(t);

  // Same encoding as optimize(): landmark = (Unit3(n_w), d_w); measurement =
  // (n_o, d_o). No negation — gtsam OrientedPlane3 uses n.x + d = 0 directly.
  const gtsam::OrientedPlane3 lm(gtsam::Unit3(n_w), d_w);
  const gtsam::Vector4 measured(n_o.x(), n_o.y(), n_o.z(), d_o);
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1, 1, 1));
  gtsam::OrientedPlane3Factor factor(measured, noise, X(0), P(0));

  gtsam::Values v;
  v.insert(X(0), pose);
  v.insert(P(0), lm);
  return factor.unwhitenedError(v).norm();
}

} // namespace reusex::geometry
