// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See PlaneGraphOptimizer.hpp for the method overview and attribution.

#include "slam/PlaneGraphOptimizer.hpp"
#include "core/logging.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
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
  int frame = -1;                 ///< index into the frames vector
  Eigen::Vector3d normal;         ///< unit normal (optical frame)
  double d = 0.0;                 ///< plane offset (optical frame)
  int inliers = 0;                ///< supporting surfel count
  Eigen::Vector3d centroid;       ///< inlier centroid (optical frame)
  double radius = 0.0;            ///< in-plane RMS extent of inliers (m)
  Eigen::Vector3d world_normal;   ///< normal in world (current pose)
  double world_d = 0.0;           ///< offset in world (current pose)
  Eigen::Vector3d world_centroid; ///< inlier centroid in world (current pose)
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

/// Refresh the world-frame fields of a FramePlane from the frame's current
/// pose. Called once per round so association always operates on the poses the
/// previous round produced (EM-style alternation).
void refresh_world(FramePlane &p, const Eigen::Matrix4d &pose) {
  const Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
  const Eigen::Vector3d t = pose.block<3, 1>(0, 3);
  plane_to_world(R, t, p.normal, p.d, p.world_normal, p.world_d);
  p.world_centroid = R * p.centroid + t;
}

/// A cross-frame plane landmark: running world plane (normal + offset) plus the
/// pooled inlier geometry the hygiene checks need (overlap gate, degeneracy
/// test).
struct Landmark {
  Eigen::Vector3d normal;
  double d = 0.0;
  Eigen::Vector3d centroid_sum = Eigen::Vector3d::Zero(); ///< sum(w*centroid_w)
  double weight = 0.0;  ///< total inlier weight
  std::vector<int> obs; ///< indices into all_planes
};

/// Inlier-weighted footprint radius of a landmark: the mean in-plane RMS extent
/// of its member detections. Used by the overlap gate to size the merge window.
double landmark_radius(const Landmark &lm,
                       const std::vector<FramePlane> &planes) {
  double num = 0.0, den = 0.0;
  for (int idx : lm.obs) {
    const double w = static_cast<double>(std::max(1, planes[idx].inliers));
    num += w * planes[idx].radius;
    den += w;
  }
  return den > 0.0 ? num / den : 0.0;
}

/// Rejects a landmark ONLY when its member observation centroids form a
/// genuine, extended near-collinear LINE — the one geometry that leaves a
/// rotation about that line unobservable and lets the landmark bend the
/// trajectory. Two cases are explicitly kept:
///   - fewer than 3 observations (nothing to judge), and
///   - centroids that are effectively coincident (primary spread below an
///     absolute floor): identical viewpoints agree strongly, they do not form
///     a harmful line and their normal+offset still constrain 3 DoF.
/// Only when the primary spread is real (above the floor) AND the secondary
/// spread is a small fraction of it (SV ratio < min_landmark_spread_ratio) is
/// the landmark treated as a degenerate line and rejected.
bool landmark_well_conditioned(const Landmark &lm,
                               const std::vector<FramePlane> &planes,
                               const PlaneGraphOptions &opt) {
  if (opt.min_landmark_spread_ratio <= 0.0f)
    return true;
  const size_t m = lm.obs.size();
  if (m < 3)
    return true; // too few points to judge collinearity; keep it
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (int idx : lm.obs)
    mean += planes[idx].world_centroid;
  mean /= static_cast<double>(m);
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (int idx : lm.obs) {
    const Eigen::Vector3d r = planes[idx].world_centroid - mean;
    cov += r * r.transpose();
  }
  cov /= static_cast<double>(m); // per-observation variance -> length^2 units
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  // Eigenvalues ascending; sqrt gives an RMS length along each axis.
  const double s1 = std::sqrt(std::max(0.0, es.eigenvalues()(2))); // primary
  const double s2 = std::sqrt(std::max(0.0, es.eigenvalues()(1))); // secondary
  // Absolute floor (m): below this the centroids are effectively coincident
  // (identical viewpoints), which is agreement, not a degenerate line.
  constexpr double kCoincidentFloor = 0.05; // 5 cm RMS
  if (s1 < kCoincidentFloor)
    return true;
  return (s2 / s1) >= static_cast<double>(opt.min_landmark_spread_ratio);
}

/// Number of independent normal directions (1, 2 or 3) among a frame's landmark
/// observations, via the rank of their outer-product scatter under a fixed
/// relative tolerance. Used by the observability guard: a frame spanning < 2
/// directions is free along the missing ones on plane factors alone.
int normal_span(const std::vector<Eigen::Vector3d> &normals) {
  if (normals.empty())
    return 0;
  Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
  for (const auto &n : normals)
    m += n * n.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(m);
  const double lmax = es.eigenvalues()(2);
  if (lmax <= 1e-12)
    return 0;
  const double tol = 0.02 * lmax; // 2% of the dominant direction's energy
  int rank = 0;
  for (int k = 0; k < 3; ++k)
    if (es.eigenvalues()(k) > tol)
      ++rank;
  return rank;
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

    // In-plane RMS extent of the inliers about the centroid: the footprint
    // "radius" used later by the overlap gate. Computed from the components of
    // (p - centroid) orthogonal to the plane normal, so out-of-plane scatter
    // (thickness) does not inflate it.
    double sum_sq = 0.0;
    for (size_t k : inlier_idx) {
      const auto &pk = pts[k];
      const Eigen::Vector3d rp = Eigen::Vector3d(pk.x, pk.y, pk.z) - centroid;
      const Eigen::Vector3d in_plane = rp - mean_normal.dot(rp) * mean_normal;
      sum_sq += in_plane.squaredNorm();
    }
    const double radius =
        std::sqrt(sum_sq / static_cast<double>(inlier_idx.size()));

    for (size_t k : inlier_idx)
      used[k] = 1;

    FramePlane fp;
    fp.frame = frame;
    fp.normal = mean_normal;
    fp.d = d;
    fp.inliers = static_cast<int>(inlier_idx.size());
    fp.centroid = centroid;
    fp.radius = radius;
    planes.push_back(fp);
  }

  return planes;
}

} // namespace

PlaneGraphOptimizer::PlaneGraphOptimizer(PlaneGraphOptions options)
    : options_(std::move(options)) {}

PlaneGraphResult
PlaneGraphOptimizer::optimize(std::vector<FrameSurfels> &frames,
                              const std::vector<LoopEdge> &loop_edges) const {
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
  // seed[] is the ORIGINAL input pose of each frame, held fixed for the whole
  // run: odometry factors and the gauge prior are always derived from it (the
  // trusted trajectory), and max_pose_shift is measured against it. cur[] is
  // the working pose, updated after each round and used to re-associate.
  std::vector<Eigen::Matrix4d> seed(N);
  for (int i = 0; i < N; ++i)
    seed[i] = frames[i].world_pose.matrix().cast<double>();
  std::vector<Eigen::Matrix4d> cur = seed;

  // --- 1. Per-frame plane detection (once; geometry is pose-independent) ----
  std::mt19937 rng(options_.seed);
  std::vector<FramePlane> all_planes;
  for (int i = 0; i < N; ++i) {
    auto fp = detect_frame_planes(frames[i], i, options_, rng);
    for (auto &p : fp)
      all_planes.push_back(p);
  }
  result.planes_detected = static_cast<int>(all_planes.size());
  core::info("PlaneGraph: {} frames, {} plane detections", N,
             all_planes.size());
  if (all_planes.empty()) {
    core::warn("PlaneGraph: no planes detected; poses unchanged");
    return result;
  }

  const double assoc_cos = std::cos(options_.assoc_normal_angle * M_PI / 180.0);
  using GncParamsLM = gtsam::GncParams<gtsam::LevenbergMarquardtParams>;

  // Self-calibrating reference for per-observation inlier weighting: the median
  // detection inlier count. Using the median (not a fixed constant) keeps the
  // weighting invariant to --sampling-factor, which scales all inlier counts.
  double ref_inliers = 1.0;
  if (options_.plane_weight_by_inliers) {
    std::vector<int> counts;
    counts.reserve(all_planes.size());
    for (const auto &p : all_planes)
      counts.push_back(std::max(1, p.inliers));
    std::nth_element(counts.begin(), counts.begin() + counts.size() / 2,
                     counts.end());
    ref_inliers = std::max(1.0, static_cast<double>(counts[counts.size() / 2]));
  }

  double round_max_shift = 0.0;

  // --- Alternating rounds: associate at cur poses -> optimize -> refit ------
  for (int round = 0; round < std::max(1, options_.assoc_rounds); ++round) {
    result.rounds = round + 1;

    // Refresh every detection's world quantities from the current poses.
    for (auto &p : all_planes)
      refresh_world(p, cur[p.frame]);

    // -- Overlap-aware greedy association ----------------------------------
    // Detections are processed largest-first so dominant planes seed clusters.
    // A detection joins the best-scoring landmark that agrees in normal AND
    // offset AND whose in-plane footprint OVERLAPS it (centroid gap projected
    // into the shared plane < sum of radii + margin). The overlap gate is what
    // keeps two offset/disjoint parallel walls from aliasing into one landmark.
    std::vector<int> order(all_planes.size());
    for (size_t i = 0; i < order.size(); ++i)
      order[i] = static_cast<int>(i);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return all_planes[a].inliers > all_planes[b].inliers;
    });

    std::vector<Landmark> landmarks;
    int rejected_overlap = 0;
    for (int idx : order) {
      const auto &pl = all_planes[idx];
      const double w = static_cast<double>(std::max(1, pl.inliers));
      int best = -1;
      double best_score = -1.0;
      for (size_t l = 0; l < landmarks.size(); ++l) {
        const double c = pl.world_normal.dot(landmarks[l].normal);
        if (c < assoc_cos)
          continue;
        if (std::abs(pl.world_d - landmarks[l].d) > options_.assoc_distance)
          continue;
        if (options_.assoc_overlap_margin > 0.0f) {
          // In-plane gap between this detection's centroid and the landmark
          // centroid (component orthogonal to the landmark normal). If it
          // exceeds the two footprint radii plus the margin, the surfaces do
          // not overlap and must NOT merge even though normal+offset agree.
          const Eigen::Vector3d lm_centroid =
              landmarks[l].centroid_sum / landmarks[l].weight;
          const Eigen::Vector3d gap = pl.world_centroid - lm_centroid;
          const Eigen::Vector3d in_plane =
              gap - landmarks[l].normal.dot(gap) * landmarks[l].normal;
          const double allowed = pl.radius +
                                 landmark_radius(landmarks[l], all_planes) +
                                 options_.assoc_overlap_margin;
          if (in_plane.norm() > allowed) {
            ++rejected_overlap;
            continue;
          }
        }
        if (c > best_score) {
          best_score = c;
          best = static_cast<int>(l);
        }
      }
      if (best < 0) {
        Landmark lm;
        lm.normal = pl.world_normal;
        lm.d = pl.world_d;
        lm.centroid_sum = pl.world_centroid * w;
        lm.weight = w;
        lm.obs.push_back(idx);
        landmarks.push_back(std::move(lm));
      } else {
        auto &lm = landmarks[best];
        const double m = static_cast<double>(lm.obs.size());
        lm.normal = (lm.normal * m + pl.world_normal).normalized();
        lm.d = (lm.d * m + pl.world_d) / (m + 1.0);
        lm.centroid_sum += pl.world_centroid * w;
        lm.weight += w;
        lm.obs.push_back(idx);
      }
    }
    result.landmarks_rejected_overlap = rejected_overlap;

    // -- Landmark hygiene: distinct-frame count + degeneracy rejection ------
    // Keep only landmarks seen by enough distinct frames whose inlier centroids
    // are NOT near-collinear. Collinear centroids leave a rotation about that
    // line unconstrained; such a landmark can bend the trajectory freely and
    // must be dropped.
    std::vector<int> landmark_key(landmarks.size(), -1);
    int kept = 0;
    int rejected_degenerate = 0;
    for (size_t l = 0; l < landmarks.size(); ++l) {
      std::vector<int> distinct;
      for (int idx : landmarks[l].obs)
        distinct.push_back(all_planes[idx].frame);
      std::sort(distinct.begin(), distinct.end());
      distinct.erase(std::unique(distinct.begin(), distinct.end()),
                     distinct.end());
      if (static_cast<int>(distinct.size()) <
          options_.min_landmark_observations)
        continue;
      if (!landmark_well_conditioned(landmarks[l], all_planes, options_)) {
        ++rejected_degenerate;
        continue;
      }
      landmark_key[l] = kept++;
    }
    result.landmarks_rejected_degenerate = rejected_degenerate;
    result.landmarks = kept;
    core::info("PlaneGraph round {}: {} raw landmarks, {} kept (>= {} obs, "
               "non-degenerate), {} overlap-rejected, {} degenerate-rejected",
               round + 1, landmarks.size(), kept,
               options_.min_landmark_observations, rejected_overlap,
               rejected_degenerate);

    // No plane landmarks: only bail out if there are ALSO no loop edges. When
    // loop edges are present the graph (gauge + odometry + loop factors) is
    // still well-posed and worth solving, so fall through to build it.
    if (kept == 0 && loop_edges.empty()) {
      if (round == 0) {
        core::warn("PlaneGraph: no landmark passed the hygiene checks; poses "
                   "unchanged. Lower --min-observations or relax association / "
                   "overlap thresholds.");
        return result;
      }
      core::warn(
          "PlaneGraph: round {} produced no valid landmarks; keeping the "
          "previous round's poses",
          round + 1);
      break;
    }

    // -- Observability guard: which frames under-constrain the trajectory ---
    // For each frame, collect the normals of the landmark observations it
    // contributes. A frame spanning < 2 independent normal directions is free
    // to slide/rotate along the missing directions on plane factors alone; we
    // tighten its odometry so the trusted trajectory holds it there.
    std::vector<std::vector<Eigen::Vector3d>> frame_normals(N);
    for (size_t l = 0; l < landmarks.size(); ++l) {
      if (landmark_key[l] < 0)
        continue;
      for (int idx : landmarks[l].obs)
        frame_normals[all_planes[idx].frame].push_back(landmarks[l].normal);
    }
    std::vector<bool> underconstrained(N, false);
    int nuc = 0;
    for (int i = 0; i < N; ++i) {
      if (normal_span(frame_normals[i]) < 2) {
        underconstrained[i] = true;
        ++nuc;
      }
    }
    result.underconstrained_frames = nuc;

    // -- Build the factor graph --------------------------------------------
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    GncParamsLM::IndexVector trusted_factors;

    for (int i = 0; i < N; ++i) {
      const gtsam::Rot3 R(cur[i].block<3, 3>(0, 0));
      const gtsam::Point3 tt(cur[i].block<3, 1>(0, 3));
      initial.insert(X(i), gtsam::Pose3(R, tt));
    }

    // Gauge: soft prior on the first pose (from the fixed seed).
    {
      auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << options_.prior_sigma_rot,
           options_.prior_sigma_rot, options_.prior_sigma_rot,
           options_.prior_sigma_trans, options_.prior_sigma_trans,
           options_.prior_sigma_trans)
              .finished());
      const gtsam::Rot3 R0(seed[0].block<3, 3>(0, 0));
      const gtsam::Point3 t0(seed[0].block<3, 1>(0, 3));
      graph.addPrior(X(0), gtsam::Pose3(R0, t0), prior_noise);
      trusted_factors.push_back(graph.size() - 1);
    }

    // Odometry from consecutive SEED poses (the trusted trajectory). A frame
    // flagged under-constrained gets both of its odometry factors tightened
    // (sigmas scaled by underconstrained_odom_scale) so planes cannot bend the
    // trajectory where they under-constrain it.
    {
      const double base_r = options_.odometry_sigma_rot;
      const double base_t = options_.odometry_sigma_trans;
      const double scale = options_.underconstrained_odom_scale;
      for (int i = 0; i + 1 < N; ++i) {
        const double s =
            (underconstrained[i] || underconstrained[i + 1]) ? scale : 1.0;
        auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << base_r * s, base_r * s, base_r * s, base_t * s,
             base_t * s, base_t * s)
                .finished());
        const Eigen::Matrix4d rel = seed[i].inverse() * seed[i + 1];
        const gtsam::Pose3 odom(gtsam::Rot3(rel.block<3, 3>(0, 0)),
                                gtsam::Point3(rel.block<3, 1>(0, 3)));
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            X(i), X(i + 1), odom, odom_noise);
        trusted_factors.push_back(graph.size() - 1);
      }
    }

    // Wide-baseline loop edges (P2). Each is a relative-pose BetweenFactor
    // between two temporally distant frames, derived from feature matching +
    // depth (see LoopClosure). They are deliberately NOT added to
    // trusted_factors, so GNC can down-weight a wrong loop edge instead of
    // letting it corrupt the trajectory (the basin problem's blast radius is
    // neutralised by construction). Endpoints out of range are skipped.
    int loop_edge_count = 0;
    for (const auto &e : loop_edges) {
      if (e.i < 0 || e.j < 0 || e.i >= N || e.j >= N || e.i == e.j)
        continue;
      auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << e.sigma_rot, e.sigma_rot, e.sigma_rot,
           e.sigma_trans, e.sigma_trans, e.sigma_trans)
              .finished());
      const gtsam::Pose3 rel(gtsam::Rot3(e.T_ij.block<3, 3>(0, 0)),
                             gtsam::Point3(e.T_ij.block<3, 1>(0, 3)));
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(e.i), X(e.j),
                                                               rel, loop_noise);
      ++loop_edge_count;
    }
    result.loop_edges = loop_edge_count;

    // Plane landmark variables + one OrientedPlane3Factor per observation.
    // Each factor's noise is (optionally) scaled by the reliability of its
    // supporting fit: sigma *= clamp(sqrt(ref / inliers), min, max). A plane
    // backed by many surfels earns a tighter sigma (pulls harder); a small,
    // weak plane is loosened so it cannot warp the trajectory. This is the fix
    // for the measured "denser extraction hurts" regression — the extra planes
    // are mostly small, and equal weighting let them dominate wrongly.
    const double base_sig_n = options_.plane_sigma_normal;
    const double base_sig_d = options_.plane_sigma_distance;

    int plane_factors = 0;
    for (size_t l = 0; l < landmarks.size(); ++l) {
      const int key = landmark_key[l];
      if (key < 0)
        continue;
      // gtsam::OrientedPlane3 stores (normal, d) in Hessian form n.x + d = 0 —
      // the SAME convention as FramePlane, so d passes through directly (see
      // the "consistent observation has zero residual" regression test).
      const gtsam::OrientedPlane3 lm_plane(gtsam::Unit3(landmarks[l].normal),
                                           landmarks[l].d);
      initial.insert(P(key), lm_plane);

      for (int idx : landmarks[l].obs) {
        const auto &fp = all_planes[idx];
        double scale = 1.0;
        if (options_.plane_weight_by_inliers) {
          scale = std::sqrt(ref_inliers / std::max(1, fp.inliers));
          scale =
              std::clamp(scale, static_cast<double>(options_.plane_weight_min),
                         static_cast<double>(options_.plane_weight_max));
        }
        auto plane_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << base_sig_n * scale, base_sig_n * scale,
             base_sig_d * scale)
                .finished());
        const gtsam::Vector4 measured(fp.normal.x(), fp.normal.y(),
                                      fp.normal.z(), fp.d);
        graph.emplace_shared<ClonableOrientedPlane3Factor>(
            measured, plane_noise, X(fp.frame), P(key));
        ++plane_factors;
      }
    }
    result.plane_factors = plane_factors;

    // -- Solve --------------------------------------------------------------
    const double round_initial_error = graph.error(initial);
    if (round == 0)
      result.initial_error = round_initial_error;

    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.setMaxIterations(options_.max_iterations);

    gtsam::Values solution;
    try {
      if (options_.use_gnc) {
        using GncLM = gtsam::GncOptimizer<GncParamsLM>;
        GncParamsLM gnc_params(lm_params);
        gnc_params.setKnownInliers(trusted_factors);
        GncLM optimizer(graph, initial, gnc_params);
        optimizer.setInlierCostThresholds(
            static_cast<double>(options_.gnc_inlier_cost));
        solution = optimizer.optimize();
      } else {
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, lm_params);
        solution = optimizer.optimize();
        result.iterations = static_cast<int>(optimizer.iterations());
      }
    } catch (const std::exception &e) {
      core::warn("PlaneGraph: optimizer failed in round {} ({}); keeping "
                 "previous poses",
                 round + 1, e.what());
      if (round == 0)
        return result; // never solved -> converged stays false
      break;
    }
    result.converged = true;
    result.final_error = graph.error(solution);

    // Adopt the round's poses into cur[] and measure this round's motion.
    round_max_shift = 0.0;
    for (int i = 0; i < N; ++i) {
      const gtsam::Pose3 pose = solution.at<gtsam::Pose3>(X(i));
      const Eigen::Matrix4d M = pose.matrix();
      round_max_shift =
          std::max(round_max_shift,
                   (M.block<3, 1>(0, 3) - cur[i].block<3, 1>(0, 3)).norm());
      cur[i] = M;
    }

    core::info("PlaneGraph round {}: error {:.4f} -> {:.4f} ({} plane factors, "
               "{} under-constrained frames tightened, round shift {:.4f} m)",
               round + 1, round_initial_error, result.final_error,
               plane_factors, nuc, round_max_shift);

    if (round_max_shift < options_.assoc_round_tol) {
      core::info(
          "PlaneGraph: round shift {:.4f} m below tol {:.4f} m; stopping",
          round_max_shift, options_.assoc_round_tol);
      break;
    }
  }

  if (!result.converged)
    return result;

  // --- Write final poses back into the frames; report shift vs seed --------
  double max_shift = 0.0;
  for (int i = 0; i < N; ++i) {
    max_shift =
        std::max(max_shift,
                 (cur[i].block<3, 1>(0, 3) - seed[i].block<3, 1>(0, 3)).norm());
    Eigen::Affine3f aff;
    aff.matrix() = cur[i].cast<float>();
    frames[i].world_pose = aff;
  }
  result.max_pose_shift = max_shift;

  core::info("PlaneGraph: {} rounds, error {:.4f} -> {:.4f}, max pose shift "
             "{:.4f} m",
             result.rounds, result.initial_error, result.final_error,
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
