// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// See JointPairwiseRegistration.hpp for the method overview and attribution.

#include "geometry/registration/JointPairwiseRegistration.hpp"
#include "core/logging.hpp"
#include "geometry/transform_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

namespace reusex::geometry {

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Per-frame geometry cached for one outer iteration at the current pose.
/// The k-d tree is held by shared_ptr: pcl::KdTreeFLANN is not safely copyable
/// (copying/moving it corrupts the internal FLANN index), so the cache must
/// move only the pointer when it is stored into / reordered within a vector.
struct FrameCache {
  std::vector<Eigen::Vector3d> world_pts;
  std::vector<Eigen::Vector3d> world_normals;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;
  std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> kdtree;
  Eigen::Vector3d center; ///< camera center in world (pose translation)
  Eigen::Vector3d view;   ///< optical +Z axis in world
};

FrameCache build_cache(const FrameSurfels &f, const Eigen::Matrix4d &T) {
  FrameCache c;
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);
  const size_t n = f.points->size();
  c.world_pts.resize(n);
  c.world_normals.resize(n);
  c.xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  c.xyz->resize(n);
  for (size_t i = 0; i < n; ++i) {
    const auto &p = (*f.points)[i];
    Eigen::Vector3d po(p.x, p.y, p.z);
    Eigen::Vector3d pw = R * po + t;
    c.world_pts[i] = pw;
    (*c.xyz)[i].x = static_cast<float>(pw.x());
    (*c.xyz)[i].y = static_cast<float>(pw.y());
    (*c.xyz)[i].z = static_cast<float>(pw.z());

    const auto &nrm = (*f.normals)[i];
    Eigen::Vector3d no(nrm.normal_x, nrm.normal_y, nrm.normal_z);
    if (!no.allFinite() || no.squaredNorm() < 1e-12)
      c.world_normals[i] = Eigen::Vector3d::Zero();
    else
      c.world_normals[i] = (R * no).normalized();
  }
  c.kdtree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  // Guard: only index a non-empty cloud (a frame whose points all turned
  // non-finite would otherwise trigger PCL's "empty input cloud" error).
  if (!c.xyz->empty())
    c.kdtree->setInputCloud(c.xyz);
  c.center = t;
  c.view = R * Eigen::Vector3d(0.0, 0.0, 1.0);
  return c;
}

/// One source->target match, stored as point indices so the residual can be
/// re-evaluated at any trial pose (needed for Levenberg-Marquardt step
/// acceptance). The robust weight is fixed at the linearization pose (IRLS).
struct Corr {
  int fa, fb; ///< frame indices (source, target)
  int si, ti; ///< source point index in fa, target point/normal index in fb
  double w;   ///< robust + geometric weight (fixed during one outer iteration)
};

double robust_weight(double r, const JprParams &p) {
  const double s = std::max(1e-6f, p.robust_width);
  if (p.kernel == JprParams::Kernel::huber) {
    const double a = std::abs(r);
    return a <= s ? 1.0 : s / a;
  }
  // Welsch (default).
  const double x = r / s;
  return std::exp(-0.5 * x * x);
}

} // namespace

JointPairwiseRegistration::JointPairwiseRegistration(JprParams params)
    : params_(std::move(params)) {}

JprResult JointPairwiseRegistration::refine(
    std::vector<FrameSurfels> &frames) const {
  JprResult result;
  const int N = static_cast<int>(frames.size());
  result.frames = N;
  if (N < 2) {
    core::warn("JPR: need at least 2 frames, got {}", N);
    return result;
  }

  // --- Seed poses (for the soft prior) ------------------------------------
  std::vector<Eigen::Matrix4d> seed(N);
  for (int i = 0; i < N; ++i)
    seed[i] = frames[i].world_pose.matrix().cast<double>();

  // --- Free-variable remapping (anchored frame is removed) ----------------
  std::vector<int> free_index(N, -1);
  int M = 0;
  for (int i = 0; i < N; ++i)
    if (i != params_.anchor_frame)
      free_index[i] = M++;
  if (M == 0) {
    core::warn("JPR: all frames anchored, nothing to optimize");
    return result;
  }
  if (params_.anchor_frame < 0 && params_.prior_weight <= 0.0f)
    core::warn("JPR: no anchor frame and prior_weight=0 — gauge is "
               "unconstrained and the solution may drift");

  // --- Candidate pair graph (temporal + spatial overlap) ------------------
  // Frames are assumed ordered by node_id (caller guarantees this), so index
  // adjacency approximates temporal adjacency.
  std::set<std::pair<int, int>> pair_set;
  const int win = std::max(1, params_.neighbor_window);
  for (int i = 0; i < N; ++i)
    for (int d = 1; d <= win && i + d < N; ++d)
      pair_set.emplace(i, i + d);

  // Spatial pairs: camera centers within a radius with agreeing view dirs.
  {
    auto centers = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    centers->resize(N);
    std::vector<Eigen::Vector3d> views(N);
    for (int i = 0; i < N; ++i) {
      const Eigen::Vector3d t =
          frames[i].world_pose.translation().cast<double>();
      (*centers)[i].x = static_cast<float>(t.x());
      (*centers)[i].y = static_cast<float>(t.y());
      (*centers)[i].z = static_cast<float>(t.z());
      views[i] = frames[i].world_pose.rotation().cast<double>() *
                 Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> ctree;
    ctree.setInputCloud(centers);
    const float spatial_radius = 1.0f; // meters; interior handheld scale
    std::vector<int> idx;
    std::vector<float> dist;
    for (int i = 0; i < N; ++i) {
      ctree.radiusSearch((*centers)[i], spatial_radius, idx, dist);
      for (int j : idx) {
        if (j <= i)
          continue;
        if (views[i].dot(views[j]) < 0.5) // > 60 deg apart
          continue;
        pair_set.emplace(i, j);
      }
    }
  }
  std::vector<std::pair<int, int>> pairs(pair_set.begin(), pair_set.end());
  core::info("JPR: {} frames, {} candidate pairs", N, pairs.size());

  const double cos_thresh =
      std::cos(params_.normal_angle_threshold * M_PI / 180.0);
  const double max_d2 =
      static_cast<double>(params_.max_corr_distance) * params_.max_corr_distance;

  const double lambda2 =
      static_cast<double>(params_.prior_weight) * params_.prior_weight;

  // Working poses (double precision). frames[].world_pose is the seed; we keep
  // Tcur as the authoritative state during optimization and write back at end.
  std::vector<Eigen::Matrix4d> Tcur(N);
  for (int i = 0; i < N; ++i)
    Tcur[i] = frames[i].world_pose.matrix().cast<double>();

  // Build correspondences for the given poses (NN search in world space).
  // Stores point indices so residuals can be re-evaluated at any trial pose.
  auto build_corrs = [&](const std::vector<FrameCache> &cache) {
    std::vector<Corr> corrs;
    std::vector<int> nn_idx(1);
    std::vector<float> nn_dist(1);
    for (const auto &pr : pairs) {
      const FrameCache &ca = cache[pr.first];
      const FrameCache &cb = cache[pr.second];
      if (cb.xyz->empty())
        continue;
      for (size_t si = 0; si < ca.world_pts.size(); ++si) {
        const Eigen::Vector3d &na = ca.world_normals[si];
        if (na.squaredNorm() < 0.5)
          continue;
        const Eigen::Vector3d &Pa = ca.world_pts[si];
        if (!Pa.allFinite())
          continue;
        pcl::PointXYZ q;
        q.x = static_cast<float>(Pa.x());
        q.y = static_cast<float>(Pa.y());
        q.z = static_cast<float>(Pa.z());
        if (cb.kdtree->nearestKSearch(q, 1, nn_idx, nn_dist) < 1)
          continue;
        if (nn_dist[0] > max_d2)
          continue;
        const int ti = nn_idx[0];
        const Eigen::Vector3d &nb = cb.world_normals[ti];
        if (nb.squaredNorm() < 0.5 || na.dot(nb) < cos_thresh)
          continue;

        const double r = nb.dot(Pa - cb.world_pts[ti]);
        const double w = robust_weight(r, params_) * std::max(0.0, na.dot(nb));
        if (w > 0.0)
          corrs.push_back(Corr{pr.first, pr.second, static_cast<int>(si), ti, w});
      }
    }
    return corrs;
  };

  // Point-to-plane residual for a correspondence at the given poses.
  // Also returns world point P (source), Q (target) and plane normal n.
  auto geom = [&](const Corr &c, const std::vector<Eigen::Matrix4d> &T,
                  Eigen::Vector3d &P, Eigen::Vector3d &Q,
                  Eigen::Vector3d &n) -> double {
    const auto &pa = (*frames[c.fa].points)[c.si];
    const auto &qb = (*frames[c.fb].points)[c.ti];
    const auto &nb = (*frames[c.fb].normals)[c.ti];
    const Eigen::Matrix3d Ra = T[c.fa].block<3, 3>(0, 0);
    const Eigen::Matrix3d Rb = T[c.fb].block<3, 3>(0, 0);
    P = Ra * Eigen::Vector3d(pa.x, pa.y, pa.z) + T[c.fa].block<3, 1>(0, 3);
    Q = Rb * Eigen::Vector3d(qb.x, qb.y, qb.z) + T[c.fb].block<3, 1>(0, 3);
    n = (Rb * Eigen::Vector3d(nb.normal_x, nb.normal_y, nb.normal_z))
            .normalized();
    return n.dot(P - Q);
  };

  // Total energy: weighted point-to-plane residuals + soft prior to seeds.
  auto energy = [&](const std::vector<Corr> &corrs,
                    const std::vector<Eigen::Matrix4d> &T) {
    double e = 0.0;
    Eigen::Vector3d P, Q, n;
    for (const auto &c : corrs) {
      const double r = geom(c, T, P, Q, n);
      e += c.w * r * r;
    }
    if (lambda2 > 0.0)
      for (int i = 0; i < N; ++i) {
        if (free_index[i] < 0)
          continue;
        e += lambda2 * se3::log(T[i] * seed[i].inverse()).squaredNorm();
      }
    return e;
  };

  auto data_rms = [&](const std::vector<Corr> &corrs,
                      const std::vector<Eigen::Matrix4d> &T) {
    if (corrs.empty())
      return 0.0;
    double s = 0.0;
    Eigen::Vector3d P, Q, n;
    for (const auto &c : corrs) {
      const double r = geom(c, T, P, Q, n);
      s += r * r;
    }
    return std::sqrt(s / static_cast<double>(corrs.size()));
  };

  // Levenberg-Marquardt damping factor (scales the diagonal); adapted by the
  // step-acceptance test so the energy decreases monotonically.
  double lambda_lm = 1e-3;
  int iter = 0;
  for (; iter < params_.max_iterations; ++iter) {
    std::vector<FrameCache> cache(N);
    for (int i = 0; i < N; ++i)
      cache[i] = build_cache(frames[i], Tcur[i]);
    std::vector<Corr> corrs = build_corrs(cache);

    const double rms = data_rms(corrs, Tcur);
    if (iter == 0)
      result.initial_rms = rms;
    core::debug("JPR iter {}: {} correspondences, point-to-plane RMS {:.5f} m",
                iter, corrs.size(), rms);
    if (corrs.empty()) {
      core::warn("JPR: no correspondences found; stopping");
      break;
    }

    // --- Assemble the Gauss-Newton system (data + prior) at Tcur ----------
    std::vector<Matrix6d> Hdiag(M, Matrix6d::Zero());
    std::map<std::pair<int, int>, Matrix6d> Hoff;
    Eigen::VectorXd g = Eigen::VectorXd::Zero(6 * M);

    auto add_self = [&](int frame, const Vector6d &J, double w, double r) {
      const int fi = free_index[frame];
      if (fi < 0)
        return;
      Hdiag[fi].noalias() += w * J * J.transpose();
      g.segment<6>(6 * fi).noalias() += (w * r) * J;
    };

    {
      Eigen::Vector3d P, Q, n;
      for (const auto &c : corrs) {
        const double r = geom(c, Tcur, P, Q, n);
        Vector6d Ja, Jb;
        Ja << P.cross(n), n;       // d r / d xi_a
        Jb << -(Q.cross(n)), -n;   // d r / d xi_b
        add_self(c.fa, Ja, c.w, r);
        add_self(c.fb, Jb, c.w, r);
        const int ia = free_index[c.fa];
        const int ib = free_index[c.fb];
        if (ia >= 0 && ib >= 0) {
          if (ia < ib)
            Hoff[{ia, ib}].noalias() += c.w * Ja * Jb.transpose();
          else
            Hoff[{ib, ia}].noalias() += c.w * Jb * Ja.transpose();
        }
      }
    }

    // Soft prior toward seed poses (regularizes gauge + prevents drift).
    if (lambda2 > 0.0)
      for (int i = 0; i < N; ++i) {
        const int fi = free_index[i];
        if (fi < 0)
          continue;
        const Vector6d rho = se3::log(Tcur[i] * seed[i].inverse());
        Hdiag[fi].diagonal().array() += lambda2;
        g.segment<6>(6 * fi).noalias() += lambda2 * rho;
      }

    const double E0 = energy(corrs, Tcur);

    // --- LM inner loop: damp until a step decreases the energy ------------
    bool accepted = false;
    double max_step = 0.0;
    for (int attempt = 0; attempt < 10 && !accepted; ++attempt) {
      std::vector<Eigen::Triplet<double>> trips;
      trips.reserve(static_cast<size_t>(M) * 36 + Hoff.size() * 72);
      auto emit_block = [&](int br, int bc, const Matrix6d &B) {
        for (int r = 0; r < 6; ++r)
          for (int col = 0; col < 6; ++col)
            if (B(r, col) != 0.0)
              trips.emplace_back(6 * br + r, 6 * bc + col, B(r, col));
      };
      for (int i = 0; i < M; ++i) {
        Matrix6d B = Hdiag[i];
        B.diagonal() *= (1.0 + lambda_lm); // Marquardt diagonal scaling
        emit_block(i, i, B);
      }
      for (const auto &kv : Hoff) {
        emit_block(kv.first.first, kv.first.second, kv.second);
        emit_block(kv.first.second, kv.first.first, kv.second.transpose());
      }

      Eigen::SparseMatrix<double> H(6 * M, 6 * M);
      H.setFromTriplets(trips.begin(), trips.end());
      Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
      solver.compute(H);
      if (solver.info() != Eigen::Success) {
        lambda_lm *= 10.0;
        continue;
      }
      const Eigen::VectorXd delta = solver.solve(-g);
      if (solver.info() != Eigen::Success || !delta.allFinite()) {
        lambda_lm *= 10.0;
        continue;
      }

      std::vector<Eigen::Matrix4d> Ttrial = Tcur;
      double step = 0.0;
      for (int i = 0; i < N; ++i) {
        const int fi = free_index[i];
        if (fi < 0)
          continue;
        const Vector6d xi = delta.segment<6>(6 * fi);
        step = std::max(step, xi.norm());
        Ttrial[i] = se3::exp(xi) * Tcur[i];
      }

      const double E1 = energy(corrs, Ttrial);
      if (E1 < E0) {
        Tcur = std::move(Ttrial);
        max_step = step;
        lambda_lm = std::max(lambda_lm * 0.33, 1e-7);
        accepted = true;
      } else {
        lambda_lm *= 10.0; // reject: damp harder, retry
      }
    }

    if (!accepted) {
      core::debug("JPR: no energy-decreasing step at iter {}, converged", iter);
      ++iter;
      break;
    }
    if (max_step < params_.convergence_eps) {
      core::debug("JPR: converged at iter {} (max step {:.2e})", iter,
                  max_step);
      ++iter;
      break;
    }
  }

  // Write refined poses back into the frames.
  for (int i = 0; i < N; ++i) {
    Eigen::Affine3f aff;
    aff.matrix() = Tcur[i].cast<float>();
    frames[i].world_pose = aff;
  }

  // Final residual.
  {
    std::vector<FrameCache> cache(N);
    for (int i = 0; i < N; ++i)
      cache[i] = build_cache(frames[i], Tcur[i]);
    result.final_rms = data_rms(build_corrs(cache), Tcur);
  }
  result.iterations = iter;
  core::info("JPR: {} iterations, point-to-plane RMS {:.5f} -> {:.5f} m", iter,
             result.initial_rms, result.final_rms);
  return result;
}

} // namespace reusex::geometry
