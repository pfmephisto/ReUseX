// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the plane-landmark pose-graph optimizer (issue #225, P1).
//
// Strategy: build several sensor frames that all observe the *same* three
// mutually-perpendicular planes (a corner). Three perpendicular planes make all
// six pose DoF observable from plane factors alone. Placing the frames at known
// world poses and then corrupting the seed poses with drift, the optimizer must
// pull the drifted poses back toward the (shared, globally-consistent) truth.
// Fixed seeds keep everything deterministic (docs/STANDARDS.md §6).

#include <reusex/geometry/Surfel.hpp>
#include <reusex/geometry/registration/PlaneGraphOptimizer.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Geometry>

#include <vector>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Matchers::WithinAbs;

// Implemented in PlaneGraphOptimizer.cpp (gtsam lives only in the .cpp).
// Returns the OrientedPlane3Factor error norm for a perfectly consistent
// off-origin observation under the optimizer's Hessian encoding.
namespace reusex::geometry {
double plane_factor_consistent_residual();
} // namespace reusex::geometry

// Regression guard for the sign-convention bug: gtsam's OrientedPlane3 stores a
// plane as n.x + d = 0 (distance = d, NOT -d). The optimizer once negated d on
// both the landmark and the measurement, which does NOT cancel (landmark is in
// world, measurement in the optical frame; the factor transforms the landmark
// into the pose frame). A perfectly consistent observation must have ~0
// residual; the buggy encoding produced ~2.1 for the same geometry. This uses
// an off-origin plane (d != 0) because origin-planes hide the sign entirely.
TEST_CASE(
    "OrientedPlane3 conventions: consistent observation has zero residual",
    "[plane_graph][optimize]") {
  const double residual = reusex::geometry::plane_factor_consistent_residual();
  REQUIRE_THAT(residual, WithinAbs(0.0, 1e-9));
}

namespace {

// A dense corner: three perpendicular planes sampled as optical-frame surfels,
// with plenty of inliers per plane so RANSAC recovers all three above the
// default --min-plane-inliers threshold. The corner is deliberately placed OFF
// the origin (offset o) so each plane has a non-zero Hessian offset d: a plane
// through the origin has d=0, which hides the OrientedPlane3 sign convention
// and let a real bug pass the earlier version of this test.
FrameSurfels make_corner_frame(int node_id) {
  auto pts = std::make_shared<Cloud>();
  auto nrm = std::make_shared<CloudN>();
  const float step = 0.03f;
  const float extent = 1.2f;
  const float o = 1.5f; // corner offset from origin -> non-zero plane offsets
  auto add = [&](float x, float y, float z, float nx, float ny, float nz) {
    PointT p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.r = p.g = p.b = 200;
    pts->push_back(p);
    NormalT n;
    n.normal_x = nx;
    n.normal_y = ny;
    n.normal_z = nz;
    n.curvature = 0.0f;
    nrm->push_back(n);
  };
  for (float a = 0.0f; a <= extent; a += step) {
    for (float b = 0.0f; b <= extent; b += step) {
      add(o + a, o + b, o, 0.0f, 0.0f, 1.0f); // z=o plane, normal +z
      add(o, o + a, o + b, 1.0f, 0.0f, 0.0f); // x=o plane, normal +x
      add(o + a, o, o + b, 0.0f, 1.0f, 0.0f); // y=o plane, normal +y
    }
  }
  FrameSurfels f;
  f.node_id = node_id;
  f.points = pts;
  f.normals = nrm;
  f.world_pose = Eigen::Affine3f::Identity();
  return f;
}

// A single dense square planar patch with normal +z, centred in world at
// (cx, cy, cz), sampled as optical-frame surfels of an identity-pose frame (so
// optical == world here). `half` is the half-side of the patch. Enough points
// to clear the default --min-plane-inliers threshold.
FrameSurfels make_patch_frame(int node_id, float cx, float cy, float cz,
                              float half) {
  auto pts = std::make_shared<Cloud>();
  auto nrm = std::make_shared<CloudN>();
  const float step = 0.03f;
  for (float a = -half; a <= half; a += step) {
    for (float b = -half; b <= half; b += step) {
      PointT p;
      p.x = cx + a;
      p.y = cy + b;
      p.z = cz;
      p.r = p.g = p.b = 200;
      pts->push_back(p);
      NormalT n;
      n.normal_x = 0.0f;
      n.normal_y = 0.0f;
      n.normal_z = 1.0f;
      n.curvature = 0.0f;
      nrm->push_back(n);
    }
  }
  FrameSurfels f;
  f.node_id = node_id;
  f.points = pts;
  f.normals = nrm;
  f.world_pose = Eigen::Affine3f::Identity();
  return f;
}

// Small rigid drift used to corrupt a seed pose.
Eigen::Affine3f drift(float angle, const Eigen::Vector3f &axis,
                      const Eigen::Vector3f &t) {
  Eigen::Affine3f d = Eigen::Affine3f::Identity();
  d.rotate(Eigen::AngleAxisf(angle, axis.normalized()));
  d.translation() = t;
  return d;
}

PlaneGraphOptions test_options() {
  PlaneGraphOptions o;
  o.max_planes_per_frame = 4;
  o.min_plane_inliers = 200;
  o.ransac_distance = 0.02f;
  o.ransac_normal_angle = 20.0f;
  o.ransac_iterations = 200;
  o.assoc_normal_angle = 15.0f;
  o.assoc_distance = 0.20f; // roomy: drift shifts world offsets a little
  o.min_landmark_observations = 2;
  // These synthetic tests probe the plane-landmark objective directly (a shared
  // corner), so they pin the noise knobs the library ships weaker-by-default
  // for real scans: TIGHT plane factors (the objective under test) and LOOSE
  // odometry (which here comes from the *drifted* seed poses and is wrong), so
  // the equilibrium is dominated by the landmarks rather than the seed chain.
  o.plane_sigma_normal = 0.05f;
  o.plane_sigma_distance = 0.03f;
  o.odometry_sigma_rot = 0.25f;
  o.odometry_sigma_trans = 0.5f;
  // Corner geometry: three big perpendicular planes shared by all frames. The
  // overlap/degeneracy hygiene is exercised by dedicated tests below; here it
  // is set permissive so it never interferes with the recovery objective.
  o.assoc_overlap_margin = 2.0f;
  o.min_landmark_spread_ratio = 0.0f;
  o.assoc_rounds = 1;
  o.underconstrained_odom_scale = 1.0f;
  o.max_iterations = 100;
  o.seed = 42;
  return o;
}

// Translational distance between two poses.
float pose_trans_error(const Eigen::Affine3f &a, const Eigen::Affine3f &b) {
  return (a.translation() - b.translation()).norm();
}

// Frobenius rotation error between two poses.
float pose_rot_error(const Eigen::Affine3f &a, const Eigen::Affine3f &b) {
  return (a.rotation() - b.rotation()).norm();
}

} // namespace

TEST_CASE("PlaneGraph recovers drifted poses toward the shared truth",
          "[plane_graph][optimize]") {
  // All frames sit at the same true pose (identity): each sees the exact same
  // corner. This is the cleanest global-consistency signal.
  const Eigen::Affine3f truth = Eigen::Affine3f::Identity();

  std::vector<FrameSurfels> frames;
  for (int i = 0; i < 4; ++i)
    frames.push_back(make_corner_frame(i));

  // Frame 0 is the gauge anchor: seed it at truth so the recovered solution is
  // expressed in the truth frame and errors are directly comparable.
  frames[0].world_pose = truth;
  // Frames 1..3 get distinct drifts (rotation + translation).
  frames[1].world_pose =
      drift(0.04f, {1, 1, 1}, {0.03f, -0.02f, 0.015f}) * truth;
  frames[2].world_pose =
      drift(0.05f, {0, 1, 0}, {-0.025f, 0.03f, -0.01f}) * truth;
  frames[3].world_pose = drift(0.03f, {1, 0, 1}, {0.02f, 0.02f, 0.02f}) * truth;

  // Record the drifted seed error for comparison.
  std::vector<float> seed_terr, seed_rerr;
  for (int i = 1; i < 4; ++i) {
    seed_terr.push_back(pose_trans_error(frames[i].world_pose, truth));
    seed_rerr.push_back(pose_rot_error(frames[i].world_pose, truth));
  }

  PlaneGraphOptimizer optimizer(test_options());
  PlaneGraphResult res = optimizer.optimize(frames);

  // The optimizer must have found the shared planes and formed landmarks.
  REQUIRE(res.landmarks >= 3);     // three perpendicular planes
  REQUIRE(res.plane_factors >= 6); // seen by multiple frames
  REQUIRE(res.converged);          // the solve itself must not have failed
  REQUIRE(res.final_error <= res.initial_error);

  // Frame 0 (gauge) must stay put.
  REQUIRE_THAT(pose_trans_error(frames[0].world_pose, truth),
               WithinAbs(0.0, 1e-3));

  // Each drifted frame must end up measurably closer to truth.
  for (int i = 1; i < 4; ++i) {
    const float terr = pose_trans_error(frames[i].world_pose, truth);
    const float rerr = pose_rot_error(frames[i].world_pose, truth);
    INFO("frame " << i << " trans " << seed_terr[i - 1] << " -> " << terr
                  << ", rot " << seed_rerr[i - 1] << " -> " << rerr);
    REQUIRE(terr < seed_terr[i - 1]);
    REQUIRE(rerr < seed_rerr[i - 1]);
    // And close to truth in absolute terms.
    REQUIRE(terr < 0.01f);
    REQUIRE(rerr < 0.03f);
  }
}

TEST_CASE("PlaneGraph is a near no-op on already-consistent poses",
          "[plane_graph][optimize]") {
  // All frames already at the true (identity) pose: no drift to correct.
  std::vector<FrameSurfels> frames;
  for (int i = 0; i < 3; ++i)
    frames.push_back(make_corner_frame(i));

  std::vector<Eigen::Affine3f> before;
  for (auto &f : frames)
    before.push_back(f.world_pose);

  PlaneGraphOptimizer optimizer(test_options());
  PlaneGraphResult res = optimizer.optimize(frames);

  REQUIRE(res.landmarks >= 3);
  // Poses must not move meaningfully.
  REQUIRE(res.max_pose_shift < 5e-3);
  for (size_t i = 0; i < frames.size(); ++i) {
    REQUIRE_THAT(pose_trans_error(frames[i].world_pose, before[i]),
                 WithinAbs(0.0, 5e-3));
    REQUIRE(pose_rot_error(frames[i].world_pose, before[i]) < 5e-3f);
  }
}

// ---------------------------------------------------------------------------
// #225 follow-up: landmark hygiene.
// ---------------------------------------------------------------------------

TEST_CASE("PlaneGraph does NOT alias two offset parallel walls",
          "[plane_graph][optimize][hygiene]") {
  // Two frames each see a co-normal (normal +z, offset d=0) patch, but the two
  // patches are far apart IN-PLANE (centres 10 m apart along x). They agree in
  // normal AND offset — the old greedy front-end would merge them into a single
  // landmark, aliasing two distinct surfaces. The overlap gate must keep them
  // separate: with min_observations=1 that means TWO kept landmarks, not one.
  std::vector<FrameSurfels> frames;
  frames.push_back(make_patch_frame(0, 0.0f, 0.0f, 0.0f, 1.0f));
  frames.push_back(make_patch_frame(1, 10.0f, 0.0f, 0.0f, 1.0f));

  PlaneGraphOptions o = test_options();
  o.min_landmark_observations = 1;    // each patch seen by exactly one frame
  o.assoc_normal_angle = 15.0f;       // normals are identical anyway
  o.assoc_distance = 0.5f;            // offsets are identical (both d=0)
  o.assoc_overlap_margin = 0.30f;     // patches are 10 m apart -> no overlap
  o.min_landmark_spread_ratio = 0.0f; // disable degeneracy check here
  o.assoc_rounds = 1;

  PlaneGraphOptimizer optimizer(o);
  PlaneGraphResult res = optimizer.optimize(frames);

  INFO("landmarks=" << res.landmarks
                    << " rejected_overlap=" << res.landmarks_rejected_overlap);
  REQUIRE(res.landmarks == 2);                  // NOT merged
  REQUIRE(res.landmarks_rejected_overlap >= 1); // the overlap gate fired

  // Sanity: with the overlap gate OFF the SAME geometry collapses to one
  // landmark (proves the gate, not the geometry, is doing the separating).
  o.assoc_overlap_margin = 0.0f; // disable overlap gate
  std::vector<FrameSurfels> frames2;
  frames2.push_back(make_patch_frame(0, 0.0f, 0.0f, 0.0f, 1.0f));
  frames2.push_back(make_patch_frame(1, 10.0f, 0.0f, 0.0f, 1.0f));
  PlaneGraphOptimizer optimizer2(o);
  PlaneGraphResult res2 = optimizer2.optimize(frames2);
  REQUIRE(res2.landmarks == 1); // aliased when the gate is off
}

TEST_CASE("PlaneGraph rejects a degenerate (near-collinear) landmark",
          "[plane_graph][optimize][hygiene]") {
  // Four frames see adjacent strips of the SAME z=0 plane whose centroids lie
  // on a straight line (along x). Merged into one landmark, those centroids are
  // near-collinear: a rotation about that line is unobservable, so the landmark
  // must be rejected (kept landmarks == 0). The generous overlap margin lets
  // the adjacent strips merge so the degeneracy test is what does the work.
  std::vector<FrameSurfels> collinear;
  for (int i = 0; i < 4; ++i)
    collinear.push_back(
        make_patch_frame(i, static_cast<float>(i) * 0.8f, 0.0f, 0.0f, 0.6f));

  PlaneGraphOptions o = test_options();
  o.min_landmark_observations = 3;
  o.assoc_normal_angle = 15.0f;
  o.assoc_distance = 0.5f;
  o.assoc_overlap_margin = 1.0f; // adjacent strips overlap -> one landmark
  o.min_landmark_spread_ratio = 0.15f; // require a genuine 2D patch
  o.assoc_rounds = 1;

  PlaneGraphOptimizer opt_c(o);
  PlaneGraphResult res_c = opt_c.optimize(collinear);
  INFO("collinear landmarks=" << res_c.landmarks << " degenerate_rejected="
                              << res_c.landmarks_rejected_degenerate);
  REQUIRE(res_c.landmarks == 0);
  REQUIRE(res_c.landmarks_rejected_degenerate >= 1);

  // Control: the same strips arranged in a 2x2 GRID (centroids span a plane)
  // form a well-conditioned landmark that is kept.
  std::vector<FrameSurfels> grid;
  int id = 0;
  for (int gx = 0; gx < 2; ++gx)
    for (int gy = 0; gy < 2; ++gy)
      grid.push_back(make_patch_frame(id++, static_cast<float>(gx) * 0.8f,
                                      static_cast<float>(gy) * 0.8f, 0.0f,
                                      0.6f));
  PlaneGraphOptimizer opt_g(o);
  PlaneGraphResult res_g = opt_g.optimize(grid);
  INFO("grid landmarks=" << res_g.landmarks);
  REQUIRE(res_g.landmarks >= 1);
}

TEST_CASE("PlaneGraph alternating rounds beat a single round on heavy drift",
          "[plane_graph][optimize][hygiene]") {
  // A larger drift than the basic recovery test: one association+optimize pass
  // under-corrects because the landmarks are formed on badly-drifted poses;
  // re-associating on the improved poses (EM-style) recovers more. Assert the
  // 3-round run ends strictly closer to truth than the 1-round run.
  const Eigen::Affine3f truth = Eigen::Affine3f::Identity();

  auto build = [&]() {
    std::vector<FrameSurfels> frames;
    for (int i = 0; i < 4; ++i)
      frames.push_back(make_corner_frame(i));
    frames[0].world_pose = truth; // gauge anchor at truth
    frames[1].world_pose =
        drift(0.10f, {1, 1, 1}, {0.08f, -0.06f, 0.05f}) * truth;
    frames[2].world_pose =
        drift(0.12f, {0, 1, 0}, {-0.07f, 0.09f, -0.05f}) * truth;
    frames[3].world_pose =
        drift(0.09f, {1, 0, 1}, {0.06f, 0.06f, 0.07f}) * truth;
    return frames;
  };

  auto total_error = [&](const std::vector<FrameSurfels> &frames) {
    float e = 0.0f;
    for (int i = 1; i < 4; ++i)
      e += pose_trans_error(frames[i].world_pose, truth) +
           pose_rot_error(frames[i].world_pose, truth);
    return e;
  };

  PlaneGraphOptions base = test_options();
  base.min_landmark_observations = 2;
  base.assoc_overlap_margin = 2.0f;      // corner planes are large & shared
  base.min_landmark_spread_ratio = 0.0f; // corner geometry, not under test here
  base.underconstrained_odom_scale = 1.0f; // isolate the rounds effect

  PlaneGraphOptions one = base;
  one.assoc_rounds = 1;
  std::vector<FrameSurfels> f1 = build();
  PlaneGraphOptimizer(one).optimize(f1);
  const float e1 = total_error(f1);

  PlaneGraphOptions many = base;
  many.assoc_rounds = 3;
  many.assoc_round_tol = 1e-5f; // do not stop early
  std::vector<FrameSurfels> f3 = build();
  PlaneGraphResult r3 = PlaneGraphOptimizer(many).optimize(f3);
  const float e3 = total_error(f3);

  INFO("1-round total err " << e1 << " vs 3-round " << e3
                            << " (rounds=" << r3.rounds << ")");
  REQUIRE(r3.rounds >= 2);
  REQUIRE(e3 < e1);
}

TEST_CASE("PlaneGraph leaves poses unchanged when no landmark is shared",
          "[plane_graph][optimize]") {
  // Two frames but require more observations than any landmark can get: the
  // optimizer must refuse to move the poses (no silent guessing).
  std::vector<FrameSurfels> frames;
  for (int i = 0; i < 2; ++i)
    frames.push_back(make_corner_frame(i));
  frames[1].world_pose = drift(0.05f, {1, 0, 0}, {0.05f, 0.0f, 0.0f});

  std::vector<Eigen::Affine3f> before;
  for (auto &f : frames)
    before.push_back(f.world_pose);

  PlaneGraphOptions o = test_options();
  o.min_landmark_observations = 10; // impossible with 2 frames

  PlaneGraphOptimizer optimizer(o);
  PlaneGraphResult res = optimizer.optimize(frames);

  REQUIRE(res.landmarks == 0);
  for (size_t i = 0; i < frames.size(); ++i)
    REQUIRE((frames[i].world_pose.matrix() - before[i].matrix()).norm() ==
            0.0f);
}
