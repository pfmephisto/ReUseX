// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for panorama-derived wide-baseline loop edges (issue #236).
//
// The ORB/PnP front-end needs RGB-D fixtures and is covered by the measured
// runs on real scans. What these tests pin is the part that MUST be exactly
// right and can be checked without a database (docs/STANDARDS.md §7):
//
//   1. the edge algebra — T_AB = T_pano_A^-1 · T_pano_B really is
//      pose(A)^-1 · pose(B), including under non-trivial rotations, and
//      crucially that it is INDEPENDENT of the seed poses (that independence is
//      the entire reason panorama edges can correct drift);
//   2. the gating chain — min_frame_gap, min/max seed disagreement, the
//      per-panorama cap, and inlier-scaled sigmas;
//   3. determinism — same input, same output, in the same order (STANDARDS §6).

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/PanoramaLoopEdges.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "../../support/temp_path.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

using namespace reusex::geometry;
using Catch::Matchers::WithinAbs;
using reusex::geometry::detail::edges_from_resections;
using reusex::geometry::detail::PanoResection;

namespace {

/// Rigid transform from an axis-angle rotation and a translation.
Eigen::Matrix4d make_pose(const Eigen::Vector3d &axis, double angle,
                          const Eigen::Vector3d &t) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  T.block<3, 1>(0, 3) = t;
  return T;
}

/// The resection a panorama at `T_world_pano` would produce against a frame at
/// `T_world_frame`, if the front-end were perfect: pano-from-frame, expressed
/// entirely in the frame's own optical coordinates.
PanoResection resect(int frame, const Eigen::Matrix4d &T_world_pano,
                     const Eigen::Matrix4d &T_world_frame, int inliers) {
  PanoResection r;
  r.frame = frame;
  r.T_pano_frame = T_world_pano.inverse() * T_world_frame;
  r.inliers = inliers;
  return r;
}

/// Options with the caps/gates wide open, so a test can enable one at a time.
PanoramaLoopOptions permissive_options() {
  PanoramaLoopOptions o;
  o.max_edges_per_panorama = 0; // no cap
  return o;
}

LoopClosureOptions permissive_gates() {
  LoopClosureOptions g;
  g.min_frame_gap = 0;
  g.min_seed_disagreement = 0.0f;
  g.max_seed_disagreement = 0.0f;
  return g;
}

} // namespace

TEST_CASE("panorama edge recovers the true relative pose under non-trivial "
          "rotations",
          "[slam][panorama][loop]") {
  // Two frames with genuinely different orientations, and a panorama placed
  // somewhere else entirely with its own arbitrary orientation.
  const Eigen::Matrix4d pose_a =
      make_pose({0.0, 0.0, 1.0}, 0.7, {1.0, -2.0, 0.5});
  const Eigen::Matrix4d pose_b =
      make_pose({0.3, 1.0, -0.2}, -1.1, {-4.0, 3.5, 1.25});
  const Eigen::Matrix4d pano =
      make_pose({0.5, -0.4, 0.9}, 2.3, {0.25, 0.75, -1.5});

  std::vector<PanoResection> res{resect(0, pano, pose_a, 40),
                                 resect(100, pano, pose_b, 55)};

  // The seed poses are DELIBERATELY wrong — 5 m of translational drift and a
  // large rotational error on frame B. If the edge algebra leaked the seed in
  // anywhere, this test would fail; that it passes is the property that lets a
  // panorama edge correct drift.
  std::vector<Eigen::Matrix4d> seed(101, Eigen::Matrix4d::Identity());
  seed[0] = pose_a;
  seed[100] = make_pose({1.0, 0.0, 0.0}, 0.9, {1.0, 8.5, -3.0});

  PanoramaLoopResult stats;
  const auto edges = edges_from_resections(res, seed, permissive_options(),
                                           permissive_gates(), &stats);

  REQUIRE(edges.size() == 1);
  CHECK(edges[0].i == 0);
  CHECK(edges[0].j == 100);

  const Eigen::Matrix4d expected = pose_a.inverse() * pose_b;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      CHECK_THAT(edges[0].T_ij(r, c), WithinAbs(expected(r, c), 1e-9));

  // Joint support is the weaker of the two resections.
  CHECK(edges[0].inliers == 40);
  CHECK(stats.proposed == 1);
  CHECK(stats.edges == 1);
}

TEST_CASE("panorama edges are independent of the panorama's own placement",
          "[slam][panorama][loop]") {
  // The panorama is a shared intermediate frame only: moving it must not change
  // the edge at all. (This is why `rux align 360` is not a prerequisite.)
  const Eigen::Matrix4d pose_a = make_pose({0, 0, 1}, 0.25, {0.0, 0.0, 0.0});
  const Eigen::Matrix4d pose_b = make_pose({0, 1, 0}, -0.8, {2.0, 1.0, 0.0});

  const Eigen::Matrix4d pano1 = make_pose({1, 0, 0}, 0.1, {0.5, 0.5, 0.0});
  const Eigen::Matrix4d pano2 =
      make_pose({0.2, 0.9, 0.3}, 2.9, {-40.0, 17.0, 6.0});

  std::vector<Eigen::Matrix4d> seed(61, Eigen::Matrix4d::Identity());
  auto gates = permissive_gates();

  PanoramaLoopResult s1, s2;
  const auto e1 = edges_from_resections(
      {resect(0, pano1, pose_a, 30), resect(60, pano1, pose_b, 30)}, seed,
      permissive_options(), gates, &s1);
  const auto e2 = edges_from_resections(
      {resect(0, pano2, pose_a, 30), resect(60, pano2, pose_b, 30)}, seed,
      permissive_options(), gates, &s2);

  REQUIRE(e1.size() == 1);
  REQUIRE(e2.size() == 1);
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      CHECK_THAT(e1[0].T_ij(r, c), WithinAbs(e2[0].T_ij(r, c), 1e-9));
}

TEST_CASE("min_frame_gap rejects temporally adjacent pairs",
          "[slam][panorama][loop]") {
  const Eigen::Matrix4d pano = make_pose({0, 0, 1}, 0.4, {0.0, 0.0, 1.0});
  std::vector<Eigen::Matrix4d> seed(200, Eigen::Matrix4d::Identity());

  // Frames 10 and 30 are 20 apart; frames 10 and 120 are 110 apart.
  std::vector<PanoResection> res{
      resect(10, pano, make_pose({0, 0, 1}, 0.0, {0.0, 0.0, 0.0}), 30),
      resect(30, pano, make_pose({0, 0, 1}, 0.1, {0.3, 0.0, 0.0}), 30),
      resect(120, pano, make_pose({0, 0, 1}, 0.9, {5.0, 2.0, 0.0}), 30)};

  auto gates = permissive_gates();
  gates.min_frame_gap = 50;

  PanoramaLoopResult stats;
  const auto edges =
      edges_from_resections(res, seed, permissive_options(), gates, &stats);

  CHECK(stats.proposed == 3);    // (10,30), (10,120), (30,120)
  CHECK(stats.dropped_gap == 1); // only (10,30) is closer than 50
  REQUIRE(edges.size() == 2);
  CHECK(edges[0].i == 10);
  CHECK(edges[0].j == 120);
  CHECK(edges[1].i == 30);
  CHECK(edges[1].j == 120);
}

TEST_CASE("seed-disagreement gates keep informative edges and drop redundant "
          "ones",
          "[slam][panorama][loop]") {
  const Eigen::Matrix4d pano = make_pose({0, 1, 0}, 0.3, {0.0, 1.0, 0.0});
  const Eigen::Matrix4d pose_a = Eigen::Matrix4d::Identity();
  const Eigen::Matrix4d pose_b = make_pose({0, 0, 1}, 0.2, {3.0, 0.0, 0.0});

  std::vector<PanoResection> res{resect(0, pano, pose_a, 30),
                                 resect(100, pano, pose_b, 30)};

  SECTION("an edge agreeing with the seed carries no drift information") {
    std::vector<Eigen::Matrix4d> seed(101, Eigen::Matrix4d::Identity());
    seed[0] = pose_a;
    seed[100] = pose_b; // seed and measurement agree exactly

    auto gates = permissive_gates();
    gates.min_seed_disagreement = 0.10f;

    PanoramaLoopResult stats;
    const auto edges =
        edges_from_resections(res, seed, permissive_options(), gates, &stats);
    CHECK(edges.empty());
    CHECK(stats.dropped_seed_gate == 1);
  }

  SECTION("a large-drift edge survives the lower gate") {
    std::vector<Eigen::Matrix4d> seed(101, Eigen::Matrix4d::Identity());
    seed[0] = pose_a;
    // Seed thinks B is 1.2 m off from where the panorama measures it.
    seed[100] = make_pose({0, 0, 1}, 0.2, {4.2, 0.0, 0.0});

    auto gates = permissive_gates();
    gates.min_seed_disagreement = 0.10f;

    PanoramaLoopResult stats;
    const auto edges =
        edges_from_resections(res, seed, permissive_options(), gates, &stats);
    REQUIRE(edges.size() == 1);
    CHECK(stats.dropped_seed_gate == 0);
  }

  SECTION("max_seed_disagreement rejects a gross mismatch when enabled") {
    std::vector<Eigen::Matrix4d> seed(101, Eigen::Matrix4d::Identity());
    seed[0] = pose_a;
    seed[100] = make_pose({0, 0, 1}, 0.2, {50.0, 0.0, 0.0}); // 47 m apart

    auto gates = permissive_gates();
    gates.max_seed_disagreement = 5.0f;

    PanoramaLoopResult stats;
    const auto edges =
        edges_from_resections(res, seed, permissive_options(), gates, &stats);
    CHECK(edges.empty());
    CHECK(stats.dropped_seed_gate == 1);
  }
}

TEST_CASE("per-panorama cap keeps the best-supported edges deterministically",
          "[slam][panorama][loop]") {
  const Eigen::Matrix4d pano = make_pose({0, 0, 1}, 0.0, {0.0, 0.0, 0.0});
  std::vector<Eigen::Matrix4d> seed(1000, Eigen::Matrix4d::Identity());

  // Six frames, all mutually far apart -> 15 candidate pairs. Support rises
  // with the frame index, so the cap must keep pairs among the high indices.
  std::vector<PanoResection> res;
  for (int k = 0; k < 6; ++k)
    res.push_back(resect(
        k * 100, pano,
        make_pose({0, 0, 1}, 0.1 * k, {static_cast<double>(k), 0.0, 0.0}),
        20 + 10 * k));

  auto opt = permissive_options();
  opt.max_edges_per_panorama = 4;

  PanoramaLoopResult stats;
  const auto edges =
      edges_from_resections(res, seed, opt, permissive_gates(), &stats);

  CHECK(stats.proposed == 15);
  CHECK(stats.dropped_cap == 11);
  REQUIRE(edges.size() == 4);

  // Output is ordered by (i, j) regardless of the support-ranked selection.
  for (size_t k = 1; k < edges.size(); ++k)
    CHECK((edges[k - 1].i < edges[k].i ||
           (edges[k - 1].i == edges[k].i && edges[k - 1].j < edges[k].j)));

  // Same input, same output — including order (STANDARDS §6).
  PanoramaLoopResult stats2;
  const auto again =
      edges_from_resections(res, seed, opt, permissive_gates(), &stats2);
  REQUIRE(again.size() == edges.size());
  for (size_t k = 0; k < edges.size(); ++k) {
    CHECK(again[k].i == edges[k].i);
    CHECK(again[k].j == edges[k].j);
    CHECK(again[k].inliers == edges[k].inliers);
  }
  CHECK(stats2.dropped_cap == stats.dropped_cap);
}

TEST_CASE("edge order does not depend on resection input order",
          "[slam][panorama][loop]") {
  const Eigen::Matrix4d pano = make_pose({1, 1, 0}, 0.6, {0.0, 0.0, 2.0});
  std::vector<Eigen::Matrix4d> seed(700, Eigen::Matrix4d::Identity());

  std::vector<PanoResection> forward;
  for (int k = 0; k < 4; ++k)
    forward.push_back(resect(
        k * 150, pano,
        make_pose({0, 1, 0}, 0.2 * k, {0.0, static_cast<double>(k), 0.0}),
        25 + k));
  std::vector<PanoResection> reversed(forward.rbegin(), forward.rend());

  const auto a = edges_from_resections(forward, seed, permissive_options(),
                                       permissive_gates(), nullptr);
  const auto b = edges_from_resections(reversed, seed, permissive_options(),
                                       permissive_gates(), nullptr);

  REQUIRE(a.size() == b.size());
  for (size_t k = 0; k < a.size(); ++k) {
    CHECK(a[k].i == b[k].i);
    CHECK(a[k].j == b[k].j);
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        CHECK_THAT(a[k].T_ij(r, c), WithinAbs(b[k].T_ij(r, c), 1e-12));
  }
}

TEST_CASE("edge sigmas shrink with inlier support down to the floors",
          "[slam][panorama][loop]") {
  const Eigen::Matrix4d pano = Eigen::Matrix4d::Identity();
  std::vector<Eigen::Matrix4d> seed(300, Eigen::Matrix4d::Identity());

  auto opt = permissive_options();
  opt.min_frame_inliers = 20;
  opt.base_sigma_trans = 0.15f;
  opt.base_sigma_rot = 0.06f;
  opt.min_sigma_trans = 0.05f;
  opt.min_sigma_rot = 0.025f;

  SECTION("bare-minimum support gets the base sigma") {
    const auto edges = edges_from_resections(
        {resect(0, pano, Eigen::Matrix4d::Identity(), 20),
         resect(200, pano, make_pose({0, 0, 1}, 0.5, {1.0, 0.0, 0.0}), 20)},
        seed, opt, permissive_gates(), nullptr);
    REQUIRE(edges.size() == 1);
    CHECK_THAT(edges[0].sigma_trans, WithinAbs(0.15, 1e-6));
    CHECK_THAT(edges[0].sigma_rot, WithinAbs(0.06, 1e-6));
  }

  SECTION("4x support halves the sigma") {
    const auto edges = edges_from_resections(
        {resect(0, pano, Eigen::Matrix4d::Identity(), 80),
         resect(200, pano, make_pose({0, 0, 1}, 0.5, {1.0, 0.0, 0.0}), 80)},
        seed, opt, permissive_gates(), nullptr);
    REQUIRE(edges.size() == 1);
    CHECK_THAT(edges[0].sigma_trans, WithinAbs(0.075, 1e-6));
    CHECK_THAT(edges[0].sigma_rot, WithinAbs(0.03, 1e-6));
  }

  SECTION("overwhelming support is clamped at the floor") {
    const auto edges = edges_from_resections(
        {resect(0, pano, Eigen::Matrix4d::Identity(), 5000),
         resect(200, pano, make_pose({0, 0, 1}, 0.5, {1.0, 0.0, 0.0}), 5000)},
        seed, opt, permissive_gates(), nullptr);
    REQUIRE(edges.size() == 1);
    CHECK_THAT(edges[0].sigma_trans, WithinAbs(0.05, 1e-6));
    CHECK_THAT(edges[0].sigma_rot, WithinAbs(0.025, 1e-6));
  }
}

TEST_CASE("a single resection cannot form an edge", "[slam][panorama][loop]") {
  const Eigen::Matrix4d pano = Eigen::Matrix4d::Identity();
  std::vector<Eigen::Matrix4d> seed(10, Eigen::Matrix4d::Identity());

  PanoramaLoopResult stats;
  const auto edges = edges_from_resections(
      {resect(0, pano, Eigen::Matrix4d::Identity(), 99)}, seed,
      permissive_options(), permissive_gates(), &stats);
  CHECK(edges.empty());
  CHECK(stats.proposed == 0);
  CHECK(stats.edges == 0);
}

TEST_CASE("resections referencing frames outside the seed vector are ignored",
          "[slam][panorama][loop]") {
  // Defensive: an out-of-range frame index must never index seed_poses.
  const Eigen::Matrix4d pano = Eigen::Matrix4d::Identity();
  std::vector<Eigen::Matrix4d> seed(5, Eigen::Matrix4d::Identity());

  PanoramaLoopResult stats;
  const auto edges = edges_from_resections(
      {resect(0, pano, Eigen::Matrix4d::Identity(), 30),
       resect(4000, pano, make_pose({0, 0, 1}, 0.3, {1.0, 0.0, 0.0}), 30)},
      seed, permissive_options(), permissive_gates(), &stats);
  CHECK(edges.empty());
  CHECK(stats.edges == 0);
}

TEST_CASE("a project with no panoramas yields zero edges without erroring",
          "[slam][panorama][loop]") {
  // The mechanism must be safe to enable unconditionally: a scan that simply
  // has no 360 imagery is a no-op, not a failure. Whether the USER explicitly
  // asked for panorama edges is the caller's knowledge, so the loud warning
  // lives in optimize_sensor_poses — but `panoramas == 0` is reported here so
  // that warning can quote a real number (docs/STANDARDS.md §5).
  reusex::test_support::TempPath tmp{"reusex_pano_loops_test", ".rux"};
  reusex::ProjectDB db(tmp.path);
  REQUIRE(db.panoramic_image_count() == 0);

  const std::vector<int> node_ids{1, 2, 3};
  const std::vector<Eigen::Matrix4d> seed(3, Eigen::Matrix4d::Identity());

  PanoramaLoopOptions opt;
  opt.enable = true;

  PanoramaLoopResult stats;
  std::vector<LoopEdge> edges;
  REQUIRE_NOTHROW(edges = detect_panorama_loop_edges(
                      db, node_ids, seed, opt, LoopClosureOptions{}, &stats));
  CHECK(edges.empty());
  CHECK(stats.panoramas == 0);
  CHECK(stats.edges == 0);
  CHECK(stats.proposed == 0);
}

TEST_CASE("mismatched node_ids / seed_poses is a loud failure",
          "[slam][panorama][loop]") {
  // A silently truncated pairing would index the wrong frame's seed pose and
  // corrupt the gating decisions rather than fail (docs/STANDARDS.md §5).
  reusex::test_support::TempPath tmp{"reusex_pano_loops_mismatch", ".rux"};
  reusex::ProjectDB db(tmp.path);

  // Give the project a panorama so the early "no panoramas" return does not
  // pre-empt the size check.
  db.save_panoramic_image("pano.jpg", std::vector<uint8_t>{1, 2, 3}, 0.0, -1);
  REQUIRE(db.panoramic_image_count() == 1);

  PanoramaLoopOptions opt;
  opt.enable = true;
  const std::vector<int> node_ids{1, 2, 3};
  const std::vector<Eigen::Matrix4d> seed(2, Eigen::Matrix4d::Identity());

  CHECK_THROWS_AS(detect_panorama_loop_edges(db, node_ids, seed, opt,
                                             LoopClosureOptions{}, nullptr),
                  std::runtime_error);
}
