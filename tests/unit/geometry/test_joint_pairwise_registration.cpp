// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/geometry/Surfel.hpp>
#include <reusex/geometry/registration/JointPairwiseRegistration.hpp>
#include <reusex/geometry/transform_utils.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Geometry>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Matchers::WithinAbs;

namespace {

// Build a 3-plane corner (z=0, x=0, y=0) as surfels. Three mutually
// perpendicular planes make all 6 pose DoF observable for point-to-plane.
FrameSurfels make_corner_frame() {
  auto pts = std::make_shared<Cloud>();
  auto nrm = std::make_shared<CloudN>();
  const float step = 0.04f;
  const float extent = 0.8f;
  auto add = [&](float x, float y, float z, float nx, float ny, float nz) {
    PointT p;
    p.x = x; p.y = y; p.z = z;
    p.r = p.g = p.b = 200;
    pts->push_back(p);
    NormalT n;
    n.normal_x = nx; n.normal_y = ny; n.normal_z = nz;
    n.curvature = 0.0f;
    nrm->push_back(n);
  };
  for (float a = 0.0f; a <= extent; a += step) {
    for (float b = 0.0f; b <= extent; b += step) {
      add(a, b, 0.0f, 0.0f, 0.0f, 1.0f); // z=0 plane
      add(0.0f, a, b, 1.0f, 0.0f, 0.0f); // x=0 plane
      add(a, 0.0f, b, 0.0f, 1.0f, 0.0f); // y=0 plane
    }
  }
  FrameSurfels f;
  f.node_id = 0;
  f.points = pts;
  f.normals = nrm;
  f.world_pose = Eigen::Affine3f::Identity();
  return f;
}

Eigen::Affine3f perturbation() {
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  t.rotate(Eigen::AngleAxisf(0.03f, Eigen::Vector3f(1, 1, 1).normalized()));
  t.translation() = Eigen::Vector3f(0.02f, -0.015f, 0.01f);
  return t;
}

} // namespace

TEST_CASE("se3 exp/log round-trip", "[jpr][se3]") {
  se3::Vector6d xi;
  xi << 0.1, -0.2, 0.05, 0.3, -0.1, 0.2;
  se3::Vector6d back = se3::log(se3::exp(xi));
  for (int i = 0; i < 6; ++i)
    REQUIRE_THAT(back(i), WithinAbs(xi(i), 1e-9));
}

TEST_CASE("to_affine / to_array16 round-trip", "[jpr][transform]") {
  Eigen::Affine3f a = perturbation();
  auto arr = to_array16(a);
  Eigen::Affine3f b = to_affine(arr);
  REQUIRE((a.matrix() - b.matrix()).norm() < 1e-5f);
}

TEST_CASE("JPR recovers a known pose offset on a corner", "[jpr]") {
  FrameSurfels f0 = make_corner_frame();
  FrameSurfels f1 = make_corner_frame();
  f0.node_id = 0;
  f1.node_id = 1;
  f1.world_pose = perturbation(); // wrong seed for frame 1

  JprParams params;
  params.max_iterations = 40;
  params.neighbor_window = 1;
  params.max_corr_distance = 0.10f;
  params.robust_width = 0.05f;
  params.anchor_frame = 0;   // fix frame 0 -> gauge
  params.prior_weight = 0.0f; // no pull toward the (wrong) seed

  std::vector<FrameSurfels> frames{f0, f1};
  JointPairwiseRegistration jpr(params);
  JprResult res = jpr.refine(frames);

  // Frame 1 should converge back toward identity (aligning with frame 0).
  REQUIRE_THAT(frames[1].world_pose.translation().norm(), WithinAbs(0.0, 0.01));
  Eigen::Matrix3f dR =
      frames[1].world_pose.rotation() - Eigen::Matrix3f::Identity();
  REQUIRE(dR.norm() < 0.05f);

  // Residual must improve.
  REQUIRE(res.final_rms <= res.initial_rms);
  REQUIRE(res.final_rms < 0.01);
}

TEST_CASE("JPR keeps the anchored frame exactly fixed", "[jpr]") {
  FrameSurfels f0 = make_corner_frame();
  FrameSurfels f1 = make_corner_frame();
  f0.node_id = 0;
  f1.node_id = 1;
  const Eigen::Affine3f seed0 = f0.world_pose; // identity
  f1.world_pose = perturbation();

  JprParams params;
  params.max_iterations = 20;
  params.neighbor_window = 1;
  params.anchor_frame = 0;
  params.prior_weight = 0.0f;

  std::vector<FrameSurfels> frames{f0, f1};
  JointPairwiseRegistration(params).refine(frames);

  // Anchored frame is removed from the system -> bit-for-bit unchanged.
  REQUIRE((frames[0].world_pose.matrix() - seed0.matrix()).norm() == 0.0f);
}

TEST_CASE("JPR is a near no-op on already-aligned frames", "[jpr]") {
  FrameSurfels f0 = make_corner_frame();
  FrameSurfels f1 = make_corner_frame();
  f0.node_id = 0;
  f1.node_id = 1; // both seeded correctly at identity

  JprParams params;
  params.max_iterations = 20;
  params.neighbor_window = 1;
  params.anchor_frame = 0;
  params.prior_weight = 1.0f;

  std::vector<FrameSurfels> frames{f0, f1};
  JointPairwiseRegistration(params).refine(frames);

  REQUIRE_THAT(frames[1].world_pose.translation().norm(),
               WithinAbs(0.0, 1e-3));
  Eigen::Matrix3f dR =
      frames[1].world_pose.rotation() - Eigen::Matrix3f::Identity();
  REQUIRE(dR.norm() < 5e-3f);
}
