// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/io/arkitscenes.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstddef>
#include <string>

using Catch::Matchers::WithinAbs;
using reusex::io::arkit_traj_to_optical_world;
using reusex::io::ArkitPincam;
using reusex::io::ArkitTrajectory;
using reusex::io::interpolate_pose;
using reusex::io::parse_frame_timestamp;
using reusex::io::parse_pincam;

TEST_CASE("ParsePincam_ValidLine_ParsesAllFields", "[io][arkitscenes]") {
  const ArkitPincam p = parse_pincam("256 192 211.5 211.9 127.9 95.8");
  CHECK_THAT(p.width, WithinAbs(256.0, 1e-9));
  CHECK_THAT(p.height, WithinAbs(192.0, 1e-9));
  CHECK_THAT(p.fx, WithinAbs(211.5, 1e-6));
  CHECK_THAT(p.fy, WithinAbs(211.9, 1e-6));
  CHECK_THAT(p.cx, WithinAbs(127.9, 1e-6));
  CHECK_THAT(p.cy, WithinAbs(95.8, 1e-6));
}

TEST_CASE("ParsePincam_MalformedLine_Throws", "[io][arkitscenes]") {
  CHECK_THROWS(parse_pincam("256 192 211.5"));
}

TEST_CASE("ArkitTrajToOpticalWorld_ZeroRotationZeroTranslation_ReturnsIdentity",
          "[io][arkitscenes]") {
  // rx=ry=rz=0 and t=0 => E=I => c2w=I => pose = I (no axis flip: ARKit's
  // camera frame is already the OpenCV optical frame reconstruct expects).
  const std::array<double, 16> pose =
      arkit_traj_to_optical_world(0, 0, 0, 0, 0, 0);

  const std::array<double, 16> identity = {1, 0, 0, 0, 0, 1, 0, 0,
                                           0, 0, 1, 0, 0, 0, 0, 1};
  for (std::size_t i = 0; i < 16; ++i)
    CHECK_THAT(pose[i], WithinAbs(identity[i], 1e-12));
}

TEST_CASE(
    "ArkitTrajToOpticalWorld_PureTranslationExtrinsic_InvertsToCameraToWorld",
    "[io][arkitscenes]") {
  // E = [I | t], t=(1,2,3), is world->camera, so pose = c2w = [I | -t]:
  //   [[ 1, 0, 0,-1],
  //    [ 0, 1, 0,-2],
  //    [ 0, 0, 1,-3],
  //    [ 0, 0, 0, 1]]
  const std::array<double, 16> pose =
      arkit_traj_to_optical_world(0, 0, 0, 1, 2, 3);

  // Full row-major matrix check.
  const std::array<double, 16> expected = {1, 0, 0, -1, 0, 1, 0, -2,
                                           0, 0, 1, -3, 0, 0, 0, 1};
  for (std::size_t i = 0; i < 16; ++i)
    CHECK_THAT(pose[i], WithinAbs(expected[i], 1e-12));

  // A point at optical (0,0,1) (1 m in front, optical z-forward) maps to
  // world = pose * [0,0,1,1]^T. Row-major: w_r = pose[r*4+2]*1 + pose[r*4+3].
  auto apply = [&](double ox, double oy, double oz) {
    std::array<double, 3> w{};
    for (int r = 0; r < 3; ++r)
      w[static_cast<std::size_t>(r)] = pose[r * 4 + 0] * ox +
                                       pose[r * 4 + 1] * oy +
                                       pose[r * 4 + 2] * oz + pose[r * 4 + 3];
    return w;
  };
  const auto w = apply(0, 0, 1);
  // = c2w * [0,0,1,1] = (-t) + (0,0,1) = (-1,-2,-2)
  CHECK_THAT(w[0], WithinAbs(-1.0, 1e-12));
  CHECK_THAT(w[1], WithinAbs(-2.0, 1e-12));
  CHECK_THAT(w[2], WithinAbs(-2.0, 1e-12));
}

TEST_CASE("ArkitTrajToOpticalWorld_ArbitraryRodriguesRotation_"
          "ReturnsOrthonormalRotation",
          "[io][arkitscenes]") {
  // Arbitrary Rodrigues rotation + translation.
  const std::array<double, 16> pose =
      arkit_traj_to_optical_world(0.3, -0.7, 1.1, 0.5, -0.2, 4.0);

  // Extract 3x3 rotation columns (row-major).
  auto col = [&](int c) {
    return std::array<double, 3>{pose[0 * 4 + c], pose[1 * 4 + c],
                                 pose[2 * 4 + c]};
  };
  auto dot = [](const std::array<double, 3> &a,
                const std::array<double, 3> &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  };
  const auto c0 = col(0), c1 = col(1), c2 = col(2);

  CHECK_THAT(dot(c0, c0), WithinAbs(1.0, 1e-9));
  CHECK_THAT(dot(c1, c1), WithinAbs(1.0, 1e-9));
  CHECK_THAT(dot(c2, c2), WithinAbs(1.0, 1e-9));
  CHECK_THAT(dot(c0, c1), WithinAbs(0.0, 1e-9));
  CHECK_THAT(dot(c0, c2), WithinAbs(0.0, 1e-9));
  CHECK_THAT(dot(c1, c2), WithinAbs(0.0, 1e-9));

  // Bottom row is [0 0 0 1].
  CHECK_THAT(pose[12], WithinAbs(0.0, 1e-12));
  CHECK_THAT(pose[13], WithinAbs(0.0, 1e-12));
  CHECK_THAT(pose[14], WithinAbs(0.0, 1e-12));
  CHECK_THAT(pose[15], WithinAbs(1.0, 1e-12));
}

// ─── Pose interpolation (10 Hz trajectory vs 60 Hz depth) ──────────────────
//
// Ground truth: a camera whose position moves linearly and whose orientation
// rotates at a constant rate about a fixed axis. Both SLERP (constant-rate
// rotation about a fixed axis is a quaternion geodesic) and lerp (linear
// translation) are *exact* for such motion, so the interpolated pose must
// reproduce the analytic pose to numerical precision.
namespace {

/// Analytic camera->world pose at device-relative time `t_ref + tau`.
struct Motion {
  Eigen::Vector3d axis = Eigen::Vector3d(0.3, -0.5, 0.81).normalized();
  double omega = 0.7;                   // rad/s
  Eigen::Vector3d p0{0.2, -1.0, 3.0};   // m
  Eigen::Vector3d v{0.25, 0.10, -0.40}; // m/s

  Eigen::Matrix4d c2w(double tau) const {
    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(omega * tau, axis).toRotationMatrix();
    M.block<3, 1>(0, 3) = p0 + v * tau;
    return M;
  }
};

/// Reference timestamp: ARKit timestamps are device uptime, not epoch, so use a
/// realistically large base value to catch any absolute-time assumptions.
constexpr double kTRef = 5045.333;

Eigen::Matrix4d to_mat(const std::array<double, 16> &a) {
  Eigen::Matrix4d M;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      M(r, c) = a[static_cast<std::size_t>(r * 4 + c)];
  return M;
}

/// Build a trajectory sample by the same route the importer does: turn the
/// desired camera->world pose back into the world->camera (rodrigues,
/// translation) pair a `.traj` line stores, then run it through
/// arkit_traj_to_optical_world.
reusex::io::ArkitPoseSample sample_at(const Motion &m, double tau) {
  const Eigen::Matrix4d w2c = m.c2w(tau).inverse();
  const Eigen::AngleAxisd aa(Eigen::Matrix3d(w2c.block<3, 3>(0, 0)));
  const Eigen::Vector3d rv = aa.axis() * aa.angle();
  const Eigen::Vector3d tr = w2c.block<3, 1>(0, 3);
  return {kTRef + tau, arkit_traj_to_optical_world(rv.x(), rv.y(), rv.z(),
                                                   tr.x(), tr.y(), tr.z())};
}

/// `n` samples at `dt` spacing starting at tau=0 (10 Hz => dt=0.1).
ArkitTrajectory make_traj(const Motion &m, double dt, int n) {
  ArkitTrajectory traj;
  for (int i = 0; i < n; ++i)
    traj.push_back(sample_at(m, i * dt));
  return traj;
}

} // namespace

TEST_CASE(
    "InterpolatePose_60HzQueriesOn10HzTrajectory_ReproducesAnalyticMotion",
    "[io][arkitscenes]") {
  const Motion m;
  const ArkitTrajectory traj = make_traj(m, 0.1, 11); // 10 Hz, 1.0 s span

  const double kDepthDt = 1.0 / 60.0; // ~16.7 ms
  int checked = 0;
  for (int i = 0; i <= 60; ++i) {
    const double tau = i * kDepthDt;
    const double t = kTRef + tau;
    if (t > traj.back().ts) // last sample may land a rounding hair past the end
      break;
    const auto pose = interpolate_pose(traj, t);
    REQUIRE(pose.has_value());
    const Eigen::Matrix4d got = to_mat(*pose);
    const Eigen::Matrix4d want = m.c2w(tau);
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        CHECK_THAT(got(r, c), WithinAbs(want(r, c), 1e-9));
    ++checked;
  }
  // ~61 queries over the 1 s span: six depth frames per trajectory sample.
  CHECK(checked >= 58);
}

TEST_CASE("InterpolatePose_QueryAtSampleTimestamp_ReturnsSamplePoseUnchanged",
          "[io][arkitscenes]") {
  const Motion m;
  const ArkitTrajectory traj = make_traj(m, 0.1, 11);
  for (const auto &s : traj) {
    const auto pose = interpolate_pose(traj, s.ts);
    REQUIRE(pose.has_value());
    for (std::size_t i = 0; i < 16; ++i)
      CHECK_THAT((*pose)[i], WithinAbs(s.pose[i], 1e-12));
  }
}

TEST_CASE("InterpolatePose_QueryBetweenSamples_AvoidsNearestMatchStaleness",
          "[io][arkitscenes]") {
  // The defect this replaces: with a 20 ms tolerance every 10 Hz pose was
  // reused for three 60 Hz frames, so a frame could sit ~16.7 ms away from the
  // pose it was given. At the motion above that is a real position error.
  const Motion m;
  const ArkitTrajectory traj = make_traj(m, 0.1, 11);

  const double tau = 0.1 + 1.0 / 60.0; // one depth frame past a sample
  const auto pose = interpolate_pose(traj, kTRef + tau);
  REQUIRE(pose.has_value());

  const Eigen::Vector3d want = m.c2w(tau).block<3, 1>(0, 3);
  const Eigen::Vector3d interp = to_mat(*pose).block<3, 1>(0, 3);
  const Eigen::Vector3d nearest = to_mat(traj[1].pose).block<3, 1>(0, 3);

  CHECK_THAT((interp - want).norm(), WithinAbs(0.0, 1e-9));
  // The nearest sample is off by |v| * 16.7 ms ~ 8 mm of camera translation
  // alone (the rotation error adds more at depth).
  CHECK((nearest - want).norm() > 5e-3);
}

TEST_CASE("InterpolatePose_TimestampOutsideTrajectorySpan_ReturnsNullopt",
          "[io][arkitscenes]") {
  const Motion m;
  const ArkitTrajectory traj = make_traj(m, 0.1, 11); // spans kTRef .. kTRef+1

  CHECK_FALSE(interpolate_pose(traj, kTRef - 0.001).has_value()); // just before
  CHECK_FALSE(interpolate_pose(traj, kTRef - 5.0).has_value());
  CHECK_FALSE(interpolate_pose(traj, kTRef + 1.001).has_value()); // just after
  CHECK_FALSE(interpolate_pose(traj, 0.0).has_value());
  // Endpoints themselves are valid (they need no extrapolation).
  CHECK(interpolate_pose(traj, traj.front().ts).has_value());
  CHECK(interpolate_pose(traj, traj.back().ts).has_value());
  // Empty trajectory.
  CHECK_FALSE(interpolate_pose(ArkitTrajectory{}, kTRef).has_value());
}

TEST_CASE("InterpolatePose_BracketingGapExceedsMaxPoseGap_ReturnsNullopt",
          "[io][arkitscenes]") {
  // Tracking loss: a 1 s hole in an otherwise 10 Hz trajectory.
  const Motion m;
  ArkitTrajectory traj;
  traj.push_back(sample_at(m, 0.0));
  traj.push_back(sample_at(m, 0.1));
  traj.push_back(sample_at(m, 1.1)); // 1.0 s gap > kArkitMaxPoseGap (0.5 s)
  traj.push_back(sample_at(m, 1.2));

  CHECK_FALSE(interpolate_pose(traj, kTRef + 0.6).has_value()); // in the hole
  CHECK(interpolate_pose(traj, kTRef + 0.05).has_value());      // normal gap
  CHECK(interpolate_pose(traj, kTRef + 1.15).has_value());      // normal gap
  // A caller willing to bridge the hole can raise the limit.
  CHECK(interpolate_pose(traj, kTRef + 0.6, 2.0).has_value());
}

TEST_CASE("ParseFrameTimestamp_VariousFilenames_ParsesTrailingTimestamp",
          "[io][arkitscenes]") {
  double ts = 0;
  REQUIRE(parse_frame_timestamp("lowres_depth/41069050_5045.334.png", ts));
  CHECK_THAT(ts, WithinAbs(5045.334, 1e-9));

  REQUIRE(parse_frame_timestamp("41069050_452.395.pincam", ts));
  CHECK_THAT(ts, WithinAbs(452.395, 1e-9));

  // Not a number / trailing junk / no digits at all.
  CHECK_FALSE(parse_frame_timestamp("41069050_abc.png", ts));
  CHECK_FALSE(parse_frame_timestamp("41069050_12ab.png", ts));
  CHECK_FALSE(parse_frame_timestamp("notimestamp.png", ts));
}
