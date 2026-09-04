// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/io/arkitscenes.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <string>

using Catch::Matchers::WithinAbs;
using reusex::io::arkit_traj_to_optical_world;
using reusex::io::ArkitPincam;
using reusex::io::parse_pincam;

TEST_CASE("parse_pincam parses width/height/fx/fy/cx/cy", "[io][arkitscenes]") {
  const ArkitPincam p = parse_pincam("256 192 211.5 211.9 127.9 95.8");
  CHECK_THAT(p.width, WithinAbs(256.0, 1e-9));
  CHECK_THAT(p.height, WithinAbs(192.0, 1e-9));
  CHECK_THAT(p.fx, WithinAbs(211.5, 1e-6));
  CHECK_THAT(p.fy, WithinAbs(211.9, 1e-6));
  CHECK_THAT(p.cx, WithinAbs(127.9, 1e-6));
  CHECK_THAT(p.cy, WithinAbs(95.8, 1e-6));
}

TEST_CASE("parse_pincam rejects malformed lines", "[io][arkitscenes]") {
  CHECK_THROWS(parse_pincam("256 192 211.5"));
}

TEST_CASE("arkit_traj_to_optical_world identity pose is the identity",
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

TEST_CASE("arkit_traj_to_optical_world inverts the world->camera extrinsic",
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

TEST_CASE("arkit_traj_to_optical_world rotation columns stay orthonormal",
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
