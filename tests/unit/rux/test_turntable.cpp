// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the pure orbit-math helper in the turntable viewer feature
// (#371).  No display, no PCL/VTK — only <array>, <cmath>, and <numbers>.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "view/turntable.hpp"

#include <cmath>
#include <numbers>

using Catch::Approx;
using rux::view::orbit_camera_position;

namespace {
constexpr double kPi = std::numbers::pi;
} // namespace

TEST_CASE("orbit_camera_position — azimuth zero points along +X",
          "[turntable]") {
  const std::array<double, 3> center = {0.0, 0.0, 0.0};
  const double radius = 10.0;
  const double elev = 0.0;

  auto pos = orbit_camera_position(center, radius, /*azimuth=*/0.0, elev);

  CHECK(pos[0] == Approx(10.0).margin(1e-9));
  CHECK(pos[1] == Approx(0.0).margin(1e-9));
  CHECK(pos[2] == Approx(0.0).margin(1e-9));
}

TEST_CASE("orbit_camera_position — azimuth 90° points along +Y",
          "[turntable]") {
  const std::array<double, 3> center = {0.0, 0.0, 0.0};
  const double radius = 10.0;
  const double elev = 0.0;

  auto pos = orbit_camera_position(center, radius, kPi / 2.0, elev);

  CHECK(pos[0] == Approx(0.0).margin(1e-9));
  CHECK(pos[1] == Approx(10.0).margin(1e-9));
  CHECK(pos[2] == Approx(0.0).margin(1e-9));
}

TEST_CASE("orbit_camera_position — elevation lifts the camera in Z",
          "[turntable]") {
  const std::array<double, 3> center = {0.0, 0.0, 0.0};
  const double radius = 10.0;
  const double elev_deg = 30.0;

  auto pos = orbit_camera_position(center, radius, /*azimuth=*/0.0, elev_deg);

  // At azimuth=0 and elevation=30°:
  //   x = radius * cos(0) * cos(30°) = 10 * cos(30°)
  //   y = radius * sin(0) * cos(30°) = 0
  //   z = radius * sin(30°)          = 5
  const double cos30 = std::cos(30.0 * kPi / 180.0);
  const double sin30 = std::sin(30.0 * kPi / 180.0);
  CHECK(pos[0] == Approx(radius * cos30).margin(1e-9));
  CHECK(pos[1] == Approx(0.0).margin(1e-9));
  CHECK(pos[2] == Approx(radius * sin30).margin(1e-9));
}

TEST_CASE("orbit_camera_position — non-zero center translates the orbit",
          "[turntable]") {
  const std::array<double, 3> center = {3.0, 4.0, 1.0};
  const double radius = 5.0;
  const double elev = 0.0;

  auto pos = orbit_camera_position(center, radius, /*azimuth=*/0.0, elev);

  // Camera should be at center + (radius, 0, 0)
  CHECK(pos[0] == Approx(center[0] + radius).margin(1e-9));
  CHECK(pos[1] == Approx(center[1]).margin(1e-9));
  CHECK(pos[2] == Approx(center[2]).margin(1e-9));
}

TEST_CASE("orbit_camera_position — distance from center equals radius",
          "[turntable]") {
  const std::array<double, 3> center = {1.0, 2.0, 3.0};
  const double radius = 7.5;

  for (double az = 0.0; az < 2.0 * kPi; az += kPi / 6.0) {
    auto pos = orbit_camera_position(center, radius, az, 20.0);
    const double dx = pos[0] - center[0];
    const double dy = pos[1] - center[1];
    const double dz = pos[2] - center[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    CHECK(dist == Approx(radius).margin(1e-9));
  }
}
