// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <reusex/utils/math.hpp>

#include <numbers>

using namespace reusex::utils;
using Catch::Approx;

TEST_CASE("Remap function basic tests", "[math][remap]") {

  SECTION("Remap to default range [0, 1]") {
    // Value at minimum of input range should map to 0
    REQUIRE(remap(0.0, 0.0, 10.0) == Approx(0.0));

    // Value at maximum of input range should map to 1
    REQUIRE(remap(10.0, 0.0, 10.0) == Approx(1.0));

    // Value at midpoint should map to 0.5
    REQUIRE(remap(5.0, 0.0, 10.0) == Approx(0.5));
  }

  SECTION("Remap to custom output range") {
    // Remap [0, 10] to [100, 200]
    REQUIRE(remap(0.0, 0.0, 10.0, 100.0, 200.0) == Approx(100.0));
    REQUIRE(remap(10.0, 0.0, 10.0, 100.0, 200.0) == Approx(200.0));
    REQUIRE(remap(5.0, 0.0, 10.0, 100.0, 200.0) == Approx(150.0));
  }

  SECTION("Remap with different input ranges") {
    // Remap [-10, 10] to [0, 1]
    REQUIRE(remap(-10.0, -10.0, 10.0) == Approx(0.0));
    REQUIRE(remap(10.0, -10.0, 10.0) == Approx(1.0));
    REQUIRE(remap(0.0, -10.0, 10.0) == Approx(0.5));
  }

  SECTION("Remap with negative output range") {
    // Remap [0, 10] to [-1, 1]
    REQUIRE(remap(0.0, 0.0, 10.0, -1.0, 1.0) == Approx(-1.0));
    REQUIRE(remap(10.0, 0.0, 10.0, -1.0, 1.0) == Approx(1.0));
    REQUIRE(remap(5.0, 0.0, 10.0, -1.0, 1.0) == Approx(0.0));
  }

  SECTION("Remap with integer types") {
    REQUIRE(remap(5, 0, 10, 0, 100) == 50);
    REQUIRE(remap(0, 0, 10, 0, 100) == 0);
    REQUIRE(remap(10, 0, 10, 0, 100) == 100);
  }

  SECTION("Remap with float types") {
    REQUIRE(remap(2.5f, 0.0f, 10.0f) == Approx(0.25f));
    REQUIRE(remap(7.5f, 0.0f, 10.0f) == Approx(0.75f));
  }

  SECTION("Remap edge cases") {
    // Value equals minimum
    REQUIRE(remap(1.0, 1.0, 5.0) == Approx(0.0));

    // Value equals maximum
    REQUIRE(remap(5.0, 1.0, 5.0) == Approx(1.0));

    // Quarter way through range
    REQUIRE(remap(2.0, 1.0, 5.0) == Approx(0.25));

    // Three quarters through range
    REQUIRE(remap(4.0, 1.0, 5.0) == Approx(0.75));
  }

  SECTION("Remap extrapolation") {
    // Values outside input range should still be mapped linearly
    REQUIRE(remap(15.0, 0.0, 10.0) == Approx(1.5));
    REQUIRE(remap(-5.0, 0.0, 10.0) == Approx(-0.5));
  }

  SECTION("Remap with inverted output range") {
    // Remap [0, 10] to [10, 0] (inverted)
    REQUIRE(remap(0.0, 0.0, 10.0, 10.0, 0.0) == Approx(10.0));
    REQUIRE(remap(10.0, 0.0, 10.0, 10.0, 0.0) == Approx(0.0));
    REQUIRE(remap(5.0, 0.0, 10.0, 10.0, 0.0) == Approx(5.0));
  }
}

TEST_CASE("Remap function precision tests", "[math][remap][precision]") {

  SECTION("Double precision") {
    double result = remap(5.123456789, 0.0, 10.0);
    REQUIRE(result == Approx(0.5123456789).epsilon(1e-10));
  }

  SECTION("Float precision") {
    float result = remap(5.12345f, 0.0f, 10.0f);
    REQUIRE(result == Approx(0.512345f).epsilon(1e-6));
  }
}

TEST_CASE("deg_to_rad converts degrees to radians", "[utils][math]") {
  REQUIRE(deg_to_rad(0.0) == Approx(0.0));
  REQUIRE(deg_to_rad(90.0) == Approx(std::numbers::pi / 2.0));
  REQUIRE(deg_to_rad(180.0) == Approx(std::numbers::pi));
  REQUIRE(deg_to_rad(360.0) == Approx(2.0 * std::numbers::pi));

  SECTION("Works with float") {
    REQUIRE(deg_to_rad(90.0f) == Approx(std::numbers::pi_v<float> / 2.0f));
  }
}

TEST_CASE("rad_to_deg converts radians to degrees", "[utils][math]") {
  REQUIRE(rad_to_deg(0.0) == Approx(0.0));
  REQUIRE(rad_to_deg(std::numbers::pi) == Approx(180.0));
  REQUIRE(rad_to_deg(std::numbers::pi / 2.0) == Approx(90.0));

  SECTION("Works with float") {
    REQUIRE(rad_to_deg(std::numbers::pi_v<float>) == Approx(180.0f));
  }
}

TEST_CASE("deg_to_rad and rad_to_deg roundtrip", "[utils][math]") {
  REQUIRE(rad_to_deg(deg_to_rad(45.0)) == Approx(45.0));
  REQUIRE(rad_to_deg(deg_to_rad(123.456)) == Approx(123.456));
  REQUIRE(deg_to_rad(rad_to_deg(1.5)) == Approx(1.5));

  SECTION("Float roundtrip") {
    REQUIRE(rad_to_deg(deg_to_rad(45.0f)) == Approx(45.0f));
  }
}
