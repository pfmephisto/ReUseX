// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * @file test_example_template.cpp
 * @brief Template file showing various Catch2 testing patterns for ReUseX
 *
 * This file serves as a reference for writing tests. Copy and adapt sections
 * as needed for testing specific ReUseX components.
 */

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/types.hpp>

using namespace ReUseX;
using Catch::Approx;
using Catch::Matchers::WithinAbs;

// =============================================================================
// Example 1: Basic test with multiple sections
// =============================================================================

TEST_CASE("Point cloud transformations", "[pointcloud][geometry]") {
  CloudPtr cloud(new Cloud());

  SECTION("Translation") {
    PointT point;
    point.x = 1.0f;
    point.y = 2.0f;
    point.z = 3.0f;
    cloud->push_back(point);

    // Apply translation (example - would use actual ReUseX functions)
    float tx = 5.0f, ty = 10.0f, tz = 15.0f;
    for (auto &p : cloud->points) {
      p.x += tx;
      p.y += ty;
      p.z += tz;
    }

    REQUIRE_THAT(cloud->points[0].x, WithinAbs(6.0f, 0.001f));
    REQUIRE_THAT(cloud->points[0].y, WithinAbs(12.0f, 0.001f));
    REQUIRE_THAT(cloud->points[0].z, WithinAbs(18.0f, 0.001f));
  }

  SECTION("Scaling") {
    PointT point;
    point.x = 2.0f;
    point.y = 4.0f;
    point.z = 6.0f;
    cloud->push_back(point);

    // Apply scaling
    float scale = 2.0f;
    for (auto &p : cloud->points) {
      p.x *= scale;
      p.y *= scale;
      p.z *= scale;
    }

    REQUIRE(cloud->points[0].x == Approx(4.0f));
    REQUIRE(cloud->points[0].y == Approx(8.0f));
    REQUIRE(cloud->points[0].z == Approx(12.0f));
  }
}

// =============================================================================
// Example 2: Testing with fixtures (setup/teardown)
// =============================================================================

class PointCloudFixture {
    protected:
  CloudPtr cloud;

  PointCloudFixture() : cloud(new Cloud()) {
    // Setup: Create a sample point cloud
    for (int i = 0; i < 100; ++i) {
      PointT point;
      point.x = static_cast<float>(i);
      point.y = static_cast<float>(i * 2);
      point.z = static_cast<float>(i * 3);
      point.r = static_cast<uint8_t>(i % 256);
      point.g = static_cast<uint8_t>((i * 2) % 256);
      point.b = static_cast<uint8_t>((i * 3) % 256);
      cloud->push_back(point);
    }
  }

  ~PointCloudFixture() {
    // Teardown (if needed)
  }
};

TEST_CASE_METHOD(PointCloudFixture, "Point cloud filtering",
                 "[pointcloud][filter]") {
  REQUIRE(cloud->size() == 100);

  SECTION("Filter by range") {
    IndicesPtr indices(new Indices());

    // Example: Select points where x < 50
    for (size_t i = 0; i < cloud->size(); ++i) {
      if (cloud->points[i].x < 50.0f) {
        indices->push_back(static_cast<int>(i));
      }
    }

    REQUIRE(indices->size() == 50);
  }

  SECTION("Color filtering") {
    int red_count = 0;

    // Count points with red channel > 50
    for (const auto &point : cloud->points) {
      if (point.r > 50) {
        red_count++;
      }
    }

    REQUIRE(red_count > 0);
  }
}

// =============================================================================
// Example 3: Testing exception handling
// =============================================================================

TEST_CASE("Error handling", "[error]") {
  SECTION("Empty cloud access") {
    CloudPtr empty_cloud(new Cloud());

    REQUIRE(empty_cloud->empty());
    REQUIRE_THROWS_AS(empty_cloud->at(0), std::out_of_range);
  }

  SECTION("Invalid index") {
    CloudPtr small_cloud(new Cloud());
    small_cloud->resize(5);

    REQUIRE_NOTHROW(small_cloud->at(4));
    REQUIRE_THROWS_AS(small_cloud->at(5), std::out_of_range);
  }
}

// =============================================================================
// Example 4: Parameterized tests using GENERATE
// =============================================================================

TEST_CASE("Point distance calculations", "[geometry][math]") {
  auto scale = GENERATE(1.0f, 2.0f, 5.0f, 10.0f);

  SECTION("Euclidean distance with scale " + std::to_string(scale)) {
    PointT p1, p2;
    p1.x = 0.0f;
    p1.y = 0.0f;
    p1.z = 0.0f;
    p2.x = scale;
    p2.y = scale;
    p2.z = scale;

    float expected_distance = scale * std::sqrt(3.0f);
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    float actual_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    REQUIRE(actual_distance == Approx(expected_distance).epsilon(0.001));
  }
}

// =============================================================================
// Example 5: Testing with Eigen matrices (common in point cloud processing)
// =============================================================================

TEST_CASE("Eigen operations", "[eigen][matrix]") {
  SECTION("Matrix-vector multiplication") {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d point(1.0, 2.0, 3.0);

    Eigen::Vector3d transformed = rotation * point;

    REQUIRE(transformed(0) == Approx(1.0));
    REQUIRE(transformed(1) == Approx(2.0));
    REQUIRE(transformed(2) == Approx(3.0));
  }

  SECTION("Vector normalization") {
    Eigen::Vector3d vec(3.0, 4.0, 0.0);
    double magnitude = vec.norm();

    REQUIRE(magnitude == Approx(5.0));

    vec.normalize();
    REQUIRE(vec.norm() == Approx(1.0));
    REQUIRE(vec(0) == Approx(0.6));
    REQUIRE(vec(1) == Approx(0.8));
  }
}

// =============================================================================
// Example 6: Benchmarking (if needed)
// =============================================================================

TEST_CASE("Performance tests", "[.benchmark]") {
  // Tests tagged with [.benchmark] are hidden by default
  // Run with: ./reusex_tests "[.benchmark]"

  SECTION("Large point cloud creation") {
    const size_t N = 1000000;
    CloudPtr large_cloud(new Cloud());
    large_cloud->reserve(N);

    for (size_t i = 0; i < N; ++i) {
      PointT point;
      point.x = static_cast<float>(i);
      point.y = static_cast<float>(i);
      point.z = static_cast<float>(i);
      large_cloud->push_back(point);
    }

    REQUIRE(large_cloud->size() == N);
  }
}
