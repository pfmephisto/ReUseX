// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/geometry/utils.hpp>
#include <ReUseX/types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ReUseX;

TEST_CASE("compute_polygon_normal - triangle", "[geometry][utils]") {
  // Create a simple triangle in the XY plane with normal pointing in +Z
  // direction
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(3);

  // Triangle vertices forming a right triangle
  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 1.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  cloud->points[2].x = 0.0f;
  cloud->points[2].y = 1.0f;
  cloud->points[2].z = 0.0f;

  pcl::Vertices poly;
  poly.vertices = {0, 1, 2};

  Eigen::Vector3f normal = geometry::compute_polygon_normal(poly, cloud);

  // The normal should point in the +Z direction (0, 0, 1)
  REQUIRE(normal.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(normal.y() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(normal.z() == Catch::Approx(1.0f).margin(1e-6));

  // Verify the normal is normalized
  REQUIRE(normal.norm() == Catch::Approx(1.0f).margin(1e-6));
}

TEST_CASE("compute_polygon_normal - square", "[geometry][utils]") {
  // Create a square in the XY plane
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(4);

  // Square vertices
  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 2.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  cloud->points[2].x = 2.0f;
  cloud->points[2].y = 2.0f;
  cloud->points[2].z = 0.0f;

  cloud->points[3].x = 0.0f;
  cloud->points[3].y = 2.0f;
  cloud->points[3].z = 0.0f;

  pcl::Vertices poly;
  poly.vertices = {0, 1, 2, 3};

  Eigen::Vector3f normal = geometry::compute_polygon_normal(poly, cloud);

  // The normal should point in the +Z direction (0, 0, 1)
  REQUIRE(normal.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(normal.y() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(normal.z() == Catch::Approx(1.0f).margin(1e-6));

  // Verify the normal is normalized
  REQUIRE(normal.norm() == Catch::Approx(1.0f).margin(1e-6));
}

TEST_CASE("compute_polygon_normal - vertical plane", "[geometry][utils]") {
  // Create a square in the XZ plane (vertical, normal pointing in +Y direction)
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(4);

  // Vertical square vertices
  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 1.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  cloud->points[2].x = 1.0f;
  cloud->points[2].y = 0.0f;
  cloud->points[2].z = 1.0f;

  cloud->points[3].x = 0.0f;
  cloud->points[3].y = 0.0f;
  cloud->points[3].z = 1.0f;

  pcl::Vertices poly;
  poly.vertices = {0, 1, 2, 3};

  Eigen::Vector3f normal = geometry::compute_polygon_normal(poly, cloud);

  // The normal should point in the -Y direction (0, -1, 0)
  // (depends on vertex winding order)
  REQUIRE(normal.x() == Catch::Approx(0.0f).margin(1e-6));
  REQUIRE(std::abs(normal.y()) == Catch::Approx(1.0f).margin(1e-6));
  REQUIRE(normal.z() == Catch::Approx(0.0f).margin(1e-6));

  // Verify the normal is normalized
  REQUIRE(normal.norm() == Catch::Approx(1.0f).margin(1e-6));
}

TEST_CASE("compute_polygon_normal - reverse winding", "[geometry][utils]") {
  // Create a triangle with reverse winding order
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(3);

  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 1.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  cloud->points[2].x = 0.0f;
  cloud->points[2].y = 1.0f;
  cloud->points[2].z = 0.0f;

  // Normal winding (CCW)
  pcl::Vertices poly_ccw;
  poly_ccw.vertices = {0, 1, 2};
  Eigen::Vector3f normal_ccw =
      geometry::compute_polygon_normal(poly_ccw, cloud);

  // Reverse winding (CW)
  pcl::Vertices poly_cw;
  poly_cw.vertices = {0, 2, 1};
  Eigen::Vector3f normal_cw = geometry::compute_polygon_normal(poly_cw, cloud);

  // Normals should be opposite
  REQUIRE(normal_ccw.x() == Catch::Approx(-normal_cw.x()).margin(1e-6));
  REQUIRE(normal_ccw.y() == Catch::Approx(-normal_cw.y()).margin(1e-6));
  REQUIRE(normal_ccw.z() == Catch::Approx(-normal_cw.z()).margin(1e-6));

  // Both should be normalized
  REQUIRE(normal_ccw.norm() == Catch::Approx(1.0f).margin(1e-6));
  REQUIRE(normal_cw.norm() == Catch::Approx(1.0f).margin(1e-6));
}

TEST_CASE("compute_polygon_normal - invalid input: too few vertices",
          "[geometry][utils]") {
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(2);

  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 1.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  // Polygon with only 2 vertices
  pcl::Vertices poly;
  poly.vertices = {0, 1};

  REQUIRE_THROWS_AS(geometry::compute_polygon_normal(poly, cloud),
                    std::invalid_argument);
}

TEST_CASE("compute_polygon_normal - degenerate polygon: collinear vertices",
          "[geometry][utils]") {
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(3);

  // All three points are collinear
  cloud->points[0].x = 0.0f;
  cloud->points[0].y = 0.0f;
  cloud->points[0].z = 0.0f;

  cloud->points[1].x = 1.0f;
  cloud->points[1].y = 0.0f;
  cloud->points[1].z = 0.0f;

  cloud->points[2].x = 2.0f;
  cloud->points[2].y = 0.0f;
  cloud->points[2].z = 0.0f;

  pcl::Vertices poly;
  poly.vertices = {0, 1, 2};

  // Should throw because the normal has zero magnitude
  REQUIRE_THROWS_AS(geometry::compute_polygon_normal(poly, cloud),
                    std::runtime_error);
}
