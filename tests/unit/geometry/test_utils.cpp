// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/geometry/utils.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

using namespace ReUseX::geometry;
using Catch::Approx;

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

TEST_CASE("Distance from point to plane", "[geometry][dist_plane_point]") {

  SECTION("Point on plane has zero distance") {
    // Plane: z = 0 (XY plane)
    Eigen::Vector4d plane(0.0, 0.0, 1.0, 0.0); // nx, ny, nz, d
    Eigen::Vector3d point(1.0, 2.0, 0.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(0.0).margin(1e-10));
  }

  SECTION("Point above XY plane") {
    // Plane: z = 0
    Eigen::Vector4d plane(0.0, 0.0, 1.0, 0.0);
    Eigen::Vector3d point(0.0, 0.0, 5.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(5.0));
  }

  SECTION("Point below XY plane") {
    // Plane: z = 0
    Eigen::Vector4d plane(0.0, 0.0, 1.0, 0.0);
    Eigen::Vector3d point(0.0, 0.0, -3.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(-3.0));
  }

  SECTION("Distance to YZ plane") {
    // Plane: x = 0
    Eigen::Vector4d plane(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d point(4.0, 0.0, 0.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(4.0));
  }

  SECTION("Distance to XZ plane") {
    // Plane: y = 0
    Eigen::Vector4d plane(0.0, 1.0, 0.0, 0.0);
    Eigen::Vector3d point(0.0, -2.5, 0.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(-2.5));
  }

  SECTION("Distance to offset plane") {
    // Plane: z = 3
    Eigen::Vector4d plane(0.0, 0.0, 1.0, -3.0);
    Eigen::Vector3d point(0.0, 0.0, 5.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(2.0));
  }

  SECTION("Distance with non-unit normal") {
    // Plane with normal (2, 0, 0) and d = 0
    // NOTE: The dist_plane_point implementation uses squaredNorm() instead of
    // norm() This differs from the standard point-to-plane distance formula
    // which uses norm() The implementation divides by squared norm: distance =
    // (n·p + d) / ||n||²
    Eigen::Vector4d plane(2.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d point(1.0, 0.0, 0.0);

    double distance = dist_plane_point(plane, point);
    // Distance = (2*1 + 0*0 + 0*0 + 0) / (4 + 0 + 0) = 2/4 = 0.5
    REQUIRE(distance == Approx(0.5).margin(1e-10));
  }

  SECTION("Distance to diagonal plane") {
    // Plane: x + y + z = 0 (passes through origin)
    // Normal is (1, 1, 1), d = 0
    Eigen::Vector4d plane(1.0, 1.0, 1.0, 0.0);
    Eigen::Vector3d point(1.0, 1.0, 1.0);

    double distance = dist_plane_point(plane, point);
    // Distance = (1 + 1 + 1 + 0) / (1 + 1 + 1) = 3 / 3 = 1.0
    REQUIRE(distance == Approx(1.0).margin(1e-10));
  }

  SECTION("Distance from origin to plane") {
    // Plane: x + y + z = 6
    Eigen::Vector4d plane(1.0, 1.0, 1.0, -6.0);
    Eigen::Vector3d point(0.0, 0.0, 0.0);

    double distance = dist_plane_point(plane, point);
    // Distance = (0 + 0 + 0 - 6) / (1 + 1 + 1) = -6 / 3 = -2.0
    REQUIRE(distance == Approx(-2.0).margin(1e-10));
  }

  SECTION("Symmetric points have opposite signed distances") {
    Eigen::Vector4d plane(0.0, 0.0, 1.0, 0.0);
    Eigen::Vector3d point1(0.0, 0.0, 3.0);
    Eigen::Vector3d point2(0.0, 0.0, -3.0);

    double dist1 = dist_plane_point(plane, point1);
    double dist2 = dist_plane_point(plane, point2);

    REQUIRE(dist1 == Approx(-dist2));
  }
}

TEST_CASE("Distance calculations with normalized normals",
          "[geometry][dist_plane_point][normalized]") {

  SECTION("Normalized normal vector") {
    // Plane with normalized normal (0, 0, 1) passing through z = 2
    Eigen::Vector4d plane(0.0, 0.0, 1.0, -2.0);
    Eigen::Vector3d point(5.0, 3.0, 7.0);

    double distance = dist_plane_point(plane, point);
    REQUIRE(distance == Approx(5.0));
  }

  SECTION("Another normalized plane") {
    // Plane with normal (1/sqrt(2), 1/sqrt(2), 0) and offset
    double sqrt2_inv = 1.0 / std::sqrt(2.0);
    Eigen::Vector4d plane(sqrt2_inv, sqrt2_inv, 0.0, 0.0);
    Eigen::Vector3d point(1.0, 1.0, 0.0);

    double distance = dist_plane_point(plane, point);
    // Distance = (1/sqrt(2) + 1/sqrt(2) + 0) / 1 = 2/sqrt(2) = sqrt(2)
    REQUIRE(distance == Approx(std::sqrt(2.0)));
  }
}
