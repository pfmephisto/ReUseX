// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <ReUseX/geometry/utils.hpp>
#include <Eigen/Core>

using namespace ReUseX::geometry;
using Catch::Approx;

TEST_CASE("Distance from point to plane", "[geometry][dist_plane_point]") {
    
    SECTION("Point on plane has zero distance") {
        // Plane: z = 0 (XY plane)
        Eigen::Vector4d plane(0.0, 0.0, 1.0, 0.0);  // nx, ny, nz, d
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
        // This should still compute correct distance
        Eigen::Vector4d plane(2.0, 0.0, 0.0, 0.0);
        Eigen::Vector3d point(1.0, 0.0, 0.0);
        
        double distance = dist_plane_point(plane, point);
        // Distance = (2*1 + 0*0 + 0*0 + 0) / sqrt(4 + 0 + 0) = 2/2 = 1
        REQUIRE(distance == Approx(2.0).margin(1e-10));
    }
    
    SECTION("Distance to diagonal plane") {
        // Plane: x + y + z = 0 (passes through origin)
        // Normal is (1, 1, 1), d = 0
        Eigen::Vector4d plane(1.0, 1.0, 1.0, 0.0);
        Eigen::Vector3d point(1.0, 1.0, 1.0);
        
        double distance = dist_plane_point(plane, point);
        // Distance = (1 + 1 + 1 + 0) / sqrt(3) = 3 / sqrt(3) = sqrt(3)
        REQUIRE(distance == Approx(3.0).margin(1e-10));
    }
    
    SECTION("Distance from origin to plane") {
        // Plane: x + y + z = 6
        Eigen::Vector4d plane(1.0, 1.0, 1.0, -6.0);
        Eigen::Vector3d point(0.0, 0.0, 0.0);
        
        double distance = dist_plane_point(plane, point);
        // Distance = (0 + 0 + 0 - 6) / sqrt(3) = -6 / sqrt(3) = -2*sqrt(3)
        REQUIRE(distance == Approx(-6.0).margin(1e-10));
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

TEST_CASE("Distance calculations with normalized normals", "[geometry][dist_plane_point][normalized]") {
    
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
