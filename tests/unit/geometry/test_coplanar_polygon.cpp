// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <geometry/CoplanarPolygon.hpp>

using namespace ReUseX::geometry;
using Catch::Approx;

// Helper: build a unit square in the XY plane (z=0)
static CoplanarPolygon make_xy_square() {
  CoplanarPolygon poly;
  poly.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  poly.plane = Eigen::Vector4d(0, 0, 1, 0); // z = 0
  return poly;
}

TEST_CASE("CoplanarPolygon - is_valid", "[geometry][coplanar_polygon]") {
  SECTION("Valid square") {
    auto poly = make_xy_square();
    REQUIRE(poly.is_valid());
  }

  SECTION("Too few vertices") {
    CoplanarPolygon poly;
    poly.vertices = {{0, 0, 0}, {1, 0, 0}};
    poly.plane = Eigen::Vector4d(0, 0, 1, 0);
    REQUIRE_FALSE(poly.is_valid());
  }

  SECTION("Zero plane normal") {
    CoplanarPolygon poly;
    poly.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    poly.plane = Eigen::Vector4d(0, 0, 0, 0);
    REQUIRE_FALSE(poly.is_valid());
  }
}

TEST_CASE("CoplanarPolygon - area of unit square", "[geometry][coplanar_polygon]") {
  auto poly = make_xy_square();
  REQUIRE(poly.area() == Approx(1.0).margin(1e-10));
}

TEST_CASE("CoplanarPolygon - area of triangle", "[geometry][coplanar_polygon]") {
  CoplanarPolygon poly;
  poly.vertices = {{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
  poly.plane = Eigen::Vector4d(0, 0, 1, 0);
  REQUIRE(poly.area() == Approx(2.0).margin(1e-10));
}

TEST_CASE("CoplanarPolygon - area of rectangle in YZ plane",
          "[geometry][coplanar_polygon]") {
  CoplanarPolygon poly;
  poly.vertices = {{0, 0, 0}, {0, 3, 0}, {0, 3, 2}, {0, 0, 2}};
  poly.plane = Eigen::Vector4d(1, 0, 0, 0); // x = 0
  REQUIRE(poly.area() == Approx(6.0).margin(1e-10));
}

TEST_CASE("CoplanarPolygon - centroid", "[geometry][coplanar_polygon]") {
  auto poly = make_xy_square();
  auto c = poly.centroid();
  REQUIRE(c.x() == Approx(0.5));
  REQUIRE(c.y() == Approx(0.5));
  REQUIRE(c.z() == Approx(0.0));
}

TEST_CASE("CoplanarPolygon - centroid of empty polygon",
          "[geometry][coplanar_polygon]") {
  CoplanarPolygon poly;
  auto c = poly.centroid();
  REQUIRE(c.x() == Approx(0.0));
  REQUIRE(c.y() == Approx(0.0));
  REQUIRE(c.z() == Approx(0.0));
}

TEST_CASE("CoplanarPolygon - normal", "[geometry][coplanar_polygon]") {
  auto poly = make_xy_square();
  auto n = poly.normal();
  REQUIRE(n.x() == Approx(0.0));
  REQUIRE(n.y() == Approx(0.0));
  REQUIRE(n.z() == Approx(1.0));
}

TEST_CASE("CoplanarPolygon - normal with non-unit plane",
          "[geometry][coplanar_polygon]") {
  CoplanarPolygon poly;
  poly.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  poly.plane = Eigen::Vector4d(0, 0, 5, 0); // scaled normal
  auto n = poly.normal();
  REQUIRE(n.norm() == Approx(1.0).margin(1e-10));
  REQUIRE(n.z() == Approx(1.0));
}

TEST_CASE("CoplanarPolygon - bounding_box", "[geometry][coplanar_polygon]") {
  auto poly = make_xy_square();
  auto [lo, hi] = poly.bounding_box();
  REQUIRE(lo.x() == Approx(0.0));
  REQUIRE(lo.y() == Approx(0.0));
  REQUIRE(lo.z() == Approx(0.0));
  REQUIRE(hi.x() == Approx(1.0));
  REQUIRE(hi.y() == Approx(1.0));
  REQUIRE(hi.z() == Approx(0.0));
}

TEST_CASE("CoplanarPolygon - serialize/deserialize round-trip",
          "[geometry][coplanar_polygon]") {
  auto poly = make_xy_square();
  auto blob = poly.serialize_vertices();

  REQUIRE(blob.size() == 4 * 3 * sizeof(double)); // 4 vertices * 24 bytes

  auto restored =
      CoplanarPolygon::deserialize_vertices(blob.data(), blob.size());
  REQUIRE(restored.size() == poly.vertices.size());

  for (size_t i = 0; i < poly.vertices.size(); ++i) {
    REQUIRE(restored[i].x() == Approx(poly.vertices[i].x()));
    REQUIRE(restored[i].y() == Approx(poly.vertices[i].y()));
    REQUIRE(restored[i].z() == Approx(poly.vertices[i].z()));
  }
}

TEST_CASE("CoplanarPolygon - deserialize invalid size throws",
          "[geometry][coplanar_polygon]") {
  std::vector<uint8_t> bad(25); // not a multiple of 24
  REQUIRE_THROWS_AS(
      CoplanarPolygon::deserialize_vertices(bad.data(), bad.size()),
      std::runtime_error);
}

TEST_CASE("CoplanarPolygon - empty polygon area is zero",
          "[geometry][coplanar_polygon]") {
  CoplanarPolygon poly;
  REQUIRE(poly.area() == Approx(0.0));
}
