// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/geometry/unweld.hpp>
#include <reusex/geometry/utils.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <pcl/conversions.h>

#include <cmath>
#include <numbers>

using namespace ReUseX;
using namespace ReUseX::geometry;
using Catch::Approx;

namespace {

/// Helper to build a pcl::PolygonMesh from points and triangle indices.
pcl::PolygonMesh make_mesh(const std::vector<std::array<float, 3>> &pts,
                           const std::vector<std::array<int, 3>> &tris) {
  CloudLocPtr cloud(new CloudLoc);
  cloud->points.resize(pts.size());
  cloud->width = static_cast<uint32_t>(pts.size());
  cloud->height = 1;
  for (size_t i = 0; i < pts.size(); ++i) {
    cloud->points[i].x = pts[i][0];
    cloud->points[i].y = pts[i][1];
    cloud->points[i].z = pts[i][2];
  }

  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(*cloud, mesh.cloud);
  mesh.polygons.resize(tris.size());
  for (size_t i = 0; i < tris.size(); ++i) {
    mesh.polygons[i].vertices = {tris[i][0], tris[i][1], tris[i][2]};
  }
  return mesh;
}

/// Extract vertex count from a PolygonMesh.
size_t vertex_count(const pcl::PolygonMesh &mesh) {
  CloudLocPtr cloud(new CloudLoc);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  return cloud->points.size();
}

/// Extract the point cloud from a PolygonMesh.
CloudLocPtr extract_cloud(const pcl::PolygonMesh &mesh) {
  CloudLocPtr cloud(new CloudLoc);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  return cloud;
}

/// Build an axis-aligned unit cube mesh (8 vertices, 12 triangles).
pcl::PolygonMesh make_cube() {
  // Vertices of a unit cube [0,1]^3
  std::vector<std::array<float, 3>> pts = {
      {0, 0, 0}, // 0
      {1, 0, 0}, // 1
      {1, 1, 0}, // 2
      {0, 1, 0}, // 3
      {0, 0, 1}, // 4
      {1, 0, 1}, // 5
      {1, 1, 1}, // 6
      {0, 1, 1}, // 7
  };

  // 12 triangles, 2 per face, consistent winding (outward normals)
  std::vector<std::array<int, 3>> tris = {
      // bottom (z=0) normal -Z
      {0, 2, 1},
      {0, 3, 2},
      // top (z=1) normal +Z
      {4, 5, 6},
      {4, 6, 7},
      // front (y=0) normal -Y
      {0, 1, 5},
      {0, 5, 4},
      // back (y=1) normal +Y
      {2, 3, 7},
      {2, 7, 6},
      // left (x=0) normal -X
      {0, 4, 7},
      {0, 7, 3},
      // right (x=1) normal +X
      {1, 2, 6},
      {1, 6, 5},
  };
  return make_mesh(pts, tris);
}

} // namespace

TEST_CASE("unweld_mesh - empty mesh", "[geometry][unweld]") {
  pcl::PolygonMesh empty;
  auto result = unweld_mesh(empty, 1.0f);
  REQUIRE(result->polygons.empty());
}

TEST_CASE("unweld_mesh - single triangle unchanged", "[geometry][unweld]") {
  auto mesh = make_mesh({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {{0, 1, 2}});
  auto result = unweld_mesh(mesh, 0.0f);

  REQUIRE(vertex_count(*result) == 3);
  REQUIRE(result->polygons.size() == 1);
  REQUIRE(result->polygons[0].vertices.size() == 3);
}

TEST_CASE("unweld_mesh - two coplanar triangles sharing edge",
          "[geometry][unweld]") {
  //   3---2
  //   | / |
  //   0---1
  // Both in XY plane (z=0), normals identical.
  auto mesh = make_mesh({{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}},
                         {{0, 1, 2}, {0, 2, 3}});

  // threshold pi/4 — normals are identical (angle=0) so vertices stay shared
  auto result =
      unweld_mesh(mesh, static_cast<float>(std::numbers::pi) / 4.0f);
  REQUIRE(vertex_count(*result) == 4);
  REQUIRE(result->polygons.size() == 2);
}

TEST_CASE("unweld_mesh - two perpendicular triangles, threshold below 90deg",
          "[geometry][unweld]") {
  // Triangle A: in XY plane (z=0)  vertices 0,1,2
  // Triangle B: in XZ plane (y=0)  vertices 0,1,3
  // Shared edge: 0-1.  Angle between normals = 90 degrees.
  auto mesh = make_mesh({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                         {{0, 1, 2}, {0, 3, 1}});

  // threshold pi/4 (45 deg) < 90 deg dihedral -> split shared edge vertices
  float threshold = static_cast<float>(std::numbers::pi) / 4.0f;
  auto result = unweld_mesh(mesh, threshold);

  // Each triangle gets its own 3 vertices = 6 total
  REQUIRE(vertex_count(*result) == 6);
  REQUIRE(result->polygons.size() == 2);
}

TEST_CASE("unweld_mesh - two perpendicular triangles, threshold above 90deg",
          "[geometry][unweld]") {
  auto mesh = make_mesh({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                         {{0, 1, 2}, {0, 3, 1}});

  // threshold 2.0 rad (~114 deg) > 90 deg dihedral -> no split
  auto result = unweld_mesh(mesh, 2.0f);
  REQUIRE(vertex_count(*result) == 4);
  REQUIRE(result->polygons.size() == 2);
}

TEST_CASE("unweld_mesh - threshold=0 splits all non-coplanar faces on cube",
          "[geometry][unweld]") {
  auto mesh = make_cube();
  auto result = unweld_mesh(mesh, 0.0f);

  // threshold=0: faces with identical normals (angle=0) stay grouped,
  // any positive angle causes a split.
  // Cube has 6 faces, each with 2 coplanar triangles sharing an edge.
  // Each face group gets its own 4 vertices = 6 * 4 = 24.
  REQUIRE(vertex_count(*result) == 24);
  REQUIRE(result->polygons.size() == 12);
}

TEST_CASE("unweld_mesh - no unweld (threshold=pi) on cube",
          "[geometry][unweld]") {
  auto mesh = make_cube();
  auto result = unweld_mesh(mesh, static_cast<float>(std::numbers::pi));

  // All sharing preserved: 8 original vertices
  REQUIRE(vertex_count(*result) == 8);
  REQUIRE(result->polygons.size() == 12);
}

TEST_CASE("unweld_mesh - vertex positions preserved", "[geometry][unweld]") {
  auto mesh = make_mesh({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
                         {{0, 1, 2}, {0, 3, 1}});

  auto result = unweld_mesh(mesh, 0.0f); // full unweld
  auto cloud_in = extract_cloud(mesh);
  auto cloud_out = extract_cloud(*result);

  // Every output vertex position must exist in the input
  for (const auto &pt : cloud_out->points) {
    bool found = false;
    for (const auto &orig : cloud_in->points) {
      if (std::abs(pt.x - orig.x) < 1e-6f && std::abs(pt.y - orig.y) < 1e-6f &&
          std::abs(pt.z - orig.z) < 1e-6f) {
        found = true;
        break;
      }
    }
    REQUIRE(found);
  }
}

TEST_CASE("unweld_mesh - face winding preserved", "[geometry][unweld]") {
  // Single triangle: winding should produce the same normal before and after
  auto mesh = make_mesh({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {{0, 1, 2}});
  auto result = unweld_mesh(mesh, 0.0f);

  auto cloud_in = extract_cloud(mesh);
  auto cloud_out = extract_cloud(*result);

  auto normal_in = compute_polygon_normal(mesh.polygons[0], cloud_in);
  auto normal_out = compute_polygon_normal(result->polygons[0], cloud_out);

  REQUIRE(normal_in.x() == Approx(normal_out.x()).margin(1e-6f));
  REQUIRE(normal_in.y() == Approx(normal_out.y()).margin(1e-6f));
  REQUIRE(normal_in.z() == Approx(normal_out.z()).margin(1e-6f));
}
