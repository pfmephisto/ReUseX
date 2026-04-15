// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/geometry/create_windows.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <pcl/conversions.h>

#include <random>

using namespace ReUseX;
using namespace ReUseX::geometry;

// Helper: Build a simple box PolygonMesh (6 faces, 8 vertices)
// The box spans [0,W] x [0,D] x [0,H].
// 4 vertical walls + floor + ceiling.
static pcl::PolygonMesh make_box_mesh(float W = 4.0f, float D = 3.0f,
                                       float H = 2.5f) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // 8 corners
  cloud.push_back({0, 0, 0});     // 0: left-front-bottom
  cloud.push_back({W, 0, 0});     // 1: right-front-bottom
  cloud.push_back({W, D, 0});     // 2: right-back-bottom
  cloud.push_back({0, D, 0});     // 3: left-back-bottom
  cloud.push_back({0, 0, H});     // 4: left-front-top
  cloud.push_back({W, 0, H});     // 5: right-front-top
  cloud.push_back({W, D, H});     // 6: right-back-top
  cloud.push_back({0, D, H});     // 7: left-back-top

  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(cloud, mesh.cloud);

  // Each quad face as 2 triangles
  auto quad = [&](int a, int b, int c, int d) {
    pcl::Vertices tri1;
    tri1.vertices = {a, b, c};
    mesh.polygons.push_back(tri1);
    pcl::Vertices tri2;
    tri2.vertices = {a, c, d};
    mesh.polygons.push_back(tri2);
  };

  // Front wall (y=0): 0,1,5,4
  quad(0, 1, 5, 4);
  // Back wall (y=D): 2,3,7,6
  quad(2, 3, 7, 6);
  // Left wall (x=0): 3,0,4,7
  quad(3, 0, 4, 7);
  // Right wall (x=W): 1,2,6,5
  quad(1, 2, 6, 5);
  // Floor (z=0): 0,3,2,1
  quad(0, 3, 2, 1);
  // Ceiling (z=H): 4,5,6,7
  quad(4, 5, 6, 7);

  return mesh;
}

// Helper: Create a point cloud with points clustered around a center
static CloudPtr make_cluster(const Eigen::Vector3f &center, int count,
                              float spread = 0.05f) {
  CloudPtr cloud(new Cloud);
  std::mt19937 gen(42);
  std::normal_distribution<float> dist(0.0f, spread);
  for (int i = 0; i < count; ++i) {
    PointT p;
    p.x = center.x() + dist(gen);
    p.y = center.y() + dist(gen);
    p.z = center.z() + dist(gen);
    p.r = 255;
    p.g = 255;
    p.b = 255;
    cloud->push_back(p);
  }
  return cloud;
}

TEST_CASE("extract_wall_candidates: box mesh", "[geometry][create_windows]") {
  auto mesh = make_box_mesh();
  auto walls = extract_wall_candidates(mesh);

  SECTION("Finds 4 vertical walls from box mesh") {
    // Floor and ceiling should be filtered (|normal.z| >= threshold)
    REQUIRE(walls.size() == 4);
  }

  SECTION("Wall normals are approximately horizontal") {
    for (const auto &w : walls) {
      REQUIRE(std::abs(w.normal.z()) < 0.3);
      REQUIRE_THAT(w.normal.norm(),
                   Catch::Matchers::WithinAbs(1.0, 1e-6));
    }
  }

  SECTION("Each wall has 2 faces (triangulated quad)") {
    for (const auto &w : walls) {
      REQUIRE(w.face_indices.size() == 2);
    }
  }

  SECTION("Wall planes are valid") {
    for (const auto &w : walls) {
      // Plane normal should match wall normal
      Eigen::Vector3d plane_n = w.plane.head<3>();
      REQUIRE_THAT(plane_n.norm(), Catch::Matchers::WithinAbs(1.0, 1e-6));
      double dot = plane_n.dot(w.normal);
      REQUIRE_THAT(std::abs(dot), Catch::Matchers::WithinAbs(1.0, 1e-6));
    }
  }
}

TEST_CASE("extract_wall_candidates: empty mesh", "[geometry][create_windows]") {
  pcl::PolygonMesh empty;
  auto walls = extract_wall_candidates(empty);
  REQUIRE(walls.empty());
}

TEST_CASE("create_windows: rectangle mode", "[geometry][create_windows]") {
  // Create a flat wall along Y-Z plane at x=0 (normal in +X direction)
  WallCandidate wall;
  wall.normal = Eigen::Vector3d(1.0, 0.0, 0.0);
  wall.centroid = Eigen::Vector3d(0.0, 2.0, 1.5);
  wall.plane = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0); // x=0 plane
  // Boundary covers y=[0,4], z=[0,3]
  wall.boundary_vertices = {
      {0, 0, 0}, {0, 4, 0}, {0, 4, 3}, {0, 0, 3}};

  // Create a window instance: cluster of points near (0.1, 2, 1.5)
  auto cloud = make_cluster({0.1f, 2.0f, 1.5f}, 200, 0.2f);

  // Instance labels: all label=1
  CloudLPtr labels(new CloudL);
  for (size_t i = 0; i < cloud->size(); ++i) {
    pcl::Label l;
    l.label = 1;
    labels->push_back(l);
  }

  std::map<uint32_t, uint32_t> inst_to_sem = {{1, 5}}; // instance 1 → semantic 5
  std::vector<uint32_t> window_labels = {5};

  CreateWindowsOptions opts;
  opts.mode = WindowBoundaryMode::rectangle;
  opts.wall_offset = 0.5f;

  auto result =
      create_windows(cloud, labels, inst_to_sem, {wall}, window_labels, opts);

  SECTION("Creates exactly one window component") {
    REQUIRE(result.components.size() == 1);
    REQUIRE(result.unmatched_instances.empty());
  }

  SECTION("Window has 4 vertices (rectangle)") {
    REQUIRE(result.components[0].boundary.vertices.size() == 4);
  }

  SECTION("Window is offset from wall plane") {
    // All vertices should have x ≈ 0.5 (offset along +X normal)
    for (const auto &v : result.components[0].boundary.vertices) {
      REQUIRE_THAT(v.x(), Catch::Matchers::WithinAbs(0.5, 0.05));
    }
  }

  SECTION("Window is approximately vertical") {
    // Rectangle vertices: z values should span the cluster's Z range
    double z_min = 1e9, z_max = -1e9;
    for (const auto &v : result.components[0].boundary.vertices) {
      z_min = std::min(z_min, v.z());
      z_max = std::max(z_max, v.z());
    }
    // Should span roughly the cluster spread
    REQUIRE(z_max > z_min);
  }

  SECTION("Component type is window") {
    REQUIRE(result.components[0].type == ComponentType::window);
  }

  SECTION("Component name follows pattern") {
    REQUIRE(result.components[0].name == "window_1");
  }
}

TEST_CASE("create_windows: no matching semantic labels",
          "[geometry][create_windows]") {
  WallCandidate wall;
  wall.normal = Eigen::Vector3d(1.0, 0.0, 0.0);
  wall.centroid = Eigen::Vector3d(0.0, 2.0, 1.5);
  wall.plane = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  wall.boundary_vertices = {{0, 0, 0}, {0, 4, 0}, {0, 4, 3}, {0, 0, 3}};

  auto cloud = make_cluster({0.1f, 2.0f, 1.5f}, 100);
  CloudLPtr labels(new CloudL);
  for (size_t i = 0; i < cloud->size(); ++i)
    labels->push_back(pcl::Label{1});

  std::map<uint32_t, uint32_t> inst_to_sem = {{1, 3}}; // semantic 3
  std::vector<uint32_t> window_labels = {5};            // looking for semantic 5

  auto result = create_windows(cloud, labels, inst_to_sem, {wall}, window_labels);

  REQUIRE(result.components.empty());
  REQUIRE(result.unmatched_instances.empty());
}

TEST_CASE("create_windows: multiple instances", "[geometry][create_windows]") {
  // Wall along Y-Z plane at x=0
  WallCandidate wall;
  wall.normal = Eigen::Vector3d(1.0, 0.0, 0.0);
  wall.centroid = Eigen::Vector3d(0.0, 5.0, 1.5);
  wall.plane = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  wall.boundary_vertices = {{0, 0, 0}, {0, 10, 0}, {0, 10, 3}, {0, 0, 3}};

  // Two window clusters at different positions on the wall
  auto cluster1 = make_cluster({0.1f, 2.0f, 1.5f}, 100);
  auto cluster2 = make_cluster({0.1f, 7.0f, 1.5f}, 100);

  CloudPtr cloud(new Cloud);
  CloudLPtr labels(new CloudL);

  for (const auto &p : *cluster1) {
    cloud->push_back(p);
    labels->push_back(pcl::Label{1});
  }
  for (const auto &p : *cluster2) {
    cloud->push_back(p);
    labels->push_back(pcl::Label{2});
  }

  std::map<uint32_t, uint32_t> inst_to_sem = {{1, 5}, {2, 5}};
  std::vector<uint32_t> window_labels = {5};

  auto result = create_windows(cloud, labels, inst_to_sem, {wall}, window_labels);

  REQUIRE(result.components.size() == 2);
  REQUIRE(result.components[0].name == "window_1");
  REQUIRE(result.components[1].name == "window_2");
}

TEST_CASE("create_windows: no walls", "[geometry][create_windows]") {
  auto cloud = make_cluster({1.0f, 1.0f, 1.0f}, 100);
  CloudLPtr labels(new CloudL);
  for (size_t i = 0; i < cloud->size(); ++i)
    labels->push_back(pcl::Label{1});

  std::map<uint32_t, uint32_t> inst_to_sem = {{1, 5}};
  std::vector<uint32_t> window_labels = {5};
  std::vector<WallCandidate> no_walls;

  auto result = create_windows(cloud, labels, inst_to_sem, no_walls, window_labels);

  REQUIRE(result.components.empty());
}

TEST_CASE("create_windows: null inputs", "[geometry][create_windows]") {
  std::vector<WallCandidate> walls;
  std::map<uint32_t, uint32_t> inst_to_sem;
  std::vector<uint32_t> window_labels = {5};

  auto result =
      create_windows(nullptr, nullptr, inst_to_sem, walls, window_labels);
  REQUIRE(result.components.empty());
}
