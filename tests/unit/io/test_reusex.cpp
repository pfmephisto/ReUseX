// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <io/reusex.hpp>
#include <types.hpp>
#include <filesystem>
#include <pcl/ModelCoefficients.h>

using Catch::Matchers::WithinAbs;

TEST_CASE("getPlanes extracts plane data from point clouds", "[io][reusex]") {
  using namespace ReUseX;

  // Create minimal test clouds - getPlanes expects per-plane data, not per-point labels
  CloudLPtr planes(new CloudL);
  CloudNPtr normals(new CloudN);
  CloudLocPtr locations(new CloudLoc);

  // Add 2 planes
  planes->resize(2);
  normals->resize(2);
  locations->resize(2);

  // Plane 1: horizontal plane at z=0
  normals->points[0].normal_x = 0.0f;
  normals->points[0].normal_y = 0.0f;
  normals->points[0].normal_z = 1.0f;
  locations->points[0] = {0.0f, 0.0f, 0.0f};
  planes->points[0].label = 1;

  // Plane 2: vertical plane
  normals->points[1].normal_x = 1.0f;
  normals->points[1].normal_y = 0.0f;
  normals->points[1].normal_z = 0.0f;
  locations->points[1] = {5.0f, 0.0f, 0.0f};
  planes->points[1].label = 2;

  auto [plane_normals, plane_centroids, plane_indices] =
      ReUseX::io::getPlanes(planes, normals, locations);

  // Should have 2 planes
  REQUIRE(plane_normals.size() == 2);
  REQUIRE(plane_centroids.size() == 2);
  REQUIRE(plane_indices.size() == 2);

  // Check first plane normal (pointing up)
  REQUIRE_THAT(plane_normals[0].z(), WithinAbs(1.0, 0.001));

  // Check centroid
  REQUIRE_THAT(plane_centroids[0].x(), WithinAbs(0.0, 0.001));
  REQUIRE_THAT(plane_centroids[0].y(), WithinAbs(0.0, 0.001));
  REQUIRE_THAT(plane_centroids[0].z(), WithinAbs(0.0, 0.001));
}

TEST_CASE("save and read plane data round-trip", "[io][reusex]") {
  using namespace ReUseX;

  // Create test data in the format expected by save()
  std::vector<pcl::ModelCoefficients> model_coefficients;
  pcl::ModelCoefficients coeff1;
  coeff1.values = {0.0, 0.0, 1.0, 0.0};  // Horizontal plane
  model_coefficients.push_back(coeff1);

  pcl::ModelCoefficients coeff2;
  coeff2.values = {1.0, 0.0, 0.0, -5.0};  // Vertical plane at x=5
  model_coefficients.push_back(coeff2);

  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroids = {
      Eigen::Vector4f(1.0f, 2.0f, 3.0f, 1.0f),
      Eigen::Vector4f(5.0f, 0.0f, 0.0f, 1.0f)
  };

  std::vector<IndicesPtr> inlier_indices = {
      std::make_shared<Indices>(Indices{0, 1, 2}),
      std::make_shared<Indices>(Indices{3, 4, 5, 6})
  };

  // Save to temp file
  std::filesystem::path temp_path = std::filesystem::temp_directory_path() / "test_planes.planes";
  bool save_result = ReUseX::io::save(temp_path, model_coefficients, centroids, inlier_indices);
  REQUIRE(save_result);

  // Verify file exists
  REQUIRE(std::filesystem::exists(temp_path));

  // Read back
  std::vector<pcl::ModelCoefficients> read_model_coefficients;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> read_centroids;
  std::vector<IndicesPtr> read_indices;

  bool read_result = ReUseX::io::read(temp_path, read_model_coefficients, read_centroids, read_indices);
  REQUIRE(read_result);

  // Verify round-trip
  REQUIRE(read_model_coefficients.size() == 2);
  REQUIRE(read_centroids.size() == 2);
  REQUIRE(read_indices.size() == 2);

  // Check plane 1 coefficients
  REQUIRE_THAT(read_model_coefficients[0].values[0], WithinAbs(0.0f, 0.001f));
  REQUIRE_THAT(read_model_coefficients[0].values[1], WithinAbs(0.0f, 0.001f));
  REQUIRE_THAT(read_model_coefficients[0].values[2], WithinAbs(1.0f, 0.001f));
  REQUIRE_THAT(read_model_coefficients[0].values[3], WithinAbs(0.0f, 0.001f));

  // Check centroid 1 (first 3 elements - xyz, ignore w)
  REQUIRE_THAT(read_centroids[0](0), WithinAbs(centroids[0](0), 0.001f));
  REQUIRE_THAT(read_centroids[0](1), WithinAbs(centroids[0](1), 0.001f));
  REQUIRE_THAT(read_centroids[0](2), WithinAbs(centroids[0](2), 0.001f));

  // Check indices 1
  REQUIRE(read_indices[0]->size() == 3);

  // Cleanup
  std::filesystem::remove(temp_path);
}

TEST_CASE("getPlanes handles empty cloud", "[io][reusex]") {
  using namespace ReUseX;

  CloudLPtr planes(new CloudL);
  CloudNPtr normals(new CloudN);
  CloudLocPtr locations(new CloudLoc);

  // Empty clouds
  auto [plane_normals, plane_centroids, plane_indices] =
      ReUseX::io::getPlanes(planes, normals, locations);

  REQUIRE(plane_normals.empty());
  REQUIRE(plane_centroids.empty());
  REQUIRE(plane_indices.empty());
}

TEST_CASE("read handles non-existent file", "[io][reusex]") {
  using namespace ReUseX;

  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> centroids;
  std::vector<IndicesPtr> inlier_indices;

  // Try to read a file that doesn't exist
  // Note: Current implementation may return true but leave vectors empty
  ReUseX::io::read("/tmp/nonexistent_planes_file_12345.planes",
                   model_coefficients, centroids, inlier_indices);

  // Verify vectors are empty (no data loaded)
  REQUIRE(model_coefficients.empty());
  REQUIRE(centroids.empty());
  REQUIRE(inlier_indices.empty());
}
