// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/geometry/downsample.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>

using namespace reusex;
using namespace reusex::geometry;

// Build a cloud whose points all fall into exactly two voxels at leaf=10.0.
// Cluster centers (15.5 and 37.3) and per-axis offsets (≤ 0.4 m) are chosen
// so that every point sits comfortably inside its voxel cell — at least
// 0.5 m of clearance from any voxel boundary after bbox-relative shifting.
// This insulates the test from float-precision artifacts that flare up
// when the subtractive voxel math lands a point exactly on a grid line.
//   bucket A: points clustered around (15.5, 15.5, 15.5)
//   bucket B: points clustered around (37.3, 37.3, 37.3)
static CloudPtr make_two_cluster_cloud() {
  auto c = std::make_shared<Cloud>();
  const std::array<std::array<float, 3>, 4> a = {{{15.50f, 15.50f, 15.50f},
                                                  {15.10f, 15.30f, 15.40f},
                                                  {15.90f, 15.70f, 15.60f},
                                                  {15.40f, 15.50f, 15.20f}}};
  const std::array<std::array<float, 3>, 4> b = {{{37.30f, 37.30f, 37.30f},
                                                  {36.90f, 37.10f, 37.20f},
                                                  {37.70f, 37.50f, 37.40f},
                                                  {37.20f, 37.30f, 37.00f}}};
  for (const auto &p : a) {
    PointT pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    pt.r = 200;
    pt.g = 100;
    pt.b = 50;
    c->push_back(pt);
  }
  for (const auto &p : b) {
    PointT pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    pt.r = 50;
    pt.g = 100;
    pt.b = 200;
    c->push_back(pt);
  }
  return c;
}

TEST_CASE("voxel_assignment: rejects bad inputs", "[downsample]") {
  auto cloud = make_two_cluster_cloud();
  REQUIRE_THROWS_AS(voxel_assignment(*cloud, 0.0f), std::invalid_argument);
  REQUIRE_THROWS_AS(voxel_assignment(*cloud, -0.5f), std::invalid_argument);

  Cloud empty;
  REQUIRE_THROWS_AS(voxel_assignment(empty, 0.1f), std::invalid_argument);
}

TEST_CASE("voxel_assignment: groups points by voxel cell", "[downsample]") {
  auto cloud = make_two_cluster_cloud();
  auto a = voxel_assignment(*cloud, 10.0f);

  REQUIRE(a.bucket_count == 2);
  REQUIRE(a.point_to_bucket.size() == cloud->size());

  // First four points share a bucket; next four share a different bucket
  uint32_t bA = a.point_to_bucket[0];
  uint32_t bB = a.point_to_bucket[4];
  REQUIRE(bA != bB);
  for (size_t i = 0; i < 4; ++i)
    REQUIRE(a.point_to_bucket[i] == bA);
  for (size_t i = 4; i < 8; ++i)
    REQUIRE(a.point_to_bucket[i] == bB);
}

TEST_CASE("voxel_assignment: marks non-finite points as skipped",
          "[downsample]") {
  Cloud cloud;
  PointT p;
  p.x = 1.0f;
  p.y = 1.0f;
  p.z = 1.0f;
  p.r = p.g = p.b = 0;
  cloud.push_back(p);
  PointT nan_pt = p;
  nan_pt.x = std::numeric_limits<float>::quiet_NaN();
  cloud.push_back(nan_pt);
  PointT q = p;
  q.x = 51.0f;
  q.y = 51.0f;
  q.z = 51.0f;
  cloud.push_back(q);

  auto a = voxel_assignment(cloud, 10.0f);
  REQUIRE(a.bucket_count == 2);
  REQUIRE(a.point_to_bucket[1] == VoxelAssignment::kSkippedPoint);
  REQUIRE(a.point_to_bucket[0] != VoxelAssignment::kSkippedPoint);
  REQUIRE(a.point_to_bucket[2] != VoxelAssignment::kSkippedPoint);
}

TEST_CASE("downsample(Cloud): centroid per voxel", "[downsample]") {
  auto cloud = make_two_cluster_cloud();
  auto a = voxel_assignment(*cloud, 10.0f);
  auto out = downsample(*cloud, a);

  REQUIRE(out);
  REQUIRE(out->size() == 2);

  // The four points in each cluster average close to the cluster center.
  // Output order is by bucket id; figure out which is which from the X.
  size_t firstIs = (*out)[0].x < 25.0f ? 0 : 1;
  size_t secondIs = 1 - firstIs;

  using Catch::Matchers::WithinAbs;
  REQUIRE_THAT((*out)[firstIs].x, WithinAbs(15.5f, 0.3f));
  REQUIRE_THAT((*out)[firstIs].y, WithinAbs(15.5f, 0.3f));
  REQUIRE_THAT((*out)[firstIs].z, WithinAbs(15.5f, 0.3f));
  REQUIRE((*out)[firstIs].r == 200);
  REQUIRE((*out)[firstIs].b == 50);

  REQUIRE_THAT((*out)[secondIs].x, WithinAbs(37.3f, 0.3f));
  REQUIRE_THAT((*out)[secondIs].z, WithinAbs(37.3f, 0.3f));
  REQUIRE((*out)[secondIs].r == 50);
  REQUIRE((*out)[secondIs].b == 200);

  REQUIRE(out->is_dense);
  REQUIRE(out->height == 1);
  REQUIRE(out->width == 2);
}

TEST_CASE("downsample(CloudN): aligned with primary cloud", "[downsample]") {
  auto cloud = make_two_cluster_cloud();
  auto a = voxel_assignment(*cloud, 10.0f);

  // Build a parallel normals cloud: bucket A normals point +Z, bucket B +X
  CloudN normals;
  for (size_t i = 0; i < 4; ++i) {
    NormalT n;
    n.normal_x = 0;
    n.normal_y = 0;
    n.normal_z = 1;
    n.curvature = 0.1f;
    normals.push_back(n);
  }
  for (size_t i = 0; i < 4; ++i) {
    NormalT n;
    n.normal_x = 1;
    n.normal_y = 0;
    n.normal_z = 0;
    n.curvature = 0.2f;
    normals.push_back(n);
  }

  auto out = downsample(normals, a);
  REQUIRE(out);
  REQUIRE(out->size() == 2);

  auto cloud_out = downsample(*cloud, a);
  REQUIRE(cloud_out->size() == out->size());

  // Bucket A row should have normal ~ (0,0,1); bucket B row ~ (1,0,0).
  // Determine row-by-row alignment via cloud_out positions.
  size_t aIdx = (*cloud_out)[0].x < 25.0f ? 0 : 1;
  size_t bIdx = 1 - aIdx;

  using Catch::Matchers::WithinAbs;
  REQUIRE_THAT((*out)[aIdx].normal_z, WithinAbs(1.0f, 1e-5f));
  REQUIRE_THAT((*out)[bIdx].normal_x, WithinAbs(1.0f, 1e-5f));
  REQUIRE_THAT((*out)[aIdx].curvature, WithinAbs(0.1f, 1e-5f));
  REQUIRE_THAT((*out)[bIdx].curvature, WithinAbs(0.2f, 1e-5f));
}

TEST_CASE("voxel_assignment: handles georeferenced (UTM-scale) coordinates",
          "[downsample]") {
  // Simulate a tiny cloud whose XY are in UTM zone 32 (eastings ~500 km,
  // northings ~6,000 km from origin). At 5 cm leaf, the *absolute* voxel
  // indices would be ~10^7 and ~10^8 — far past anything that fits in
  // 21 bits. The assignment must shift to bbox-relative coords so this
  // still works.
  Cloud cloud;
  const double base_x = 555000.0;
  const double base_y = 6320000.0;
  const double base_z = 12.0;
  for (int dx : {0, 1, 2}) {
    for (int dy : {0, 1, 2}) {
      PointT p;
      p.x = static_cast<float>(base_x + dx * 0.5); // 50 cm spacing
      p.y = static_cast<float>(base_y + dy * 0.5);
      p.z = static_cast<float>(base_z);
      p.r = p.g = p.b = 100;
      cloud.push_back(p);
    }
  }

  // leaf=0.5 → each unique (dx,dy) lands in its own voxel
  auto a = voxel_assignment(cloud, 0.5f);
  REQUIRE(a.bucket_count == 9);

  // Origin should equal bbox min, give or take float precision
  using Catch::Matchers::WithinAbs;
  REQUIRE_THAT(a.origin_x, WithinAbs(base_x, 1.0));
  REQUIRE_THAT(a.origin_y, WithinAbs(base_y, 1.0));
}

TEST_CASE("voxel_assignment: rejects cloud larger than packed range",
          "[downsample]") {
  // Two points ~150 km apart along X at 5 cm leaf → 3e6 voxels, exceeds
  // the 2^21-1 packed limit per axis.
  Cloud cloud;
  PointT a;
  a.x = 0.0f;
  a.y = 0.0f;
  a.z = 0.0f;
  a.r = a.g = a.b = 0;
  cloud.push_back(a);
  PointT b = a;
  b.x = 150'000.0f;
  cloud.push_back(b);

  REQUIRE_THROWS_AS(voxel_assignment(cloud, 0.05f), std::out_of_range);
}

TEST_CASE("downsample(CloudN): rejects mismatched sizes", "[downsample]") {
  auto cloud = make_two_cluster_cloud();
  auto a = voxel_assignment(*cloud, 10.0f);

  CloudN normals;
  for (size_t i = 0; i < 3; ++i) {
    NormalT n;
    n.normal_x = 0;
    n.normal_y = 0;
    n.normal_z = 1;
    normals.push_back(n);
  }
  REQUIRE_THROWS_AS(downsample(normals, a), std::invalid_argument);
}
