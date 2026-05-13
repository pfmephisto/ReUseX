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

// Build a cloud whose points all fall into exactly two voxels at leaf=1.0:
//   bucket A: points clustered around (0.25, 0.25, 0.25)
//   bucket B: points clustered around (5.25, 5.25, 5.25)
static CloudPtr make_two_cluster_cloud() {
  auto c = std::make_shared<Cloud>();
  const std::array<std::array<float, 3>, 4> a = {{{0.1f, 0.1f, 0.1f},
                                                  {0.4f, 0.4f, 0.4f},
                                                  {0.2f, 0.3f, 0.1f},
                                                  {0.3f, 0.2f, 0.4f}}};
  const std::array<std::array<float, 3>, 4> b = {{{5.1f, 5.1f, 5.1f},
                                                  {5.4f, 5.4f, 5.4f},
                                                  {5.2f, 5.3f, 5.1f},
                                                  {5.3f, 5.2f, 5.4f}}};
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
  auto a = voxel_assignment(*cloud, 1.0f);

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
  p.x = 0.1f;
  p.y = 0.1f;
  p.z = 0.1f;
  p.r = p.g = p.b = 0;
  cloud.push_back(p);
  PointT nan_pt = p;
  nan_pt.x = std::numeric_limits<float>::quiet_NaN();
  cloud.push_back(nan_pt);
  PointT q = p;
  q.x = 5.1f;
  q.y = 5.1f;
  q.z = 5.1f;
  cloud.push_back(q);

  auto a = voxel_assignment(cloud, 1.0f);
  REQUIRE(a.bucket_count == 2);
  REQUIRE(a.point_to_bucket[1] == VoxelAssignment::kSkippedPoint);
  REQUIRE(a.point_to_bucket[0] != VoxelAssignment::kSkippedPoint);
  REQUIRE(a.point_to_bucket[2] != VoxelAssignment::kSkippedPoint);
}

TEST_CASE("downsample(Cloud): centroid per voxel", "[downsample]") {
  auto cloud = make_two_cluster_cloud();
  auto a = voxel_assignment(*cloud, 1.0f);
  auto out = downsample(*cloud, a);

  REQUIRE(out);
  REQUIRE(out->size() == 2);

  // Expected centroids: (0.25, 0.25, 0.25) and (5.25, 5.25, 5.25)
  // Order in the output is by bucket id; figure out which is which.
  size_t firstIs = (*out)[0].x < 1.0f ? 0 : 1;
  size_t secondIs = 1 - firstIs;

  using Catch::Matchers::WithinAbs;
  REQUIRE_THAT((*out)[firstIs].x, WithinAbs(0.25f, 1e-5f));
  REQUIRE_THAT((*out)[firstIs].y, WithinAbs(0.25f, 1e-5f));
  REQUIRE_THAT((*out)[firstIs].z, WithinAbs(0.25f, 1e-5f));
  REQUIRE((*out)[firstIs].r == 200);
  REQUIRE((*out)[firstIs].b == 50);

  REQUIRE_THAT((*out)[secondIs].x, WithinAbs(5.25f, 1e-5f));
  REQUIRE_THAT((*out)[secondIs].z, WithinAbs(5.25f, 1e-5f));
  REQUIRE((*out)[secondIs].r == 50);
  REQUIRE((*out)[secondIs].b == 200);

  REQUIRE(out->is_dense);
  REQUIRE(out->height == 1);
  REQUIRE(out->width == 2);
}

TEST_CASE("downsample(CloudN): aligned with primary cloud", "[downsample]") {
  auto cloud = make_two_cluster_cloud();
  auto a = voxel_assignment(*cloud, 1.0f);

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
  size_t aIdx = (*cloud_out)[0].x < 1.0f ? 0 : 1;
  size_t bIdx = 1 - aIdx;

  using Catch::Matchers::WithinAbs;
  REQUIRE_THAT((*out)[aIdx].normal_z, WithinAbs(1.0f, 1e-5f));
  REQUIRE_THAT((*out)[bIdx].normal_x, WithinAbs(1.0f, 1e-5f));
  REQUIRE_THAT((*out)[aIdx].curvature, WithinAbs(0.1f, 1e-5f));
  REQUIRE_THAT((*out)[bIdx].curvature, WithinAbs(0.2f, 1e-5f));
}

TEST_CASE("downsample(CloudN): rejects mismatched sizes", "[downsample]") {
  auto cloud = make_two_cluster_cloud();
  auto a = voxel_assignment(*cloud, 1.0f);

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
