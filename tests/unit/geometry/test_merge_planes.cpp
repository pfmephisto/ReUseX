// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for agglomerative plane merging (issue #215).
//
// merge_planes() clusters planes that are both coplanar (normal + offset within
// tolerance) AND spatially adjacent (inlier boxes overlap or gap below
// tolerance). It replaces the old greedy first-match pass whose 0.8 inlier
// overlap gate kept split walls apart. These tests pin the intended behavior:
//   - split wall (same plane, disjoint halves)      -> MERGES
//   - perpendicular planes                           -> NEVER merge
//   - parallel-but-offset walls (> tolerance apart)  -> NEVER merge
//   - result ordering is deterministic

#include <reusex/geometry/utils.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

using namespace reusex;
using namespace reusex::geometry;

namespace {

/// Append a rectangular grid of points on the plane x = x0, spanning
/// y in [y0, y1] and z in [z0, z1], to `cloud`. Returns the inlier indices.
IndicesPtr add_x_wall_patch(CloudPtr &cloud, double x0, double y0, double y1,
                            double z0, double z1, double step = 0.05) {
  IndicesPtr idx(new Indices);
  for (double y = y0; y <= y1 + 1e-9; y += step) {
    for (double z = z0; z <= z1 + 1e-9; z += step) {
      PointT p;
      p.x = static_cast<float>(x0);
      p.y = static_cast<float>(y);
      p.z = static_cast<float>(z);
      idx->push_back(static_cast<int>(cloud->size()));
      cloud->push_back(p);
    }
  }
  return idx;
}

Eigen::Vector3d centroid_of(const CloudPtr &cloud, const IndicesPtr &idx) {
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for (int i : *idx)
    c += cloud->points[i].getVector3fMap().cast<double>();
  return c / static_cast<double>(idx->size());
}

} // namespace

TEST_CASE("merge_planes: split wall (disjoint halves) merges",
          "[geometry][merge_planes]") {
  auto cloud = std::make_shared<Cloud>();

  // Two disjoint halves of the SAME wall x = 0: one at y in [0,2], the other at
  // y in [2.5,4.5]. Same plane, a small gap between the inlier patches.
  auto ia = add_x_wall_patch(cloud, 0.0, 0.0, 2.0, 0.0, 2.5);
  auto ib = add_x_wall_patch(cloud, 0.0, 2.5, 4.5, 0.0, 2.5);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(1, 0, 0, 0)};
  std::vector<IndicesPtr> inliers{ia, ib};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, ia),
                                            centroid_of(cloud, ib)};

  auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);

  REQUIRE(Pm.size() == 1);
  REQUIRE(Im.size() == 1);
  REQUIRE(Cm.size() == 1);
  // Merged inlier list is the union of both halves.
  REQUIRE(Im[0]->size() == ia->size() + ib->size());
  // Normal still points along +x, plane still at x = 0.
  Eigen::Vector3d n = Pm[0].head<3>().normalized();
  REQUIRE(std::abs(n.x()) == Catch::Approx(1.0).margin(1e-6));
  REQUIRE(std::abs(Pm[0][3] / Pm[0].head<3>().norm()) ==
          Catch::Approx(0.0).margin(1e-3));
}

TEST_CASE("merge_planes: perpendicular planes never merge",
          "[geometry][merge_planes]") {
  auto cloud = std::make_shared<Cloud>();

  // Wall x = 0 and wall y = 0 (perpendicular), meeting at the corner.
  auto ix = add_x_wall_patch(cloud, 0.0, 0.0, 3.0, 0.0, 2.5);
  IndicesPtr iy(new Indices);
  for (double x = 0.0; x <= 3.0 + 1e-9; x += 0.05)
    for (double z = 0.0; z <= 2.5 + 1e-9; z += 0.05) {
      PointT p;
      p.x = static_cast<float>(x);
      p.y = 0.0F;
      p.z = static_cast<float>(z);
      iy->push_back(static_cast<int>(cloud->size()));
      cloud->push_back(p);
    }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(0, 1, 0, 0)};
  std::vector<IndicesPtr> inliers{ix, iy};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, ix),
                                            centroid_of(cloud, iy)};

  auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);

  REQUIRE(Pm.size() == 2);
}

TEST_CASE("merge_planes: parallel-but-offset walls never merge",
          "[geometry][merge_planes]") {
  auto cloud = std::make_shared<Cloud>();

  // Two parallel walls x = 0 and x = 3 (3 m apart, far beyond tolerance).
  auto i0 = add_x_wall_patch(cloud, 0.0, 0.0, 3.0, 0.0, 2.5);
  auto i3 = add_x_wall_patch(cloud, 3.0, 0.0, 3.0, 0.0, 2.5);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(1, 0, 0, -3.0)};
  std::vector<IndicesPtr> inliers{i0, i3};
  EigenVectorContainer<double, 3> centroids{centroid_of(cloud, i0),
                                            centroid_of(cloud, i3)};

  auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);

  REQUIRE(Pm.size() == 2);
}

TEST_CASE("merge_planes: deterministic result ordering",
          "[geometry][merge_planes]") {
  auto cloud = std::make_shared<Cloud>();

  // Three planes: two coplanar+adjacent halves of wall x=0, plus a distinct
  // wall x=3. The two halves must merge; the result must be stable across runs.
  auto ia = add_x_wall_patch(cloud, 0.0, 0.0, 2.0, 0.0, 2.5);
  auto ib = add_x_wall_patch(cloud, 0.0, 2.5, 4.5, 0.0, 2.5);
  auto ic = add_x_wall_patch(cloud, 3.0, 0.0, 4.5, 0.0, 2.5);
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;

  EigenVectorContainer<double, 4> planes{Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(1, 0, 0, 0),
                                         Eigen::Vector4d(1, 0, 0, -3.0)};
  std::vector<IndicesPtr> inliers{ia, ib, ic};
  EigenVectorContainer<double, 3> centroids{
      centroid_of(cloud, ia), centroid_of(cloud, ib), centroid_of(cloud, ic)};

  auto run = [&]() {
    auto [Pm, Im, Cm] = merge_planes(planes, inliers, centroids, cloud);
    std::vector<size_t> sizes;
    for (const auto &I : Im)
      sizes.push_back(I->size());
    return sizes;
  };

  auto first = run();
  REQUIRE(first.size() == 2); // halves merged, distant wall separate
  for (int rep = 0; rep < 3; ++rep)
    REQUIRE(run() == first); // identical output every run
}
