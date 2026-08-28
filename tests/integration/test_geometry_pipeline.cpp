// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// End-to-end integration test for the geometry pipeline:
//   synthetic room cloud -> segment_planes -> segment_rooms -> mesh
//
// This is the canary for the whole reconstruction chain: if any stage starts
// failing on the simplest possible input (a clean single-room box), every
// real scan is broken too. Keep it green (docs/STANDARDS.md §7).

#include "../support/synthetic_scene.hpp"

#include <reusex/geometry/mesh.hpp>
#include <reusex/geometry/segment_planes.hpp>
#include <reusex/geometry/segment_rooms.hpp>
#include <reusex/io/reusex.hpp>
#include <reusex/types.hpp>

#include <pcl/common/common.h>
#include <pcl/conversions.h>

#include <catch2/catch_test_macros.hpp>

#include <map>

namespace {

/// Count points per label, ignoring label == 0 (unlabeled).
std::map<uint32_t, size_t> label_histogram(const reusex::CloudL &labels) {
  std::map<uint32_t, size_t> hist;
  for (const auto &pt : labels.points) {
    if (pt.label > 0) {
      ++hist[pt.label];
    }
  }
  return hist;
}

} // namespace

TEST_CASE("Geometry pipeline: synthetic room to mesh",
          "[integration][pipeline]") {
  // 4m x 3m x 2.5m room, 5cm sampling, 5mm noise, fixed seed.
  const auto scene = reusex::test_support::make_room();
  REQUIRE(scene.cloud->size() > 10'000);
  REQUIRE(scene.cloud->size() == scene.normals->size());

  // ── Stage 1: plane segmentation ────────────────────────────────────────
  auto [plane_labels, plane_centroids, plane_normals] =
      reusex::geometry::segment_planes(scene.cloud, scene.normals);

  REQUIRE(plane_labels != nullptr);
  REQUIRE(plane_labels->size() == scene.cloud->size());

  const auto plane_hist = label_histogram(*plane_labels);
  INFO("planes found: " << plane_hist.size());
  // A box has exactly 6 faces; allow slack for fragmentation but fail hard
  // if segmentation collapses (<4) or explodes (>12) on clean input.
  CHECK(plane_hist.size() >= 4);
  CHECK(plane_hist.size() <= 12);

  // ── Stage 2: room segmentation ─────────────────────────────────────────
  auto rooms =
      reusex::geometry::segment_rooms(scene.cloud, scene.normals, plane_labels);
  REQUIRE(rooms != nullptr);
  REQUIRE(rooms->size() == scene.cloud->size());

  const auto room_hist = label_histogram(*rooms);
  INFO("rooms found: " << room_hist.size());
  REQUIRE_FALSE(room_hist.empty());

  // A single box must be dominated by one room label.
  size_t dominant = 0;
  for (const auto &[label, count] : room_hist) {
    dominant = std::max(dominant, count);
  }
  CHECK(static_cast<double>(dominant) /
            static_cast<double>(scene.cloud->size()) >
        0.5);

  // ── Stage 3: solid mesh generation (cell complex + MIP) ────────────────
  auto [planes, centroids, inliers] =
      reusex::io::getPlanes(plane_labels, plane_normals, plane_centroids);
  REQUIRE_FALSE(planes.empty());
  REQUIRE(planes.size() == centroids.size());
  REQUIRE(planes.size() == inliers.size());

  auto mesh = reusex::geometry::mesh(scene.cloud, scene.normals, planes,
                                     centroids, inliers, rooms);

  REQUIRE(mesh != nullptr);
  const size_t vertex_count =
      static_cast<size_t>(mesh->cloud.width) * mesh->cloud.height;
  INFO("mesh: " << vertex_count << " vertices, " << mesh->polygons.size()
                << " faces");
  REQUIRE(vertex_count > 0);
  REQUIRE_FALSE(mesh->polygons.empty());

  // The reconstructed solid should roughly match the room's bounding box.
  reusex::CloudLoc mesh_points;
  pcl::fromPCLPointCloud2(mesh->cloud, mesh_points);
  reusex::LocT min_pt;
  reusex::LocT max_pt;
  pcl::getMinMax3D(mesh_points, min_pt, max_pt);

  INFO("mesh bbox: [" << min_pt.x << "," << min_pt.y << "," << min_pt.z
                      << "] - [" << max_pt.x << "," << max_pt.y << ","
                      << max_pt.z << "]");
  CHECK(max_pt.x - min_pt.x > 3.0F);
  CHECK(max_pt.x - min_pt.x < 5.5F);
  CHECK(max_pt.y - min_pt.y > 2.0F);
  CHECK(max_pt.y - min_pt.y < 4.5F);
  CHECK(max_pt.z - min_pt.z > 1.5F);
  CHECK(max_pt.z - min_pt.z < 4.0F);
}
