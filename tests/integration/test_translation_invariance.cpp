// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Translation-invariance of the geometry pipeline (issue #216).
//
// Georeferenced (UTM / 500 km-scale) coordinates fed to CGAL's
// inexact-construction kernel degrade precision. CellComplex now recenters the
// geometry to its bounding-box centroid before building the arrangement and
// restores the offset on output vertices. This test runs the full synthetic
// room -> segment_planes -> mesh pipeline at the origin and at a +500 km
// offset and asserts the two meshes are equivalent: identical vertex count and
// matching bounding-box dimensions (translation-invariant).

#include "../support/synthetic_scene.hpp"

#include <reusex/geometry/mesh.hpp>
#include <reusex/geometry/segment_planes.hpp>
#include <reusex/geometry/segment_rooms.hpp>
#include <reusex/io/reusex.hpp>
#include <reusex/types.hpp>

#include <pcl/common/common.h>
#include <pcl/conversions.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace {

struct MeshStats {
  size_t vertex_count;
  Eigen::Vector3f dims; // bbox extents (max - min)
};

/// Segmented, mesh-ready geometry for one scan (planes + inliers + centroids +
/// the room/normal clouds the mesh stage consumes).
struct SegmentedScene {
  reusex::CloudPtr cloud;
  reusex::CloudNPtr normals;
  reusex::CloudLPtr rooms;
  reusex::EigenVectorContainer<double, 4> planes;
  reusex::EigenVectorContainer<double, 3> centroids;
  std::vector<reusex::IndicesPtr> inliers;
};

SegmentedScene segment(const reusex::test_support::SyntheticScene &scene) {
  auto [plane_labels, plane_centroids, plane_normals] =
      reusex::geometry::segment_planes(scene.cloud, scene.normals);
  REQUIRE(plane_labels != nullptr);
  auto rooms =
      reusex::geometry::segment_rooms(scene.cloud, scene.normals, plane_labels);
  REQUIRE(rooms != nullptr);
  auto [planes, centroids, inliers] =
      reusex::io::getPlanes(plane_labels, plane_normals, plane_centroids);
  REQUIRE_FALSE(planes.empty());
  return {scene.cloud,       scene.normals,        rooms,
          std::move(planes), std::move(centroids), std::move(inliers)};
}

MeshStats mesh_of(const SegmentedScene &s) {
  auto planes = s.planes;       // mesh() mutates these in place
  auto centroids = s.centroids; //  (regularize / merge / orthogonal)
  auto inliers = s.inliers;
  auto mesh = reusex::geometry::mesh(s.cloud, s.normals, planes, centroids,
                                     inliers, s.rooms);
  REQUIRE(mesh != nullptr);

  reusex::CloudLoc pts;
  pcl::fromPCLPointCloud2(mesh->cloud, pts);
  const size_t vc = pts.size();
  REQUIRE(vc > 0);

  reusex::LocT lo, hi;
  pcl::getMinMax3D(pts, lo, hi);
  return {vc, Eigen::Vector3f(hi.x - lo.x, hi.y - lo.y, hi.z - lo.z)};
}

/// Translate segmented geometry rigidly by `offset`: the cloud points and the
/// plane offsets / centroids move together, so it is the *same* scan expressed
/// in a georeferenced frame. This isolates the CGAL-arrangement recentering
/// (issue #216) from float32 storage-precision loss in segmentation.
SegmentedScene shift(const SegmentedScene &in, const Eigen::Vector3d &offset) {
  SegmentedScene out;
  out.cloud = std::make_shared<reusex::Cloud>(*in.cloud);
  out.normals = in.normals;
  out.rooms = in.rooms;
  const Eigen::Vector3f of = offset.cast<float>();
  for (auto &p : out.cloud->points)
    p.getVector3fMap() += of;
  // Plane n·x + d = 0 -> in shifted frame x' = x + offset, so d' = d -
  // n·offset.
  out.planes = in.planes;
  for (auto &pl : out.planes)
    pl[3] -= pl.template head<3>().dot(offset);
  out.centroids = in.centroids;
  for (auto &c : out.centroids)
    c += offset;
  out.inliers = in.inliers;
  return out;
}

} // namespace

TEST_CASE("Mesh stage is translation-invariant at 500 km offset",
          "[integration][pipeline][translation]") {
  // Segment once at the origin, then reconstruct the mesh both at the origin
  // and after rigidly translating the segmented scan by +500 km on every axis.
  // Without recentering, CGAL's inexact kernel loses precision at that scale
  // and the arrangement/solid collapses. With recentering (issue #216) the two
  // meshes must be equivalent.
  const auto scene = reusex::test_support::make_room();
  REQUIRE(scene.cloud->size() > 10'000);

  const auto seg = segment(scene);
  const auto base = mesh_of(seg);

  const Eigen::Vector3d offset(500'000.0, 500'000.0, 500'000.0);
  const auto shifted = mesh_of(shift(seg, offset));

  INFO("origin:  " << base.vertex_count << " verts, dims (" << base.dims.x()
                   << ", " << base.dims.y() << ", " << base.dims.z() << ")");
  INFO("shifted: " << shifted.vertex_count << " verts, dims ("
                   << shifted.dims.x() << ", " << shifted.dims.y() << ", "
                   << shifted.dims.z() << ")");

  // Equivalent mesh: same number of vertices ...
  CHECK(shifted.vertex_count == base.vertex_count);
  // ... and matching bounding-box dimensions within 1 mm.
  CHECK(shifted.dims.x() == Catch::Approx(base.dims.x()).margin(1e-3));
  CHECK(shifted.dims.y() == Catch::Approx(base.dims.y()).margin(1e-3));
  CHECK(shifted.dims.z() == Catch::Approx(base.dims.z()).margin(1e-3));
}
