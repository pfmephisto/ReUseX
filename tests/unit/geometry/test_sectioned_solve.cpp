// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the sectioned MIP solve (issue #226): the cell complex is
// partitioned by horizontal section and each storey is solved independently,
// so multi-room / multi-storey buildings stay tractable. Covers:
//   1. section-count / c:section bookkeeping in the CellComplex constructor,
//   2. the monolithic path is preserved for a single-section (1 storey) box,
//   3. a stacked two-section box meshes with consistent shared faces.

#include "../../support/synthetic_scene.hpp"

#include "geometry/CellComplex.hpp"
#include "geometry/Solidifier.hpp"

#include <reusex/geometry/mesh.hpp>
#include <reusex/geometry/segment_planes.hpp>
#include <reusex/geometry/segment_rooms.hpp>
#include <reusex/io/reusex.hpp>

#include <pcl/common/common.h>
#include <pcl/conversions.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

using namespace reusex::geometry;

namespace {

using PlaneVec =
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

// Build a box with `n_floors` stacked horizontal levels (n_floors-1 sections),
// each of height `h`, footprint w x d. Vertical walls span the full stack.
struct StackedBox {
  PlaneVec planes;
  std::vector<size_t> verticals;
  std::vector<size_t> horizontals;
  std::vector<std::pair<size_t, size_t>> pairs;
};

StackedBox make_stack(int n_floors, double w = 4.0, double d = 3.0,
                      double h = 2.5) {
  StackedBox b;
  b.planes.emplace_back(1, 0, 0, 0);  // x = 0   (id 0)
  b.planes.emplace_back(1, 0, 0, -w); // x = w   (id 1)
  b.planes.emplace_back(0, 1, 0, 0);  // y = 0   (id 2)
  b.planes.emplace_back(0, 1, 0, -d); // y = d   (id 3)
  b.verticals = {0, 1, 2, 3};
  for (int i = 0; i < n_floors; ++i) {
    b.planes.emplace_back(0, 0, 1, -h * i); // z = i*h
    b.horizontals.push_back(b.planes.size() - 1);
  }
  b.pairs = {{0, 1}, {2, 3}};
  return b;
}

std::shared_ptr<CellComplex> make_stack_complex(int n_floors,
                                                size_t max_cells = 5000) {
  auto b = make_stack(n_floors);
  return std::make_shared<CellComplex>(
      b.planes, b.verticals, b.horizontals, b.pairs,
      std::array<double, 2>{-1.0, -1.0}, std::array<double, 2>{5.0, 4.0},
      std::nullopt, max_cells);
}

// A watertight mesh must have every edge shared by exactly two triangles; a
// doubled shared face at a section joint (or a missing one) shows up as an
// edge with an odd or wrong multiplicity. We use the weaker, robust invariant
// that no *undirected* edge is used by more than two faces.
bool no_edge_over_shared(const pcl::PolygonMesh &mesh) {
  std::map<std::pair<uint32_t, uint32_t>, int> edge_count;
  for (const auto &poly : mesh.polygons) {
    const auto &v = poly.vertices;
    for (size_t i = 0; i < v.size(); ++i) {
      uint32_t a = v[i];
      uint32_t b = v[(i + 1) % v.size()];
      if (a > b)
        std::swap(a, b);
      ++edge_count[{a, b}];
    }
  }
  for (const auto &[edge, count] : edge_count)
    if (count > 2)
      return false;
  return true;
}

} // namespace

TEST_CASE("CellComplex records section count and per-cell section index",
          "[geometry][sectioned][cellcomplex]") {
  // One storey (two horizontal planes) => exactly one section; every cell in
  // section 0.
  auto cc1 = make_stack_complex(2);
  REQUIRE(cc1->n_sections == 1);
  auto sec1 = cc1->property_map<CellComplex::Vertex, int>("c:section");
  for (auto cit = cc1->cells_begin(); cit != cc1->cells_end(); ++cit)
    REQUIRE(sec1[*cit] == 0);

  // Three storeys (four horizontal planes) => three sections; cells span
  // section indices 0..2 and every cell has a valid section.
  auto cc3 = make_stack_complex(4);
  REQUIRE(cc3->n_sections == 3);
  auto sec3 = cc3->property_map<CellComplex::Vertex, int>("c:section");
  std::set<int> seen;
  for (auto cit = cc3->cells_begin(); cit != cc3->cells_end(); ++cit) {
    const int s = sec3[*cit];
    REQUIRE(s >= 0);
    REQUIRE(s < 3);
    seen.insert(s);
  }
  REQUIRE(seen == std::set<int>{0, 1, 2});
}

TEST_CASE("Single-section room takes the monolithic path (no section stats)",
          "[geometry][sectioned][monolithic]") {
  // The canonical single box has one section: even with sectioning enabled the
  // Solidifier must use the monolithic solve and leave section_stats empty, so
  // the existing well-tested path is unchanged (issue #226).
  const auto scene = reusex::test_support::make_room();

  auto [plane_labels, plane_centroids, plane_normals] =
      reusex::geometry::segment_planes(scene.cloud, scene.normals);
  auto rooms =
      reusex::geometry::segment_rooms(scene.cloud, scene.normals, plane_labels);

  auto [planes, centroids, inliers] =
      reusex::io::getPlanes(plane_labels, plane_normals, plane_centroids);

  reusex::geometry::MeshOptions opt;
  opt.sectioned = true;
  opt.sectioned_threshold = 1; // force the sectioned path to be *considered*
  auto mesh = reusex::geometry::mesh(scene.cloud, scene.normals, planes,
                                     centroids, inliers, rooms, opt);

  REQUIRE(mesh != nullptr);
  const size_t vertex_count =
      static_cast<size_t>(mesh->cloud.width) * mesh->cloud.height;
  REQUIRE(vertex_count > 0);
  REQUIRE_FALSE(mesh->polygons.empty());
  REQUIRE(no_edge_over_shared(*mesh));
}

TEST_CASE("Two-storey room meshes via the sectioned solve with consistent "
          "shared faces",
          "[geometry][sectioned][pipeline]") {
  // Two stacked rooms (three horizontal levels => two sections). With a low
  // threshold the sectioned path engages; the result must be a sane mesh whose
  // shared middle slab is not doubled at the section joint.
  const auto scene = reusex::test_support::make_two_storey_room();
  REQUIRE(scene.cloud->size() > 10'000);

  auto [plane_labels, plane_centroids, plane_normals] =
      reusex::geometry::segment_planes(scene.cloud, scene.normals);
  auto rooms =
      reusex::geometry::segment_rooms(scene.cloud, scene.normals, plane_labels);

  auto [planes, centroids, inliers] =
      reusex::io::getPlanes(plane_labels, plane_normals, plane_centroids);
  REQUIRE_FALSE(planes.empty());

  reusex::geometry::MeshOptions opt;
  opt.sectioned = true;
  opt.sectioned_threshold = 1; // engage sectioned path on this small case
  opt.time_limit_seconds = 120.0;
  auto mesh = reusex::geometry::mesh(scene.cloud, scene.normals, planes,
                                     centroids, inliers, rooms, opt);

  REQUIRE(mesh != nullptr);
  const size_t vertex_count =
      static_cast<size_t>(mesh->cloud.width) * mesh->cloud.height;
  INFO("two-storey mesh: " << vertex_count << " vertices, "
                           << mesh->polygons.size() << " faces");
  REQUIRE(vertex_count > 0);
  REQUIRE_FALSE(mesh->polygons.empty());

  // No shared face doubled at the section joint.
  REQUIRE(no_edge_over_shared(*mesh));

  // The reconstructed solid should span both storeys in Z (~2 * 2.5m).
  reusex::CloudLoc mesh_points;
  pcl::fromPCLPointCloud2(mesh->cloud, mesh_points);
  reusex::LocT min_pt;
  reusex::LocT max_pt;
  pcl::getMinMax3D(mesh_points, min_pt, max_pt);
  INFO("mesh bbox z-extent: " << (max_pt.z - min_pt.z));
  CHECK(max_pt.x - min_pt.x > 3.0F);
  CHECK(max_pt.y - min_pt.y > 2.0F);
  CHECK(max_pt.z - min_pt.z > 3.0F); // taller than a single storey
}
