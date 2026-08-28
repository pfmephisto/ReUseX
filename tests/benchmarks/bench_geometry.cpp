// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Performance benchmarks for the geometry hot paths. Run via scripts/bench.sh
// (stores an XML report that can be diffed against a baseline).
//
// Changes to these code paths must be benchmark-neutral or better
// (docs/STANDARDS.md §8). Inputs are deterministic (fixed seeds) so runs are
// comparable across commits.

#include "../support/synthetic_scene.hpp"

#include <reusex/geometry/downsample.hpp>
#include <reusex/geometry/segment_planes.hpp>
#include <reusex/geometry/segment_rooms.hpp>

#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Benchmark: voxel downsample", "[benchmark][geometry]") {
  const auto scene = reusex::test_support::make_room(
      /*sx=*/4.0F, /*sy=*/3.0F, /*sz=*/2.5F, /*spacing=*/0.02F);

  BENCHMARK("voxel_assignment 5cm") {
    return reusex::geometry::voxel_assignment(*scene.cloud, 0.05);
  };
}

TEST_CASE("Benchmark: plane segmentation", "[benchmark][geometry]") {
  const auto scene = reusex::test_support::make_room();

  BENCHMARK("segment_planes (synthetic room, ~25k pts)") {
    return reusex::geometry::segment_planes(scene.cloud, scene.normals);
  };
}

TEST_CASE("Benchmark: room segmentation", "[benchmark][geometry]") {
  const auto scene = reusex::test_support::make_room();
  auto [plane_labels, plane_centroids, plane_normals] =
      reusex::geometry::segment_planes(scene.cloud, scene.normals);

  BENCHMARK("segment_rooms (synthetic room, ~25k pts)") {
    return reusex::geometry::segment_rooms(scene.cloud, scene.normals,
                                           plane_labels);
  };
}
