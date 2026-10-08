// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The clouds stage builds the spatial tile index (#395) itself, so every
// front end that runs it — `rux create clouds`, the GUI's POST /jobs — leaves
// the same project behind. Before the index moved into reusex_pipeline only
// the CLI wrapper built it, and a GUI-submitted clouds job left none.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/pipeline/tile_index.hpp>
#include <reusex/types/point_types.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <cstdint>

using namespace reusex;
using namespace reusex::test_support;

namespace {

/// A dense 64x48 frame at 1.5 m: survives the clouds stage's voxel grid and
/// outlier filters with several hundred points (see test_pose_guards.cpp).
void seed_frame(ProjectDB &db, int node_id) {
  constexpr int kW = 64;
  constexpr int kH = 48;
  db.save_sensor_frame(node_id, make_color(kW, kH), make_depth(kW, kH, 1.5),
                       cv::Mat(), translation_pose(0.1 * node_id, 0, 0),
                       make_intrinsics(64.0, 64.0, kW / 2.0, kH / 2.0, kW, kH));
}

Cloud grid_cloud(int n) {
  Cloud cloud;
  for (int i = 0; i < n; ++i) {
    PointT p;
    p.x = static_cast<float>(i % 10);
    p.y = static_cast<float>((i / 10) % 10);
    p.z = static_cast<float>(i / 100);
    cloud.push_back(p);
  }
  return cloud;
}

} // namespace

TEST_CASE("RunStage_Clouds_BuildsTileIndexForTheMortonCloud",
          "[pipeline][stages][tiles]") {
  TempPath project("test_pipeline_tile_index");
  ProjectDB db(project.path);
  seed_frame(db, 1);
  seed_frame(db, 2);

  pipeline::StageContext ctx;
  ctx.project = project.path;
  ctx.stage = pipeline::JobStage::clouds;
  ctx.parameters =
      R"({"resolution":0.05,"min_distance":0.0,)"
      R"("max_distance":4.0,"sampling_factor":1,"confidence_threshold":2})";
  const auto result = pipeline::run_stage(db, ctx);
  INFO(result.message);
  REQUIRE(result.ok);
  REQUIRE(db.point_cloud_storage_order("cloud") ==
          pipeline::kMortonStorageOrder);

  const auto blob = db.tile_index("cloud");
  REQUIRE_FALSE(blob.empty());
  const auto [hdr, tiles] = pipeline::parse_tile_index(blob);
  CHECK(hdr.magic == pipeline::kTileIndexMagic);
  CHECK(hdr.point_count == db.point_cloud_xyzrgb("cloud")->size());
  uint64_t sum = 0;
  for (const auto &t : tiles)
    sum += t.count;
  CHECK(sum == hdr.point_count);
}

TEST_CASE("BuildTileIndex_UnorderedOrMissingCloud_StoresNothing",
          "[pipeline][tiles]") {
  TempPath project("test_pipeline_tile_index");
  ProjectDB db(project.path);

  CHECK(pipeline::build_tile_index(db, "cloud") == 0);

  db.save_point_cloud("cloud", grid_cloud(1000)); // no storage order
  CHECK(pipeline::build_tile_index(db, "cloud") == 0);
  CHECK(db.tile_index("cloud").empty());
}

TEST_CASE("BuildTileIndex_MortonCloud_StoresParseableIndex",
          "[pipeline][tiles]") {
  TempPath project("test_pipeline_tile_index");
  ProjectDB db(project.path);
  db.save_point_cloud("cloud", grid_cloud(1000), "test",
                      R"({"storage_order":"morton_10bit_bitrev"})");

  const auto bytes = pipeline::build_tile_index(db, "cloud");
  REQUIRE(bytes > 0);
  const auto blob = db.tile_index("cloud");
  CHECK(blob.size() == bytes);
  const auto [hdr, tiles] = pipeline::parse_tile_index(blob);
  CHECK(hdr.point_count == 1000u);
  CHECK(tiles.size() == 64u);
}
