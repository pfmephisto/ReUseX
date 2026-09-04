// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the external loop-edge JSON bridge (issue #221 / #225 P2).
//
// load_loop_edges is the license-clean seam that lets an out-of-process matcher
// (a commercial-safe learned matcher, or an offline MASt3R ceiling oracle) feed
// wide-baseline relative-pose constraints into the pose graph as plain DATA.
// These tests pin the two guarantees the optimizer relies on: (1) DB node ids
// are correctly mapped to the optimizer's frame indices, and (2) malformed /
// out-of-range / redundant edges are dropped loudly, never silently trusted.

#include <reusex/slam/LoopClosure.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace reusex::geometry;
using Catch::Matchers::WithinAbs;

namespace {

// RAII temp file holding the given JSON text.
struct TempJson {
  std::filesystem::path path;
  explicit TempJson(const std::string &contents) {
    path = std::filesystem::temp_directory_path() /
           ("reusex_loop_edges_test_" +
            std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json");
    std::ofstream(path) << contents;
  }
  ~TempJson() {
    std::error_code ec;
    std::filesystem::remove(path, ec);
  }
};

} // namespace

TEST_CASE("load_loop_edges maps node ids to frame indices", "[loop_edges]") {
  // Frame k has node id node_ids[k]; the file references node ids, not indices.
  const std::vector<int> node_ids{10, 20, 30, 40};

  TempJson f(R"({
    "schema": "reusex.loop_edges.v1",
    "producer": "unit-test",
    "edges": [
      {"node_i": 10, "node_j": 40,
       "T_ij": [1,0,0,0.5, 0,1,0,0, 0,0,1,0, 0,0,0,1],
       "sigma_rot": 0.03, "sigma_trans": 0.07, "inliers": 123}
    ]
  })");

  LoopClosureResult stats;
  auto edges = load_loop_edges(f.path.string(), node_ids, &stats);

  REQUIRE(edges.size() == 1);
  // node 10 -> index 0, node 40 -> index 3
  CHECK(edges[0].i == 0);
  CHECK(edges[0].j == 3);
  CHECK_THAT(edges[0].T_ij(0, 3), WithinAbs(0.5, 1e-12)); // row-major tx read
  CHECK_THAT(edges[0].sigma_rot, WithinAbs(0.03, 1e-12));
  CHECK_THAT(edges[0].sigma_trans, WithinAbs(0.07, 1e-12));
  CHECK(edges[0].inliers == 123);
  CHECK(stats.edges == 1);
  CHECK(stats.total_inliers == 123);
}

TEST_CASE("load_loop_edges drops unknown/self/duplicate/malformed edges",
          "[loop_edges]") {
  const std::vector<int> node_ids{10, 20, 30};

  TempJson f(R"({
    "schema": "reusex.loop_edges.v1",
    "edges": [
      {"node_i": 10, "node_j": 30,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]},
      {"node_i": 10, "node_j": 99,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]},
      {"node_i": 20, "node_j": 20,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]},
      {"node_i": 30, "node_j": 10,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]},
      {"node_i": 10, "node_j": 20, "T_ij": [1,2,3]}
    ]
  })");

  auto edges = load_loop_edges(f.path.string(), node_ids, nullptr);

  // Only the first (10<->30) survives: 10<->99 is unknown node, 20<->20 is a
  // self-loop, 30<->10 duplicates the first pair (order-independent), and the
  // last has a 3-element T_ij (malformed).
  REQUIRE(edges.size() == 1);
  CHECK(edges[0].i == 0);
  CHECK(edges[0].j == 2);
  // Missing sigma/inliers fall back to the LoopEdge struct defaults.
  CHECK_THAT(edges[0].sigma_rot, WithinAbs(0.05, 1e-12));
  CHECK(edges[0].inliers == 0);
}

TEST_CASE("load_loop_edges fails loudly on a missing or invalid file",
          "[loop_edges]") {
  const std::vector<int> node_ids{1, 2};

  SECTION("missing file throws") {
    CHECK_THROWS(load_loop_edges("/no/such/reusex_edges_file.json", node_ids));
  }

  SECTION("valid JSON without an edges array throws") {
    TempJson f(R"({"schema": "reusex.loop_edges.v1", "producer": "x"})");
    CHECK_THROWS(load_loop_edges(f.path.string(), node_ids));
  }

  SECTION("non-JSON garbage throws") {
    TempJson f("this is not json {{{");
    CHECK_THROWS(load_loop_edges(f.path.string(), node_ids));
  }
}
