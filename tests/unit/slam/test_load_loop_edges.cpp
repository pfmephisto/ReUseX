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

#include "../../support/temp_path.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace reusex::geometry;
using Catch::Matchers::WithinAbs;

namespace {

// RAII temp file holding the given JSON text. Naming/uniqueness/cleanup is
// delegated to the shared TempPath helper (#262); this struct only adds the
// "write the JSON contents up front" behaviour on top.
struct TempJson {
  reusex::test_support::TempPath tmp{"reusex_loop_edges_test", ".json"};
  const std::filesystem::path &path = tmp.path;
  explicit TempJson(const std::string &contents) {
    std::ofstream(path) << contents;
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

// The numeric payload is UNTRUSTED input from another process, so shape checks
// are not enough: a matrix that is not SE(3), or a sigma that is not a usable
// Gaussian std, corrupts the solve instead of failing. One case per rejection
// class. Each file pairs the bad edge with a valid control edge (10<->30) that
// must still be accepted, so a rejection cannot be confused with the loader
// bailing out on the whole file.
TEST_CASE("load_loop_edges rejects non-SE(3) and bad-sigma payloads",
          "[loop_edges]") {
  const std::vector<int> node_ids{10, 20, 30};

  // Builds a file with `bad_edge` first and a known-good 10<->30 edge second.
  auto file_with = [](const std::string &bad_edge) {
    return std::string(R"({"schema": "reusex.loop_edges.v1", "edges": [)") +
           bad_edge + R"(,
      {"node_i": 10, "node_j": 30,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]}
    ]})";
  };

  auto only_control_survives = [&](const std::string &bad_edge) {
    TempJson f(file_with(bad_edge));
    LoopClosureResult stats;
    auto edges = load_loop_edges(f.path.string(), node_ids, &stats);
    REQUIRE(edges.size() == 1);
    CHECK(edges[0].i == 0); // node 10
    CHECK(edges[0].j == 2); // node 30
    CHECK(stats.candidates == 2);
    CHECK(stats.edges == 1);
  };

  SECTION("non-finite value fails the whole file loudly") {
    // A non-finite number cannot survive into a parsed edge: RFC 8259 has no
    // Infinity/NaN literal, and nlohmann rejects an overflowing exponent with
    // out_of_range.406 while parsing. So the reachable behaviour for a
    // non-finite payload is a THROW, not a skip — which still satisfies the
    // loader's contract (a bad edge file must fail loudly). The `allFinite()`
    // guard in the loader remains as defence in depth for any future path that
    // builds a LoopEdge without going through this parser.
    TempJson f(file_with(R"({"node_i": 10, "node_j": 20,
       "T_ij": [1,0,0,1e999, 0,1,0,0, 0,0,1,0, 0,0,0,1]})"));
    CHECK_THROWS(load_loop_edges(f.path.string(), node_ids, nullptr));
  }

  SECTION("non-orthonormal rotation block") {
    // A uniform 2x scale: R^T R = 4I, far outside the 1e-3 tolerance. A
    // similarity transform like this is what a pointmap model can emit.
    only_control_survives(R"({"node_i": 10, "node_j": 20,
       "T_ij": [2,0,0,0, 0,2,0,0, 0,0,2,0, 0,0,0,1]})");
  }

  SECTION("reflection (det R < 0)") {
    // Orthonormal but improper: mirrored geometry drags the solve into a wrong
    // basin rather than merely adding noise, so orthonormality alone is not a
    // sufficient check.
    only_control_survives(R"({"node_i": 10, "node_j": 20,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,-1,0, 0,0,0,1]})");
  }

  SECTION("bottom row is not [0,0,0,1]") {
    // A general projective matrix, not an SE(3) element.
    only_control_survives(R"({"node_i": 10, "node_j": 20,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0.1,0,0,2]})");
  }

  SECTION("zero sigma_trans") {
    // The dangerous case: gtsam turns a zero sigma into a Constrained
    // (hard-equality) model, welding two poses together.
    only_control_survives(R"({"node_i": 10, "node_j": 20,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],
       "sigma_trans": 0.0})");
  }

  SECTION("negative sigma_rot") {
    only_control_survives(R"({"node_i": 10, "node_j": 20,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],
       "sigma_rot": -0.05})");
  }

  SECTION("huge-but-finite sigma is accepted (only <=0 and non-finite fail)") {
    // The gate is finiteness and positivity, not magnitude: an absurdly loose
    // sigma is a near-zero-information edge, which GNC handles, not a corrupt
    // one. Pinning this keeps the validator from growing an arbitrary upper
    // bound that would silently drop a legitimately uncertain edge.
    TempJson f(R"({"schema": "reusex.loop_edges.v1", "edges": [
      {"node_i": 10, "node_j": 20,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],
       "sigma_trans": 1e30}
    ]})");
    auto edges = load_loop_edges(f.path.string(), node_ids, nullptr);
    CHECK(edges.size() == 1);
  }

  SECTION("wrong-typed field (json::type_error is counted, not thrown)") {
    // `sigma_trans: null` makes nlohmann's get<double>() throw type_error. That
    // is a malformed EDGE, not a malformed FILE: it must be counted as skipped
    // and parsing must continue to the control edge.
    only_control_survives(R"({"node_i": 10, "node_j": 20,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],
       "sigma_trans": null})");
  }

  SECTION("string node id (json::type_error is counted, not thrown)") {
    only_control_survives(R"({"node_i": "10", "node_j": 20,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]})");
  }
}

TEST_CASE("load_loop_edges dedups only on accepted edges", "[loop_edges]") {
  // A rejected edge must NOT consume its (i,j) slot: the valid second entry for
  // the same pair has to be accepted, not counted as a duplicate.
  const std::vector<int> node_ids{10, 20, 30};

  TempJson f(R"({
    "schema": "reusex.loop_edges.v1",
    "edges": [
      {"node_i": 10, "node_j": 30,
       "T_ij": [1,0,0,0, 0,1,0,0, 0,0,-1,0, 0,0,0,1]},
      {"node_i": 10, "node_j": 30,
       "T_ij": [1,0,0,0.5, 0,1,0,0, 0,0,1,0, 0,0,0,1]}
    ]
  })");

  auto edges = load_loop_edges(f.path.string(), node_ids, nullptr);
  REQUIRE(edges.size() == 1);
  CHECK_THAT(edges[0].T_ij(0, 3), WithinAbs(0.5, 1e-12)); // the VALID one
}

TEST_CASE("load_loop_edges accepts a genuine rotation", "[loop_edges]") {
  // Guard against an over-strict validator: a real 90-degree rotation about z,
  // with a translation, must pass every SE(3) check.
  const std::vector<int> node_ids{10, 20};

  TempJson f(R"({
    "schema": "reusex.loop_edges.v1",
    "edges": [
      {"node_i": 10, "node_j": 20,
       "T_ij": [0,-1,0,1.25, 1,0,0,-2.5, 0,0,1,0.75, 0,0,0,1],
       "sigma_rot": 0.02, "sigma_trans": 0.04, "inliers": 88}
    ]
  })");

  auto edges = load_loop_edges(f.path.string(), node_ids, nullptr);
  REQUIRE(edges.size() == 1);
  CHECK_THAT(edges[0].T_ij(0, 1), WithinAbs(-1.0, 1e-12));
  CHECK_THAT(edges[0].T_ij(1, 3), WithinAbs(-2.5, 1e-12));
  CHECK(edges[0].inliers == 88);
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
