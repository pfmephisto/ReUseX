// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/label_semantics.hpp>
#include <reusex/geometry/mesh.hpp>
#include <reusex/geometry/reconstruct.hpp>
#include <reusex/geometry/segment_instances.hpp>
#include <reusex/geometry/segment_planes.hpp>
#include <reusex/geometry/segment_rooms.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace reusex;
using namespace reusex::geometry;

namespace {

// Build a labels cloud sized to `cloud`, all unlabeled (0).
CloudLPtr make_unlabeled(CloudConstPtr cloud) {
  CloudLPtr labels(new CloudL);
  labels->resize(cloud->size());
  for (auto &p : labels->points)
    p.label = core::kUnlabeled;
  return labels;
}

PointT pt(float x, float y, float z) {
  PointT p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.r = p.g = p.b = 255;
  return p;
}

} // namespace

TEST_CASE("propagate_room_labels: within-radius point gets majority label",
          "[geometry][segment_rooms][propagation]") {
  // Seeds: a tight cluster of room-1 points around the origin (plus one
  // stray room-2 seed) and a missing point sitting inside the cluster.
  CloudPtr cloud(new Cloud);
  // 0..3 seeds (room 1), 4 seed (room 2, the "misclassified" outlier).
  cloud->push_back(pt(0.00F, 0.0F, 0.0F));
  cloud->push_back(pt(0.05F, 0.0F, 0.0F));
  cloud->push_back(pt(0.00F, 0.05F, 0.0F));
  cloud->push_back(pt(0.05F, 0.05F, 0.0F));
  cloud->push_back(pt(0.02F, 0.02F, 0.0F)); // stray room-2 seed inside cluster
  // 5: the missing point, close to all seeds.
  cloud->push_back(pt(0.025F, 0.025F, 0.0F));

  IndicesPtr sampled(new Indices{0, 1, 2, 3, 4});
  IndicesPtr missing(new Indices{5});

  auto labels = make_unlabeled(cloud);
  labels->points[0].label = 1;
  labels->points[1].label = 1;
  labels->points[2].label = 1;
  labels->points[3].label = 1;
  labels->points[4].label = 2; // outlier

  const size_t unprop =
      propagate_room_labels(cloud, labels, sampled, missing, 5, 0.5F);

  // Majority (four room-1 vs one room-2) wins: a single misclassified seed
  // does NOT smear into the missing point.
  CHECK(unprop == 0);
  CHECK(labels->points[5].label == 1U);
}

TEST_CASE("propagate_room_labels: point beyond radius stays unlabeled",
          "[geometry][segment_rooms][propagation]") {
  CloudPtr cloud(new Cloud);
  cloud->push_back(pt(0.0F, 0.0F, 0.0F)); // seed, room 1
  cloud->push_back(pt(0.1F, 0.0F, 0.0F)); // seed, room 1
  // Missing point 10 m away -> outside any sensible radius.
  cloud->push_back(pt(10.0F, 0.0F, 0.0F));

  IndicesPtr sampled(new Indices{0, 1});
  IndicesPtr missing(new Indices{2});

  auto labels = make_unlabeled(cloud);
  labels->points[0].label = 1;
  labels->points[1].label = 1;

  const size_t unprop =
      propagate_room_labels(cloud, labels, sampled, missing, 5, 0.5F);

  // No labelled neighbour within 0.5 m: stays kUnlabeled and is counted.
  CHECK(unprop == 1);
  CHECK(labels->points[2].label == core::kUnlabeled);
}

TEST_CASE("propagate_room_labels: deterministic tie-break to smallest label",
          "[geometry][segment_rooms][propagation]") {
  // Two seeds equidistant from the missing point, different labels (3 and 7).
  // The tie must break towards the smaller label (3), deterministically.
  CloudPtr cloud(new Cloud);
  cloud->push_back(pt(-0.1F, 0.0F, 0.0F)); // seed, room 7
  cloud->push_back(pt(0.1F, 0.0F, 0.0F));  // seed, room 3
  cloud->push_back(pt(0.0F, 0.0F, 0.0F));  // missing, exactly between

  IndicesPtr sampled(new Indices{0, 1});
  IndicesPtr missing(new Indices{2});

  auto run_once = [&]() {
    auto labels = make_unlabeled(cloud);
    labels->points[0].label = 7;
    labels->points[1].label = 3;
    const size_t unprop =
        propagate_room_labels(cloud, labels, sampled, missing, 5, 0.5F);
    CHECK(unprop == 0);
    return labels->points[2].label;
  };

  const uint32_t first = run_once();
  CHECK(first == 3U); // smallest label wins the tie
  // Determinism: repeated runs give identical output.
  CHECK(run_once() == first);
}

TEST_CASE("propagate_room_labels: never propagates the unlabeled sentinel",
          "[geometry][segment_rooms][propagation]") {
  // Nearest seed is itself unlabeled; the majority vote must ignore it and
  // fall back to the labelled seed farther away (still in radius).
  CloudPtr cloud(new Cloud);
  cloud->push_back(pt(0.01F, 0.0F, 0.0F)); // closest seed, UNLABELED
  cloud->push_back(pt(0.20F, 0.0F, 0.0F)); // labelled seed, room 4
  cloud->push_back(pt(0.0F, 0.0F, 0.0F));  // missing

  IndicesPtr sampled(new Indices{0, 1});
  IndicesPtr missing(new Indices{2});

  auto labels = make_unlabeled(cloud);
  labels->points[0].label = core::kUnlabeled; // explicit
  labels->points[1].label = 4;

  const size_t unprop =
      propagate_room_labels(cloud, labels, sampled, missing, 5, 0.5F);

  CHECK(unprop == 0);
  CHECK(labels->points[2].label == 4U);
}

// ─────────────────────────────────────────────────────────────────────────
// Single-source-of-defaults contract (docs/STANDARDS.md §4).
//
// The CLI option structs in apps/rux/include/create/*.hpp initialise their
// fields FROM these library structs (e.g.
// `SubcommandSegRoomsOptions::resolution =
//  reusex::geometry::SegmentRoomsOptions{}.resolution`), so the CLI can never
// disagree with the library by construction. This test pins the *library*
// side — the single source — so any accidental drift of a library default is
// caught, and documents the values that issue #217 reconciled.
// ─────────────────────────────────────────────────────────────────────────
TEST_CASE("Library option defaults are the single source of truth",
          "[geometry][defaults][STANDARDS]") {
  SECTION("SegmentRoomsOptions") {
    SegmentRoomsOptions o;
    CHECK(o.resolution == 1.0F);
    CHECK(o.beta == 0.01F);
    // Leiden bound is now finite (issue #217): must NOT be negative.
    CHECK(o.max_iter == 100);
    CHECK(o.max_iter > 0);
    CHECK(o.grid_size == 0.5F);
    CHECK(o.propagate_k == 5);
    CHECK(o.propagate_max_radius == 0.5F);
  }

  SECTION("SegmentPlanesOptions") {
    SegmentPlanesOptions o;
    CHECK(o.angle_threshold == 25.0F);
    CHECK(o.plane_dist_threshold == 0.07F);
    CHECK(o.min_inliers == 1000);
    CHECK(o.radius == 0.5F);
    CHECK(o.interval_0 == 16.0F);
    CHECK(o.interval_factor == 1.5F);
  }

  SECTION("MeshOptions (search/offset reconciled to tuned CLI values)") {
    MeshOptions o;
    CHECK(o.search_threshold == 0.60F);
    CHECK(o.new_plane_offset == 0.25F);
  }

  SECTION("ReconstructionParams") {
    ReconstructionParams o;
    CHECK(o.resolution == 0.05F);
    CHECK(o.min_distance == 0.0F);
    CHECK(o.max_distance == 4.0F);
    CHECK(o.sampling_factor == 4);
    CHECK(o.confidence_threshold == 2);
  }

  SECTION("SegmentInstancesRequest") {
    SegmentInstancesRequest o;
    CHECK(o.cluster_tolerance == 0.5F);
    CHECK(o.min_cluster_size == 50);
    CHECK(o.max_cluster_size == 1000000);
  }
}
