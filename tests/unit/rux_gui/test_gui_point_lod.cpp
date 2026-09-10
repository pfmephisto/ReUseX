// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Voxel LOD for the points endpoint (#320).
//
// The property that matters is *not* "a smaller page came back" — a prefix
// does that too, and a prefix is exactly what this exists to replace. What
// matters is that the smaller page still describes the whole scene. So the
// coverage tests below are built on clouds whose storage order is deliberately
// adversarial: the first several thousand points sit in one corner, so any
// selection that is really a prefix in disguise fails them.
//
// Everything here runs against a real ProjectDB, because the selection reads
// storage records through it — there is no in-memory shortcut to test instead.

#include <catch2/catch_test_macros.hpp>

#include <gui/binary_points.hpp>
#include <gui/point_lod.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <string>
#include <vector>

using reusex::ProjectDB;
using reusex::test_support::TempPath;
using rux::gui::gather_points;
using rux::gui::kRuxpFlagLod;
using rux::gui::lod_supports;
using rux::gui::voxel_lod;

namespace {

struct TempDB : TempPath {
  TempDB() : TempPath("test_gui_point_lod") {}
};

/// A deterministic pseudo-random source, so a coverage assertion means the
/// same thing on every machine and every run (STANDARDS §6). Nothing here may
/// use <random>, whose distributions are not specified across libstdc++
/// versions.
struct Lcg {
  uint64_t state = 0x2545F4914F6CDD1DULL;

  /// Next value in [0, 1).
  double next() {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<double>(state >> 11) / static_cast<double>(1ULL << 53);
  }
};

/// A cloud filling `[0, 10)^3`, in an order that punishes a prefix.
///
/// The first `corner_points` points are packed into the `[0, 1)^3` corner;
/// everything after them is spread over the whole box. A "first N points"
/// answer therefore sees one 1/1000th of the volume, while a spatial
/// subsample sees all of it — which is the entire difference #320 is about.
reusex::Cloud make_corner_loaded_cloud(size_t total, size_t corner_points) {
  Lcg rng;
  reusex::Cloud cloud;
  cloud.width = static_cast<uint32_t>(total);
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.resize(total);

  for (size_t i = 0; i < total; ++i) {
    const double scale = i < corner_points ? 1.0 : 10.0;
    cloud.points[i].x = static_cast<float>(rng.next() * scale);
    cloud.points[i].y = static_cast<float>(rng.next() * scale);
    cloud.points[i].z = static_cast<float>(rng.next() * scale);
    // Colour is carried through the selection untouched; making it a function
    // of the index is what lets a test check *which* points came back.
    cloud.points[i].r = static_cast<uint8_t>(i & 0xFF);
    cloud.points[i].g = static_cast<uint8_t>((i >> 8) & 0xFF);
    cloud.points[i].b = static_cast<uint8_t>((i >> 16) & 0xFF);
    cloud.points[i].a = 255;
  }
  return cloud;
}

float x_of(const ProjectDB::CloudPage &page, size_t i) {
  float value = 0.0F;
  std::memcpy(&value, page.data.data() + i * page.point_step, sizeof(value));
  return value;
}

float axis_of(const ProjectDB::CloudPage &page, size_t i, int axis) {
  float value = 0.0F;
  std::memcpy(&value,
              page.data.data() + i * page.point_step +
                  4 * static_cast<size_t>(axis),
              sizeof(value));
  return value;
}

/// How many of the 8 octants of `[0, 10)^3` the page has a point in.
size_t octants_touched(const ProjectDB::CloudPage &page) {
  std::set<int> seen;
  for (size_t i = 0; i < static_cast<size_t>(page.count); ++i) {
    int octant = 0;
    for (int axis = 0; axis < 3; ++axis)
      octant |= (axis_of(page, i, axis) >= 5.0F ? 1 : 0) << axis;
    seen.insert(octant);
  }
  return seen.size();
}

} // namespace

TEST_CASE("VoxelLod_CloudOverBudget_RespectsMaxPoints", "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_corner_loaded_cloud(20000, 4000), "test");

  // A spread of budgets, including ones that fall between the dyadic levels
  // the grid can actually hit — the bound is a hard ceiling, not an estimate.
  for (uint64_t budget : {1U, 7U, 100U, 999U, 5000U, 19999U}) {
    const auto selection = voxel_lod(db, "cloud", budget);
    INFO("budget " << budget << " returned " << selection.page.count);
    CHECK(selection.subsampled);
    CHECK(selection.page.count <= budget);
    CHECK(selection.page.count >= 1);
    CHECK(selection.page.total == 20000);
    CHECK(selection.page.offset == 0);
    CHECK(selection.indices.size() == selection.page.count);
    // The payload must be exactly as long as the header claims, or the RUXP
    // encoder writes a page that parses and is wrong.
    CHECK(selection.page.data.size() ==
          selection.page.count * selection.page.point_step);
    CHECK(selection.voxel_size > 0.0);
  }
}

TEST_CASE("VoxelLod_CloudWithCrowdedPrefix_CoversTheWholeExtent",
          "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  // 4000 of the 20000 points are in the [0,1)^3 corner, so a 1000-point page
  // of the cloud in storage order never leaves that corner.
  db.save_point_cloud("cloud", make_corner_loaded_cloud(20000, 4000), "test");

  const auto prefix = db.point_cloud_page("cloud", 0, 1000);
  REQUIRE(octants_touched(prefix) == 1);

  const auto selection = voxel_lod(db, "cloud", 1000);
  CHECK(octants_touched(selection.page) == 8);

  // Not just "some point in each half" — the spread has to be even enough to
  // read as the scene. Every octant of a uniform cloud should hold roughly an
  // eighth of the selection; 3 % is far below that and far above zero.
  const size_t floor_per_octant =
      static_cast<size_t>(selection.page.count) / 32;
  std::vector<size_t> per_octant(8, 0);
  for (size_t i = 0; i < static_cast<size_t>(selection.page.count); ++i) {
    int octant = 0;
    for (int axis = 0; axis < 3; ++axis)
      octant |= (axis_of(selection.page, i, axis) >= 5.0F ? 1 : 0) << axis;
    ++per_octant[static_cast<size_t>(octant)];
  }
  for (size_t octant = 0; octant < 8; ++octant) {
    INFO("octant " << octant << " holds " << per_octant[octant]);
    CHECK(per_octant[octant] > floor_per_octant);
  }
}

TEST_CASE("VoxelLod_SameCloudAndBudget_IsDeterministic", "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_corner_loaded_cloud(20000, 4000), "test");

  // Twice through the same database, and once through a second database
  // holding the same points: neither hash iteration order nor the storage
  // chunking may reach the answer.
  const auto first = voxel_lod(db, "cloud", 1500);
  const auto second = voxel_lod(db, "cloud", 1500);
  CHECK(first.indices == second.indices);
  CHECK(first.page.data == second.page.data);
  CHECK(first.voxel_size == second.voxel_size);

  TempDB other;
  ProjectDB copy(other.path);
  copy.save_point_cloud("cloud", make_corner_loaded_cloud(20000, 4000), "test");
  CHECK(voxel_lod(copy, "cloud", 1500).indices == first.indices);
}

TEST_CASE("VoxelLod_Selection_IsInAscendingStorageOrder", "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_corner_loaded_cloud(20000, 4000), "test");

  const auto selection = voxel_lod(db, "cloud", 900);
  CHECK(std::is_sorted(selection.indices.begin(), selection.indices.end()));
  CHECK(std::adjacent_find(selection.indices.begin(),
                           selection.indices.end()) == selection.indices.end());

  // And the records really are the records of those indices, not a reshuffle:
  // the encoded colour of point i is a function of i.
  const auto whole = db.point_cloud_page("cloud", 0, 20000);
  for (size_t i = 0; i < selection.indices.size(); ++i) {
    const size_t source = static_cast<size_t>(selection.indices[i]);
    CHECK(x_of(selection.page, i) == x_of(whole, source));
    const uint8_t *got =
        selection.page.data.data() + i * selection.page.point_step;
    const uint8_t *want = whole.data.data() + source * whole.point_step;
    CHECK(std::memcmp(got, want, whole.point_step) == 0);
  }
}

TEST_CASE("VoxelLod_CloudUnderBudget_ReturnsEveryPointUnsubsampled",
          "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_corner_loaded_cloud(500, 100), "test");

  const auto selection = voxel_lod(db, "cloud", 5000);
  // Subsampling a cloud that already fits would throw points away for nothing,
  // and claiming LOD on the wire would tell a client to distrust an offset
  // that is perfectly good.
  CHECK_FALSE(selection.subsampled);
  CHECK(selection.voxel_size == 0.0);
  CHECK(selection.page.count == 500);
  CHECK(selection.page.total == 500);
  CHECK(selection.indices.size() == 500);
  CHECK(selection.indices.front() == 0);
  CHECK(selection.indices.back() == 499);
}

TEST_CASE("VoxelLod_NonFinitePoints_AreSkippedAndCounted", "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  auto cloud = make_corner_loaded_cloud(2000, 200);
  cloud.is_dense = false;
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  cloud.points[5].x = nan;
  cloud.points[6].y = std::numeric_limits<float>::infinity();
  cloud.points[7].z = nan;
  db.save_point_cloud("cloud", cloud, "test");

  const auto selection = voxel_lod(db, "cloud", 200);
  // Binning a NaN would put the whole grid at a nonsense scale, so they are
  // dropped — but never silently (STANDARDS §5).
  CHECK(selection.skipped_non_finite == 3);
  for (uint64_t index : selection.indices)
    CHECK((index != 5 && index != 6 && index != 7));
}

TEST_CASE("VoxelLod_CloudWithoutPositions_Throws", "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  reusex::CloudL labels;
  labels.width = 4;
  labels.height = 1;
  labels.points.resize(4);
  for (size_t i = 0; i < 4; ++i)
    labels.points[i].label = static_cast<uint32_t>(i);
  db.save_point_cloud("labels", labels, "test");

  CHECK_FALSE(lod_supports("Label"));
  CHECK_FALSE(lod_supports("Normal"));
  CHECK(lod_supports("PointXYZRGB"));
  CHECK(lod_supports("PointXYZ"));
  // A Label cloud has nothing to voxelise on its own; the caller must name a
  // geometry cloud to drive the selection instead.
  CHECK_THROWS_AS(voxel_lod(db, "labels", 10), std::runtime_error);
}

TEST_CASE("VoxelLod_ZeroBudget_Throws", "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_corner_loaded_cloud(100, 10), "test");
  CHECK_THROWS_AS(voxel_lod(db, "cloud", 0), std::runtime_error);
}

TEST_CASE("GatherPoints_IndexAlignedSibling_ReturnsTheSamePoints",
          "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto geometry = make_corner_loaded_cloud(20000, 4000);
  db.save_point_cloud("cloud", geometry, "test");

  reusex::CloudL labels;
  labels.width = static_cast<uint32_t>(geometry.size());
  labels.height = 1;
  labels.points.resize(geometry.size());
  for (size_t i = 0; i < geometry.size(); ++i)
    labels.points[i].label = static_cast<uint32_t>(i);
  db.save_point_cloud("labels", labels, "test");

  const auto selection = voxel_lod(db, "cloud", 1000);
  const auto sibling = gather_points(db, "labels", selection.indices);

  CHECK(sibling.point_type == "Label");
  CHECK(sibling.count == selection.page.count);
  CHECK(sibling.total == 20000);
  CHECK(sibling.offset == 0);

  // The whole point of `lod_source`: the two pages describe the *same* points,
  // so they can still be zipped positionally (docs/CONTRACTS.md).
  for (size_t i = 0; i < static_cast<size_t>(sibling.count); ++i) {
    uint32_t label = 0;
    std::memcpy(&label, sibling.data.data() + i * sibling.point_step,
                sizeof(label));
    CHECK(label == selection.indices[i]);
  }
}

TEST_CASE("GatherPoints_EmptySelection_ReturnsAnEmptyPage", "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_corner_loaded_cloud(100, 10), "test");

  const auto page = gather_points(db, "cloud", {});
  CHECK(page.count == 0);
  CHECK(page.total == 100);
  CHECK(page.data.empty());
}

TEST_CASE("GatherPoints_IndexPastTheEnd_Throws", "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_corner_loaded_cloud(100, 10), "test");
  // Not index-aligned with whatever produced the indices — silently returning
  // a short page would zip the wrong labels onto the wrong points.
  CHECK_THROWS_AS(gather_points(db, "cloud", {0, 50, 100}), std::runtime_error);
}

TEST_CASE("EncodeRuxp_LodFlag_IsWrittenIntoTheHeaderFlagsWord", "[gui][lod]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_corner_loaded_cloud(20000, 4000), "test");

  const auto selection = voxel_lod(db, "cloud", 1000);
  const auto encoded = rux::gui::encode_ruxp(selection.page, kRuxpFlagLod);

  // flags is the u32 at +8. The layout is otherwise untouched, which is why
  // this needed no version bump: the u16 at +4 is still 1.
  REQUIRE(encoded.size() >= 12);
  uint32_t flags = 0;
  for (size_t i = 0; i < 4; ++i)
    flags |= static_cast<uint32_t>(encoded[8 + i]) << (8 * i);
  CHECK(flags == kRuxpFlagLod);
  CHECK(encoded[4] == 1);
  CHECK(encoded[5] == 0);

  // And an undefined bit is a programming error, not something to emit and
  // let a client puzzle over.
  CHECK_THROWS_AS(rux::gui::encode_ruxp(selection.page, 0x8000'0000U),
                  std::runtime_error);
}
