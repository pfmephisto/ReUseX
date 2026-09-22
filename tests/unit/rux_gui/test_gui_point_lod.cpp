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
#include <numeric>
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

// ── Morton LOD tests (#394) ────────────────────────────────────────────────
//
// Helpers shared by the Morton tests below.
namespace {

/// Spread one 10-bit value into a 30-bit word (bit k → position 3k).
/// Mirrors the implementation in reconstruct.cpp exactly.
uint32_t expand3(uint32_t v) {
  v &= 0x000003ffu;
  v = (v | (v << 16u)) & 0x030000ffu;
  v = (v | (v << 8u)) & 0x0300f00fu;
  v = (v | (v << 4u)) & 0x030c30c3u;
  v = (v | (v << 2u)) & 0x09249249u;
  return v;
}

/// 30-bit Morton code for quantised (xi, yi, zi) each in [0, 1023].
uint32_t morton30(uint32_t xi, uint32_t yi, uint32_t zi) {
  return expand3(xi) | (expand3(yi) << 1u) | (expand3(zi) << 2u);
}

/// Reverse the 30 significant bits of a Morton code.
/// Mirrors reverse_bits30 in reconstruct.cpp: standard 32-bit reversal then
/// >>2 shifts the reversed 30 bits into positions 0..29.
uint32_t reverse_bits30(uint32_t v) {
  v = ((v >> 1u) & 0x55555555u) | ((v & 0x55555555u) << 1u);
  v = ((v >> 2u) & 0x33333333u) | ((v & 0x33333333u) << 2u);
  v = ((v >> 4u) & 0x0f0f0f0fu) | ((v & 0x0f0f0f0fu) << 4u);
  v = ((v >> 8u) & 0x00ff00ffu) | ((v & 0x00ff00ffu) << 8u);
  v = (v >> 16u) | (v << 16u);
  return v >> 2u;
}

/// Build a cloud of `n` points spread across all 8 octants of [0, 10)^3,
/// then sort them by their Morton code — exactly as reconstruct.cpp does.
/// Returns both the sorted cloud and the permutation that produced it so
/// the caller can build an index-aligned label array.
struct MortonCloud {
  reusex::Cloud cloud;
  std::vector<size_t> perm;
};

MortonCloud make_morton_cloud(size_t n) {
  Lcg rng;
  reusex::Cloud unsorted;
  unsorted.width = static_cast<uint32_t>(n);
  unsorted.height = 1;
  unsorted.is_dense = true;
  unsorted.points.resize(n);
  for (size_t i = 0; i < n; ++i) {
    unsorted.points[i].x = static_cast<float>(rng.next() * 10.0);
    unsorted.points[i].y = static_cast<float>(rng.next() * 10.0);
    unsorted.points[i].z = static_cast<float>(rng.next() * 10.0);
    // Colour encodes the original index so an alignment test can read it back.
    unsorted.points[i].r = static_cast<uint8_t>(i & 0xFFu);
    unsorted.points[i].g = static_cast<uint8_t>((i >> 8u) & 0xFFu);
    unsorted.points[i].b = static_cast<uint8_t>((i >> 16u) & 0xFFu);
    unsorted.points[i].a = 255;
  }

  // Compute bbox for quantisation.
  constexpr float kMax10 = 1023.0f;
  float lo[3] = {std::numeric_limits<float>::max(),
                 std::numeric_limits<float>::max(),
                 std::numeric_limits<float>::max()};
  float hi[3] = {-std::numeric_limits<float>::max(),
                 -std::numeric_limits<float>::max(),
                 -std::numeric_limits<float>::max()};
  for (const auto &p : unsorted.points) {
    lo[0] = std::min(lo[0], p.x);
    lo[1] = std::min(lo[1], p.y);
    lo[2] = std::min(lo[2], p.z);
    hi[0] = std::max(hi[0], p.x);
    hi[1] = std::max(hi[1], p.y);
    hi[2] = std::max(hi[2], p.z);
  }
  float range[3] = {std::max(hi[0] - lo[0], 1e-9f),
                    std::max(hi[1] - lo[1], 1e-9f),
                    std::max(hi[2] - lo[2], 1e-9f)};

  std::vector<uint32_t> codes(n);
  for (size_t i = 0; i < n; ++i) {
    const auto &p = unsorted.points[i];
    const auto clamp01 = [](float v) {
      return std::max(0.0f, std::min(1.0f, v));
    };
    const uint32_t xi =
        static_cast<uint32_t>(clamp01((p.x - lo[0]) / range[0]) * kMax10);
    const uint32_t yi =
        static_cast<uint32_t>(clamp01((p.y - lo[1]) / range[1]) * kMax10);
    const uint32_t zi =
        static_cast<uint32_t>(clamp01((p.z - lo[2]) / range[2]) * kMax10);
    codes[i] = reverse_bits30(morton30(xi, yi, zi));
  }

  std::vector<size_t> perm(n);
  std::iota(perm.begin(), perm.end(), size_t{0});
  std::stable_sort(perm.begin(), perm.end(),
                   [&](size_t a, size_t b) { return codes[a] < codes[b]; });

  reusex::Cloud sorted;
  sorted.width = static_cast<uint32_t>(n);
  sorted.height = 1;
  sorted.is_dense = true;
  sorted.points.resize(n);
  for (size_t i = 0; i < n; ++i)
    sorted.points[i] = unsorted.points[perm[i]];

  return {std::move(sorted), std::move(perm)};
}

/// Save a cloud with the "morton_10bit_bitrev" storage_order.
void save_morton_cloud(ProjectDB &db, std::string_view name,
                       const reusex::Cloud &cloud) {
  db.save_point_cloud(name, cloud, "test",
                      R"({"storage_order":"morton_10bit_bitrev"})");
}

} // namespace

// ── Test 1: metadata round-trips through point_cloud_storage_order() ──────
TEST_CASE("MortonLod_StorageOrderMetadata_RoundTrips", "[gui][lod][morton]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const auto mc = make_morton_cloud(500);
  save_morton_cloud(db, "cloud", mc.cloud);

  // Single-cloud accessor
  CHECK(db.point_cloud_storage_order("cloud") == "morton_10bit_bitrev");

  // project_summary() path
  const auto summary = db.project_summary();
  REQUIRE(summary.clouds.size() == 1);
  CHECK(summary.clouds[0].storage_order == "morton_10bit_bitrev");

  // Absent key = empty string (sequential/unspecified)
  db.save_point_cloud("plain", make_corner_loaded_cloud(100, 10), "test");
  CHECK(db.point_cloud_storage_order("plain").empty());
  const auto summary2 = db.project_summary();
  REQUIRE(summary2.clouds.size() == 2);
  const auto &plain_info = summary2.clouds[0].name == "plain"
                               ? summary2.clouds[0]
                               : summary2.clouds[1];
  CHECK(plain_info.storage_order.empty());
}

// ── Test 2: Morton LOD returns a prefix, not voxels ───────────────────────
//
// A Morton-sorted cloud with `storage_order=="morton_10bit"` should make
// voxel_lod() skip the voxel pass entirely and return indices [0..N-1].
TEST_CASE("MortonLod_FlaggedCloud_ReturnsPrefixNotVoxels",
          "[gui][lod][morton]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const size_t total = 5000;
  const uint64_t budget = 1000;
  const auto mc = make_morton_cloud(total);
  save_morton_cloud(db, "cloud", mc.cloud);

  const auto selection = voxel_lod(db, "cloud", budget);

  CHECK(selection.subsampled);
  CHECK(selection.page.count == budget);
  CHECK(selection.page.total == total);
  CHECK(selection.page.offset == 0);
  CHECK(selection.indices.size() == budget);

  // Prefix: indices must be exactly 0, 1, 2, …, budget-1.
  for (uint64_t i = 0; i < budget; ++i) {
    INFO("index " << i);
    CHECK(selection.indices[i] == i);
  }

  // voxel_size sentinel: 0.0 means "prefix, no voxel grid".
  CHECK(selection.voxel_size == 0.0);

  // Non-flagged cloud still runs the voxel path (regression guard).
  db.save_point_cloud("plain", make_corner_loaded_cloud(total, 1000), "test");
  const auto vox = voxel_lod(db, "plain", budget);
  CHECK(vox.subsampled);
  CHECK(vox.voxel_size > 0.0); // voxel pass ran
}

// ── Test 3: bit-reversed Morton prefix is spatially stratified ───────────
//
// Bit-reversal makes the coarsest octant bits most-significant in the sort
// key, so any prefix visits all octants before refining any single one.  A
// 5 % prefix of a uniform [0,10)^3 cloud should therefore span ≥ 80 % of
// the full cloud's bbox on every axis — proving it covers the whole room
// rather than a corner.
//
// Contrast: plain ascending Morton order puts the first 12.5 % of points
// entirely in the lowest-code octant (x<mid ∧ y<mid ∧ z<mid), so the same
// ≥80 % check fails against plain Morton — the test is sensitive to the
// defect it is guarding against.
TEST_CASE("MortonLod_Prefix_IsStratiified", "[gui][lod][morton]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const size_t total = 8000;
  const uint64_t budget = 400; // 5 % — small enough to be a real test
  const auto mc = make_morton_cloud(total);
  save_morton_cloud(db, "cloud", mc.cloud);

  // Measure the full cloud's per-axis range.
  const auto whole = db.point_cloud_page("cloud", 0, total);
  float full_lo[3] = {std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max()};
  float full_hi[3] = {-std::numeric_limits<float>::max(),
                      -std::numeric_limits<float>::max(),
                      -std::numeric_limits<float>::max()};
  for (size_t i = 0; i < total; ++i)
    for (int ax = 0; ax < 3; ++ax) {
      const float v = axis_of(whole, i, ax);
      full_lo[ax] = std::min(full_lo[ax], v);
      full_hi[ax] = std::max(full_hi[ax], v);
    }

  const auto selection = voxel_lod(db, "cloud", budget);
  REQUIRE(selection.page.count == budget);

  // Measure the prefix's per-axis range.
  float pfx_lo[3] = {std::numeric_limits<float>::max(),
                     std::numeric_limits<float>::max(),
                     std::numeric_limits<float>::max()};
  float pfx_hi[3] = {-std::numeric_limits<float>::max(),
                     -std::numeric_limits<float>::max(),
                     -std::numeric_limits<float>::max()};
  for (size_t i = 0; i < budget; ++i)
    for (int ax = 0; ax < 3; ++ax) {
      const float v = axis_of(selection.page, i, ax);
      pfx_lo[ax] = std::min(pfx_lo[ax], v);
      pfx_hi[ax] = std::max(pfx_hi[ax], v);
    }

  // Prefix range must be ≥ 80 % of the full cloud range on every axis.
  // Plain Morton would give ≈ 50 % (one octant), so this threshold is
  // clearly distinguishing between the two orderings.
  for (int ax = 0; ax < 3; ++ax) {
    const float full_range = full_hi[ax] - full_lo[ax];
    const float pfx_range = pfx_hi[ax] - pfx_lo[ax];
    const float fraction = pfx_range / full_range;
    INFO("axis " << ax << ": prefix range " << pfx_range << " / full range "
                 << full_range << " = " << fraction);
    CHECK(fraction >= 0.80f);
  }
}

// ── Test 4: permutation alignment — same perm applied to cloud and labels ─
//
// Simulate what reconstruct.cpp does: apply the Morton permutation to both
// a geometry cloud and an index-aligned label cloud, save both, then verify
// that for every stored position i the label encodes the same original index
// as the geometry point — i.e. the two arrays stayed index-aligned through
// the reorder (STANDARDS §3.2).
//
// label[stored_i] = perm[i] (original index before sort). After LOD prefix
// fetch, gather_points on the label cloud with the same indices must return
// that same value, confirming both arrays were sorted by the identical perm.
TEST_CASE("MortonLod_PermutationAlignedLabels_MatchGeometryPoints",
          "[gui][lod][morton]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const size_t n = 1000;
  // mc.cloud is sorted by Morton code; mc.perm[i] = original index at stored i.
  const auto mc = make_morton_cloud(n);

  // Build a label cloud that encodes the original index at each stored slot.
  reusex::CloudL labels;
  labels.width = static_cast<uint32_t>(n);
  labels.height = 1;
  labels.points.resize(n);
  for (size_t i = 0; i < n; ++i)
    labels.points[i].label = static_cast<uint32_t>(mc.perm[i]);

  save_morton_cloud(db, "cloud", mc.cloud);
  db.save_point_cloud("labels", labels, "test",
                      R"({"storage_order":"morton_10bit_bitrev"})");

  // Fetch a prefix of the geometry cloud via the Morton LOD path.
  const uint64_t budget = 200;
  const auto selection = voxel_lod(db, "cloud", budget);
  REQUIRE(selection.page.count == budget);
  // Morton prefix: selection.indices == [0, 1, ..., budget-1].
  REQUIRE(selection.indices[0] == 0);
  REQUIRE(selection.indices[budget - 1] == budget - 1);

  // Gather the matching labels for the same indices.
  const auto sibling = gather_points(db, "labels", selection.indices);
  REQUIRE(sibling.count == budget);

  // For each stored position i in the prefix:
  // - label[i] == mc.perm[i] (original index of this point)
  // - the geometry x coordinate must match mc.cloud.points[i].x (prefix is
  //   storage-ordered, so stored position i == mc.cloud[i])
  for (size_t i = 0; i < budget; ++i) {
    uint32_t label = 0;
    std::memcpy(&label, sibling.data.data() + i * sibling.point_step,
                sizeof(label));

    float geo_x = 0.0f;
    std::memcpy(&geo_x,
                selection.page.data.data() + i * selection.page.point_step,
                sizeof(geo_x));

    INFO("stored position " << i << ": label=" << label
                            << " perm=" << mc.perm[i] << " geo_x=" << geo_x
                            << " mc_x=" << mc.cloud.points[i].x);
    CHECK(label == static_cast<uint32_t>(mc.perm[i]));
    CHECK(geo_x == mc.cloud.points[i].x);
  }
}

// ── Tile index tests (#395) ─────────────────────────────────────────────────

TEST_CASE("TileIndex_MortonCloud_HasCorrectTileCount", "[gui][tiles]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto mc = make_morton_cloud(8000);
  save_morton_cloud(db, "cloud", mc.cloud);

  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  REQUIRE_FALSE(blob.empty());

  const auto [hdr, tiles] = rux::gui::parse_tile_index(blob);
  CHECK(hdr.magic == rux::gui::kTileIndexMagic);
  CHECK(hdr.version == 2u);
  CHECK(hdr.tile_bits == 6u); // default: K=64
  CHECK(hdr.point_count == 8000u);
  CHECK(tiles.size() == 64u);
}

TEST_CASE("TileIndex_AllTilesBboxCoverCloud", "[gui][tiles]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto mc = make_morton_cloud(8000);
  save_morton_cloud(db, "cloud", mc.cloud);

  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  const auto [hdr, tiles] = rux::gui::parse_tile_index(blob);

  // Union of all tile bboxes must cover the whole cloud.
  float cloud_min[3] = {std::numeric_limits<float>::max(),
                        std::numeric_limits<float>::max(),
                        std::numeric_limits<float>::max()};
  float cloud_max[3] = {-std::numeric_limits<float>::max(),
                        -std::numeric_limits<float>::max(),
                        -std::numeric_limits<float>::max()};
  uint32_t total_count = 0;
  for (const auto &t : tiles) {
    total_count += t.count;
    for (int ax = 0; ax < 3; ++ax) {
      cloud_min[ax] = std::min(cloud_min[ax], t.min[ax]);
      cloud_max[ax] = std::max(cloud_max[ax], t.max[ax]);
    }
  }
  CHECK(total_count == 8000u); // every point assigned to a tile

  // Verify bbox covers the actual cloud extent.
  const auto whole = db.point_cloud_page("cloud", 0, 8000);
  for (size_t i = 0; i < 8000; ++i) {
    for (int ax = 0; ax < 3; ++ax) {
      const float v = axis_of(whole, i, ax);
      CHECK(v >= cloud_min[ax] - 1e-4f);
      CHECK(v <= cloud_max[ax] + 1e-4f);
    }
  }
}

TEST_CASE("TileIndex_GatherTilePoints_CountMatchesTileInfo", "[gui][tiles]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  // Need >= K=64 points for a non-empty tile index.
  const auto mc = make_morton_cloud(8000);
  save_morton_cloud(db, "cloud", mc.cloud);

  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  const auto [hdr, tiles] = rux::gui::parse_tile_index(blob);

  // gather_tile_points must return the same count as recorded in TileInfo.
  for (uint32_t k : {0u, 1u, 32u, 63u}) {
    const auto page = rux::gui::gather_tile_points(db, "cloud", hdr, k);
    CHECK(page.count == static_cast<uint64_t>(tiles[k].count));
  }
}

TEST_CASE("TileIndex_SaveAndLoad_RoundTrips", "[gui][tiles]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto mc = make_morton_cloud(2000);
  save_morton_cloud(db, "cloud", mc.cloud);

  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  db.save_tile_index("cloud", blob);

  const auto loaded = db.tile_index("cloud");
  CHECK(loaded == blob);
  CHECK(loaded.size() > 0);
}

TEST_CASE("TileIndex_NonMortonCloud_IsEmpty", "[gui][tiles]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  // A plain (non-Morton) cloud with <K points should return empty.
  db.save_point_cloud("small", make_corner_loaded_cloud(10, 2), "test");
  const auto blob = rux::gui::compute_tile_index(db, "small");
  CHECK(blob.empty());
}

TEST_CASE("TileIndex_CloudRewrite_ClearsIndex", "[gui][tiles]") {
  // Regression for BLOCKER 3: savePointCloudMeta upsert must set tile_index =
  // NULL so a re-run of create clouds does not leave a stale index in place.
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto mc = make_morton_cloud(2000);
  save_morton_cloud(db, "cloud", mc.cloud);

  // Build and persist the tile index.
  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  db.save_tile_index("cloud", blob);
  REQUIRE_FALSE(db.tile_index("cloud").empty());

  // Re-save the cloud (simulating a second create clouds run).
  db.save_point_cloud("cloud", mc.cloud, "test2");

  // The old tile index must be gone — it was built for a different geometry
  // run and cannot be trusted against the new storage order.
  CHECK(db.tile_index("cloud").empty());
}

TEST_CASE("TileIndex_GatherTileIndices_MatchGatherTilePoints", "[gui][tiles]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto mc = make_morton_cloud(4000);
  save_morton_cloud(db, "cloud", mc.cloud);

  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  const auto [hdr, tiles] = rux::gui::parse_tile_index(blob);

  // Build a label cloud that encodes the storage index as its label value.
  reusex::CloudL labels;
  labels.width = static_cast<uint32_t>(mc.cloud.size());
  labels.height = 1;
  labels.points.resize(mc.cloud.size());
  for (size_t i = 0; i < mc.cloud.size(); ++i)
    labels.points[i].label = static_cast<uint32_t>(i);
  db.save_point_cloud("labels", labels, "test");

  // For tile 0, gather_tile_indices and gather_tile_points must agree on which
  // storage positions belong to the tile.
  const auto indices = rux::gui::gather_tile_indices(db, "cloud", hdr, 0u);
  const auto page = rux::gui::gather_tile_points(db, "cloud", hdr, 0u);
  CHECK(indices.size() == static_cast<size_t>(page.count));

  // Gather the label cloud by those indices; each label must equal its index.
  const auto sibling = rux::gui::gather_points(db, "labels", indices);
  CHECK(sibling.count == page.count);
  for (size_t i = 0; i < indices.size(); ++i) {
    uint32_t label = 0;
    std::memcpy(&label, sibling.data.data() + i * sibling.point_step,
                sizeof(label));
    CHECK(label == static_cast<uint32_t>(indices[i]));
  }
}

// ── Multi-level tile LOD tests (#396) ──────────────────────────────────────
//
// The key property to verify: within a tile, the stored order is a
// bit-reversed Morton ordering of the sub-octant. A prefix of a tile's
// points is therefore a spatially stratified sample of that tile.

TEST_CASE("TileLod_WithinTilePrefix_IsStratiified", "[gui][tiles][lod396]") {
  // Build a Morton-sorted cloud large enough to have non-trivial tiles.
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto mc = make_morton_cloud(8000);
  save_morton_cloud(db, "cloud", mc.cloud);

  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  const auto [hdr, tiles] = rux::gui::parse_tile_index(blob);

  // Pick the tile with the most points (so a 25% prefix is still meaningful).
  uint32_t best_tile = 0;
  uint32_t best_count = 0;
  for (uint32_t k = 0; k < static_cast<uint32_t>(tiles.size()); ++k) {
    if (tiles[k].count > best_count) {
      best_count = tiles[k].count;
      best_tile = k;
    }
  }
  REQUIRE(best_count >=
          20); // need enough points for the check to be meaningful

  // Full tile fetch: establishes ground-truth bbox.
  const auto full = rux::gui::gather_tile_points(db, "cloud", hdr, best_tile);
  REQUIRE(full.count == best_count);

  float full_lo[3] = {std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max()};
  float full_hi[3] = {-std::numeric_limits<float>::max(),
                      -std::numeric_limits<float>::max(),
                      -std::numeric_limits<float>::max()};
  for (size_t i = 0; i < full.count; ++i)
    for (int ax = 0; ax < 3; ++ax) {
      const float v = axis_of(full, i, ax);
      full_lo[ax] = std::min(full_lo[ax], v);
      full_hi[ax] = std::max(full_hi[ax], v);
    }

  // Prefix fetch (first 25%): must cover >= 60% of the full tile bbox per axis.
  // A plain-Morton ordering of the same cloud would put the first 25% of the
  // tile into one spatial sub-octant (≈ 50% on each axis at best), while
  // bit-reversal distributes across all sub-octants — so 60% is a realistic
  // threshold distinguishing the two.
  const uint64_t prefix_count = std::max<uint64_t>(1, best_count / 4);
  const auto prefix = rux::gui::gather_tile_points(db, "cloud", hdr, best_tile,
                                                   /*skip=*/0, prefix_count);
  REQUIRE(static_cast<uint64_t>(prefix.count) == prefix_count);

  for (int ax = 0; ax < 3; ++ax) {
    const float full_range = full_hi[ax] - full_lo[ax];
    if (full_range < 1e-4f)
      continue; // degenerate axis (e.g. a flat scan) — skip coverage check
    float pfx_lo = std::numeric_limits<float>::max();
    float pfx_hi = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < prefix.count; ++i) {
      const float v = axis_of(prefix, i, ax);
      pfx_lo = std::min(pfx_lo, v);
      pfx_hi = std::max(pfx_hi, v);
    }
    const float coverage = (pfx_hi - pfx_lo) / full_range;
    INFO("tile " << best_tile << " axis " << ax << ": prefix covers "
                 << coverage * 100.0f << "% of tile range");
    CHECK(coverage >= 0.60f);
  }
}

TEST_CASE("TileLod_Limit_ReturnsExactCount", "[gui][tiles][lod396]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto mc = make_morton_cloud(4000);
  save_morton_cloud(db, "cloud", mc.cloud);

  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  const auto [hdr, tiles] = rux::gui::parse_tile_index(blob);

  // Find a tile with at least 10 points.
  uint32_t test_tile = 0;
  for (uint32_t k = 0; k < static_cast<uint32_t>(tiles.size()); ++k) {
    if (tiles[k].count >= 10) {
      test_tile = k;
      break;
    }
  }

  // A limit smaller than the tile count must be respected exactly.
  const uint64_t limit = static_cast<uint64_t>(tiles[test_tile].count / 2);
  const auto page = rux::gui::gather_tile_points(db, "cloud", hdr, test_tile,
                                                 /*skip=*/0, limit);
  CHECK(static_cast<uint64_t>(page.count) == limit);
  CHECK(page.count < static_cast<uint64_t>(tiles[test_tile].count));

  // limit == 0 returns the full tile.
  const auto full = rux::gui::gather_tile_points(db, "cloud", hdr, test_tile,
                                                 /*skip=*/0, /*limit=*/0);
  CHECK(full.count == static_cast<uint64_t>(tiles[test_tile].count));
}

TEST_CASE("TileLod_OffsetPlusLimit_PartitionsTheTile", "[gui][tiles][lod396]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto mc = make_morton_cloud(4000);
  save_morton_cloud(db, "cloud", mc.cloud);

  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  const auto [hdr, tiles] = rux::gui::parse_tile_index(blob);

  // Find a tile with at least 6 points so we can split into halves.
  uint32_t test_tile = 0;
  for (uint32_t k = 0; k < static_cast<uint32_t>(tiles.size()); ++k) {
    if (tiles[k].count >= 6) {
      test_tile = k;
      break;
    }
  }
  const uint64_t total = tiles[test_tile].count;
  const uint64_t half = total / 2;

  const auto first_half = rux::gui::gather_tile_points(
      db, "cloud", hdr, test_tile, /*skip=*/0, half);
  const auto second_half = rux::gui::gather_tile_points(
      db, "cloud", hdr, test_tile, /*skip=*/half, total - half);
  const auto full = rux::gui::gather_tile_points(db, "cloud", hdr, test_tile,
                                                 /*skip=*/0, /*limit=*/0);

  REQUIRE(first_half.count == half);
  REQUIRE(second_half.count == total - half);
  REQUIRE(full.count == total);

  // The two halves together must exactly reconstruct the full tile.
  const size_t step = full.point_step;
  for (size_t i = 0; i < half; ++i) {
    CHECK(std::memcmp(first_half.data.data() + i * step,
                      full.data.data() + i * step, step) == 0);
  }
  for (size_t i = 0; i < total - half; ++i) {
    CHECK(std::memcmp(second_half.data.data() + i * step,
                      full.data.data() + (half + i) * step, step) == 0);
  }
}

TEST_CASE("TileLod_IndicesAndPoints_HaveSameSkipLimit",
          "[gui][tiles][lod396]") {
  // gather_tile_indices with skip/limit must return exactly the storage indices
  // of the points gather_tile_points would return — the label-alignment
  // contract from docs/CONTRACTS.md, STANDARDS §3.2.
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto mc = make_morton_cloud(4000);
  save_morton_cloud(db, "cloud", mc.cloud);

  // Build a label cloud encoding each stored position's index.
  reusex::CloudL labels;
  labels.width = static_cast<uint32_t>(mc.cloud.size());
  labels.height = 1;
  labels.points.resize(mc.cloud.size());
  for (size_t i = 0; i < mc.cloud.size(); ++i)
    labels.points[i].label = static_cast<uint32_t>(i);
  db.save_point_cloud("labels", labels, "test");

  const auto blob = rux::gui::compute_tile_index(db, "cloud");
  const auto [hdr, tiles] = rux::gui::parse_tile_index(blob);

  // Find a tile with >= 8 points.
  uint32_t test_tile = 0;
  for (uint32_t k = 0; k < static_cast<uint32_t>(tiles.size()); ++k) {
    if (tiles[k].count >= 8) {
      test_tile = k;
      break;
    }
  }
  const uint64_t total = tiles[test_tile].count;
  const uint64_t skip = total / 4;
  const uint64_t lim = total / 2;

  const auto indices =
      rux::gui::gather_tile_indices(db, "cloud", hdr, test_tile, skip, lim);
  const auto page =
      rux::gui::gather_tile_points(db, "cloud", hdr, test_tile, skip, lim);

  REQUIRE(indices.size() == static_cast<size_t>(page.count));

  // Gather labels by these indices: each label value must match the index.
  const auto sibling = rux::gui::gather_points(db, "labels", indices);
  REQUIRE(sibling.count == page.count);
  for (size_t i = 0; i < indices.size(); ++i) {
    uint32_t label = 0;
    std::memcpy(&label, sibling.data.data() + i * sibling.point_step,
                sizeof(label));
    CHECK(label == static_cast<uint32_t>(indices[i]));
  }
}
