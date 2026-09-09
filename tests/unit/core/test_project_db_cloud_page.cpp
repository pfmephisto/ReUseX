// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// ProjectDB::point_cloud_page() — the partial read behind both the JSON and
// the RUXP (#283) point endpoints.
//
// Two things are being pinned here. First, that a page is byte-identical to
// the corresponding slice of the full read, so the two paths cannot disagree.
// Second, the *memory* property the API exists for: a page allocates
// `count * point_step` bytes and no more, which is only true if it never
// concatenated the whole cloud on the way.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

using reusex::ProjectDB;

namespace {

using reusex::test_support::TempPath;

struct CloudPageDB : TempPath {
  CloudPageDB() : TempPath("test_projectdb_cloud_page") {}
};

/// Read a little-endian float out of a stored record, independently of
/// whatever ProjectDB's own deserializers do.
float f32_at(const std::vector<uint8_t> &data, size_t at) {
  REQUIRE(at + 4 <= data.size());
  uint32_t bits = 0;
  for (size_t i = 0; i < 4; ++i)
    bits |= static_cast<uint32_t>(data[at + i]) << (8 * i);
  float f = 0.0f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

uint32_t u32_at(const std::vector<uint8_t> &data, size_t at) {
  REQUIRE(at + 4 <= data.size());
  uint32_t bits = 0;
  for (size_t i = 0; i < 4; ++i)
    bits |= static_cast<uint32_t>(data[at + i]) << (8 * i);
  return bits;
}

reusex::Cloud make_rgb_cloud(size_t n) {
  reusex::Cloud cloud;
  cloud.width = static_cast<uint32_t>(n);
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(n);
  for (size_t i = 0; i < n; ++i) {
    auto &pt = cloud.points[i];
    pt.x = static_cast<float>(i) * 0.125f; // exact in binary32
    pt.y = static_cast<float>(i) * 0.25f;
    pt.z = -static_cast<float>(i) * 0.5f;
    pt.r = static_cast<uint8_t>(i % 251);
    pt.g = static_cast<uint8_t>((i * 3) % 253);
    pt.b = static_cast<uint8_t>((i * 7) % 257);
    pt.a = 255;
  }
  return cloud;
}

} // namespace

// ===========================================================================
// Agreement with the full read
// ===========================================================================

TEST_CASE("PointCloudPage_VariousOffsetsAndLimits_MatchesFullReadSlice",
          "[projectdb][cloudpage]") {
  CloudPageDB tmp;
  ProjectDB db(tmp.path);

  constexpr size_t kPoints = 1000;
  db.save_point_cloud("cloud", make_rgb_cloud(kPoints), "test");

  const auto full = db.point_cloud_xyzrgb("cloud");
  REQUIRE(full->size() == kPoints);

  struct Window {
    uint64_t offset;
    uint64_t limit;
  };
  // Includes a window starting at an interior offset and one that crosses
  // one (250..549), plus the whole cloud and the final single point.
  const std::vector<Window> windows = {{0, kPoints}, {0, 1},   {0, 7},
                                       {250, 300},   {999, 1}, {512, 64}};

  for (const auto &w : windows) {
    INFO("offset=" << w.offset << " limit=" << w.limit);
    const auto page = db.point_cloud_page("cloud", w.offset, w.limit);

    REQUIRE(page.point_type == "PointXYZRGB");
    REQUIRE(page.point_step == 16);
    REQUIRE(page.offset == w.offset);
    REQUIRE(page.count == w.limit);
    REQUIRE(page.total == kPoints);
    REQUIRE(page.data.size() == w.limit * 16u);

    for (uint64_t i = 0; i < page.count; ++i) {
      const size_t rec = static_cast<size_t>(i) * 16;
      const auto &src = full->points[w.offset + i];
      CHECK(f32_at(page.data, rec + 0) == src.x);
      CHECK(f32_at(page.data, rec + 4) == src.y);
      CHECK(f32_at(page.data, rec + 8) == src.z);
      // The stored word is the packed rgba, verbatim.
      CHECK(u32_at(page.data, rec + 12) == src.rgba);
    }
  }
}

// ===========================================================================
// Clamping
// ===========================================================================

TEST_CASE("PointCloudPage_OffsetOrLimitBeyondBounds_ClampsToTotal",
          "[projectdb][cloudpage]") {
  CloudPageDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_rgb_cloud(10), "test");

  SECTION("offset past the end is an empty page, not an error") {
    const auto page = db.point_cloud_page("cloud", 10, 5);
    CHECK(page.offset == 10);
    CHECK(page.count == 0);
    CHECK(page.total == 10);
    CHECK(page.data.empty());
  }

  SECTION("offset far past the end clamps to total") {
    const auto page = db.point_cloud_page("cloud", 1'000'000, 5);
    CHECK(page.offset == 10); // clamped
    CHECK(page.count == 0);
    CHECK(page.total == 10);
    CHECK(page.data.empty());
  }

  SECTION("limit larger than the remainder yields a short page") {
    const auto page = db.point_cloud_page("cloud", 7, 100);
    CHECK(page.offset == 7);
    CHECK(page.count == 3);
    CHECK(page.total == 10);
    CHECK(page.data.size() == 3 * 16u);
  }

  SECTION("limit 0 yields an empty page with the offset intact") {
    const auto page = db.point_cloud_page("cloud", 4, 0);
    CHECK(page.offset == 4);
    CHECK(page.count == 0);
    CHECK(page.total == 10);
    CHECK(page.data.empty());
  }

  SECTION("an empty cloud reports total 0") {
    reusex::Cloud empty;
    empty.width = 0;
    empty.height = 1;
    db.save_point_cloud("empty", empty, "test");

    const auto page = db.point_cloud_page("empty", 0, 100);
    CHECK(page.point_type == "PointXYZRGB");
    CHECK(page.point_step == 16);
    CHECK(page.offset == 0);
    CHECK(page.count == 0);
    CHECK(page.total == 0);
    CHECK(page.data.empty());
  }
}

TEST_CASE("PointCloudPage_UnknownCloudName_Throws", "[projectdb][cloudpage]") {
  CloudPageDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_rgb_cloud(4), "test");

  REQUIRE_THROWS_AS(db.point_cloud_page("missing", 0, 10), std::runtime_error);
  // Also on an empty database, where the metadata query finds nothing at all.
  CloudPageDB tmp2;
  ProjectDB empty_db(tmp2.path);
  REQUIRE_THROWS_AS(empty_db.point_cloud_page("cloud", 0, 10),
                    std::runtime_error);
}

// ===========================================================================
// Every point type
// ===========================================================================

TEST_CASE("PointCloudPage_EveryStoredPointType_RoundTripsCorrectly",
          "[projectdb][cloudpage]") {
  CloudPageDB tmp;
  ProjectDB db(tmp.path);

  SECTION("PointXYZRGB") {
    const auto src = make_rgb_cloud(20);
    db.save_point_cloud("rgb", src, "test");
    const auto page = db.point_cloud_page("rgb", 5, 10);
    REQUIRE(page.point_type == db.point_cloud_type("rgb"));
    REQUIRE(page.point_type == "PointXYZRGB");
    REQUIRE(page.point_step == 16);
    REQUIRE(page.data.size() == 10 * 16u);
    for (size_t i = 0; i < 10; ++i) {
      const auto &pt = src.points[5 + i];
      CHECK(f32_at(page.data, i * 16 + 0) == pt.x);
      CHECK(f32_at(page.data, i * 16 + 4) == pt.y);
      CHECK(f32_at(page.data, i * 16 + 8) == pt.z);
      CHECK(u32_at(page.data, i * 16 + 12) == pt.rgba);
    }
  }

  SECTION("PointXYZ") {
    pcl::PointCloud<pcl::PointXYZ> src;
    src.width = 20;
    src.height = 1;
    src.points.resize(20);
    for (size_t i = 0; i < 20; ++i) {
      src.points[i].x = static_cast<float>(i) * 0.5f;
      src.points[i].y = static_cast<float>(i) * 0.25f;
      src.points[i].z = static_cast<float>(i) * 0.125f;
    }
    db.save_point_cloud("xyz", src, "test");

    const auto page = db.point_cloud_page("xyz", 3, 6);
    REQUIRE(page.point_type == db.point_cloud_type("xyz"));
    REQUIRE(page.point_type == "PointXYZ");
    REQUIRE(page.point_step == 12);
    REQUIRE(page.data.size() == 6 * 12u);
    for (size_t i = 0; i < 6; ++i) {
      CHECK(f32_at(page.data, i * 12 + 0) == src.points[3 + i].x);
      CHECK(f32_at(page.data, i * 12 + 4) == src.points[3 + i].y);
      CHECK(f32_at(page.data, i * 12 + 8) == src.points[3 + i].z);
    }
  }

  SECTION("Normal") {
    reusex::CloudN src;
    src.width = 20;
    src.height = 1;
    src.points.resize(20);
    for (size_t i = 0; i < 20; ++i) {
      src.points[i].normal_x = static_cast<float>(i) * 0.5f;
      src.points[i].normal_y = -static_cast<float>(i) * 0.25f;
      src.points[i].normal_z = 0.125f;
      src.points[i].curvature = static_cast<float>(i) + 0.5f;
    }
    db.save_point_cloud("normals", src, "test");

    const auto page = db.point_cloud_page("normals", 2, 5);
    REQUIRE(page.point_type == db.point_cloud_type("normals"));
    REQUIRE(page.point_type == "Normal");
    REQUIRE(page.point_step == 16); // curvature is part of the record
    REQUIRE(page.data.size() == 5 * 16u);
    for (size_t i = 0; i < 5; ++i) {
      CHECK(f32_at(page.data, i * 16 + 0) == src.points[2 + i].normal_x);
      CHECK(f32_at(page.data, i * 16 + 4) == src.points[2 + i].normal_y);
      CHECK(f32_at(page.data, i * 16 + 8) == src.points[2 + i].normal_z);
      CHECK(f32_at(page.data, i * 16 + 12) == src.points[2 + i].curvature);
    }
  }

  SECTION("Label") {
    reusex::CloudL src;
    src.width = 20;
    src.height = 1;
    src.points.resize(20);
    for (size_t i = 0; i < 20; ++i)
      src.points[i].label = static_cast<uint32_t>(i * 1000 + 1);
    db.save_point_cloud("planes", src, "test");

    const auto page = db.point_cloud_page("planes", 11, 9);
    REQUIRE(page.point_type == db.point_cloud_type("planes"));
    REQUIRE(page.point_type == "Label");
    REQUIRE(page.point_step == 4);
    REQUIRE(page.data.size() == 9 * 4u);
    for (size_t i = 0; i < 9; ++i)
      CHECK(u32_at(page.data, i * 4) == src.points[11 + i].label);
  }
}

// ===========================================================================
// The memory property
// ===========================================================================

TEST_CASE("PointCloudPage_LargeCloud_AllocatesOnlyPageNotWholeCloud",
          "[projectdb][cloudpage]") {
  CloudPageDB tmp;
  ProjectDB db(tmp.path);

  // 200k points * 16 B = 3.2 MB on disk. Large enough that a page is a small
  // fraction of it, small enough to stay polite about disk and wall time.
  constexpr size_t kPoints = 200'000;
  db.save_point_cloud("big", make_rgb_cloud(kPoints), "test");

  const auto page = db.point_cloud_page("big", 100'000, 1'000);
  REQUIRE(page.total == kPoints);
  REQUIRE(page.count == 1'000);

  // The load-bearing assertion: exactly the page, to the byte. A read that
  // concatenated the cloud first would have to shrink back down to this
  // exact size, and nothing in the API asks it to.
  REQUIRE(page.data.size() == 1'000u * 16u);
  REQUIRE(page.data.capacity() < static_cast<size_t>(kPoints) * 16u);

  // ...and the bytes are the ones at that offset, not the ones at the start.
  const auto source = make_rgb_cloud(kPoints);
  for (size_t i = 0; i < 1'000; i += 97) {
    const auto &pt = source.points[100'000 + i];
    CHECK(f32_at(page.data, i * 16 + 0) == pt.x);
    CHECK(u32_at(page.data, i * 16 + 12) == pt.rgba);
  }

  // A one-point probe is enough to discover the length of the cloud.
  const auto probe = db.point_cloud_page("big", 0, 1);
  CHECK(probe.total == kPoints);
  CHECK(probe.data.size() == 16u);
}
