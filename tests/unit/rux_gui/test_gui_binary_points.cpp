// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Byte-level tests for RUXP v1, the binary point transport served by
// `GET /api/v1/clouds/{name}/points?format=binary` (#283).
//
// The normative layout is docs/gui/binary-points.md. The point of this file is
// to pin the *bytes*, so the expectations here are written out literally
// rather than recomputed with the same arithmetic the encoder uses — a test
// that recomputes the layout would agree with any consistent bug.
//
// Everything is decoded with the small little-endian readers below, which are
// deliberately independent of rux::gui.

#include <catch2/catch_test_macros.hpp>

#include <gui/binary_points.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using reusex::ProjectDB;
using reusex::test_support::TempPath;
using rux::gui::encode_ruxp;
using rux::gui::ruxp_supports;

namespace {

// ---------------------------------------------------------------------------
// An independent little-endian reader. Nothing here shares code with the
// encoder; that is the whole point.
// ---------------------------------------------------------------------------

uint8_t rd_u8(const std::vector<uint8_t> &buf, size_t at) {
  REQUIRE(at + 1 <= buf.size());
  return buf[at];
}

uint16_t rd_u16(const std::vector<uint8_t> &buf, size_t at) {
  REQUIRE(at + 2 <= buf.size());
  return static_cast<uint16_t>(buf[at]) |
         static_cast<uint16_t>(static_cast<uint16_t>(buf[at + 1]) << 8);
}

uint32_t rd_u32(const std::vector<uint8_t> &buf, size_t at) {
  REQUIRE(at + 4 <= buf.size());
  uint32_t v = 0;
  for (size_t i = 0; i < 4; ++i)
    v |= static_cast<uint32_t>(buf[at + i]) << (8 * i);
  return v;
}

uint64_t rd_u64(const std::vector<uint8_t> &buf, size_t at) {
  REQUIRE(at + 8 <= buf.size());
  uint64_t v = 0;
  for (size_t i = 0; i < 8; ++i)
    v |= static_cast<uint64_t>(buf[at + i]) << (8 * i);
  return v;
}

float rd_f32(const std::vector<uint8_t> &buf, size_t at) {
  const uint32_t bits = rd_u32(buf, at);
  float f = 0.0f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

std::string rd_name(const std::vector<uint8_t> &buf, size_t at) {
  REQUIRE(at + 8 <= buf.size());
  std::string s;
  for (size_t i = 0; i < 8 && buf[at + i] != 0; ++i)
    s.push_back(static_cast<char>(buf[at + i]));
  return s;
}

/// A parsed field descriptor, read straight out of the buffer.
struct Descriptor {
  std::string name;
  uint8_t type = 0;
  uint8_t components = 0;
  uint16_t reserved = 0;
  uint32_t byte_offset = 0;
};

/// A parsed header plus its descriptor table.
struct Parsed {
  std::string magic;
  uint16_t version = 0;
  uint16_t header_size = 0;
  uint32_t flags = 0;
  uint32_t field_count = 0;
  uint32_t count = 0;
  uint32_t reserved = 0;
  uint64_t offset = 0;
  uint64_t total = 0;
  std::vector<Descriptor> fields;
};

Parsed parse(const std::vector<uint8_t> &buf) {
  REQUIRE(buf.size() >= 40);
  Parsed p;
  p.magic.assign(reinterpret_cast<const char *>(buf.data()), 4);
  p.version = rd_u16(buf, 4);
  p.header_size = rd_u16(buf, 6);
  p.flags = rd_u32(buf, 8);
  p.field_count = rd_u32(buf, 12);
  p.count = rd_u32(buf, 16);
  p.reserved = rd_u32(buf, 20);
  p.offset = rd_u64(buf, 24);
  p.total = rd_u64(buf, 32);
  for (uint32_t f = 0; f < p.field_count; ++f) {
    const size_t at = 40 + 16 * static_cast<size_t>(f);
    Descriptor d;
    d.name = rd_name(buf, at);
    d.type = rd_u8(buf, at + 8);
    d.components = rd_u8(buf, at + 9);
    d.reserved = rd_u16(buf, at + 10);
    d.byte_offset = rd_u32(buf, at + 12);
    p.fields.push_back(d);
  }
  return p;
}

size_t width_of(uint8_t type) {
  switch (type) {
  case 1: // f32
  case 3: // u32
    return 4;
  case 2: // u8
    return 1;
  default:
    FAIL("unknown RUXP type code " << static_cast<int>(type));
    return 0;
  }
}

/// Everything docs/gui/binary-points.md says must hold of *any* v1 page,
/// checked without reference to which cloud produced it.
void check_invariants(const std::vector<uint8_t> &buf) {
  const Parsed p = parse(buf);

  INFO("buffer size " << buf.size());
  CHECK(p.magic == "RUXP");
  CHECK(p.version == 1);
  CHECK(p.flags == 0);
  CHECK(p.reserved == 0);
  CHECK(p.field_count >= 1);

  // header_size == 40 + 16 * field_count, and therefore a multiple of 8.
  CHECK(p.header_size == 40 + 16 * p.field_count);
  CHECK(p.header_size % 8 == 0);

  // Sections are tightly packed in descriptor order, starting at the header
  // end. Walk a running total independently and compare.
  size_t running = p.header_size;
  for (const auto &d : p.fields) {
    INFO("field '" << d.name << "'");
    CHECK(d.reserved == 0);
    CHECK(d.components >= 1);
    CHECK(d.byte_offset == running);
    // f32 / u32 sections must start 4-byte aligned or the client's
    // `new Float32Array(buffer, byteOffset, …)` throws a RangeError.
    if (d.type == 1 || d.type == 3)
      CHECK(d.byte_offset % 4 == 0);
    running += static_cast<size_t>(p.count) * d.components * width_of(d.type);
    CHECK(running <= buf.size());
  }

  // Header + all sections is exactly the buffer: no padding, no slack.
  CHECK(running == buf.size());
  CHECK(p.offset + p.count <= p.total);
}

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

struct TempDB : TempPath {
  TempDB() : TempPath("test_gui_binary_points") {}
};

/// Three points whose float bit patterns are exactly writable by hand:
///   0.0f = 00 00 00 00, 0.5f = 00 00 00 3F,
///   1.0f = 00 00 80 3F, 2.0f = 00 00 00 40
/// and three RGB triples that are not grey, so a reversed swizzle cannot pass.
reusex::Cloud make_known_rgb_cloud() {
  reusex::Cloud cloud;
  cloud.width = 3;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(3);

  cloud.points[0].x = 1.0f;
  cloud.points[0].y = 2.0f;
  cloud.points[0].z = 0.5f;
  cloud.points[0].r = 1;
  cloud.points[0].g = 2;
  cloud.points[0].b = 3;
  cloud.points[0].a = 255;

  cloud.points[1].x = 0.0f;
  cloud.points[1].y = 0.5f;
  cloud.points[1].z = 1.0f;
  cloud.points[1].r = 10;
  cloud.points[1].g = 20;
  cloud.points[1].b = 30;
  cloud.points[1].a = 255;

  cloud.points[2].x = 2.0f;
  cloud.points[2].y = 1.0f;
  cloud.points[2].z = 0.0f;
  cloud.points[2].r = 255;
  cloud.points[2].g = 0;
  cloud.points[2].b = 128;
  cloud.points[2].a = 255;

  return cloud;
}

} // namespace

// ===========================================================================
// Byte-exact encoding
// ===========================================================================

TEST_CASE("EncodeRuxp_KnownPointXYZRGBPage_MatchesExactBytes", "[gui][ruxp]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("cloud", make_known_rgb_cloud(), "test");

  const auto page = db.point_cloud_page("cloud", 0, 3);
  REQUIRE(page.point_type == "PointXYZRGB");
  REQUIRE(page.count == 3);

  const std::vector<uint8_t> actual = encode_ruxp(page);

  // Written out by hand from docs/gui/binary-points.md. Do NOT replace this
  // with a computed expectation.
  const std::vector<uint8_t> expected = {
      // ---- header (72 bytes) ----
      0x52,
      0x55,
      0x58,
      0x50, // [ 0] magic "RUXP"
      0x01,
      0x00, // [ 4] version = 1
      0x48,
      0x00, // [ 6] header_size = 72
      0x00,
      0x00,
      0x00,
      0x00, // [ 8] flags = 0
      0x02,
      0x00,
      0x00,
      0x00, // [12] field_count = 2
      0x03,
      0x00,
      0x00,
      0x00, // [16] count = 3
      0x00,
      0x00,
      0x00,
      0x00, // [20] reserved = 0
      0x00,
      0x00,
      0x00,
      0x00, //
      0x00,
      0x00,
      0x00,
      0x00, // [24] offset = 0 (u64)
      0x03,
      0x00,
      0x00,
      0x00, //
      0x00,
      0x00,
      0x00,
      0x00, // [32] total = 3 (u64)

      // ---- descriptor 0 (16 bytes) ----
      0x78,
      0x79,
      0x7A,
      0x00, //
      0x00,
      0x00,
      0x00,
      0x00, // [40] name "xyz", NUL-padded
      0x01, // [48] type = 1 (f32)
      0x03, // [49] components = 3
      0x00,
      0x00, // [50] reserved = 0
      0x48,
      0x00,
      0x00,
      0x00, // [52] byte_offset = 72

      // ---- descriptor 1 (16 bytes) ----
      0x72,
      0x67,
      0x62,
      0x00, //
      0x00,
      0x00,
      0x00,
      0x00, // [56] name "rgb", NUL-padded
      0x02, // [64] type = 2 (u8)
      0x03, // [65] components = 3
      0x00,
      0x00, // [66] reserved = 0
      0x6C,
      0x00,
      0x00,
      0x00, // [68] byte_offset = 108

      // ---- xyz section, 3 points * 3 floats, offset 72 ----
      0x00,
      0x00,
      0x80,
      0x3F, // [ 72] p0.x = 1.0f
      0x00,
      0x00,
      0x00,
      0x40, // [ 76] p0.y = 2.0f
      0x00,
      0x00,
      0x00,
      0x3F, // [ 80] p0.z = 0.5f
      0x00,
      0x00,
      0x00,
      0x00, // [ 84] p1.x = 0.0f
      0x00,
      0x00,
      0x00,
      0x3F, // [ 88] p1.y = 0.5f
      0x00,
      0x00,
      0x80,
      0x3F, // [ 92] p1.z = 1.0f
      0x00,
      0x00,
      0x00,
      0x40, // [ 96] p2.x = 2.0f
      0x00,
      0x00,
      0x80,
      0x3F, // [100] p2.y = 1.0f
      0x00,
      0x00,
      0x00,
      0x00, // [104] p2.z = 0.0f

      // ---- rgb section, 3 points * 3 bytes, offset 108 ----
      0x01,
      0x02,
      0x03, // [108] p0 = (1, 2, 3)
      0x0A,
      0x14,
      0x1E, // [111] p1 = (10, 20, 30)
      0xFF,
      0x00,
      0x80, // [114] p2 = (255, 0, 128)
  };

  REQUIRE(expected.size() == 117); // 72 + 3 * 15
  REQUIRE(actual.size() == expected.size());
  CHECK(actual == expected);

  check_invariants(actual);
}

// ===========================================================================
// The RGB swizzle
// ===========================================================================

TEST_CASE("EncodeRuxp_RgbField_ReordersFromStoredBgra", "[gui][ruxp]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  reusex::Cloud cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(1);
  cloud.points[0].x = 0.0f;
  cloud.points[0].y = 0.0f;
  cloud.points[0].z = 0.0f;
  cloud.points[0].r = 255;
  cloud.points[0].g = 0;
  cloud.points[0].b = 128;
  cloud.points[0].a = 255;

  db.save_point_cloud("cloud", cloud, "test");

  // Guard the premise: PCL stores the colour as a BGRA word, so the *stored*
  // record really is b,g,r,a at byte 12. If this ever changes, the swizzle
  // assertion below stops meaning what it says.
  const auto page = db.point_cloud_page("cloud", 0, 1);
  REQUIRE(page.point_step == 16);
  REQUIRE(page.data.size() == 16);
  CHECK(page.data[12] == 128); // b
  CHECK(page.data[13] == 0);   // g
  CHECK(page.data[14] == 255); // r

  const auto buf = encode_ruxp(page);
  const Parsed p = parse(buf);
  REQUIRE(p.field_count == 2);
  REQUIRE(p.fields[1].name == "rgb");

  const size_t at = p.fields[1].byte_offset;
  REQUIRE(buf.size() >= at + 3);
  CHECK(buf[at + 0] == 255); // r
  CHECK(buf[at + 1] == 0);   // g
  CHECK(buf[at + 2] == 128); // b

  check_invariants(buf);
}

// ===========================================================================
// Paging
// ===========================================================================

TEST_CASE("EncodeRuxp_PagingWindows_ReportsCorrectCountOffsetTotal",
          "[gui][ruxp]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto source = make_known_rgb_cloud();
  db.save_point_cloud("cloud", source, "test");

  struct Window {
    uint64_t offset;
    uint64_t limit;
    uint64_t want_offset;
    uint64_t want_count;
  };

  // total is 3 in every case: a page past the end still reports the length of
  // the cloud (docs/gui/binary-points.md, "Paging").
  const std::vector<Window> windows = {
      {0, 2, 0, 2},  // first page
      {1, 2, 1, 2},  // interior page
      {2, 10, 2, 1}, // short last page
      {3, 5, 3, 0},  // exactly at the end: empty, total intact
      {99, 5, 3, 0}, // far past the end: offset clamps to total
  };

  for (const auto &w : windows) {
    INFO("offset=" << w.offset << " limit=" << w.limit);
    const auto page = db.point_cloud_page("cloud", w.offset, w.limit);
    const auto buf = encode_ruxp(page);
    const Parsed p = parse(buf);

    CHECK(p.count == w.want_count);
    CHECK(p.offset == w.want_offset);
    CHECK(p.total == 3);
    CHECK(p.header_size == 72);
    // 15 bytes/point for PointXYZRGB: 3 floats + 3 colour bytes.
    CHECK(buf.size() == 72 + w.want_count * 15);

    check_invariants(buf);

    // The window really is the window: point i of the page is point
    // offset + i of the source cloud.
    for (uint64_t i = 0; i < w.want_count; ++i) {
      const auto &src = source.points[w.want_offset + i];
      const size_t xyz = p.fields[0].byte_offset + 12 * static_cast<size_t>(i);
      const size_t rgb = p.fields[1].byte_offset + 3 * static_cast<size_t>(i);
      CHECK(rd_f32(buf, xyz + 0) == src.x);
      CHECK(rd_f32(buf, xyz + 4) == src.y);
      CHECK(rd_f32(buf, xyz + 8) == src.z);
      CHECK(buf[rgb + 0] == src.r);
      CHECK(buf[rgb + 1] == src.g);
      CHECK(buf[rgb + 2] == src.b);
    }
  }
}

TEST_CASE("EncodeRuxp_EmptyCloud_WritesHeaderOnly", "[gui][ruxp]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  reusex::Cloud empty;
  empty.width = 0;
  empty.height = 1;
  empty.is_dense = false;
  db.save_point_cloud("empty", empty, "test");

  const auto page = db.point_cloud_page("empty", 0, 100);
  const auto buf = encode_ruxp(page);
  const Parsed p = parse(buf);

  CHECK(p.magic == "RUXP");
  CHECK(p.version == 1);
  CHECK(p.header_size == 72);
  CHECK(p.field_count == 2);
  CHECK(p.count == 0);
  CHECK(p.offset == 0);
  CHECK(p.total == 0);
  CHECK(buf.size() == p.header_size);

  check_invariants(buf);
}

// ===========================================================================
// The other cloud types
// ===========================================================================

TEST_CASE("EncodeRuxp_PointXYZCloud_WritesSingleXyzField", "[gui][ruxp]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 4;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(4);
  for (size_t i = 0; i < 4; ++i) {
    cloud.points[i].x = static_cast<float>(i) + 0.25f;
    cloud.points[i].y = static_cast<float>(i) * 2.0f;
    cloud.points[i].z = -static_cast<float>(i);
  }
  db.save_point_cloud("xyz", cloud, "test");

  const auto page = db.point_cloud_page("xyz", 0, 4);
  REQUIRE(page.point_type == "PointXYZ");
  const auto buf = encode_ruxp(page);
  const Parsed p = parse(buf);

  REQUIRE(p.field_count == 1);
  CHECK(p.header_size == 56); // 40 + 16
  CHECK(p.count == 4);
  CHECK(p.total == 4);
  CHECK(p.fields[0].name == "xyz");
  CHECK(p.fields[0].type == 1); // f32
  CHECK(p.fields[0].components == 3);
  CHECK(p.fields[0].byte_offset == 56);
  CHECK(buf.size() == 56 + 4 * 12);

  for (size_t i = 0; i < 4; ++i) {
    const size_t at = p.fields[0].byte_offset + 12 * i;
    CHECK(rd_f32(buf, at + 0) == cloud.points[i].x);
    CHECK(rd_f32(buf, at + 4) == cloud.points[i].y);
    CHECK(rd_f32(buf, at + 8) == cloud.points[i].z);
  }

  check_invariants(buf);
}

TEST_CASE("EncodeRuxp_NormalCloud_WritesNormalFieldExcludingCurvature",
          "[gui][ruxp]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  reusex::CloudN cloud;
  cloud.width = 3;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(3);
  for (size_t i = 0; i < 3; ++i) {
    cloud.points[i].normal_x = 1.0f + static_cast<float>(i);
    cloud.points[i].normal_y = -0.5f * static_cast<float>(i);
    cloud.points[i].normal_z = 0.25f;
    // Deliberately distinctive: if curvature leaked into the payload the
    // value comparison below would land on 7, 8 or 9.
    cloud.points[i].curvature = 7.0f + static_cast<float>(i);
  }
  db.save_point_cloud("normals", cloud, "test");

  const auto page = db.point_cloud_page("normals", 0, 3);
  REQUIRE(page.point_type == "Normal");
  REQUIRE(page.point_step == 16); // curvature IS in storage...

  const auto buf = encode_ruxp(page);
  const Parsed p = parse(buf);

  // ...but not on the wire: one field, three components, 12 bytes/point.
  REQUIRE(p.field_count == 1);
  CHECK(p.fields[0].name == "normal"); // not "xyz"
  CHECK(p.fields[0].type == 1);        // f32
  CHECK(p.fields[0].components == 3);
  CHECK(p.header_size == 56);
  CHECK(buf.size() == 56 + 3 * 12);

  for (size_t i = 0; i < 3; ++i) {
    const size_t at = p.fields[0].byte_offset + 12 * i;
    CHECK(rd_f32(buf, at + 0) == cloud.points[i].normal_x);
    CHECK(rd_f32(buf, at + 4) == cloud.points[i].normal_y);
    CHECK(rd_f32(buf, at + 8) == cloud.points[i].normal_z);
  }

  check_invariants(buf);
}

TEST_CASE("EncodeRuxp_LabelCloud_WritesU32PerPoint", "[gui][ruxp]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  const std::vector<uint32_t> labels = {0, 1, 42, 4294967295u, 7};
  reusex::CloudL cloud;
  cloud.width = static_cast<uint32_t>(labels.size());
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(labels.size());
  for (size_t i = 0; i < labels.size(); ++i)
    cloud.points[i].label = labels[i];
  db.save_point_cloud("planes", cloud, "test");

  const auto page = db.point_cloud_page("planes", 0, labels.size());
  REQUIRE(page.point_type == "Label");
  const auto buf = encode_ruxp(page);
  const Parsed p = parse(buf);

  REQUIRE(p.field_count == 1);
  CHECK(p.fields[0].name == "label");
  CHECK(p.fields[0].type == 3); // u32
  CHECK(p.fields[0].components == 1);
  CHECK(p.header_size == 56);
  CHECK(buf.size() == 56 + labels.size() * 4);

  for (size_t i = 0; i < labels.size(); ++i)
    CHECK(rd_u32(buf, p.fields[0].byte_offset + 4 * i) == labels[i]);

  check_invariants(buf);
}

// ===========================================================================
// ruxp_supports
// ===========================================================================

TEST_CASE("RuxpSupports_StoredPointTypes_AcceptsExactlyFour", "[gui][ruxp]") {
  CHECK(ruxp_supports("PointXYZRGB"));
  CHECK(ruxp_supports("PointXYZ"));
  CHECK(ruxp_supports("Normal"));
  CHECK(ruxp_supports("Label"));

  CHECK_FALSE(ruxp_supports(""));
  CHECK_FALSE(ruxp_supports("junk"));
  CHECK_FALSE(ruxp_supports("PointXYZRGBA"));
  CHECK_FALSE(ruxp_supports("pointxyzrgb")); // matching is case-sensitive
  CHECK_FALSE(ruxp_supports("PointXYZI"));
  CHECK_FALSE(ruxp_supports("Labels"));
}

TEST_CASE("EncodeRuxp_UnsupportedPointType_Throws", "[gui][ruxp]") {
  ProjectDB::CloudPage page;
  page.point_type = "PointXYZI";
  page.point_step = 16;
  page.count = 0;
  page.total = 0;
  CHECK_THROWS_AS(encode_ruxp(page), std::runtime_error);
}

TEST_CASE("EncodeRuxp_TruncatedPageData_Throws", "[gui][ruxp]") {
  ProjectDB::CloudPage page;
  page.point_type = "PointXYZRGB";
  page.point_step = 16;
  page.offset = 0;
  page.count = 4;
  page.total = 4;
  page.data.resize(3 * 16); // one record missing
  CHECK_THROWS_AS(encode_ruxp(page), std::runtime_error);
}
