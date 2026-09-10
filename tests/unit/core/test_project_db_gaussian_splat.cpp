// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Gaussian splat storage in ProjectDB (schema v12, #322), plus the INRIA PLY
// header parser it is built on.
//
// Two properties are being pinned. First, that the header parser tells a
// trained splat apart from the point-cloud PLY `rux export ply` writes — that
// is the failure users actually hit, and the whole reason save validates at
// all. Second, that the payload survives the chunked round trip byte for
// byte: the splat is handed to a renderer verbatim, so a single lost or
// duplicated byte at a chunk seam is a corrupt file, not a rounding error.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/gaussian_splat.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

using reusex::ProjectDB;
using reusex::core::parse_gaussian_splat_ply_header;
using reusex::test_support::TempPath;

namespace {

/// Floats per Gaussian in the INRIA layout at spherical-harmonic degree
/// @p degree: 3 position + 3 normal + 3 f_dc + 3*((d+1)^2-1) f_rest
/// + 1 opacity + 3 scale + 4 rotation.
size_t float_count_for_degree(int degree) {
  const size_t rest =
      static_cast<size_t>(3 * ((degree + 1) * (degree + 1) - 1));
  return 3 + 3 + 3 + rest + 1 + 3 + 4;
}

/// The exact property list the INRIA trainer emits, in its order.
std::string inria_header(uint64_t count, int degree,
                         const std::string &format = "binary_little_endian",
                         const std::string &eol = "\n") {
  std::string h = "ply" + eol;
  h += "format " + format + " 1.0" + eol;
  h += "element vertex " + std::to_string(count) + eol;
  for (const char *p : {"x", "y", "z", "nx", "ny", "nz"})
    h += std::string("property float ") + p + eol;
  for (int i = 0; i < 3; ++i)
    h += "property float f_dc_" + std::to_string(i) + eol;
  const int rest = 3 * ((degree + 1) * (degree + 1) - 1);
  for (int i = 0; i < rest; ++i)
    h += "property float f_rest_" + std::to_string(i) + eol;
  h += "property float opacity" + eol;
  for (int i = 0; i < 3; ++i)
    h += "property float scale_" + std::to_string(i) + eol;
  for (int i = 0; i < 4; ++i)
    h += "property float rot_" + std::to_string(i) + eol;
  h += "end_header" + eol;
  return h;
}

/// A structurally honest INRIA splat file: a real header for @p count
/// Gaussians at @p degree, followed by count * float_count * 4 body bytes.
///
/// The body is filled from a fixed-seed LCG rather than zeroed, so the
/// byte-exactness assertions have something to catch: an all-zero payload
/// would survive a truncation or a duplicated chunk unnoticed (STANDARDS §6 —
/// the seed is fixed, so the file is identical on every run).
std::vector<uint8_t> make_splat_ply(uint64_t count, int degree) {
  const std::string header = inria_header(count, degree);
  const size_t body_bytes =
      static_cast<size_t>(count) * float_count_for_degree(degree) * 4;

  std::vector<uint8_t> ply;
  ply.reserve(header.size() + body_bytes);
  ply.assign(header.begin(), header.end());

  uint32_t state = 0x9E3779B9u;
  for (size_t i = 0; i < body_bytes; ++i) {
    state = state * 1664525u + 1013904223u;
    ply.push_back(static_cast<uint8_t>(state >> 24));
  }
  return ply;
}

/// What `rux export ply` writes: valid PLY, no Gaussian properties.
std::vector<uint8_t> make_point_cloud_ply(uint64_t count) {
  std::string text = "ply\nformat binary_little_endian 1.0\n";
  text += "element vertex " + std::to_string(count) + "\n";
  text += "property float x\nproperty float y\nproperty float z\n";
  text += "property uchar red\nproperty uchar green\nproperty uchar blue\n";
  text += "end_header\n";
  std::vector<uint8_t> ply(text.begin(), text.end());
  ply.resize(ply.size() + static_cast<size_t>(count) * 15, 0);
  return ply;
}

std::string_view head_of(const std::vector<uint8_t> &ply) {
  return std::string_view(reinterpret_cast<const char *>(ply.data()),
                          ply.size());
}

/// Rows in `gaussian_splat_data`, read straight out of sqlite so the
/// assertion does not depend on the API under test to report on itself.
int chunk_row_count(const std::filesystem::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open_v2(path.string().c_str(), &db, SQLITE_OPEN_READONLY,
                          nullptr) == SQLITE_OK);
  sqlite3_stmt *stmt = nullptr;
  int rows = -1;
  if (sqlite3_prepare_v2(db, "SELECT COUNT(*) FROM gaussian_splat_data;", -1,
                         &stmt, nullptr) == SQLITE_OK) {
    if (sqlite3_step(stmt) == SQLITE_ROW)
      rows = sqlite3_column_int(stmt, 0);
    sqlite3_finalize(stmt);
  }
  sqlite3_close(db);
  REQUIRE(rows >= 0);
  return rows;
}

/// Turn a freshly created project back into a v11 one: drop the splat tables
/// and rewind schema_version. Read-only opens never migrate, so this is what
/// an old project on disk looks like to today's readers.
void downgrade_to_v11(const std::filesystem::path &path) {
  sqlite3 *db = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &db) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(db,
                              "DROP TABLE IF EXISTS gaussian_splat_data;"
                              "DROP TABLE IF EXISTS gaussian_splats;"
                              "DELETE FROM schema_version WHERE version >= 12;",
                              nullptr, nullptr, &err);
  sqlite3_free(err);
  sqlite3_close(db);
  REQUIRE(rc == SQLITE_OK);
}

} // namespace

// ── Header parsing ────────────────────────────────────────────────────

TEST_CASE("ParseGaussianSplatPlyHeader_InriaHeader_ReportsCountAndDegree",
          "[core][gsplat]") {
  const auto degree0 = inria_header(1234, 0);
  const auto h0 = parse_gaussian_splat_ply_header(degree0);
  CHECK(h0.gaussian_count == 1234);
  CHECK(h0.sh_degree == 0);

  // Degree 3 is what the trainer defaults to: 45 f_rest_* properties.
  const auto degree3 = inria_header(987654, 3);
  const auto h3 = parse_gaussian_splat_ply_header(degree3);
  CHECK(h3.gaussian_count == 987654);
  CHECK(h3.sh_degree == 3);
}

TEST_CASE("ParseGaussianSplatPlyHeader_PointCloudPly_"
          "ThrowsNamingTheMissingProperties",
          "[core][gsplat]") {
  const auto ply = make_point_cloud_ply(100);

  // The message is the product here: a user who points the tool at an
  // exported point cloud must learn which command makes the right file.
  try {
    parse_gaussian_splat_ply_header(head_of(ply));
    FAIL("expected a point-cloud PLY to be rejected");
  } catch (const std::runtime_error &e) {
    const std::string msg = e.what();
    CHECK(msg.find("f_dc_") != std::string::npos);
    CHECK(msg.find("rux export ply") != std::string::npos);
    CHECK(msg.find("rux create gsplat") != std::string::npos);
  }
}

TEST_CASE("ParseGaussianSplatPlyHeader_BigEndian_Throws", "[core][gsplat]") {
  const auto ply = inria_header(10, 0, "binary_big_endian");
  REQUIRE_THROWS_AS(parse_gaussian_splat_ply_header(ply), std::runtime_error);
}

TEST_CASE("ParseGaussianSplatPlyHeader_TruncatedHeader_Throws",
          "[core][gsplat]") {
  auto text = inria_header(10, 3);
  // Cut before end_header — exactly what a probe window that is too small,
  // or a half-written file, looks like.
  text.resize(text.size() / 2);
  REQUIRE_THROWS_AS(parse_gaussian_splat_ply_header(text), std::runtime_error);

  REQUIRE_THROWS_AS(parse_gaussian_splat_ply_header(""), std::runtime_error);
  REQUIRE_THROWS_AS(parse_gaussian_splat_ply_header("not a ply at all\n"),
                    std::runtime_error);
}

TEST_CASE("ParseGaussianSplatPlyHeader_CrlfLineEndings_Parses",
          "[core][gsplat]") {
  const auto crlf = inria_header(4242, 2, "binary_little_endian", "\r\n");
  const auto header = parse_gaussian_splat_ply_header(crlf);
  CHECK(header.gaussian_count == 4242);
  CHECK(header.sh_degree == 2);
}

// ── ProjectDB storage ─────────────────────────────────────────────────

TEST_CASE("SaveGaussianSplat_TrainedSplat_RoundTripsBytesVerbatim",
          "[core][gsplat]") {
  TempPath tmp("test_projectdb_gaussian_splat");

  // 160k Gaussians at degree 0 is 68 bytes each, ~10.4 MiB — more than the
  // 8 MiB chunk size, so this exercises the multi-chunk reassembly path
  // (a single-chunk payload would never catch a seam bug).
  const auto ply = make_splat_ply(160000, 0);
  REQUIRE(ply.size() > 8ull * 1024 * 1024);

  {
    ProjectDB db(tmp.path);
    REQUIRE(db.schema_version() == 12);
    db.save_gaussian_splat("scene", ply, "gsplat", R"({"iters":30000})");
    REQUIRE(db.has_gaussian_splat("scene"));
  }

  // Reopen: the bytes must survive the connection, not just the cache.
  ProjectDB db(tmp.path, /*readOnly=*/true);
  const auto loaded = db.gaussian_splat_blob("scene");
  REQUIRE(loaded.size() == ply.size());
  CHECK(loaded == ply);

  // More than one chunk row actually exists.
  CHECK(chunk_row_count(tmp.path) > 1);
}

TEST_CASE("SaveGaussianSplat_HeaderMetadata_IsDerivedFromThePly",
          "[core][gsplat]") {
  TempPath tmp("test_projectdb_gaussian_splat");
  ProjectDB db(tmp.path);

  const auto ply = make_splat_ply(777, 3);
  db.save_gaussian_splat("scene", ply, "gsplat", R"({"sh":3})");

  const auto meta = db.gaussian_splat_metadata("scene");
  CHECK(meta.name == "scene");
  CHECK(meta.format == "ply");
  CHECK(meta.gaussian_count == 777);
  CHECK(meta.sh_degree == 3);
  CHECK(meta.byte_size == ply.size());
  CHECK(meta.stage == "gsplat");
  CHECK(meta.parameters == R"({"sh":3})");
  CHECK_FALSE(meta.created_at.empty());
}

TEST_CASE("SaveGaussianSplat_SameName_ReplacesThePrevious", "[core][gsplat]") {
  TempPath tmp("test_projectdb_gaussian_splat");
  ProjectDB db(tmp.path);

  const auto first = make_splat_ply(50000, 0); // ~3.2 MiB, one chunk
  const auto second = make_splat_ply(1000, 1); // much smaller, degree 1
  REQUIRE(second.size() < first.size());

  db.save_gaussian_splat("scene", first);
  db.save_gaussian_splat("scene", second);

  // Only one splat, and it is the second one — metadata and payload both.
  CHECK(db.list_gaussian_splats().size() == 1);
  const auto meta = db.gaussian_splat_metadata("scene");
  CHECK(meta.gaussian_count == 1000);
  CHECK(meta.sh_degree == 1);
  CHECK(meta.byte_size == second.size());

  // The real hazard: leftover chunks from the longer first payload would
  // silently append a tail of the old file to the new one.
  const auto loaded = db.gaussian_splat_blob("scene");
  REQUIRE(loaded.size() == second.size());
  CHECK(loaded == second);
}

TEST_CASE("SaveGaussianSplat_PointCloudPly_ThrowsAndStoresNothing",
          "[core][gsplat]") {
  TempPath tmp("test_projectdb_gaussian_splat");
  ProjectDB db(tmp.path);

  const auto ply = make_point_cloud_ply(500);
  REQUIRE_THROWS_AS(db.save_gaussian_splat("scene", ply), std::runtime_error);

  // Rejected at the door: no metadata row, no chunks, nothing to clean up.
  CHECK_FALSE(db.has_gaussian_splat("scene"));
  CHECK(db.list_gaussian_splats().empty());
  CHECK(chunk_row_count(tmp.path) == 0);
}

TEST_CASE("ListGaussianSplats_MultipleStored_ReturnsEveryName",
          "[core][gsplat]") {
  TempPath tmp("test_projectdb_gaussian_splat");
  ProjectDB db(tmp.path);

  CHECK(db.list_gaussian_splats().empty());

  db.save_gaussian_splat("ground_floor", make_splat_ply(10, 0));
  db.save_gaussian_splat("first_floor", make_splat_ply(20, 1));
  db.save_gaussian_splat("attic", make_splat_ply(30, 2));

  const auto names = db.list_gaussian_splats();
  REQUIRE(names.size() == 3);
  // Insertion order (ORDER BY id), which is the order they were trained in.
  CHECK(names[0] == "ground_floor");
  CHECK(names[1] == "first_floor");
  CHECK(names[2] == "attic");
}

TEST_CASE("GaussianSplatBlob_UnknownName_Throws", "[core][gsplat]") {
  TempPath tmp("test_projectdb_gaussian_splat");
  ProjectDB db(tmp.path);

  CHECK_FALSE(db.has_gaussian_splat("nope"));
  REQUIRE_THROWS_AS(db.gaussian_splat_blob("nope"), std::runtime_error);
  REQUIRE_THROWS_AS(db.gaussian_splat_metadata("nope"), std::runtime_error);
}

TEST_CASE("DeleteGaussianSplat_StoredSplat_RemovesItAndItsChunks",
          "[core][gsplat]") {
  TempPath tmp("test_projectdb_gaussian_splat");
  ProjectDB db(tmp.path);

  db.save_gaussian_splat("scene", make_splat_ply(20000, 0));
  REQUIRE(db.has_gaussian_splat("scene"));
  REQUIRE(chunk_row_count(tmp.path) > 0);

  CHECK(db.delete_gaussian_splat("scene"));
  CHECK_FALSE(db.has_gaussian_splat("scene"));
  CHECK(db.list_gaussian_splats().empty());
  // ON DELETE CASCADE, not a manual sweep — the chunks must go with the row.
  CHECK(chunk_row_count(tmp.path) == 0);

  // Nothing left to delete the second time.
  CHECK_FALSE(db.delete_gaussian_splat("scene"));

  // And the name is reusable afterwards.
  const auto again = make_splat_ply(11, 0);
  db.save_gaussian_splat("scene", again);
  CHECK(db.gaussian_splat_blob("scene") == again);
}

TEST_CASE("ProjectSummary_StoredSplat_ListsItWithoutTheBlob",
          "[core][gsplat]") {
  TempPath tmp("test_projectdb_gaussian_splat");
  ProjectDB db(tmp.path);

  CHECK(db.project_summary().gaussian_splats.empty());

  const auto ply = make_splat_ply(2500, 2);
  db.save_gaussian_splat("scene", ply, "gsplat", R"({"iters":7000})");

  const auto summary = db.project_summary();
  REQUIRE(summary.gaussian_splats.size() == 1);
  const auto &info = summary.gaussian_splats[0];
  CHECK(info.name == "scene");
  CHECK(info.format == "ply");
  CHECK(info.gaussian_count == 2500);
  CHECK(info.sh_degree == 2);
  CHECK(info.byte_size == ply.size());
  CHECK(info.stage == "gsplat");
}

// A read-only open never runs migrations, so every splat read path has to
// answer "nothing here" on a project whose schema predates the tables — not
// surface sqlite's "no such table". `rux info` runs read-only, so
// project_summary() failing here would break inspection of every old project.
TEST_CASE(
    "ProjectSummary_PreV12SchemaReadOnlyOpen_ReportsNoSplatsWithoutThrowing",
    "[core][gsplat][migration]") {
  TempPath tmp("test_projectdb_gaussian_splat");
  {
    ProjectDB fresh(tmp.path);
  }
  downgrade_to_v11(tmp.path);

  ProjectDB db(tmp.path, /*readOnly=*/true);
  REQUIRE(db.schema_version() == 11);

  ProjectDB::ProjectSummary summary;
  REQUIRE_NOTHROW(summary = db.project_summary());
  CHECK(summary.gaussian_splats.empty());

  CHECK_FALSE(db.has_gaussian_splat("scene"));
  CHECK(db.list_gaussian_splats().empty());
  CHECK_FALSE(db.delete_gaussian_splat("scene"));
  REQUIRE_THROWS_AS(db.gaussian_splat_blob("scene"), std::runtime_error);
  REQUIRE_THROWS_AS(db.gaussian_splat_metadata("scene"), std::runtime_error);
}

// The other half of the same story: opened read-write, the same project
// migrates to v12 and the tables appear.
TEST_CASE("ProjectDb_PreV12ProjectReadWriteOpen_MigratesToV12",
          "[core][gsplat][migration]") {
  TempPath tmp("test_projectdb_gaussian_splat");
  {
    ProjectDB fresh(tmp.path);
  }
  downgrade_to_v11(tmp.path);

  const auto ply = make_splat_ply(64, 1);
  {
    ProjectDB db(tmp.path);
    REQUIRE(db.schema_version() == 12);
    db.save_gaussian_splat("scene", ply);
  }

  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK(ro.schema_version() == 12);
  CHECK(ro.gaussian_splat_blob("scene") == ply);
}
