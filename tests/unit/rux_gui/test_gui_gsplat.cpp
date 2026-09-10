// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Contract tests for the Gaussian-splat routes of `rux gui` (#322).
//
// The splat lives in the project (schema v12), so these are ordinary ProjectDB
// reads and are tested the same way the mesh routes are — against a real
// database, without a socket.

#include <catch2/catch_test_macros.hpp>

#include <gui/gsplat.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>

#include <cstdint>
#include <string>
#include <vector>

using json = nlohmann::json;
using namespace rux::gui;

namespace {

using reusex::test_support::TempPath;

/// The property list the reference 3DGS implementation writes, for @p degree.
std::string inria_properties(int degree) {
  std::string out = "property float x\nproperty float y\nproperty float z\n"
                    "property float nx\nproperty float ny\nproperty float nz\n";
  for (int i = 0; i < 3; ++i)
    out += "property float f_dc_" + std::to_string(i) + "\n";
  const int rest = 3 * ((degree + 1) * (degree + 1) - 1);
  for (int i = 0; i < rest; ++i)
    out += "property float f_rest_" + std::to_string(i) + "\n";
  out += "property float opacity\n";
  for (int i = 0; i < 3; ++i)
    out += "property float scale_" + std::to_string(i) + "\n";
  for (int i = 0; i < 4; ++i)
    out += "property float rot_" + std::to_string(i) + "\n";
  return out;
}

/// A minimal but structurally honest INRIA splat: real header, a body of the
/// right length filled with zeroes. Nothing here decodes the body, and a GPU
/// is what would be needed to care about its contents.
std::vector<uint8_t> splat_ply(uint64_t count, int degree) {
  const std::string header = "ply\nformat binary_little_endian 1.0\n"
                             "element vertex " +
                             std::to_string(count) + "\n" +
                             inria_properties(degree) + "end_header\n";
  const size_t floats = static_cast<size_t>(
      3 + 3 + 3 + 3 * ((degree + 1) * (degree + 1) - 1) + 1 + 3 + 4);
  std::vector<uint8_t> bytes(header.begin(), header.end());
  bytes.resize(bytes.size() +
               static_cast<size_t>(count) * floats * sizeof(float));
  return bytes;
}

} // namespace

// ===========================================================================
// GET /api/v1/gsplats
// ===========================================================================

TEST_CASE("GsplatsJson_EmptyProject_ReturnsAnEmptyCollection",
          "[gui][gsplat]") {
  TempPath project("test_gui_gsplat");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);

  const auto body = gsplats_json(db, Params{});
  REQUIRE(body.contains("gsplats"));
  CHECK(body.at("gsplats").empty());
}

TEST_CASE("GsplatsJson_StoredSplats_ListsEachWithItsHeaderMetadata",
          "[gui][gsplat]") {
  TempPath project("test_gui_gsplat");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  db.save_gaussian_splat("splat", splat_ply(64, 2), "gsplat", "{}");
  db.save_gaussian_splat("detailed", splat_ply(8, 0), "gsplat", "{}");

  const auto body = gsplats_json(db, Params{});
  REQUIRE(body.at("gsplats").size() == 2);

  // The count and degree come from the PLY header, not from the caller, so a
  // row cannot describe a different model than the one stored.
  for (const auto &entry : body.at("gsplats")) {
    REQUIRE(entry.contains("name"));
    CHECK(entry.at("format") == "ply");
    CHECK(entry.contains("byte_size"));
    if (entry.at("name") == "splat") {
      CHECK(entry.at("gaussian_count") == 64);
      CHECK(entry.at("sh_degree") == 2);
    } else {
      CHECK(entry.at("gaussian_count") == 8);
      CHECK(entry.at("sh_degree") == 0);
    }
  }
}

// ===========================================================================
// GET /api/v1/gsplats/{name}
// ===========================================================================

TEST_CASE("GsplatJson_StoredSplat_ReportsMetadataWithoutTheBlob",
          "[gui][gsplat]") {
  TempPath project("test_gui_gsplat");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  const auto ply = splat_ply(32, 1);
  db.save_gaussian_splat("splat", ply, "gsplat", R"({"iterations":2000})");

  const auto body = gsplat_json(db, "splat");
  CHECK(body.at("name") == "splat");
  CHECK(body.at("gaussian_count") == 32);
  CHECK(body.at("sh_degree") == 1);
  CHECK(body.at("byte_size") == ply.size());
  CHECK(body.at("stage") == "gsplat");
  // Metadata only: a listing that carried hundreds of megabytes of PLY would
  // make the layer panel unusable.
  CHECK_FALSE(body.contains("data"));
}

TEST_CASE("GsplatJson_UnknownName_ThrowsNotFoundNamingTheFix",
          "[gui][gsplat]") {
  TempPath project("test_gui_gsplat");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);

  try {
    gsplat_json(db, "splat");
    FAIL("an absent splat must not produce a record");
  } catch (const HttpError &e) {
    CHECK(e.status() == 404);
    // A project nobody has trained is the normal state; the message has to
    // say what to run, not merely that the lookup failed.
    CHECK(std::string(e.what()).find("rux create gsplat") != std::string::npos);
  }
}

// ===========================================================================
// GET /api/v1/gsplats/{name}/data
// ===========================================================================

TEST_CASE("GsplatBlob_StoredSplat_ReturnsTheStoredBytesVerbatim",
          "[gui][gsplat]") {
  TempPath project("test_gui_gsplat");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  const auto ply = splat_ply(48, 1);
  db.save_gaussian_splat("splat", ply, "gsplat", "{}");

  const auto blob = gsplat_blob(db, "splat");
  CHECK(blob.content_type == "application/octet-stream");
  // Verbatim: the renderer parses the PLY itself, so any transcoding here
  // would be a bug rather than a feature.
  CHECK(blob.data == ply);
}

TEST_CASE("GsplatBlob_UnknownName_ThrowsNotFound", "[gui][gsplat]") {
  TempPath project("test_gui_gsplat");
  reusex::ProjectDB db(project.path, /*readOnly=*/false);

  try {
    gsplat_blob(db, "splat");
    FAIL("an absent splat must not produce a body");
  } catch (const HttpError &e) {
    CHECK(e.status() == 404);
  }
}
