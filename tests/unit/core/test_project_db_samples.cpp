// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v22 samples: codes, stage/result storage, link replacement,
// per-type lookup, and cascade on type deletion.

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <optional>
#include <stdexcept>
#include <string>

using reusex::ProjectDB;
namespace core = reusex::core;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_samples") {}
};
int64_t add_type(ProjectDB &db, const char *name) {
  ProjectDB::SurveyTypeRecord t;
  t.name = name;
  return db.add_survey_type(t).id;
}
void add_part(ProjectDB &db, const char *code, int64_t type_id) {
  db.add_survey_part({code,
                      type_id,
                      std::nullopt,
                      std::nullopt,
                      std::nullopt,
                      "Office Zone",
                      1,
                      false,
                      "",
                      {},
                      {}});
}
} // namespace

TEST_CASE("Samples_Add_AssignsSequentialCodes", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a =
      db.add_sample("PCB i fugemasse", "Fugemasse omkring vinduespartier");
  const auto b = db.add_sample("Bly i maling", "Malede indervægge");
  CHECK(a.code == "P-01");
  CHECK(b.code == "P-02");
  CHECK(a.stage == core::SampleStage::planlagt);
  CHECK(a.result == core::SampleResult::none);
  CHECK(db.samples().size() == 2);
}

TEST_CASE("Samples_Delete_DoesNotReuseCodes", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.add_sample("a", "");
  const auto b = db.add_sample("b", "");
  CHECK(db.delete_sample(b.id));
  CHECK_FALSE(db.delete_sample(b.id));
  // A lab report quoting P-02 must never come to mean a different sample.
  CHECK(db.add_sample("c", "").code == "P-03");
}

TEST_CASE("Samples_UpdateStageAndResult", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto s = db.add_sample("Asbest i linoleumslim", "Gulvlim, Office Zone");
  ProjectDB::SamplePatch p;
  p.stage = core::SampleStage::svar;
  p.result = core::SampleResult::forurenet;
  const auto u = db.update_sample(s.id, p);
  CHECK(u.stage == core::SampleStage::svar);
  CHECK(u.result == core::SampleResult::forurenet);
  CHECK(u.title == "Asbest i linoleumslim");
  CHECK_THROWS_AS(db.update_sample(s.id + 50, p), std::out_of_range);
}

TEST_CASE("Samples_Links_ReplaceSet_PerTypeLookup_AtomicOnUnknownType",
          "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto walls = add_type(db, "Indvendige murvægge, malet");
  const auto windows = add_type(db, "Vinduespartier");
  const auto s = db.add_sample("Bly i maling", "");
  db.set_sample_links(s.id, {windows, walls});
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{walls, windows});
  db.set_sample_links(s.id, {walls});
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{walls});
  CHECK(db.samples_for_type(walls).size() == 1);
  CHECK(db.samples_for_type(windows).empty());
  CHECK_THROWS_AS(db.set_sample_links(s.id, {walls, 9999}), std::out_of_range);
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{walls}); // unchanged
}

// GUI Phase 6 (On-site): a sample registered at a bygningsdel records it.

TEST_CASE("Samples_Add_WithPartCode_RoundTrips", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = add_type(db, "Vinduespartier, aluminium");
  add_part(db, "RX-008", t);
  const auto s = db.add_sample("Asbest i fugemasse", "Fuge mod nord",
                               std::string("RX-008"));
  CHECK(s.part_code == std::optional<std::string>("RX-008"));
  CHECK(db.sample(s.id)->part_code == std::optional<std::string>("RX-008"));
  CHECK(db.samples().front().part_code == std::optional<std::string>("RX-008"));
  CHECK_FALSE(db.add_sample("Uden del", "").part_code.has_value());
}

TEST_CASE("Samples_Add_UnknownPart_ThrowsAndWritesNothing",
          "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK_THROWS_AS(db.add_sample("x", "", std::string("RX-404")),
                  std::out_of_range);
  CHECK(db.samples().empty());
  // The refused sample did not consume a code either.
  CHECK(db.add_sample("y", "").code == "P-01");
}

TEST_CASE("Samples_MigratesFromV23_PartCodeNull",
          "[ProjectDB][samples][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    db.add_sample("PCB i fugemasse", "Fugemasse");
  }
  {
    // Roll back to v23: drop the v24 column and every later version row.
    sqlite3 *raw = nullptr;
    REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
    const char *sql = "ALTER TABLE samples DROP COLUMN part_code;"
                      "DELETE FROM schema_version WHERE version >= 24;";
    REQUIRE(sqlite3_exec(raw, sql, nullptr, nullptr, nullptr) == SQLITE_OK);
    sqlite3_close(raw);
  }
  {
    // Read-only opens never migrate; the list must still work on v23.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    const auto list = ro.samples();
    REQUIRE(list.size() == 1);
    CHECK_FALSE(list[0].part_code.has_value());
    CHECK_FALSE(ro.sample(list[0].id)->part_code.has_value());
  }
  ProjectDB db(tmp.path, /*readOnly=*/false);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK_FALSE(db.samples().front().part_code.has_value());
  const auto t = add_type(db, "Vinduer");
  add_part(db, "RX-001", t);
  CHECK(db.add_sample("Ny", "", std::string("RX-001")).part_code ==
        std::optional<std::string>("RX-001"));
}
