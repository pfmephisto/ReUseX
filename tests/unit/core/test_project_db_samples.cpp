// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v22 samples: codes, stage/result storage, link replacement,
// per-type lookup, and cascade on type deletion.

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <stdexcept>

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
