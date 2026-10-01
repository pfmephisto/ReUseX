// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Survey operations that need a project: the sample approval gate, quantity
// redistribution across parts, totals, and the checked sample edit.

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>
#include <core/survey_service.hpp>

#include "../../support/temp_path.hpp"

#include <stdexcept>

using reusex::ProjectDB;
using namespace reusex::core;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_survey_service") {}
};
int64_t type_with_parts(ProjectDB &db, std::vector<double> quantities) {
  ProjectDB::SurveyTypeRecord t;
  t.name = "Betonsøjler, bærende";
  t.mass_t = 58.0;
  const auto id = db.add_survey_type(t).id;
  int n = db.max_survey_part_number();
  for (double q : quantities)
    db.add_survey_part({part_code(++n),
                        id,
                        std::nullopt,
                        std::nullopt,
                        std::nullopt,
                        "",
                        q,
                        false,
                        "",
                        {},
                        {}});
  return id;
}
} // namespace

TEST_CASE("SetReviewStatus_PendingSample_BlocksApproval_NotRejection",
          "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = type_with_parts(db, {1});
  const auto s = db.add_sample("PCB i fugemasse", "");
  db.set_sample_links(s.id, {id});
  CHECK(environment_status_of(db, id) == EnvironmentStatus::afventer);
  CHECK_THROWS_AS(set_review_status(db, id, ReviewStatus::approved),
                  SamplePendingError);
  CHECK(db.survey_type(id)->review_status == ReviewStatus::queue);
  CHECK(set_review_status(db, id, ReviewStatus::rejected).review_status ==
        ReviewStatus::rejected);

  ProjectDB::SamplePatch answered;
  answered.stage = SampleStage::svar;
  answered.result = SampleResult::ren;
  update_sample_checked(db, s.id, answered);
  CHECK(environment_status_of(db, id) == EnvironmentStatus::ren_proevesvar);
  CHECK(set_review_status(db, id, ReviewStatus::approved).review_status ==
        ReviewStatus::approved);
}

TEST_CASE("SetTypeQuantity_RedistributesAcrossParts", "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = type_with_parts(db, {18, 6});
  const auto parts = set_type_quantity(db, id, 30);
  REQUIRE(parts.size() == 2);
  CHECK(parts[0].quantity == 22.5);
  CHECK(parts[1].quantity == 7.5);
  const auto empty = type_with_parts(db, {});
  CHECK_THROWS_AS(set_type_quantity(db, empty, 5), std::invalid_argument);
  CHECK_THROWS_AS(set_type_quantity(db, id, -1), std::invalid_argument);
}

TEST_CASE("UpdateSampleChecked_ResultBeforeAnswer_Throws",
          "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto s = db.add_sample("x", "");
  ProjectDB::SamplePatch p;
  p.result = SampleResult::ren;
  CHECK_THROWS_AS(update_sample_checked(db, s.id, p), std::invalid_argument);
  CHECK(db.sample(s.id)->result == SampleResult::none);
}

TEST_CASE("TypeTotals_CarryEnvironment", "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = type_with_parts(db, {1});
  type_with_parts(db, {1});
  const auto s = db.add_sample("x", "");
  db.set_sample_links(s.id, {a});
  const auto totals = type_totals(db);
  REQUIRE(totals.size() == 2);
  CHECK(totals[0].environment == EnvironmentStatus::afventer);
  CHECK(totals[1].environment == EnvironmentStatus::ren_screening);
  CHECK(totals[0].mass_t == 58.0);
}
