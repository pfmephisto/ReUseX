// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Survey and sample JSON (Kortlægning): shapes, counts, derived miljøstatus,
// circularity and fractions as the frontend reads them.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <gui/survey.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>
#include <core/survey_service.hpp>

#include <nlohmann/json.hpp>

using namespace rux::gui;
using reusex::ProjectDB;
namespace core = reusex::core;
using Catch::Approx;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_gui_survey") {}
};
int64_t add_type(ProjectDB &db, const char *name, core::Treatment tr,
                 double mass, core::ReviewStatus st = core::ReviewStatus::queue,
                 const char *eak = "17.01.01") {
  ProjectDB::SurveyTypeRecord t;
  t.name = name;
  t.treatment = tr;
  t.mass_t = mass;
  t.review_status = st;
  t.eak_code = eak;
  return db.add_survey_type(t).id;
}
} // namespace

TEST_CASE("SurveyJson_TypesWithParts_Counts_Environment", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = add_type(db, "Betonsøjler", core::Treatment::genbrug, 58);
  add_type(db, "Beton, fundament", core::Treatment::bevaring, 640,
           core::ReviewStatus::approved);
  add_type(db, "Fejl", core::Treatment::genbrug, 1,
           core::ReviewStatus::rejected);
  db.add_survey_part({"RX-002",
                      a,
                      std::nullopt,
                      std::nullopt,
                      6,
                      "Entrance",
                      6,
                      false,
                      "",
                      {},
                      {}});
  db.add_survey_part({"RX-001",
                      a,
                      std::nullopt,
                      std::nullopt,
                      4,
                      "Production Hall",
                      18,
                      false,
                      "",
                      {},
                      {}});
  const auto s = db.add_sample("PCB", "");
  db.set_sample_links(s.id, {a});

  const auto j = survey_json(db);
  CHECK(j.at("counts").at("queue") == 1);
  CHECK(j.at("counts").at("approved") == 1);
  CHECK(j.at("counts").at("rejected") == 1);
  CHECK(j.at("counts").at("all") == 2);
  const auto &t = j.at("types").at(0);
  CHECK(t.at("name") == "Betonsøjler");
  CHECK(t.at("treatment") == "genbrug");
  CHECK(t.at("environment_status") == "afventer");
  CHECK(t.at("sample_ids") == nlohmann::json::array({s.id}));
  CHECK(t.at("quantity").get<double>() == Approx(24));
  CHECK(t.at("eak_name") == "Beton");
  CHECK(t.at("confidence").is_null());
  REQUIRE(t.at("parts").size() == 2);
  CHECK(t.at("parts").at(0).at("code") == "RX-001");
  CHECK(t.at("parts").at(0).at("cloud").is_null());
  CHECK(t.at("parts").at(0).at("room_name") == "Production Hall");
}

TEST_CASE("SurveySummaryJson_Circularity_ReuseShare_PendingSamples",
          "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_type(db, "a", core::Treatment::bevaring, 60);
  add_type(db, "b", core::Treatment::genanvendelse, 40);
  db.add_sample("pending", "");
  const auto j = survey_summary_json(db);
  CHECK(j.at("circularity").at("bevaring").get<double>() == Approx(60));
  CHECK(j.at("circularity").at("nyttiggoerelse").get<double>() == Approx(0));
  CHECK(j.at("total_mass_t").get<double>() == Approx(100));
  CHECK(j.at("reuse_share").get<double>() == Approx(0.6));
  CHECK(j.at("pending_samples") == 1);
  CHECK(j.at("unlabeled_points").is_null()); // no instance cloud
  CHECK(j.at("rooms_without_parts") == nlohmann::json::array());
}

TEST_CASE("SurveySummaryJson_EmptyProject_ReuseShareNull", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK(survey_summary_json(db).at("reuse_share").is_null());
}

TEST_CASE("SurveyFractionsJson_ApprovedOnly_ReadyFlag", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_type(db, "a", core::Treatment::genanvendelse, 380,
           core::ReviewStatus::approved);
  add_type(db, "b", core::Treatment::genbrug, 58);
  auto j = survey_fractions_json(db);
  REQUIRE(j.at("fractions").size() == 1);
  CHECK(j.at("fractions").at(0).at("name") == "Beton");
  CHECK(j.at("fractions").at(0).at("treatment") == "genanvendelse");
  CHECK(j.at("blocking_types") == 1);
  CHECK(j.at("ready") == false);
}

TEST_CASE("SamplesJson_ResultNullWhenNone", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.add_sample("PCB i fugemasse", "Fugemasse");
  const auto j = samples_json(db);
  REQUIRE(j.at("samples").size() == 1);
  CHECK(j.at("samples").at(0).at("code") == "P-01");
  CHECK(j.at("samples").at(0).at("stage") == "planlagt");
  CHECK(j.at("samples").at(0).at("result").is_null());
}
