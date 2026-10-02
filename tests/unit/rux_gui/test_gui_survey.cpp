// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Survey and sample JSON (Kortlægning): shapes, counts, derived miljøstatus,
// circularity and fractions as the frontend reads them.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "gui/ViewRenderer.hpp"
#include <gui/survey.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>
#include <core/survey_service.hpp>

#include <nlohmann/json.hpp>

#include <pcl/point_types.h>

#include <algorithm>
#include <functional>

using namespace rux::gui;
using reusex::ProjectDB;
namespace core = reusex::core;
using Catch::Approx;
using Catch::Matchers::WithinAbs;

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
  CHECK(j.at("fractions").at(0).at("contaminated") == false);
  REQUIRE(j.at("blocking").size() == 1);
  CHECK(j.at("blocking").at(0).at("name") == "b");
  CHECK(j.at("blocking").at(0).at("reason") == "review");
  CHECK(j.at("blocking").at(0).at("treatment") == "genbrug");
  CHECK(j.at("blocking").at(0).at("mass_t").get<double>() == Approx(58));
  CHECK(j.at("blocking").at(0).at("type_id").get<int64_t>() > 0);
}

TEST_CASE("SurveyFractionsJson_BlockingMassNullWhenUnset", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  ProjectDB::SurveyTypeRecord t;
  t.name = "Uden tonnage";
  t.eak_code = "17.02.01";
  db.add_survey_type(t); // mass_t stays nullopt
  const auto j = survey_fractions_json(db);
  REQUIRE(j.at("blocking").size() == 1);
  CHECK(j.at("blocking").at(0).at("mass_t").is_null());
  CHECK(j.at("ready") == false);
}

TEST_CASE("SurveyFractionsJson_EmptySurveyIsNotReady", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  {
    const auto j = survey_fractions_json(db);
    CHECK(j.at("fractions").empty());
    CHECK(j.at("blocking_types") == 0);
    CHECK(j.at("ready") == false);
  }
  // Only bevaring: approved, nothing blocks, but no fraction row to send.
  add_type(db, "Fundament", core::Treatment::bevaring, 640,
           core::ReviewStatus::approved);
  {
    const auto j = survey_fractions_json(db);
    CHECK(j.at("fractions").empty());
    CHECK(j.at("blocking_types") == 0);
    CHECK(j.at("ready") == false);
  }
  add_type(db, "Beton", core::Treatment::genanvendelse, 12,
           core::ReviewStatus::approved);
  const auto j = survey_fractions_json(db);
  CHECK(j.at("fractions").size() == 1);
  CHECK(j.at("ready") == true);
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

// ===========================================================================
// Writes (#265 Phase 2)
// ===========================================================================

namespace {
int status_of(const std::function<void()> &f) {
  try {
    f();
  } catch (const HttpError &e) {
    return e.status();
  }
  return 200;
}
} // namespace

TEST_CASE("PatchSurveyType_SparseFields_AndQuantityRedistribution",
          "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = add_type(db, "Betonsøjler", core::Treatment::genbrug, 58);
  db.add_survey_part({"RX-001",
                      id,
                      std::nullopt,
                      std::nullopt,
                      std::nullopt,
                      "",
                      18,
                      false,
                      "",
                      {},
                      {}});
  db.add_survey_part({"RX-002",
                      id,
                      std::nullopt,
                      std::nullopt,
                      std::nullopt,
                      "",
                      6,
                      false,
                      "",
                      {},
                      {}});
  const auto j = patch_survey_type_json(
      db, id,
      R"({"treatment":"genanvendelse","mass_t":null,"starred":true,"quantity":30})");
  CHECK(j.at("treatment") == "genanvendelse");
  CHECK(j.at("mass_t").is_null());
  CHECK(j.at("starred") == true);
  CHECK(j.at("quantity").get<double>() == Approx(30));
  CHECK(j.at("parts").at(0).at("quantity").get<double>() == Approx(22.5));
}

TEST_CASE("PatchSurveyType_Errors_MapToStatuses", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = add_type(db, "Vinduer", core::Treatment::genbrug, 3.1);
  const auto s = db.add_sample("PCB", "");
  db.set_sample_links(s.id, {id});
  CHECK(status_of([&] {
          patch_survey_type_json(db, id, R"({"review_status":"approved"})");
        }) == 422);
  CHECK(status_of([&] {
          patch_survey_type_json(db, id, R"({"treatment":"Genbrug"})");
        }) == 400);
  CHECK(status_of([&] {
          patch_survey_type_json(db, id, R"({"quantity":-1})");
        }) == 400);
  CHECK(status_of([&] {
          patch_survey_type_json(db, id, R"({"quantity":5})");
        }) == 422); // no parts
  CHECK(status_of([&] {
          patch_survey_type_json(db, id + 99, R"({"starred":true})");
        }) == 404);
  CHECK(status_of([&] { patch_survey_type_json(db, id, "not json"); }) == 400);
  // A rejected combination must not half-apply: nothing changed.
  CHECK(status_of([&] {
          patch_survey_type_json(
              db, id, R"({"starred":true,"review_status":"approved"})");
        }) == 422);
  CHECK_FALSE(db.survey_type(id)->starred);
}

TEST_CASE("PatchSurveyType_RejectsNonFiniteAndOutOfRange",
          "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = add_type(db, "Betonsøjler", core::Treatment::genbrug, 58);
  db.add_survey_part({"RX-001",
                      id,
                      std::nullopt,
                      std::nullopt,
                      std::nullopt,
                      "",
                      18,
                      false,
                      "",
                      {},
                      {}});
  // 1e999 overflows double on parse, producing +inf.
  CHECK(status_of([&] {
          patch_survey_type_json(db, id, R"({"quantity":1e999})");
        }) == 400);
  CHECK(status_of([&] {
          patch_survey_type_json(db, id, R"({"mass_t":1e999})");
        }) == 400);
  CHECK(status_of([&] {
          patch_survey_type_json(db, id, R"({"confidence":5})");
        }) == 400);
  CHECK(status_of([&] {
          patch_survey_type_json(db, id, R"({"confidence":-0.1})");
        }) == 400);
  CHECK(status_of([&] { patch_survey_type_json(db, id, R"({"name":""})"); }) ==
        400);
  // None of the rejected patches changed anything.
  CHECK(db.survey_type(id)->mass_t == 58.0);
  CHECK(db.survey_part("RX-001")->quantity == 18.0);
}

TEST_CASE("PatchSurveyPart_RejectsNonFiniteQuantity", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = add_type(db, "Betonsøjler", core::Treatment::genbrug, 58);
  db.add_survey_part({"RX-001",
                      id,
                      std::nullopt,
                      std::nullopt,
                      std::nullopt,
                      "",
                      18,
                      false,
                      "",
                      {},
                      {}});
  CHECK(status_of([&] {
          patch_survey_part_json(db, "RX-001", R"({"quantity":1e999})");
        }) == 400);
  CHECK(db.survey_part("RX-001")->quantity == 18.0);
}

TEST_CASE("CreateSurveyType_RequiresName", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto j = create_survey_type_json(
      db, R"({"name":"Trapezplader, tag","eak_code":"17.04.05"})");
  CHECK(j.at("id").get<int64_t>() > 0);
  CHECK(j.at("review_status") == "queue");
  CHECK(status_of([&] { create_survey_type_json(db, R"({"name":""})"); }) ==
        400);
}

TEST_CASE("CreateSurveyType_IsManual_NeverMatchedBySync",
          "[gui][survey][edits][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  create_survey_type_json(db, R"({"name":"Manuelt tilføjet type"})");
  CHECK(db.survey_types().at(0).semantic_class == core::kManualSemanticClass);

  // An unclassified (class -1) instance must get its own "Uklassificeret"
  // type, not the manual one.
  reusex::CloudL labels;
  for (std::uint32_t l : {1u, 0u}) {
    pcl::Label p;
    p.label = l;
    labels.push_back(p);
  }
  db.save_point_cloud("instances", labels, "test", "{}");
  db.save_instances("instances", {{1, "g1", -1, 1}});

  const auto r = core::sync_survey(db);
  CHECK(r.types_created == 1);
  CHECK(r.parts_created == 1);
  const auto types = db.survey_types();
  REQUIRE(types.size() == 2);
  const auto manual =
      std::find_if(types.begin(), types.end(), [](const auto &t) {
        return t.semantic_class == core::kManualSemanticClass;
      });
  REQUIRE(manual != types.end());
  for (const auto &p : db.survey_parts())
    CHECK(p.type_id != manual->id);
}

TEST_CASE("SurveyPartJson_OrphanedFlag", "[gui][survey]") {
  reusex::ProjectDB::SurveyPartRecord linked;
  linked.code = "RX-001";
  linked.type_id = 1;
  linked.cloud_name = "instances";
  linked.instance_id = 1;
  linked.instance_guid = "g1";
  CHECK_FALSE(survey_part_json(linked).at("orphaned").get<bool>());

  // instance_guid set but cloud/instance_id read back null: the guid no
  // longer resolves to an instances row.
  reusex::ProjectDB::SurveyPartRecord orphaned = linked;
  orphaned.cloud_name = std::nullopt;
  orphaned.instance_id = std::nullopt;
  CHECK(survey_part_json(orphaned).at("orphaned").get<bool>());
}

TEST_CASE("PatchSurveyPart_MoveAndErrors", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = add_type(db, "a", core::Treatment::genbrug, 1);
  const auto b = add_type(db, "b", core::Treatment::genbrug, 1);
  db.add_survey_part({"RX-001",
                      a,
                      std::nullopt,
                      std::nullopt,
                      std::nullopt,
                      "",
                      1,
                      false,
                      "",
                      {},
                      {}});
  const auto j = patch_survey_part_json(db, "RX-001",
                                        R"({"type_id":)" + std::to_string(b) +
                                            R"(,"note":"flyttet"})");
  CHECK(j.at("type_id") == b);
  CHECK(j.at("note") == "flyttet");
  CHECK(status_of([&] {
          patch_survey_part_json(db, "RX-404", R"({"starred":true})");
        }) == 404);
  CHECK(status_of([&] {
          patch_survey_part_json(db, "RX-001", R"({"type_id":9999})");
        }) == 404);
}

TEST_CASE("SampleEndpoints_CreateLinkAnswerDelete", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = add_type(db, "Vinduer", core::Treatment::genbrug, 3.1);
  auto s = create_sample_json(db, R"({"title":"PCB i fugemasse","type_ids":[)" +
                                      std::to_string(t) + "]}");
  const auto id = s.at("id").get<int64_t>();
  CHECK(s.at("code") == "P-01");
  CHECK(s.at("type_ids") == nlohmann::json::array({t}));
  CHECK(status_of([&] { patch_sample_json(db, id, R"({"result":"ren"})"); }) ==
        422);
  s = patch_sample_json(db, id, R"({"stage":"svar","result":"forurenet"})");
  CHECK(s.at("result") == "forurenet");
  CHECK(status_of([&] { patch_sample_json(db, id, R"({"stage":"lab"})"); }) ==
        400);
  s = set_sample_links_json(db, id, R"({"type_ids":[]})");
  CHECK(s.at("type_ids").empty());
  CHECK(status_of([&] {
          set_sample_links_json(db, id, R"({"type_ids":[9999]})");
        }) == 404);
  delete_sample(db, id);
  CHECK(status_of([&] { delete_sample(db, id); }) == 404);
}

TEST_CASE("SyncSurveyJson_NoInstances_Is422", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK(status_of([&] { sync_survey_json(db, "{}"); }) == 422);
}

TEST_CASE("SyncSurveyJson_ReportsInstancesSeenAndBackfilled",
          "[gui][survey][edits][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  // A pre-v10 instance cloud: labels and definitions, no `instances` rows.
  reusex::CloudL inst;
  for (std::uint32_t l : {1u, 1u, 2u, 0u}) {
    pcl::Label p;
    p.label = l;
    inst.push_back(p);
  }
  db.save_point_cloud("instances", inst, "test", "{}");
  db.save_label_definitions("instances",
                            {{1, "SM2-1 (2p)"}, {2, "SM2-2 (1p)"}});

  const auto first = sync_survey_json(db, "{}");
  CHECK(first.at("instances_backfilled") == 2);
  CHECK(first.at("instances_seen") == 2);
  CHECK(first.at("types_created") == 1);
  CHECK(first.at("parts_created") == 2);

  const auto again = sync_survey_json(db, "{}");
  CHECK(again.at("instances_backfilled") == 0);
  CHECK(again.at("instances_seen") == 2);
  CHECK(again.at("parts_created") == 0);
  CHECK(again.at("parts_existing") == 2);
  CHECK(again.at("links_restored") == 0);

  // A `rux create instances` re-run drops the instance links; sync puts
  // back the one whose part owns a passport.
  std::string guid;
  {
    ProjectDB::Transaction tx(db);
    guid = db.ensure_resource_passport("RX-001");
    tx.commit();
  }
  db.save_instances("instances", db.instances("instances"));
  const auto restored = sync_survey_json(db, "{}");
  CHECK(restored.at("links_restored") == 1);
  CHECK(db.instance_material_guid("instances", 1) == guid);
}

namespace {
struct FakeRenderer : IViewRenderer {
  RenderRequest last;
  enum class Mode { ok, no_gl, missing_data } mode = Mode::ok;
  std::vector<std::uint8_t> render_png(const reusex::ProjectDB &,
                                       const RenderRequest &req) override {
    last = req;
    if (mode == Mode::no_gl)
      throw RenderUnavailable("no EGL");
    if (mode == Mode::missing_data)
      throw std::runtime_error(
          "needs the label cloud 'rooms' — run `rux create rooms` first");
    return {0x89, 'P', 'N', 'G'};
  }
};
Params
params_of(std::initializer_list<std::pair<const char *, const char *>> kv) {
  Params p;
  for (auto [k, v] : kv)
    p.set(k, v);
  return p;
}
} // namespace

TEST_CASE("RenderBlob_ParsesQuery_DefaultsHighlightCloud", "[gui][render]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  FakeRenderer r;
  const auto blob = render_blob(db, &r,
                                params_of({{"view", "orbit"},
                                           {"orbit_index", "3"},
                                           {"layers", "cloud,rooms"},
                                           {"highlight_instance", "12"}}));
  CHECK(blob.content_type == "image/png");
  CHECK(blob.data.size() == 4);
  CHECK(r.last.view == "orbit");
  CHECK(r.last.orbit_index == 3);
  CHECK(r.last.layers == std::vector<std::string>{"cloud", "rooms"});
  CHECK(r.last.highlight_cloud == "instances");
  CHECK(r.last.highlight_instance == 12u);
}

TEST_CASE("RenderBlob_StatusMapping", "[gui][render]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  FakeRenderer r;
  CHECK(status_of([&] { render_blob(db, nullptr, {}); }) == 503);
  r.mode = FakeRenderer::Mode::no_gl;
  CHECK(status_of([&] { render_blob(db, &r, {}); }) == 503);
  r.mode = FakeRenderer::Mode::missing_data;
  CHECK(status_of([&] { render_blob(db, &r, {}); }) == 422);
  r.mode = FakeRenderer::Mode::ok;
  CHECK(status_of([&] {
          render_blob(db, &r, params_of({{"view", "sideways"}}));
        }) == 400);
  CHECK(status_of([&] { render_blob(db, &r, params_of({{"width", "5"}})); }) ==
        400);
  CHECK(status_of([&] {
          render_blob(db, &r, params_of({{"orbit_index", "8"}}));
        }) == 400);
}

TEST_CASE("SurveyFractionsJson_ApprovedWithoutMass_BlocksAndNotReady",
          "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_type(db, "a", core::Treatment::genanvendelse, 380,
           core::ReviewStatus::approved);
  ProjectDB::SurveyTypeRecord t;
  t.name = "Uden tonnage";
  t.eak_code = "17.02.01";
  t.treatment = core::Treatment::genanvendelse;
  t.review_status = core::ReviewStatus::approved; // mass_t stays nullopt
  db.add_survey_type(t);
  const auto j = survey_fractions_json(db);
  REQUIRE(j.at("fractions").size() == 1);
  REQUIRE(j.at("blocking").size() == 1);
  CHECK(j.at("blocking").at(0).at("name") == "Uden tonnage");
  CHECK(j.at("blocking").at(0).at("reason") == "mass");
  CHECK(j.at("blocking").at(0).at("mass_t").is_null());
  CHECK(j.at("blocking_types") == 1);
  CHECK(j.at("ready") == false);
}

TEST_CASE("SurveyFractionsJson_TonnesRoundedOnTheWire", "[gui][survey]") {
  // Demo-seed-style sum: 190 + 6.8 + 2.4 is 199.20000000000002 in doubles.
  TempDB tmp;
  ProjectDB db(tmp.path);
  // One code, summed in input order, so the row and the total both carry it.
  add_type(db, "a", core::Treatment::genanvendelse, 190,
           core::ReviewStatus::approved);
  add_type(db, "b", core::Treatment::genanvendelse, 6.8,
           core::ReviewStatus::approved);
  add_type(db, "c", core::Treatment::genanvendelse, 2.4,
           core::ReviewStatus::approved);
  const auto j = survey_fractions_json(db);
  CHECK_THAT(j.at("total_t").get<double>(), WithinAbs(199.2, 1e-9));
  CHECK(j.at("total_t").dump() == "199.2");
  REQUIRE(j.at("fractions").size() == 1);
  CHECK_THAT(j.at("fractions").at(0).at("mass_t").get<double>(),
             WithinAbs(199.2, 1e-9));
  CHECK(j.at("fractions").at(0).at("mass_t").dump() == "199.2");
  CHECK(j.at("ready") == true);
}

TEST_CASE("SurveySummaryJson_ClassifiedShare_ContaminatedTypes",
          "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  reusex::CloudL labels;
  for (std::uint32_t l : {0u, 1u, 1u, 2u}) {
    pcl::Label p;
    p.label = l;
    labels.push_back(p);
  }
  db.save_point_cloud("instances", labels, "test", "{}");

  const auto walls =
      add_type(db, "Murvægge", core::Treatment::bortskaffelse, 38);
  const auto gone = add_type(db, "Afvist", core::Treatment::bortskaffelse, 1,
                             core::ReviewStatus::rejected);
  const auto lead = db.add_sample("Bly i maling", "");
  ProjectDB::SamplePatch answered;
  answered.stage = core::SampleStage::svar;
  answered.result = core::SampleResult::forurenet;
  db.update_sample(lead.id, answered);
  db.set_sample_links(lead.id, {walls, gone});

  const auto j = survey_summary_json(db);
  CHECK(j.at("classified_share").get<double>() == Approx(0.75));
  CHECK(j.at("unlabeled_points") == 1);
  CHECK(j.at("contaminated_types") == 1); // the rejected one never counts
}

TEST_CASE("SurveySummaryJson_NoInstanceCloud_ClassifiedShareNull",
          "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto j = survey_summary_json(db);
  CHECK(j.at("classified_share").is_null());
  CHECK(j.at("contaminated_types") == 0);
}

TEST_CASE("CreateSample_Refusals_WriteNothing_OnsiteFieldsIgnored",
          "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK(status_of([&] { create_sample_json(db, R"({"title":""})"); }) == 400);
  CHECK(status_of([&] { create_sample_json(db, R"({"what":"x"})"); }) == 400);
  CHECK(status_of([&] {
          create_sample_json(db, R"({"title":"x","type_ids":[9999]})");
        }) == 404);
  CHECK(db.samples().empty());
  // On-site's fields are gone: the route ignores them like any unknown key.
  const auto s = create_sample_json(
      db, R"({"title":"PCB","part_code":"RX-404","stage":"svar"})");
  CHECK(s.at("stage") == "planlagt");
  CHECK_FALSE(s.contains("part_code"));
  CHECK_FALSE(samples_json(db).at("samples").at(0).contains("part_code"));
}
