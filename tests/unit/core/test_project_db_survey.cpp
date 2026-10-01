// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v22: survey_types / survey_parts round trips, patches, the
// instance-deleted SET NULL rule, and the v21 -> v22 migration.

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <stdexcept>
#include <string>

using reusex::ProjectDB;
namespace core = reusex::core;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_survey") {}
};

/// A 3-point instance cloud with instance 1 on two points and instance 2 on
/// one.
void add_instance_cloud(ProjectDB &db) {
  reusex::CloudL labels;
  for (std::uint32_t l : {1u, 1u, 2u}) {
    pcl::Label p;
    p.label = l;
    labels.push_back(p);
  }
  db.save_point_cloud("instances", labels, "test", "{}");
  db.save_instances("instances",
                    {{1, "guid-inst-1", 3, 2}, {2, "guid-inst-2", 5, 1}});
}

ProjectDB::SurveyTypeRecord window_type() {
  ProjectDB::SurveyTypeRecord t;
  t.name = "Vinduespartier, aluminium";
  t.eak_code = "17.04.02";
  t.bim7aa_code = "312 Udv. vinduer";
  t.treatment = core::Treatment::genbrug;
  t.confidence = 0.82;
  t.mass_t = 3.1;
  t.note = "Ved ren fuge: salg som brugte partier.";
  t.semantic_class = 3;
  return t;
}
} // namespace

TEST_CASE("SurveyTypes_AddListGet_RoundTrip", "[ProjectDB][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto added = db.add_survey_type(window_type());
  REQUIRE(added.id > 0);
  CHECK(!added.created_at.empty());
  const auto all = db.survey_types();
  REQUIRE(all.size() == 1);
  const auto &t = all.front();
  CHECK(t.name == "Vinduespartier, aluminium");
  CHECK(t.treatment == core::Treatment::genbrug);
  CHECK(t.review_status == core::ReviewStatus::queue);
  CHECK(t.confidence == 0.82);
  CHECK(t.mass_t == 3.1);
  CHECK(t.unit == "stk");
  CHECK(t.semantic_class == 3);
  CHECK(db.survey_type(added.id).has_value());
  CHECK_FALSE(db.survey_type(added.id + 100).has_value());
}

TEST_CASE("SurveyTypes_Patch_ChangesOnlyEngagedFields_AndClearsNullables",
          "[ProjectDB][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = db.add_survey_type(window_type()).id;
  ProjectDB::SurveyTypePatch p;
  p.review_status = core::ReviewStatus::approved;
  p.mass_t = std::optional<double>{}; // clear
  p.starred = true;
  const auto t = db.update_survey_type(id, p);
  CHECK(t.review_status == core::ReviewStatus::approved);
  CHECK_FALSE(t.mass_t.has_value());
  CHECK(t.starred);
  CHECK(t.name == "Vinduespartier, aluminium"); // untouched
  CHECK(t.confidence == 0.82);
  CHECK_THROWS_AS(db.update_survey_type(id + 100, p), std::out_of_range);
}

TEST_CASE("SurveyParts_AddList_JoinsMaterialGuid_AndCodeOrder",
          "[ProjectDB][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_instance_cloud(db);
  const auto type_id = db.add_survey_type(window_type()).id;
  db.add_survey_part({"RX-009",
                      type_id,
                      "instances",
                      2,
                      7,
                      "Production Hall",
                      12,
                      false,
                      "",
                      {}});
  db.add_survey_part({"RX-008",
                      type_id,
                      "instances",
                      1,
                      4,
                      "Office Zone",
                      26,
                      true,
                      "note",
                      {}});
  const auto parts = db.survey_parts();
  REQUIRE(parts.size() == 2);
  CHECK(parts[0].code == "RX-008");
  CHECK(parts[0].cloud_name == "instances");
  CHECK(parts[0].instance_id == 1u);
  CHECK(parts[0].room_name == "Office Zone");
  CHECK(parts[0].quantity == 26);
  CHECK(parts[0].starred);
  CHECK_FALSE(parts[0].material_guid.has_value()); // no passport linked yet
  CHECK(db.has_survey_part_for("instances", 2));
  CHECK_FALSE(db.has_survey_part_for("instances", 9));
  CHECK(db.max_survey_part_number() == 9);
}

TEST_CASE("SurveyParts_Patch_MovesToOtherType_RejectsUnknownType",
          "[ProjectDB][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = db.add_survey_type(window_type()).id;
  auto other = window_type();
  other.name = "Indvendige døre, træ";
  const auto b = db.add_survey_type(other).id;
  db.add_survey_part({"RX-001",
                      a,
                      std::nullopt,
                      std::nullopt,
                      std::nullopt,
                      "",
                      1,
                      false,
                      "",
                      {}});
  ProjectDB::SurveyPartPatch p;
  p.type_id = b;
  p.quantity = 14;
  const auto part = db.update_survey_part("RX-001", p);
  CHECK(part.type_id == b);
  CHECK(part.quantity == 14);
  p.type_id = b + 100;
  CHECK_THROWS_AS(db.update_survey_part("RX-001", p), std::out_of_range);
  CHECK_THROWS_AS(db.update_survey_part("RX-404", {}), std::out_of_range);
}

TEST_CASE("SurveyParts_InstanceDeleted_KeepsPartWithNullInstance",
          "[ProjectDB][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_instance_cloud(db);
  const auto type_id = db.add_survey_type(window_type()).id;
  db.add_survey_part(
      {"RX-001", type_id, "instances", 1, std::nullopt, "", 5, false, "", {}});
  db.delete_point_cloud("instances"); // cascades to instances(cloud_id, …)
  const auto part = db.survey_part("RX-001");
  REQUIRE(part.has_value());
  CHECK(part->quantity == 5);
  CHECK_FALSE(part->cloud_name.has_value());
  CHECK_FALSE(part->instance_id.has_value());
}

TEST_CASE("SurveySchema_MigratesFromV21", "[ProjectDB][survey][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
  } // fresh DB at the latest version
  {
    // Roll it back to v21 by removing everything v22 adds.
    sqlite3 *raw = nullptr;
    REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
    const char *sql =
        "DROP TABLE sample_links; DROP TABLE samples; DROP TABLE survey_parts;"
        "DROP TABLE survey_types; DELETE FROM schema_version WHERE version = "
        "22;";
    REQUIRE(sqlite3_exec(raw, sql, nullptr, nullptr, nullptr) == SQLITE_OK);
    sqlite3_close(raw);
  }
  ProjectDB db(tmp.path, /*readOnly=*/false);
  CHECK(db.survey_types().empty());
  CHECK(db.add_survey_type(window_type()).id > 0);
}
