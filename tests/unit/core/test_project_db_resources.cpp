// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Resource storage (schema v25): transactions, the lazy per-part passport
// and its instance_materials link, part delete, stored-field rename, and
// set_instance_material keeping a part's passport in step.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resource_keys.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <optional>
#include <string>

using reusex::ProjectDB;
namespace core = reusex::core;
using namespace reusex::test_support;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_resources") {}
};
std::string guid_of_field(const char *field) {
  for (const auto &f : core::leksikon_fields())
    if (f.field_name == field)
      return f.guid;
  FAIL("no leksikon field " << field);
  return {};
}
} // namespace

TEST_CASE("ResourceStore_Transaction_CommitsOrRollsBack",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  ProjectDB::SurveyPartPatch p;
  p.note = "rolled back";
  {
    ProjectDB::Transaction tx(db);
    db.update_survey_part("RX-001", p);
  }
  CHECK(db.survey_part("RX-001")->note.empty());
  p.note = "kept";
  {
    ProjectDB::Transaction tx(db);
    db.update_survey_part("RX-001", p);
    tx.commit();
  }
  CHECK(db.survey_part("RX-001")->note == "kept");
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK_THROWS_AS(ProjectDB::Transaction(ro), std::runtime_error);
}

TEST_CASE("ResourceStore_EnsurePassport_ManualPart_OnceAndLeksikonReady",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  std::string guid;
  {
    ProjectDB::Transaction tx(db);
    guid = db.ensure_resource_passport("RX-001");
    CHECK(db.ensure_resource_passport("RX-001") == guid);
    // A leksikon field lands under its leksikon definition, not "custom:".
    db.set_passport_property(guid, "width_mm", "600");
    tx.commit();
  }
  CHECK(db.survey_part("RX-001")->material_guid == guid);
  CHECK(db.passport_stored_properties(guid).at("width_mm") == "600");
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  sqlite3_prepare_v2(raw, "SELECT property_id FROM passport_property_values;",
                     -1, &s, nullptr);
  REQUIRE(sqlite3_step(s) == SQLITE_ROW);
  CHECK(std::string(reinterpret_cast<const char *>(
            sqlite3_column_text(s, 0))) == guid_of_field("width_mm"));
  sqlite3_finalize(s);
  sqlite3_close(raw);
  CHECK_THROWS_AS(db.ensure_resource_passport("RX-404"), std::out_of_range);
}

TEST_CASE("ResourceStore_EnsurePassport_InstancePart_UpsertsLink",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 2);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  std::string guid;
  {
    ProjectDB::Transaction tx(db);
    guid = db.ensure_resource_passport("RX-001");
    tx.commit();
  }
  CHECK(db.instance_material_guid("instances", 1) == guid);
  CHECK(db.is_passport_linked(guid));
}

TEST_CASE("ResourceStore_EnsurePassport_AdoptsUnownedInstanceLink",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  make_passport(db, "guid-cli");
  // Link first, part after: set_instance_material's sync has no part to
  // move yet, so passport_guid stays NULL and the adopt branch is reached.
  db.set_instance_material("instances", 1, "guid-cli");
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  CHECK(db.survey_part("RX-001")->material_guid ==
        std::optional<std::string>("guid-cli")); // fallback read
  ProjectDB::Transaction tx(db);
  CHECK(db.ensure_resource_passport("RX-001") == "guid-cli");
  tx.commit();
  CHECK(db.instance_material_guid("instances", 1) ==
        std::optional<std::string>("guid-cli"));
}

TEST_CASE("ResourceStore_InstanceLinkOwnedByOtherPart_ReadsEmptyThenOwnCopy",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 2);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  make_part(db, "RX-002", t, 2u);
  std::string a;
  {
    ProjectDB::Transaction tx(db);
    a = db.ensure_resource_passport("RX-001");
    tx.commit();
  }
  // Instance 2 is pointed at RX-001's passport; RX-002 has none, so the
  // sync cannot give it one and the fallback must not either (no two parts
  // writing through one passport).
  db.set_instance_material("instances", 2, a);
  CHECK(db.instance_material_guid("instances", 2) ==
        std::optional<std::string>(a));
  CHECK_FALSE(db.survey_part("RX-002")->material_guid.has_value());
  for (const auto &p : db.survey_parts())
    if (p.code == "RX-002")
      CHECK_FALSE(p.material_guid.has_value());
  std::string b;
  {
    ProjectDB::Transaction tx(db);
    b = db.ensure_resource_passport("RX-002");
    tx.commit();
  }
  CHECK(b != a);
  CHECK(db.survey_part("RX-002")->material_guid ==
        std::optional<std::string>(b));
  CHECK(db.survey_part("RX-001")->material_guid ==
        std::optional<std::string>(a));
  CHECK(db.instance_material_guid("instances", 2) ==
        std::optional<std::string>(b));
}

TEST_CASE("ResourceStore_SetInstanceMaterial_MovesPartPassportUnlessOwned",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 2);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  make_part(db, "RX-002", t, 2u);
  std::string a, c;
  {
    ProjectDB::Transaction tx(db);
    a = db.ensure_resource_passport("RX-001");
    c = db.ensure_resource_passport("RX-002");
    tx.commit();
  }
  make_passport(db, "guid-b");
  db.set_instance_material("instances", 1, "guid-b");
  CHECK(db.survey_part("RX-001")->material_guid == "guid-b");
  // guid-b now belongs to RX-001: linking it to instance 2 cannot move
  // RX-002's passport (warn), so RX-002 keeps its own.
  db.set_instance_material("instances", 2, "guid-b");
  CHECK(db.survey_part("RX-002")->material_guid == c);
  CHECK(db.instance_material_guid("instances", 2) == "guid-b");
}

TEST_CASE("ResourceStore_DeletePart_AndRenameField", "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  std::string guid;
  {
    ProjectDB::Transaction tx(db);
    guid = db.ensure_resource_passport("RX-001");
    db.set_passport_property(guid, "Stand", "God");
    db.set_passport_property(guid, "Gammel", "x");
    tx.commit();
  }
  {
    ProjectDB::Transaction tx(db);
    db.rename_passport_field("Stand", "Tilstand");
    tx.commit();
  }
  const auto props = db.passport_stored_properties(guid);
  CHECK(props.at("Tilstand") == "God");
  CHECK(props.count("Stand") == 0);
  // The old name is free again: a new value under it does not collide.
  db.set_passport_property(guid, "Stand", "Ny");
  CHECK(db.passport_stored_properties(guid).at("Stand") == "Ny");
  CHECK_THROWS_AS(db.rename_passport_field("Gammel", "Tilstand"),
                  core::NameConflictError);
  db.rename_passport_field("Findes ikke", "Andet"); // nothing stored: no-op
  db.delete_survey_part("RX-001");
  CHECK_FALSE(db.survey_part("RX-001").has_value());
  CHECK_FALSE(db.is_passport_linked(guid));
  CHECK_THROWS_AS(db.delete_survey_part("RX-001"), std::out_of_range);
}

TEST_CASE("ResourceStore_RenameField_OntoDefinitionWithoutValues",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  // A definition with no stored values does not block the rename (only
  // stored values would merge); the values join that definition.
  db.add_property_definition("Bredde", "number", {}, 0);
  std::string guid;
  {
    ProjectDB::Transaction tx(db);
    guid = db.ensure_resource_passport("RX-001");
    db.set_passport_property(guid, "Gammel", "x");
    tx.commit();
  }
  db.rename_passport_field("Gammel", "Bredde");
  const auto props = db.passport_stored_properties(guid);
  CHECK(props.at("Bredde") == "x");
  CHECK(props.count("Gammel") == 0);
}
