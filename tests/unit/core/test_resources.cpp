// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Resources (spec §4): values by key id, writes routed by scope (type,
// part, passport), lazy passports, add/delete, and user columns whose
// rename carries their stored values.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resource_keys.hpp>
#include <core/resources.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <algorithm>
#include <string>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::core;
using namespace reusex::test_support;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_resources") {}
};
std::string lex(const char *field) {
  for (const auto &f : leksikon_fields())
    if (f.field_name == field)
      return "lex:" + f.guid;
  FAIL("no leksikon field " << field);
  return {};
}
} // namespace

TEST_CASE("Resources_List_NoTemplate_BuiltinsPlusStored", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-002", t, std::nullopt, 2.5);
  make_part(db, "RX-001", t);
  const auto list = list_resources(db);
  REQUIRE(list.size() == 2);
  CHECK(list[0].code == "RX-001");
  CHECK(list[0].manual);
  CHECK(list[0].values.size() == builtin_keys().size());
  CHECK(value_of(list[1], "sys:quantity") == "2.5");
  CHECK(value_of(list[1], "sys:name") == "Døre");
  CHECK(value_of(list[1], "sys:treatment") == "genbrug");
  CHECK(value_of(list[1], "sys:environment") == "ren_screening");
  CHECK(value_of(list[1], "sys:starred") == "false");
  CHECK_FALSE(value_of(list[1], "sys:mass_t").has_value());
  patch_resource(db, "RX-001", {{lex("width_mm"), "600"}});
  const auto r = resource(db, "RX-001");
  CHECK(r.values.size() == builtin_keys().size() + 1);
  CHECK(value_of(r, lex("width_mm")) == "600");
}

TEST_CASE("Resources_List_WithKeys_OrderAndNullForMissing", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  const auto r = resource(
      db, "RX-001",
      std::vector<std::string>{lex("width_mm"), "sys:name", "col:gone"});
  REQUIRE(r.values.size() == 3);
  CHECK(r.values[0].key == lex("width_mm"));
  CHECK_FALSE(r.values[0].value.has_value());
  CHECK(r.values[1].value == "Døre");
  CHECK_FALSE(r.values[2].value.has_value());
  CHECK_THROWS_AS(resource(db, "RX-404"), std::out_of_range);
}

TEST_CASE("Resources_Patch_RoutesByScope_ReturnsSiblings", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  make_part(db, "RX-002", t);
  const auto res = patch_resource(db, "RX-001",
                                  {{"sys:eak", "17.02.01"},
                                   {"sys:mass_t", "0,9"},
                                   {"sys:note", "ved trappen"},
                                   {"sys:starred", "true"},
                                   {"sys:quantity", "4"}});
  CHECK(db.survey_type(t)->eak_code == "17.02.01");
  CHECK(db.survey_type(t)->mass_t == 0.9);
  CHECK(db.survey_part("RX-001")->note == "ved trappen");
  CHECK(db.survey_part("RX-001")->starred);
  CHECK(db.survey_part("RX-001")->quantity == 4.0);
  CHECK(db.survey_part("RX-002")->note.empty());
  REQUIRE(res.siblings.size() == 1);
  CHECK(res.siblings[0].code == "RX-002");
  CHECK(value_of(res.siblings[0], "sys:eak") == "17.02.01");
  // Part-only writes have no siblings to refresh.
  CHECK(
      patch_resource(db, "RX-001", {{"sys:room", "Kælder"}}).siblings.empty());
  // Clearing: mass_t to NULL, eak to "".
  patch_resource(db, "RX-001", {{"sys:mass_t", std::nullopt}, {"sys:eak", ""}});
  CHECK_FALSE(db.survey_type(t)->mass_t.has_value());
  CHECK(db.survey_type(t)->eak_code.empty());
  // A null clear of a NOT NULL text column writes "".
  patch_resource(db, "RX-001",
                 {{"sys:note", std::nullopt},
                  {"sys:room", std::nullopt},
                  {"sys:bim7aa", std::nullopt},
                  {"sys:eak", std::nullopt}});
  CHECK(db.survey_part("RX-001")->note.empty());
  CHECK(db.survey_part("RX-001")->room_name.empty());
  CHECK(db.survey_type(t)->bim7aa_code.empty());
  CHECK(value_of(resource(db, "RX-001"), "sys:note") == "");
}

TEST_CASE("Resources_Patch_LazyPassport_InstanceLinkFollows", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  make_part(db, "RX-002", t);
  // Clearing an absent value succeeds and creates no passport.
  patch_resource(db, "RX-002", {{lex("width_mm"), std::nullopt}});
  CHECK_FALSE(db.survey_part("RX-002")->material_guid.has_value());
  patch_resource(db, "RX-001", {{lex("width_mm"), "600"}});
  const auto guid = db.survey_part("RX-001")->material_guid;
  REQUIRE(guid.has_value());
  CHECK(db.instance_material_guid("instances", 1) == guid);
  patch_resource(db, "RX-001", {{lex("width_mm"), std::nullopt}});
  CHECK(db.passport_stored_properties(*guid).count("width_mm") == 0);
}

TEST_CASE("Resources_Patch_ValidatesAllBeforeWriting", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  for (const std::vector<ResourceWrite> &w :
       std::vector<std::vector<ResourceWrite>>{
           {{"sys:note", "x"}, {"sys:environment", "forurenet"}},
           {{"sys:note", "x"}, {"sys:nope", "1"}},
           {{"sys:note", "x"}, {"sys:treatment", "smid ud"}},
           {{"sys:note", "x"}, {"sys:quantity", "mange"}}}) {
    CHECK_THROWS_AS(patch_resource(db, "RX-001", w), KeyValueError);
  }
  CHECK(db.survey_part("RX-001")->note.empty());
  CHECK_THROWS_AS(patch_resource(db, "RX-404", {{"sys:note", "x"}}),
                  std::out_of_range);
}

TEST_CASE("Resources_CreateAndDelete", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  const auto r = create_resource(db, t, std::string("Branddør"));
  CHECK(r.code == "RX-002");
  CHECK(r.manual);
  CHECK(value_of(r, lex("designation")) == "Branddør");
  const auto guid = db.survey_part("RX-002")->material_guid;
  REQUIRE(guid.has_value());
  CHECK(create_resource(db, t).code == "RX-003");
  CHECK_THROWS_AS(create_resource(db, 999), std::out_of_range);
  CHECK_THROWS_AS(create_resource(db, t, std::string("")),
                  std::invalid_argument);
  delete_resource(db, "RX-002");
  CHECK_FALSE(db.survey_part("RX-002").has_value());
  const auto guids = db.list_passport_guids();
  CHECK(std::find(guids.begin(), guids.end(), *guid) == guids.end());
  CHECK_THROWS_AS(delete_resource(db, "RX-001"), ResourceConflictError);
  CHECK_THROWS_AS(delete_resource(db, "RX-404"), std::out_of_range);
}

TEST_CASE("Resources_RenameColumn_CarriesValues", "[resources][columns]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  ProjectDB::PropertyDefinition def;
  def.name = "Stand";
  def.type = "select";
  def.options = {"God", "Dårlig"};
  const auto col = create_column(db, def);
  CHECK_FALSE(col.id.empty());
  patch_resource(db, "RX-001", {{"col:" + col.id, "God"}});
  ColumnPatch p;
  p.name = "Tilstand";
  CHECK(update_column(db, col.id, p).name == "Tilstand");
  CHECK(value_of(resource(db, "RX-001"), "col:" + col.id) == "God");
  ProjectDB::PropertyDefinition dup;
  dup.name = "Tilstand";
  dup.type = "text";
  CHECK_THROWS_AS(create_column(db, dup), reusex::core::NameConflictError);
  ProjectDB::PropertyDefinition lexname;
  lexname.name = "width_mm";
  lexname.type = "text";
  CHECK_THROWS_AS(create_column(db, lexname), reusex::core::NameConflictError);
  CHECK_THROWS_AS(update_column(db, "nope", p), std::out_of_range);
}

TEST_CASE("Resources_RenameColumn_RefusesNameWithStoredValues",
          "[resources][columns]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  ProjectDB::PropertyDefinition a;
  a.name = "Gammel";
  a.type = "text";
  const auto old_col = create_column(db, a);
  patch_resource(db, "RX-001", {{"col:" + old_col.id, "rest"}});
  db.delete_property_definition(old_col.id); // values stay, column gone
  ProjectDB::PropertyDefinition b;
  b.name = "Ny";
  b.type = "text";
  const auto new_col = create_column(db, b);
  patch_resource(db, "RX-001", {{"col:" + new_col.id, "frisk"}});
  ColumnPatch p;
  p.name = "Gammel";
  CHECK_THROWS_AS(update_column(db, new_col.id, p),
                  reusex::core::NameConflictError);
  CHECK(value_of(resource(db, "RX-001"), "col:" + new_col.id) == "frisk");
  p.name = "Ny"; // same name: no-op
  CHECK(update_column(db, new_col.id, p).name == "Ny");
  p.name = "Ny 2";
  update_column(db, new_col.id, p);
  p.name = "Ny"; // and back again
  update_column(db, new_col.id, p);
  CHECK(value_of(resource(db, "RX-001"), "col:" + new_col.id) == "frisk");
}
