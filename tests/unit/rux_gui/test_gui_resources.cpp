// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// The resources HTTP handlers (spec §4.4, §6.3, §7): shapes and status
// codes over a real ProjectDB.

#include <catch2/catch_test_macros.hpp>

#include <gui/api.hpp>
#include <gui/resources.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>
#include <core/resource_keys.hpp>
#include <core/resource_templates.hpp>

#include <nlohmann/json.hpp>

#include <functional>
#include <string>

using namespace rux::gui;
using reusex::ProjectDB;
using json = nlohmann::json;
using namespace reusex::test_support;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_gui_resources") {}
};
int status_of(const std::function<void()> &f) {
  try {
    f();
  } catch (const HttpError &e) {
    return e.status();
  }
  return 200;
}
Params
params_of(std::initializer_list<std::pair<std::string, std::string>> kv) {
  Params p;
  for (const auto &[k, v] : kv)
    p.set(k, v);
  return p;
}
int64_t screening_id(const ProjectDB &db) {
  for (const auto &t : db.resource_templates())
    if (t.seed == std::optional<std::string>("screening"))
      return t.id;
  FAIL("no screening seed");
  return 0;
}
} // namespace

TEST_CASE("GuiResources_Keys_IsTheCatalogueArray", "[gui][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto keys = resource_keys_json(db);
  REQUIRE(keys.is_array());
  CHECK(keys.size() == reusex::core::key_catalogue(db).size());
  CHECK(keys[0] == json::parse(R"({"id":"sys:name","label":"Betegnelse",
        "category":"Kortlægning","scope":"type","data_type":"text",
        "unit":null,"options":[],"editable":true})"));
  CHECK(keys[6].at("editable") == false);
  CHECK(keys[8].at("unit") == "t");
  CHECK_FALSE(keys[0].contains("field"));
}

TEST_CASE("GuiResources_List_WithAndWithoutTemplate", "[gui][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  const auto all = resources_json(db, {});
  REQUIRE(all.at("resources").size() == 1);
  CHECK(all.at("resources")[0].at("manual") == true);
  CHECK_FALSE(all.contains("template"));
  const auto id = screening_id(db);
  const auto shaped =
      resources_json(db, params_of({{"template", std::to_string(id)}}));
  CHECK(shaped.at("template").at("id") == id);
  CHECK(shaped.at("template").at("resolved_keys").size() == 11);
  CHECK(shaped.at("template").at("missing") == json::array());
  const auto &values = shaped.at("resources")[0].at("values");
  CHECK(values.size() == 11);
  CHECK(values.at("sys:mass_t").is_null());
  CHECK(values.at("sys:name") == "Døre");
  CHECK(status_of([&] {
          resources_json(db, params_of({{"template", "999"}}));
        }) == 404);
  CHECK(status_of([&] {
          resources_json(db, params_of({{"template", "x"}}));
        }) == 400);
}

TEST_CASE("GuiResources_Patch_StatusesAndSiblings", "[gui][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  make_part(db, "RX-002", t);
  const auto r =
      patch_resource_json(db, "RX-001", {}, R"({"values":{"sys:unit":"m²"}})");
  CHECK(r.at("resource").at("values").at("sys:unit") == "m²");
  REQUIRE(r.at("siblings").size() == 1);
  CHECK(r.at("siblings")[0].at("code") == "RX-002");
  CHECK(status_of([&] {
          patch_resource_json(
              db, "RX-001", {},
              R"({"values":{"sys:environment":"ren_screening"}})");
        }) == 400);
  CHECK(status_of([&] {
          patch_resource_json(db, "RX-001", {}, R"({"values":{"sys:x":"1"}})");
        }) == 400);
  CHECK(status_of([&] {
          patch_resource_json(db, "RX-001", {}, R"({"values":{"sys:note":1}})");
        }) == 400);
  CHECK(status_of([&] { patch_resource_json(db, "RX-001", {}, R"({})"); }) ==
        400);
  CHECK(status_of([&] {
          patch_resource_json(db, "RX-404", {}, R"({"values":{}})");
        }) == 404);
  // A 400 names the key.
  try {
    patch_resource_json(db, "RX-001", {},
                        R"({"values":{"sys:treatment":"smid ud"}})");
    FAIL("an unknown treatment must be refused");
  } catch (const HttpError &e) {
    CHECK(std::string(e.what()).find("sys:treatment") != std::string::npos);
  }
}

TEST_CASE("GuiResources_CreateDelete", "[gui][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  const auto r = create_resource_json(db, R"({"type_id":)" + std::to_string(t) +
                                              R"(,"name":"Branddør"})");
  CHECK(r.at("code") == "RX-002");
  CHECK(r.at("manual") == true);
  CHECK(status_of([&] { create_resource_json(db, R"({"type_id":999})"); }) ==
        404);
  CHECK(status_of([&] { create_resource_json(db, R"({"name":"x"})"); }) == 400);
  CHECK(status_of([&] { delete_resource(db, "RX-001"); }) == 409);
  CHECK(status_of([&] { delete_resource(db, "RX-404"); }) == 404);
  delete_resource(db, "RX-002");
  CHECK_FALSE(db.survey_part("RX-002").has_value());
}

TEST_CASE("GuiResources_Csv_RequiresTemplate", "[gui][resources][csv]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "=Døre");
  make_part(db, "RX-001", t);
  const auto blob = resources_csv_blob(
      db, params_of({{"template", std::to_string(screening_id(db))}}));
  CHECK(blob.content_type == "text/csv; charset=utf-8");
  const std::string csv(blob.data.begin(), blob.data.end());
  CHECK(csv.rfind("\xEF\xBB\xBF"
                  "Betegnelse;Mængde;",
                  0) == 0);
  CHECK(csv.find("\r\n'=Døre;1;") != std::string::npos);
  CHECK(status_of([&] { resources_csv_blob(db, {}); }) == 400);
  CHECK(status_of([&] {
          resources_csv_blob(db, params_of({{"template", "999"}}));
        }) == 404);
}

TEST_CASE("GuiResources_Columns_ConflictIs409", "[gui][resources][columns]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a =
      create_material_column(db, R"({"name":"Stand","type":"text"})");
  CHECK(status_of([&] {
          create_material_column(db, R"({"name":"Stand","type":"text"})");
        }) == 409);
  const auto b =
      create_material_column(db, R"({"name":"Andet","type":"text"})");
  CHECK(status_of([&] {
          patch_material_column(db, b.at("id"), R"({"name":"Stand"})");
        }) == 409);
  CHECK(status_of([&] {
          patch_material_column(db, "nope", R"({"name":"x"})");
        }) == 404);
  CHECK(patch_material_column(db, a.at("id"), R"({"width":300})").at("width") ==
        300);
}

TEST_CASE("GuiTemplates_ListShape", "[gui][templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto body = templates_json(db);
  REQUIRE(body.at("templates").size() == 2);
  const auto &s = body.at("templates")[1];
  CHECK(s.at("name") == "Hurtig genbrugsscreening");
  CHECK(s.at("seed") == "screening");
  CHECK(s.at("members").size() == 11);
  CHECK(s.at("members")[0] == json::parse(R"({"key":"sys:name"})"));
  CHECK(s.at("resolved_keys").size() == 11);
  CHECK(s.at("missing") == json::array());
  CHECK(s.at("csv") == json::parse(R"({"delimiter":";","encoding":"utf-8-bom",
        "header":"label"})"));
  CHECK(body.at("templates")[0].at("seed") == "materialepas");
}

TEST_CASE("GuiTemplates_CrudAndStatuses", "[gui][templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = create_template_json(
      db, R"({"name":"Mit","members":[{"key":"sys:note"},{"key":"col:x"}]})");
  const int64_t id = t.at("id");
  CHECK(t.at("seed").is_null());
  CHECK(t.at("missing") == json::parse(R"([{"key":"col:x"}])"));
  CHECK(status_of([&] { create_template_json(db, R"({"name":"Mit"})"); }) ==
        409);
  CHECK(status_of([&] { create_template_json(db, R"({})"); }) == 400);
  CHECK(status_of([&] {
          create_template_json(db, R"({"name":"X","members":{}})");
        }) == 400);
  CHECK(status_of([&] {
          create_template_json(db, R"({"name":"X","csv":{"delimiter":"|"}})");
        }) == 400);
  const auto p = patch_template_json(db, id, R"({"csv":{"header":"key"}})");
  CHECK(p.at("csv").at("header") == "key");
  CHECK(p.at("name") == "Mit");
  CHECK(status_of([&] { patch_template_json(db, 999, R"({"name":"Y"})"); }) ==
        404);
  CHECK(status_of([&] {
          patch_template_json(db, id, R"({"name":"Hurtig genbrugsscreening"})");
        }) == 409);
  CHECK(duplicate_template_json(db, id).at("name") == "Mit (kopi)");
  CHECK(status_of([&] { duplicate_template_json(db, 999); }) == 404);
  delete_template(db, id);
  CHECK(status_of([&] { delete_template(db, id); }) == 404);
}

TEST_CASE("GuiTemplates_RestoreSeeds", "[gui][templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  delete_template(db, screening_id(db));
  const auto r = restore_seed_templates_json(db);
  CHECK(r.at("restored") == json::parse(R"(["Hurtig genbrugsscreening"])"));
  CHECK(r.at("templates").size() == 2);
  CHECK(restore_seed_templates_json(db).at("restored") == json::array());
}
