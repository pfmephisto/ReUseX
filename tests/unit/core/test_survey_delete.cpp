// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Reject != delete (Kortlægning fixes spec A3): deleting any survey part or a
// whole type, with tombstones that keep sync_survey from re-creating deleted
// scan-backed parts, and the passport rule (deleted unless linked elsewhere).

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resources.hpp>
#include <core/survey_service.hpp>

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
  TempDB() : reusex::test_support::TempPath("test_survey_delete") {}
};
bool has_passport(const ProjectDB &db, const std::string &guid) {
  const auto g = db.list_passport_guids();
  return std::find(g.begin(), g.end(), guid) != g.end();
}
std::vector<std::string> codes(const ProjectDB &db) {
  std::vector<std::string> out;
  for (const auto &p : db.survey_parts())
    out.push_back(p.code);
  return out;
}
} // namespace

TEST_CASE("DeleteResource_InstanceBacked_TombstonedAndSyncSkipsIt",
          "[survey][delete]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 3);
  REQUIRE(sync_survey(db).parts_created == 3);
  const auto rx2 = db.survey_part("RX-002");
  REQUIRE(rx2);
  REQUIRE(rx2->instance_guid == std::optional<std::string>("guid-inst-2"));
  const auto guid = db.ensure_resource_passport("RX-002"); // outside a tx: ok
  REQUIRE(db.instance_material_guid("instances", 2) == guid);

  delete_resource(db, "RX-002");
  CHECK_FALSE(db.survey_part("RX-002").has_value());
  CHECK(db.is_instance_dismissed("guid-inst-2"));
  // Its own instance link went with it, so nothing links the passport.
  CHECK_FALSE(db.instance_material_guid("instances", 2).has_value());
  CHECK_FALSE(has_passport(db, guid));

  const auto r = sync_survey(db);
  CHECK(r.parts_created == 0);
  CHECK(r.parts_existing == 2);
  CHECK(r.parts_dismissed == 1);
  CHECK(r.instances_seen == 3);
  CHECK(codes(db) == std::vector<std::string>{"RX-001", "RX-003"});
}

TEST_CASE("DeleteResource_Manual_PassportDeletedUnlessShared",
          "[survey][delete]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  const auto a = create_resource(db, t, std::string("Branddør"));
  const auto b = create_resource(db, t, std::string("Glasdør"));
  const auto ga = *db.survey_part(a.code)->material_guid;
  const auto gb = *db.survey_part(b.code)->material_guid;
  db.set_instance_material("instances", 1, gb); // b's passport shared

  delete_resource(db, a.code);
  CHECK_FALSE(has_passport(db, ga));
  delete_resource(db, b.code);
  CHECK(has_passport(db, gb));
  CHECK(db.instance_material_guid("instances", 1) == gb);
  CHECK(db.dismissed_instances().empty()); // manual parts leave no tombstone
}

TEST_CASE("DeleteResource_InstanceBacked_KeepsForeignInstanceLink",
          "[survey][delete]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 2);
  sync_survey(db);
  const auto g1 = db.ensure_resource_passport("RX-001");
  // Instance 2 is (also) linked to RX-001's passport — not RX-002's own.
  db.set_instance_material("instances", 2, g1);
  // RX-002 has no own passport; its material_guid falls back to the
  // instance link, which another part owns — so it reads empty.
  CHECK_FALSE(db.survey_part("RX-002")->material_guid.has_value());
  delete_resource(db, "RX-002");
  CHECK(db.instance_material_guid("instances", 2) == g1);
  CHECK(has_passport(db, g1));
  CHECK(db.is_instance_dismissed("guid-inst-2"));
}

TEST_CASE("DeleteSurveyType_CascadesPartsAndTombstones", "[survey][delete]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 3);
  sync_survey(db); // one type (class 3), RX-001..RX-003
  const auto types = db.survey_types();
  REQUIRE(types.size() == 1);
  const auto t = types[0].id;
  const auto manual = create_resource(db, t, std::string("Ekstra"));
  const auto gm = *db.survey_part(manual.code)->material_guid;
  const auto g1 = db.ensure_resource_passport("RX-001");
  const auto other = make_type(db, "Vinduer");
  make_part(db, "RX-100", other);
  const auto s = db.add_sample("PCB", "");
  db.set_sample_links(s.id, {t, other});

  const auto r = delete_survey_type(db, t);
  CHECK(r.parts_deleted == 4);
  CHECK(r.instances_dismissed == 3);
  CHECK_FALSE(db.survey_type(t).has_value());
  CHECK(codes(db) == std::vector<std::string>{"RX-100"});
  CHECK(db.dismissed_instances() ==
        std::vector<std::string>{"guid-inst-1", "guid-inst-2", "guid-inst-3"});
  CHECK_FALSE(has_passport(db, gm));
  CHECK_FALSE(has_passport(db, g1));
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{other});

  // A re-sync neither re-creates the type nor its parts.
  const auto again = sync_survey(db);
  CHECK(again.types_created == 0);
  CHECK(again.parts_created == 0);
  CHECK(again.parts_dismissed == 3);
  CHECK(db.survey_types().size() == 1);

  CHECK_THROWS_AS(delete_survey_type(db, t), std::out_of_range);
}
