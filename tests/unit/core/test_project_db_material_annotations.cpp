// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <types.hpp>

#include "../../support/temp_path.hpp"

#include <string>
#include <vector>

using namespace reusex;
using reusex::core::MaterialPassport;

namespace {

struct TempDB : reusex::test_support::TempPath {
  TempDB() : TempPath("test_projectdb_matannot") {}
};

CloudLPtr makeInstanceCloud(size_t n) {
  auto cloud = std::make_shared<CloudL>();
  cloud->width = static_cast<uint32_t>(n);
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(n);
  for (size_t i = 0; i < n; ++i)
    cloud->points[i].label = static_cast<uint32_t>(i + 1);
  return cloud;
}

void makeInstances(ProjectDB &db, const std::string &cloud, size_t n) {
  std::vector<ProjectDB::InstanceRecord> recs;
  for (size_t i = 1; i <= n; ++i)
    recs.push_back(
        {static_cast<uint32_t>(i), "inst-guid-" + std::to_string(i), 1, 10});
  db.save_instances(cloud, recs);
}

void makePassport(ProjectDB &db, const std::string &guid) {
  MaterialPassport p;
  p.metadata.document_guid = guid;
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  db.add_material_passport(p, "test-project");
}

} // namespace

TEST_CASE("SaveMaterialAnnotation_DescriptionAndKv_RoundTrips",
          "[projectdb][material_annotations]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(2), "segment_instances");
  makeInstances(db, "instances", 2);
  makePassport(db, "mat-A");

  REQUIRE_FALSE(db.material_annotation("mat-A").has_value());

  ProjectDB::MaterialAnnotation ann;
  ann.description = "a weathered oak beam";
  ann.attributes = {
      {"material", "wood"}, {"colour", "brown"}, {"condition", "weathered"}};
  ann.provider_model = "http://localhost:11434/v1|qwen2.5vl";
  ann.raw_json = R"({"description":"a weathered oak beam"})";
  db.save_material_annotation("mat-A", ann);

  auto got = db.material_annotation("mat-A");
  REQUIRE(got.has_value());
  CHECK(got->description == "a weathered oak beam");
  CHECK(got->provider_model == "http://localhost:11434/v1|qwen2.5vl");
  CHECK(got->raw_json == R"({"description":"a weathered oak beam"})");

  // kv pairs come back key-sorted (deterministic).
  REQUIRE(got->attributes.size() == 3);
  CHECK(got->attributes[0] ==
        std::make_pair(std::string("colour"), std::string("brown")));
  CHECK(got->attributes[1] ==
        std::make_pair(std::string("condition"), std::string("weathered")));
  CHECK(got->attributes[2] ==
        std::make_pair(std::string("material"), std::string("wood")));
}

TEST_CASE("SaveMaterialAnnotation_DescriptionOnly_RoundTrips",
          "[projectdb][material_annotations]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(1), "segment_instances");
  makeInstances(db, "instances", 1);
  makePassport(db, "mat-desc");

  ProjectDB::MaterialAnnotation ann;
  ann.description = "just text";
  db.save_material_annotation("mat-desc", ann);

  auto got = db.material_annotation("mat-desc");
  REQUIRE(got.has_value());
  CHECK(got->description == "just text");
  CHECK(got->attributes.empty());
}

TEST_CASE("SaveMaterialAnnotation_Upsert_ReplacesKvRowsNoStaleKeys",
          "[projectdb][material_annotations]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(1), "segment_instances");
  makeInstances(db, "instances", 1);
  makePassport(db, "mat-U");

  ProjectDB::MaterialAnnotation a1;
  a1.description = "first";
  a1.attributes = {{"material", "wood"}, {"stale", "gone"}};
  db.save_material_annotation("mat-U", a1);

  ProjectDB::MaterialAnnotation a2;
  a2.description = "second";
  a2.attributes = {{"material", "metal"}, {"finish", "matte"}};
  db.save_material_annotation("mat-U", a2);

  auto got = db.material_annotation("mat-U");
  REQUIRE(got.has_value());
  CHECK(got->description == "second");
  // The stale key from the first save must be gone; only the second set's keys
  // remain, and "material" holds the new value.
  REQUIRE(got->attributes.size() == 2);
  bool has_stale = false, has_finish = false;
  std::string material_val;
  for (const auto &[k, v] : got->attributes) {
    if (k == "stale")
      has_stale = true;
    if (k == "finish")
      has_finish = true;
    if (k == "material")
      material_val = v;
  }
  CHECK_FALSE(has_stale);
  CHECK(has_finish);
  CHECK(material_val == "metal");
}

TEST_CASE("SaveMaterialAnnotation_NonexistentGuid_Throws",
          "[projectdb][material_annotations][integrity]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  // No passport created.
  ProjectDB::MaterialAnnotation ann;
  ann.description = "x";
  REQUIRE_THROWS_AS(db.save_material_annotation("ghost-guid", ann),
                    std::runtime_error);
}

TEST_CASE("DeleteMaterialPassport_CascadesAwayAnnotationAndKv",
          "[projectdb][material_annotations][integrity]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(1), "segment_instances");
  makeInstances(db, "instances", 1);
  makePassport(db, "mat-D");

  ProjectDB::MaterialAnnotation ann;
  ann.description = "to be deleted";
  ann.attributes = {{"material", "glass"}, {"colour", "clear"}};
  db.save_material_annotation("mat-D", ann);
  REQUIRE(db.material_annotation("mat-D").has_value());

  // Deleting the owning passport must cascade the annotation AND its kv rows.
  db.delete_material_passport("mat-D");

  CHECK_FALSE(db.material_annotation("mat-D").has_value());

  // Prove the kv rows are gone too, not just the parent row: re-create a
  // passport with the SAME guid and re-read — a fresh annotation carries no
  // leftover kv from before the delete.
  makePassport(db, "mat-D");
  ProjectDB::MaterialAnnotation fresh;
  fresh.description = "fresh";
  db.save_material_annotation("mat-D", fresh);
  auto got = db.material_annotation("mat-D");
  REQUIRE(got.has_value());
  CHECK(got->description == "fresh");
  CHECK(got->attributes.empty()); // no stale kv survived the cascade
}

TEST_CASE("RegenerateInstances_CascadesAwayMaterialLinkAndAnnotation",
          "[projectdb][material_annotations][integrity]") {
  // The real regeneration path: re-running `rux create instances` deletes and
  // reinserts the `instances` rows for the same cloud. That cascades away the
  // `instance_materials` links (FK onto instances), and deleting the passport
  // afterwards (or via a `--clear`) removes the annotation. Here we prove the
  // annotation does not dangle once the owning material is deleted after a
  // regeneration removed the link.
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(2), "segment_instances");
  makeInstances(db, "instances", 2);
  makePassport(db, "mat-R");
  db.set_instance_material("instances", 1, "mat-R");

  ProjectDB::MaterialAnnotation ann;
  ann.description = "linked";
  db.save_material_annotation("mat-R", ann);
  REQUIRE(db.material_annotation("mat-R").has_value());

  // Regenerate instances on the SAME cloud: the instance_materials link for the
  // old instance_id cascades away.
  std::vector<ProjectDB::InstanceRecord> regen;
  for (uint32_t i = 1; i <= 2; ++i)
    regen.push_back({i, "regen-guid-" + std::to_string(i), 1, 10});
  db.save_instances("instances", regen);
  CHECK(db.instance_materials("instances").empty());

  // The annotation is still owned by the (still-present) passport; removing the
  // passport cascades it away.
  db.delete_material_passport("mat-R");
  CHECK_FALSE(db.material_annotation("mat-R").has_value());
}
