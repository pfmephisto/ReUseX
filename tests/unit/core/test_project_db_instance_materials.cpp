// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <types.hpp>

#include <filesystem>
#include <string>
#include <vector>

using namespace reusex;
using reusex::core::MaterialPassport;
namespace fs = std::filesystem;

namespace {

struct TempDB {
  fs::path path;
  TempDB()
      : path(fs::temp_directory_path() /
             ("test_projectdb_instmat_" +
              std::to_string(reinterpret_cast<uintptr_t>(this)) + ".rux")) {}
  ~TempDB() noexcept {
    std::error_code ec;
    fs::remove(path, ec);
  }
};

// Build a small instance-label cloud so getCloudId() resolves.
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

// Persist `n` instances (ids 1..n) with fresh guids into the instances table.
void makeInstances(ProjectDB &db, const std::string &cloud, size_t n) {
  std::vector<ProjectDB::InstanceRecord> recs;
  for (size_t i = 1; i <= n; ++i)
    recs.push_back(
        {static_cast<uint32_t>(i), "inst-guid-" + std::to_string(i), 1, 10});
  db.save_instances(cloud, recs);
}

// Create a passport with the given document guid.
void makePassport(ProjectDB &db, const std::string &guid) {
  MaterialPassport p;
  p.metadata.document_guid = guid;
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  db.add_material_passport(p, "test-project");
}

} // namespace

TEST_CASE("ProjectDB instance material link round-trip",
          "[projectdb][instance_materials]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(5), "segment_instances");
  makeInstances(db, "instances", 5);
  makePassport(db, "guid-A");
  makePassport(db, "guid-B");

  REQUIRE_FALSE(db.instance_material_guid("instances", 1).has_value());

  db.set_instance_material("instances", 1, "guid-A");
  db.set_instance_material("instances", 2, "guid-B");

  auto g1 = db.instance_material_guid("instances", 1);
  REQUIRE(g1.has_value());
  REQUIRE(*g1 == "guid-A");
  REQUIRE(*db.instance_material_guid("instances", 2) == "guid-B");
  // Unlinked instance.
  REQUIRE_FALSE(db.instance_material_guid("instances", 3).has_value());
}

TEST_CASE("ProjectDB instance material upserts on re-link",
          "[projectdb][instance_materials]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(3), "segment_instances");
  makeInstances(db, "instances", 3);
  makePassport(db, "guid-old");
  makePassport(db, "guid-new");

  db.set_instance_material("instances", 1, "guid-old");
  db.set_instance_material("instances", 1, "guid-new");

  REQUIRE(*db.instance_material_guid("instances", 1) == "guid-new");
  // Only one row for the instance.
  auto all = db.instance_materials("instances");
  REQUIRE(all.size() == 1);
  REQUIRE(all.at(1) == "guid-new");
}

TEST_CASE("ProjectDB instance_materials returns full map",
          "[projectdb][instance_materials]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(4), "segment_instances");
  makeInstances(db, "instances", 4);
  makePassport(db, "g1");
  makePassport(db, "g2");
  makePassport(db, "g4");

  REQUIRE(db.instance_materials("instances").empty());

  db.set_instance_material("instances", 1, "g1");
  db.set_instance_material("instances", 2, "g2");
  db.set_instance_material("instances", 4, "g4");

  auto m = db.instance_materials("instances");
  REQUIRE(m.size() == 3);
  REQUIRE(m.at(1) == "g1");
  REQUIRE(m.at(2) == "g2");
  REQUIRE(m.at(4) == "g4");
  REQUIRE(m.find(3) == m.end());
}

TEST_CASE("ProjectDB set_instance_material throws for unknown cloud",
          "[projectdb][instance_materials]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  REQUIRE_THROWS_AS(db.set_instance_material("nope", 1, "g"),
                    std::runtime_error);
}

TEST_CASE("ProjectDB set_instance_material rejects ghost instance",
          "[projectdb][instance_materials][integrity]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(3), "segment_instances");
  makeInstances(db, "instances", 2); // only instances 1,2 exist
  makePassport(db, "guid-A");

  // Instance 3 has no instances row.
  REQUIRE_THROWS_AS(db.set_instance_material("instances", 3, "guid-A"),
                    std::runtime_error);
}

TEST_CASE("ProjectDB set_instance_material rejects missing passport",
          "[projectdb][instance_materials][integrity]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(3), "segment_instances");
  makeInstances(db, "instances", 3);
  // No passport created.
  REQUIRE_THROWS_AS(db.set_instance_material("instances", 1, "ghost-guid"),
                    std::runtime_error);
}

TEST_CASE("ProjectDB instances table round-trip and guid lookup",
          "[projectdb][instances]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(3), "segment_instances");

  std::vector<ProjectDB::InstanceRecord> recs = {{1, "guid-1", 5, 100},
                                                 {2, "guid-2", 7, 200}};
  db.save_instances("instances", recs);

  auto got = db.instances("instances");
  REQUIRE(got.size() == 2);
  REQUIRE(got[0].instance_id == 1);
  REQUIRE(got[0].guid == "guid-1");
  REQUIRE(got[0].semantic_class == 5);
  REQUIRE(got[0].point_count == 100);

  REQUIRE(db.instance_guid("instances", 2) == "guid-2");
  REQUIRE_THROWS_AS(db.instance_guid("instances", 99), std::runtime_error);
}

TEST_CASE("ProjectDB save_instances rejects empty and duplicate guids",
          "[projectdb][instances]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(3), "segment_instances");

  std::vector<ProjectDB::InstanceRecord> empty_guid = {{1, "", 1, 10}};
  REQUIRE_THROWS_AS(db.save_instances("instances", empty_guid),
                    std::runtime_error);

  std::vector<ProjectDB::InstanceRecord> dup = {{1, "dup", 1, 10},
                                                {2, "dup", 1, 10}};
  REQUIRE_THROWS_AS(db.save_instances("instances", dup), std::runtime_error);
}

TEST_CASE("ProjectDB deleting a cloud cascades instance rows and links",
          "[projectdb][instances][integrity]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(2), "segment_instances");
  makeInstances(db, "instances", 2);
  makePassport(db, "guid-A");
  db.set_instance_material("instances", 1, "guid-A");

  db.delete_point_cloud("instances");

  // Recreate cloud; instances rows and links should be gone (cascaded).
  db.save_point_cloud("instances", *makeInstanceCloud(2), "segment_instances");
  REQUIRE(db.instances("instances").empty());
  REQUIRE(db.instance_materials("instances").empty());
}
