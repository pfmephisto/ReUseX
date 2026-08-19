// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <types.hpp>

#include <filesystem>
#include <string>

using namespace reusex;
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

} // namespace

TEST_CASE("ProjectDB instance material link round-trip",
          "[projectdb][instance_materials]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.save_point_cloud("instances", *makeInstanceCloud(5), "segment_instances");

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
