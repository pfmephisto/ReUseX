// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>
#include <core/MaterialPassport.hpp>
#include <core/materialepas_types.hpp>

#include <cstdio>
#include <filesystem>
#include <iostream>

using namespace reusex;
using reusex::core::MaterialPassport;

namespace fs = std::filesystem;

// Helper: create a temp database path that auto-cleans
struct TempDB {
  fs::path path;
  TempDB()
      : path(fs::temp_directory_path() /
             ("test_projectdb_materials_" +
              std::to_string(reinterpret_cast<uintptr_t>(this)) + ".rux")) {}
  ~TempDB() noexcept {
    std::error_code ec;
    fs::remove(path, ec);
    if (ec) {
      std::cerr << "Warning: Failed to remove temp DB file " << path << ": "
                << ec.message() << std::endl;
    }
  }
};

TEST_CASE("ProjectDB::project_summary() with material passports",
          "[ProjectDB][materials]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  SECTION("Empty database has no materials") {
    auto summary = db.project_summary();
    REQUIRE(summary.materials.empty());
  }

  SECTION("Database with material passports populates materials vector") {
    // Create a material passport
    MaterialPassport passport;
    passport.metadata.document_guid = "test-guid-12345";
    passport.metadata.creation_date = "2025-01-15T10:30:00Z";
    passport.metadata.version_number = "1.0.0";

    // Add designation (name) property
    passport.description.designation = "Test Material Concrete";

    // Add some other properties (images)
    passport.description.images.push_back("http://example.com/image.jpg");

    // Save to database (add_material_passport requires projectId)
    db.add_material_passport(passport, "test-project");

    // Get summary
    auto summary = db.project_summary();

    // Verify materials vector is populated
    REQUIRE(summary.materials.size() == 1);

    const auto &material = summary.materials[0];
    REQUIRE(material.guid == "test-guid-12345");
    REQUIRE(!material.id.empty());  // ID should be populated
    REQUIRE(material.created_at == "2025-01-15T10:30:00Z");
    REQUIRE(material.version_number == "1.0.0");
    // Should have at least designation property stored
    REQUIRE(material.property_count >= 1);
  }

  SECTION("Multiple material passports are all listed") {
    // Create first passport
    MaterialPassport passport1;
    passport1.metadata.document_guid = "guid-1";
    passport1.metadata.creation_date = "2025-01-01T00:00:00Z";
    passport1.metadata.version_number = "0.1.0";
    passport1.description.designation = "Brick Wall";
    db.add_material_passport(passport1, "test-project");

    // Create second passport
    MaterialPassport passport2;
    passport2.metadata.document_guid = "guid-2";
    passport2.metadata.creation_date = "2025-01-02T00:00:00Z";
    passport2.metadata.version_number = "0.2.0";
    passport2.description.designation = "Concrete Floor";
    db.add_material_passport(passport2, "test-project");

    // Create third passport
    MaterialPassport passport3;
    passport3.metadata.document_guid = "guid-3";
    passport3.metadata.creation_date = "2025-01-03T00:00:00Z";
    passport3.metadata.version_number = "1.5.2";
    passport3.description.designation = "Steel Beam";
    db.add_material_passport(passport3, "test-project");

    // Get summary
    auto summary = db.project_summary();

    // Verify all materials are present
    REQUIRE(summary.materials.size() == 3);

    // Check they're ordered by creation date and all have IDs
    REQUIRE(summary.materials[0].guid == "guid-1");
    REQUIRE(!summary.materials[0].id.empty());
    REQUIRE(summary.materials[1].guid == "guid-2");
    REQUIRE(!summary.materials[1].id.empty());
    REQUIRE(summary.materials[2].guid == "guid-3");
    REQUIRE(!summary.materials[2].id.empty());
  }

  SECTION("Material name is always the row ID") {
    // Create passport without setting designation
    MaterialPassport passport;
    passport.metadata.document_guid = "test-guid-no-name";
    passport.metadata.creation_date = "2025-01-15T10:30:00Z";
    passport.metadata.version_number = "1.0.0";
    // Don't set designation field (leave it empty)

    db.add_material_passport(passport, "test-project");

    auto summary = db.project_summary();

    REQUIRE(summary.materials.size() == 1);
    const auto &material = summary.materials[0];
    REQUIRE(material.guid == "test-guid-no-name");
    // ID should be populated (the row ID)
    REQUIRE(!material.id.empty());
  }

  SECTION("Property count reflects stored properties") {
    MaterialPassport passport;
    passport.metadata.document_guid = "test-guid-props";
    passport.metadata.creation_date = "2025-01-15T10:30:00Z";
    passport.metadata.version_number = "1.0.0";
    passport.description.designation = "Material With Properties";

    // Add multiple properties
    passport.description.images.push_back("http://example.com/img1.jpg");
    passport.description.images.push_back("http://example.com/img2.jpg");

    db.add_material_passport(passport, "test-project");

    auto summary = db.project_summary();

    REQUIRE(summary.materials.size() == 1);
    const auto &material = summary.materials[0];
    // Should count all non-empty properties stored
    REQUIRE(material.property_count > 1);
  }
}
