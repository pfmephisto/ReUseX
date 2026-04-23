// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>

#include <cstdio>
#include <filesystem>
#include <iostream>

using namespace reusex;

namespace fs = std::filesystem;

// Helper: create a temp database path that auto-cleans
struct TempDB {
  fs::path path;
  TempDB()
      : path(fs::temp_directory_path() /
             ("test_projectdb_project_info_" +
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

TEST_CASE("ProjectDB::project_summary() includes project information",
          "[ProjectDB][projects]") {
  TempDB tmp;
  ProjectDB db(tmp.path);

  SECTION("Empty database initially has no projects in summary") {
    auto summary = db.project_summary();
    // New database should have no projects initially
    REQUIRE(summary.projects.empty());
  }

  SECTION("Database with project metadata populates projects vector") {
    // Create and set project metadata using update_project_metadata
    ProjectDB::ProjectMetadata metadata;
    metadata.id = "test-project-1";
    metadata.name = "Test Building";
    metadata.building_address = "123 Test Street";
    metadata.year_of_construction = 1950;
    metadata.survey_date = "2025-04-16";
    metadata.survey_organisation = "Test Organization";
    metadata.notes = "This is a test note";

    db.update_project_metadata(metadata);

    // Get summary
    auto summary = db.project_summary();

    // Verify projects vector contains exactly one project
    REQUIRE(summary.projects.size() == 1);

    // Find our test project
    bool found = false;
    for (const auto &project : summary.projects) {
      if (project.id == "test-project-1") {
        found = true;
        REQUIRE(project.name == "Test Building");
        REQUIRE(project.building_address == "123 Test Street");
        REQUIRE(project.year_of_construction == 1950);
        REQUIRE(project.survey_date == "2025-04-16");
        REQUIRE(project.survey_organisation == "Test Organization");
        REQUIRE(project.notes == "This is a test note");
        break;
      }
    }
    REQUIRE(found);
  }

  SECTION("Database with partial project metadata") {
    // Set only some fields
    ProjectDB::ProjectMetadata metadata;
    metadata.id = "partial-project";
    metadata.name = "Partial Project";
    metadata.survey_date = "2025-03-01";
    // Other fields left as defaults

    db.update_project_metadata(metadata);

    auto summary = db.project_summary();
    REQUIRE(summary.projects.size() == 1);

    // Find our partial project
    bool found = false;
    for (const auto &project : summary.projects) {
      if (project.id == "partial-project") {
        found = true;
        REQUIRE(project.name == "Partial Project");
        REQUIRE(project.building_address.empty());
        REQUIRE(project.year_of_construction == 0); // Not set
        REQUIRE(project.survey_date == "2025-03-01");
        REQUIRE(project.survey_organisation.empty());
        REQUIRE(project.notes.empty());
        break;
      }
    }
    REQUIRE(found);
  }

  SECTION("Multiple projects are all listed") {
    // Create first project
    ProjectDB::ProjectMetadata metadata1;
    metadata1.id = "project-first";
    metadata1.name = "First Building";
    metadata1.building_address = "100 Main St";
    db.update_project_metadata(metadata1);

    // Create second project
    ProjectDB::ProjectMetadata metadata2;
    metadata2.id = "project-second";
    metadata2.name = "Second Building";
    metadata2.building_address = "200 Oak Ave";
    metadata2.year_of_construction = 2000;
    db.update_project_metadata(metadata2);

    auto summary = db.project_summary();
    REQUIRE(summary.projects.size() == 2);

    // Find and verify both projects
    bool found_first = false;
    bool found_second = false;

    for (const auto &project : summary.projects) {
      if (project.id == "project-first") {
        found_first = true;
        REQUIRE(project.name == "First Building");
        REQUIRE(project.building_address == "100 Main St");
      } else if (project.id == "project-second") {
        found_second = true;
        REQUIRE(project.name == "Second Building");
        REQUIRE(project.building_address == "200 Oak Ave");
        REQUIRE(project.year_of_construction == 2000);
      }
    }

    REQUIRE(found_first);
    REQUIRE(found_second);
  }

  SECTION("Summary includes basic database info") {
    auto summary = db.project_summary();

    REQUIRE(summary.path == tmp.path);
    REQUIRE(summary.schema_version > 0);
  }
}
