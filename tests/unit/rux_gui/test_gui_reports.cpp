// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ReportPdfVersion JSON (GUI Phase 5, R7): version ordinal and the blocking
// count, null for a version without one.

#include <catch2/catch_test_macros.hpp>

#include <gui/api.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>

#include <nlohmann/json.hpp>

using namespace rux::gui;
using reusex::ProjectDB;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_gui_reports") {}
};
} // namespace

TEST_CASE("ReportVersionsJson_VersionAndBlockingTypes", "[gui][reports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.add_report_pdf({'%', 'P', 'D', 'F'}, "Ressourcekortlægning", 2);
  db.add_report_pdf({'%', 'P', 'D', 'F'}, "Ressourcekortlægning");
  const auto j = list_report_pdfs_json(db);
  const auto &v = j.at("versions");
  REQUIRE(v.size() == 2);
  CHECK(v.at(0).at("version") == 2);
  CHECK(v.at(0).at("blocking_types").is_null());
  CHECK(v.at(1).at("version") == 1);
  CHECK(v.at(1).at("blocking_types") == 2);
  CHECK(v.at(1).at("label") == "Ressourcekortlægning");
  CHECK(v.at(1).at("size_bytes") == 4);
}
