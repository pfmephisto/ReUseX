// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ReportPdfVersion JSON (GUI Phase 5, R7): version ordinal and the blocking
// count, null for a version without one.

#include <catch2/catch_test_macros.hpp>

#include <api/api.hpp>
#include <api/edits.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>

#include <nlohmann/json.hpp>

using namespace ruxd::api;
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

TEST_CASE("GuiReport_TemplateField_400And404BeforeGenerating",
          "[gui][reports]") {
  reusex::test_support::TempPath tmp("test_gui_reports_template");
  reusex::ProjectDB db(tmp.path);
  auto status = [&](const std::string &body) {
    try {
      ruxd::api::generate_report_pdf_json(db, body);
    } catch (const ruxd::api::HttpError &e) {
      return e.status();
    }
    return 201;
  };
  CHECK(status(R"({"resource_template_id":"2"})") == 400);
  CHECK(status(R"({"resource_template_id":999})") == 404);
  CHECK(status("[]") == 400);
  CHECK(db.list_report_pdfs().empty());
}
