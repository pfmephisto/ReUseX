// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// CSV and PDF output through a template (spec §6.3): the formula-injection
// guard, quoting, header modes, display values, and the PDF's tables of at
// most 8 columns led by Betegnelse.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resource_export.hpp>
#include <core/resource_templates.hpp>
#include <core/resources.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::core;
using namespace reusex::test_support;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_resource_export") {}
};
} // namespace

TEST_CASE("ResourceExport_CsvCell_GuardAndQuote", "[resources][csv]") {
  for (const char *risky : {"=SUM(A1)", "+1", "-3", "@x", "\tx", "\rx"}) {
    INFO(risky);
    CHECK((csv_cell(risky, ";").rfind("'", 0) == 0 ||
           csv_cell(risky, ";").rfind("\"'", 0) == 0));
  }
  CHECK(csv_cell("-3", ";") == "'-3");
  CHECK(csv_cell("a;b", ";") == "\"a;b\"");
  CHECK(csv_cell("a,b", ";") == "a,b");
  CHECK(csv_cell("a,b", ",") == "\"a,b\"");
  CHECK(csv_cell("say \"hi\"", ";") == "\"say \"\"hi\"\"\"");
  CHECK(csv_cell("two\nlines", ";") == "\"two\nlines\"");
  CHECK(csv_cell("\rx", ";") == "\"'\rx\"");
  CHECK(csv_cell("plain", ";") == "plain");
}

TEST_CASE("ResourceExport_Csv_HeaderModesDisplayValuesBom",
          "[resources][csv]") {
  const auto cat = key_catalogue({});
  const std::vector<ResourceKey> cols{
      *find_key(cat, "sys:name"), *find_key(cat, "sys:treatment"),
      *find_key(cat, "sys:starred"), *find_key(cat, "sys:mass_t")};
  const std::vector<Resource> rows{{"RX-001",
                                    1,
                                    true,
                                    {{"sys:name", "=Døre"},
                                     {"sys:treatment", "nyttiggoerelse"},
                                     {"sys:starred", "true"},
                                     {"sys:mass_t", std::nullopt}}}};
  CsvOptions o; // ";", utf-8-bom, label
  CHECK(build_resource_csv(cols, rows, o) ==
        "\xEF\xBB\xBF"
        "Betegnelse;Behandling;Vigtig;Tons\r\n"
        "'=Døre;Nyttiggørelse;Ja;\r\n");
  o.header = "key";
  o.encoding = "utf-8";
  o.delimiter = ",";
  CHECK(build_resource_csv(cols, rows, o) ==
        "sys:name,sys:treatment,sys:starred,sys:mass_t\r\n"
        "'=Døre,nyttiggoerelse,true,\r\n");
}

TEST_CASE("ResourceExport_Tables_ChunkedLedByBetegnelse", "[resources][pdf]") {
  const auto cat = key_catalogue({});
  std::vector<ResourceKey> cols{*find_key(cat, "sys:name")};
  const auto lex = leksikon_keys();
  for (std::size_t i = 0; i < 15; ++i)
    cols.push_back(lex[i]);
  Resource r{"RX-007", 1, false, {{"sys:name", "Døre"}}};
  const auto tables = resource_tables(cols, {r});
  REQUIRE(tables.size() == 3); // 15 others / 7 per table
  CHECK(tables[0].headers.size() == 8);
  CHECK(tables[0].headers.front() == "Betegnelse");
  CHECK(tables[1].headers.size() == 8);
  CHECK(tables[2].headers.size() == 2);
  CHECK(tables[2].headers[1] == lex[14].label);
  for (const auto &t : tables) {
    REQUIRE(t.rows.size() == 1);
    CHECK(t.rows[0].front() == "Døre · RX-007");
    CHECK(t.rows[0].size() == t.headers.size());
  }
  CHECK(resource_tables(cols, {}).empty());
  const auto only_name = resource_tables({*find_key(cat, "sys:name")}, {r});
  REQUIRE(only_name.size() == 1);
  CHECK(only_name[0].headers == std::vector<std::string>{"Betegnelse"});
}

TEST_CASE("ResourceExport_ThroughATemplate", "[resources][csv][pdf]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto keep = make_type(db, "Døre");
  const auto drop = make_type(db, "Fejl");
  make_part(db, "RX-001", keep);
  make_part(db, "RX-002", drop);
  ProjectDB::SurveyTypePatch rej;
  rej.review_status = ReviewStatus::rejected;
  db.update_survey_type(drop, rej);
  TemplateInput in;
  in.name = "Kort";
  in.members = nlohmann::json::parse(R"([{"key":"sys:quantity"}])");
  in.csv = nlohmann::json::parse(R"({"encoding":"utf-8"})");
  const auto id = create_template(db, in).record.id;
  CHECK(export_resources_csv(db, id) ==
        "Mængde\r\n1\r\n1\r\n"); // CSV: every resource
  const auto section = resource_report_section(db, id);
  CHECK(section.template_name == "Kort");
  REQUIRE(section.tables.size() == 1);
  REQUIRE(section.tables[0].rows.size() == 1); // PDF: rejected type left out
  CHECK(section.tables[0].rows[0] ==
        std::vector<std::string>{"Døre · RX-001", "1"});
  CHECK_THROWS_AS(export_resources_csv(db, 999), std::out_of_range);
  CHECK_THROWS_AS(resource_report_section(db, 999), std::out_of_range);
}
