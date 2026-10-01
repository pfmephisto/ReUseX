// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Endpoint contract tests for the Ressourcekortlægning report routes (#456).
// Verifies that the three routes are registered in the endpoint table with the
// expected method/path/auth shape. No server is started; the EndpointRegistry
// is checked directly.

#include <auth.hpp>
#include <endpoints.hpp>
#include <handlers.hpp>

#include <gui/api.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <nlohmann/json.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include "../../support/temp_path.hpp"

namespace {

struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_reports_routes") {}
};

// Returns the endpoint matching method+path, or nullptr.
const ruxd::Endpoint *find_endpoint(const ruxd::EndpointRegistry &reg,
                                    const std::string &method,
                                    const std::string &path) {
  for (const auto &e : reg.endpoints()) {
    if (e.method == method && e.path == path)
      return &e;
  }
  return nullptr;
}

} // namespace

TEST_CASE("ReportRoutes_RegistrationShape", "[ruxd][reports]") {
  TempDB tmp;
  reusex::ProjectDB db(tmp.path, /*readOnly=*/false);

  ruxd::App app;
  ruxd::EndpointRegistry reg;
  ruxd::register_report_routes(app, reg, db);

  SECTION(
      "POST /reports/ressourcekortlaegning is registered and requires auth") {
    const auto *ep =
        find_endpoint(reg, "POST", "/reports/ressourcekortlaegning");
    REQUIRE(ep != nullptr);
    REQUIRE(ep->requires_auth);
  }

  SECTION(
      "GET /reports/ressourcekortlaegning is registered and requires auth") {
    const auto *ep =
        find_endpoint(reg, "GET", "/reports/ressourcekortlaegning");
    REQUIRE(ep != nullptr);
    REQUIRE(ep->requires_auth);
  }

  SECTION("GET /reports/ressourcekortlaegning/<int> is registered and requires "
          "auth") {
    const auto *ep =
        find_endpoint(reg, "GET", "/reports/ressourcekortlaegning/<int>");
    REQUIRE(ep != nullptr);
    REQUIRE(ep->requires_auth);
  }

  SECTION("Exactly three report routes are registered") {
    int count = 0;
    for (const auto &e : reg.endpoints()) {
      if (e.path.find("/reports/ressourcekortlaegning") != std::string::npos)
        ++count;
    }
    REQUIRE(count == 3);
  }
}

TEST_CASE("ReportRoutes_ListMatchesRuxGuiSerialiser", "[ruxd][reports]") {
  // F9: ruxd and `rux gui` both answer GET /reports/ressourcekortlaegning with
  // ReportPdfVersion objects; version and blocking_types (null before v23)
  // must mean the same on both servers.
  TempDB tmp;
  reusex::ProjectDB db(tmp.path);
  db.add_report_pdf({'%', 'P', 'D', 'F'}, "Ressourcekortlægning", 3);
  db.add_report_pdf({'%', 'P', 'D', 'F'}, "Ressourcekortlægning", 0);
  db.add_report_pdf({'%', 'P', 'D', 'F'}, "Ressourcekortlægning");

  ruxd::App app;
  ruxd::EndpointRegistry reg;
  ruxd::register_report_routes(app, reg, db);
  app.get_middleware<ruxd::BearerAuthMiddleware>().configure(&reg, "");
  app.validate();

  crow::request req;
  req.method = crow::HTTPMethod::Get;
  req.url = "/reports/ressourcekortlaegning";
  req.raw_url = req.url;
  crow::response res;
  app.handle_full(req, res);
  REQUIRE(res.code == 200);

  const auto ruxd_json = nlohmann::json::parse(res.body);
  const auto gui_json = rux::gui::list_report_pdfs_json(db);
  CHECK(ruxd_json == gui_json);

  const auto &v = ruxd_json.at("versions");
  REQUIRE(v.size() == 3);
  CHECK(v.at(0).at("version") == 3);
  CHECK(v.at(0).at("blocking_types").is_null());
  CHECK(v.at(1).at("version") == 2);
  CHECK(v.at(1).at("blocking_types") == 0);
  CHECK(v.at(2).at("version") == 1);
  CHECK(v.at(2).at("blocking_types") == 3);
}
