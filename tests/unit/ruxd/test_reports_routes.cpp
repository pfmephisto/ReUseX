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

#include <reusex/core/ProjectDB.hpp>

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
