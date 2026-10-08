// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd's command line (src/cli.cpp): the local-mode port default and the
// rule that local-mode options need --local.

#include <catch2/catch_test_macros.hpp>

#include <cli.hpp>

#include <CLI/CLI.hpp>

#include <cstdlib>
#include <string>

namespace {

/// Parse @p args (no program name) the way run() does, minus running.
ruxd::Invocation parse(const std::string &args) {
  CLI::App app{"test"};
  ruxd::Invocation inv;
  ruxd::configure_cli(app, inv);
  app.parse(args, /*program_name_included=*/false);
  ruxd::finish_invocation(app, inv);
  return inv;
}

/// Unset RUXD_PORT for the test's duration, restoring nothing: each ctest
/// test is its own process.
void clear_port_env() { ::unsetenv("RUXD_PORT"); }

} // namespace

TEST_CASE("RuxdCli_LocalWithoutPort_DefaultsTo8420", "[ruxd][cli]") {
  clear_port_env();
  const auto inv = parse("--local scan.rux");
  CHECK(inv.is_local());
  CHECK(inv.local.target == "scan.rux");
  CHECK(inv.local.server.port == ruxd::kLocalDefaultPort);
  CHECK(inv.local.server.port == 8420);
  CHECK(inv.local.server.bind_address == "127.0.0.1");
  CHECK_FALSE(inv.local.server.open_browser);
}

TEST_CASE("RuxdCli_LocalWithPortOrEnv_UsesThatPort", "[ruxd][cli]") {
  clear_port_env();
  SECTION("--port") {
    CHECK(parse("--local scan.rux --port 9000").local.server.port == 9000);
  }
  SECTION("RUXD_PORT") {
    ::setenv("RUXD_PORT", "9100", 1);
    CHECK(parse("--local scan.rux").local.server.port == 9100);
    ::unsetenv("RUXD_PORT");
  }
}

TEST_CASE("RuxdCli_LocalSharedOptions_ReachTheServer", "[ruxd][cli]") {
  clear_port_env();
  const auto inv = parse("--local scan.rux --threads 3 --auth-token t "
                         "--bind 0.0.0.0 --allow-origin http://box:8420");
  CHECK(inv.local.server.threads == 3u);
  CHECK(inv.local.server.auth_token == "t");
  CHECK(inv.local.server.bind_address == "0.0.0.0");
  REQUIRE(inv.local.server.allowed_origins.size() == 1);
}

TEST_CASE("RuxdCli_ServiceMode_KeepsItsOwnPortDefault", "[ruxd][cli]") {
  clear_port_env();
  const auto inv = parse("");
  CHECK_FALSE(inv.is_local());
  CHECK(inv.config.port == ruxd::Config{}.port);
}

TEST_CASE("RuxdCli_LocalOptionWithoutLocal_IsAnError", "[ruxd][cli]") {
  clear_port_env();
  for (const char *args :
       {"--bind 0.0.0.0", "--allow-origin http://x", "--open-browser",
        "--sam3-model /m", "--models-dir /m", "--no-segment-cuda"}) {
    INFO(args);
    CHECK_THROWS_AS(parse(args), CLI::RequiresError);
  }
}
