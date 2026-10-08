// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd's command line (src/cli.cpp): the local-mode port default, the case
// options, and the rule that local-mode options need --local.

#include <catch2/catch_test_macros.hpp>

#include <cli.hpp>

#include <CLI/CLI.hpp>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "../../support/temp_path.hpp"

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

TEST_CASE("RuxdCli_OpenBrowserWithoutLocal_IsAnError", "[ruxd][cli]") {
  clear_port_env();
  // Only opening a browser is local-mode-only; the web GUI's other options
  // serve the multi-user server too (phase S3).
  CHECK_THROWS_AS(parse("--open-browser"), CLI::RequiresError);
}

TEST_CASE("RuxdCli_ServerMode_WebOptionsReachTheServer", "[ruxd][cli]") {
  clear_port_env();
  const auto inv =
      parse("--pg-url postgresql:///x --data-dir /srv/ruxd --bind 0.0.0.0 "
            "--allow-origin https://ruxd.example.dk --job-workers 2 "
            "--auth-token su --cookie-secure always --port 9443");
  CHECK_FALSE(inv.is_local());
  CHECK(inv.local.server.data_dir == "/srv/ruxd");
  CHECK(inv.local.server.bind_address == "0.0.0.0");
  CHECK(inv.local.server.job_workers == 2u);
  CHECK(inv.local.server.port == 9443);
  CHECK(inv.local.server.cookie_secure == ruxd::api::CookieSecure::always);
  // In server mode the token is the superuser's, not a shared access token.
  CHECK(inv.config.auth_token == "su");
  CHECK(inv.local.server.auth_token.empty());
  CHECK_THROWS(parse("--cookie-secure maybe"));
}

TEST_CASE("RuxdCli_Admin_SubcommandsAndNoPasswordFlag", "[ruxd][cli]") {
  clear_port_env();
  using Kind = ruxd::pg::AdminCommand::Kind;
  auto inv = parse("admin create-user --email a@x.dk --name A --admin");
  CHECK(inv.is_admin());
  CHECK(inv.admin.kind == Kind::create_user);
  CHECK(inv.admin.email == "a@x.dk");
  CHECK(inv.admin.display_name == "A");
  CHECK(inv.admin.is_admin);
  // Parent options may come after the subcommand.
  inv = parse("admin list-users --pg-url postgresql:///x");
  CHECK(inv.admin.kind == Kind::list_users);
  CHECK(inv.config.pg_url == "postgresql:///x");
  inv = parse("admin create-token --email a@x.dk --name ci --case kontor");
  CHECK(inv.admin.kind == Kind::create_token);
  CHECK(inv.admin.case_id == "kontor");
  inv = parse("admin disable-user --email a@x.dk --enable");
  CHECK(inv.admin.kind == Kind::disable_user);
  CHECK(inv.admin.enable);
  // There is no way to pass a password on the command line.
  CHECK_THROWS(parse("admin create-user --email a@x.dk --password x"));
  CHECK_THROWS(parse("admin set-password --email a@x.dk --password x"));
  CHECK_THROWS(parse("admin"));
}

TEST_CASE("RuxdCli_CaseOptions_ReachTheServer", "[ruxd][cli]") {
  clear_port_env();
  SECTION("defaults mirror ServerOptions") {
    const auto inv = parse("--local sager");
    const ruxd::api::ServerOptions defaults;
    CHECK(inv.local.server.job_workers == defaults.job_workers);
    CHECK(inv.local.server.job_workers == 1u);
    CHECK(inv.local.server.max_open_cases == defaults.max_open_cases);
    CHECK(inv.local.server.case_idle_timeout == defaults.case_idle_timeout);
    CHECK(inv.local.server.upload_limits.max_bytes ==
          defaults.upload_limits.max_bytes);
    CHECK(inv.local.server.data_dir.empty());
  }
  SECTION("explicit values") {
    const auto inv =
        parse("--local sager --data-dir /srv/sager --job-workers 2 "
              "--max-open-cases 4 --case-idle-minutes 3 --max-upload-mb 100");
    CHECK(inv.local.server.data_dir == "/srv/sager");
    CHECK(inv.local.server.job_workers == 2u);
    CHECK(inv.local.server.max_open_cases == 4u);
    CHECK(inv.local.server.case_idle_timeout == std::chrono::minutes(3));
    CHECK(inv.local.server.upload_limits.max_bytes == 100ull << 20);
  }
  SECTION("job workers must be at least one") {
    CHECK_THROWS(parse("--local sager --job-workers 0"));
  }
}

TEST_CASE("RuxdCli_TrustedProxyRetentionAndSecretFiles", "[ruxd][cli]") {
  clear_port_env();
  auto inv = parse("--trusted-proxy 127.0.0.1 --trusted-proxy 10.0.0.0/8 "
                   "--audit-retention-days 30");
  REQUIRE(inv.local.server.trusted_proxies.size() == 2);
  CHECK(inv.local.server.audit_retention == std::chrono::hours(24 * 30));
  CHECK_THROWS(parse("--trusted-proxy not-an-address"));
  CHECK_THROWS(parse("--trusted-proxy 10.0.0.0/40"));

  // Secrets from files win over (and keep them off) the command line.
  reusex::test_support::TempDir dir("ruxd_cli_secrets");
  const auto token = dir.path / "token";
  const auto dsn = dir.path / "dsn";
  std::ofstream(token) << "  0123456789abcdef0123456789abcdef\n";
  std::ofstream(dsn) << "postgresql:///ruxd\n";
  inv = parse("--auth-token-file " + token.string() + " --pg-url-file " +
              dsn.string());
  ruxd::load_secret_files(inv);
  CHECK(inv.config.auth_token == "0123456789abcdef0123456789abcdef");
  CHECK(inv.config.pg_url == "postgresql:///ruxd");

  // Secrets passed on argv are spotted (run() warns about them).
  const char *argv[] = {"ruxd",           "--pg-url", "x",
                        "--auth-token=y", "--port",   "1"};
  const auto found = ruxd::secrets_on_argv(6, const_cast<char **>(argv));
  CHECK(found == std::vector<std::string>{"--pg-url", "--auth-token"});
}

TEST_CASE("RuxdCli_DeploymentEnvironment_SetsBindDataDirAndPgUrlFile",
          "[ruxd][cli]") {
  // Final review #3: the OCI image configures ruxd through the environment
  // (RUXD_BIND=0.0.0.0, RUXD_DATA_DIR=/data); a flag still wins.
  clear_port_env();
  reusex::test_support::TempPath url_file("test_cli_pg_url", ".txt");
  {
    std::ofstream out(url_file.path);
    out << "postgresql://ruxd@db/ruxd\n";
  }
  ::setenv("RUXD_BIND", "0.0.0.0", 1);
  ::setenv("RUXD_DATA_DIR", "/data", 1);
  ::setenv("DATABASE_URL_FILE", url_file.path.c_str(), 1);
  const auto from_env = parse("");
  CHECK(from_env.local.server.bind_address == "0.0.0.0");
  CHECK(from_env.local.server.data_dir == "/data");
  CHECK(from_env.pg_url_file == url_file.path.string());
  CHECK(parse("--bind 127.0.0.1").local.server.bind_address == "127.0.0.1");
  ::unsetenv("RUXD_BIND");
  ::unsetenv("RUXD_DATA_DIR");
  ::unsetenv("DATABASE_URL_FILE");
}
