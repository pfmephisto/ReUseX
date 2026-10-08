// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// ruxd's command line: option set, post-parse defaults, and the entry point
// main() calls. Kept in ruxd_lib so the parsing rules are unit-tested
// (tests/unit/ruxd/test_cli.cpp) — main.cpp is one statement (STANDARDS §1.1).

#include <config.hpp>
#include <local.hpp>
#include <pg/admin.hpp>

#include <CLI/CLI.hpp>

#include <cstdint>

namespace ruxd {

/// Port `ruxd --local` listens on when neither --port nor RUXD_PORT is given.
/// The Vite dev proxy and the docs assume it.
inline constexpr std::uint16_t kLocalDefaultPort = 8420;

/// Everything the command line asks for.
struct Invocation {
  Config config; ///< Server settings (port, threads, Postgres, token).
  /// The web GUI's settings, both modes (`local.server`), plus the
  /// `--local` target: empty = server mode.
  LocalOptions local;
  /// `ruxd admin …`: Kind::none unless that subcommand was given.
  pg::AdminCommand admin;
  /// `--cookie-secure`: auto | always | never.
  std::string cookie_secure = "auto";

  bool is_local() const { return !local.target.empty(); }
  bool is_admin() const { return admin.kind != pg::AdminCommand::Kind::none; }
};

/// Register every ruxd option and the `admin` subcommands on @p app, bound
/// into @p inv. The web GUI's options serve both modes; only --open-browser
/// requires --local.
void configure_cli(CLI::App &app, Invocation &inv);

/// Post-parse defaults: the server's port (in local mode kLocalDefaultPort
/// unless --port / RUXD_PORT was given), thread count, access token (local
/// mode only: in server mode it is the superuser token), idle timeout and
/// upload limit.
void finish_invocation(const CLI::App &app, Invocation &inv);

/// Parse, then run local mode or the service. @return process exit code.
int run(int argc, char **argv);

} // namespace ruxd
