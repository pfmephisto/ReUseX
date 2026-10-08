// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// ruxd's command line: option set, post-parse defaults, and the entry point
// main() calls. Kept in ruxd_lib so the parsing rules are unit-tested
// (tests/unit/ruxd/test_cli.cpp) — main.cpp is one statement (STANDARDS §1.1).

#include <config.hpp>
#include <local.hpp>

#include <CLI/CLI.hpp>

#include <cstdint>

namespace ruxd {

/// Port `ruxd --local` listens on when neither --port nor RUXD_PORT is given.
/// The Vite dev proxy and the docs assume it.
inline constexpr std::uint16_t kLocalDefaultPort = 8420;

/// Everything the command line asks for.
struct Invocation {
  Config config;      ///< Service mode settings (shared: port, threads, token).
  LocalOptions local; ///< `--local` settings; `local.target` empty = service.

  bool is_local() const { return !local.target.empty(); }
};

/// Register every ruxd option on @p app, bound into @p inv. Local-mode options
/// other than --local itself require --local.
void configure_cli(CLI::App &app, Invocation &inv);

/// Post-parse defaults: in local mode, the port (kLocalDefaultPort unless
/// --port / RUXD_PORT was given), the thread count and the access token.
void finish_invocation(const CLI::App &app, Invocation &inv);

/// Parse, then run local mode or the service. @return process exit code.
int run(int argc, char **argv);

} // namespace ruxd
