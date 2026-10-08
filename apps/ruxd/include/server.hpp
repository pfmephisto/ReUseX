// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// ruxd without --local: the multi-user server, and `ruxd admin`.

#include <cli.hpp>

namespace ruxd {

/// Migrate Postgres, build the Postgres-backed stores and serve the web GUI
/// with logins until SIGINT/SIGTERM. @return process exit code; startup
/// errors are logged, not thrown.
int run_server(Invocation inv);

/// Run the `ruxd admin` command in @p inv. @return process exit code.
int run_admin(Invocation inv);

} // namespace ruxd
