// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace rux {

/// The whole of the `rux` CLI, minus the `main()` symbol itself (#249).
///
/// Installs the fatal-signal handlers and the async spdlog logger, builds the
/// `CLI::App`, registers every subcommand, parses `argv` and runs the selected
/// subcommand. The return value is already a process exit code (0 on success,
/// 1-125 on failure — see `exit_status.hpp`), so `main()` only has to forward
/// it.
///
/// Splitting this out of `main()` is what lets `rux_lib` be a static library
/// the unit tests can link: an object file containing `main` cannot be linked
/// into a Catch2 binary that supplies its own.
///
/// Not re-entrant: it mutates process-global state (signal dispositions, the
/// spdlog default logger, the ReUseX log handler) and calls
/// `spdlog::shutdown()` on the way out. Call it once, from `main()`.
int run(int argc, char **argv);

} // namespace rux
