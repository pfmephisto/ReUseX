// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <reusex/pipeline/stages.hpp>

#include <filesystem>
#include <functional>
#include <optional>

namespace rux {

/// What plain `rux` (no subcommand) asks the desktop app to do.
struct GuiLaunch {
  /// Set only for an explicit `-p` — never the ./project.rux default.
  std::optional<std::filesystem::path> project;
  /// Dev/test `--quit-after-ms`; negative = run normally.
  int quit_after_ms = -1;
  /// How the Pipeline workspace runs a stage in-process: rux_lib's executor,
  /// which can also run `optimize` (the slam module).
  reusex::pipeline::StageExecutor stage_executor;
};

/// Starts the desktop app and returns its exit code. Injected by the `rux`
/// executable's main() (which alone links the Qt client), so rux_lib — and
/// every test binary that links it — stays free of Qt Widgets.
using GuiLauncher =
    std::function<int(int argc, char **argv, const GuiLaunch &)>;

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
///
/// With no subcommand: calls @p launch_gui when there is a display and a
/// launcher was given, and prints the help text otherwise (exit 0).
int run(int argc, char **argv, GuiLauncher launch_gui = {});

/// True when run() returned with spdlog still up because detached Qt-client
/// work (rux_qt/background.hpp) outlived the window. main() then either ends
/// with std::quick_exit (still running) or calls finish_logging() (it has
/// finished since) — so the logger is torn down exactly once, and never
/// under a thread that may still log.
bool logging_left_up();
/// The spdlog teardown run() skipped.
void finish_logging();

} // namespace rux
