// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Entry point of the native Qt client: what plain `rux` (or `rux -p x.rux`
// with no subcommand) runs when there is a display. rux.cpp decides that
// (rux_qt/launch.hpp) and calls run_app().

#include <QString>

namespace rux::qt {

struct AppOptions {
  /// Project to open at start; empty shows the start page alone.
  QString project;
  /// Dev/test: quit this many ms after the main window is shown, printing
  /// a one-line state report to stderr. Negative = run normally.
  int quit_after_ms = -1;
};

/// Create the QApplication, theme and main window and run the event loop.
/// Returns the process exit code.
int run_app(int argc, char **argv, const AppOptions &options);

} // namespace rux::qt
