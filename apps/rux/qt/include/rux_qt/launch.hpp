// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// What plain `rux` does, and how a failed project open is explained — the
// Qt-free half of the app shell's start-up, unit-tested without a display.

#include <string>
#include <string_view>

namespace rux::qt {

// ------------------------------------------------------------------ launch --

enum class LaunchAction {
  run_subcommand, ///< a subcommand was given: the CLI runs it
  open_app,       ///< no subcommand and a display: start the Qt client
  print_help,     ///< no subcommand and no display: print help, exit 0
};

struct LaunchInputs {
  bool has_subcommand = false;
  bool qt_client_built = true; ///< false when built with BUILD_QT_CLIENT=OFF
  std::string_view display;    ///< $DISPLAY
  std::string_view wayland_display; ///< $WAYLAND_DISPLAY
  std::string_view qpa_platform;    ///< $QT_QPA_PLATFORM
};

/// True when a window can be shown: an X or Wayland display, or Qt's
/// offscreen platform explicitly selected (tests, screenshots). Any other
/// QT_QPA_PLATFORM value alone is not a display: a desktop profile exports
/// "wayland;xcb" into SSH sessions too.
bool has_display(const LaunchInputs &in);

LaunchAction decide_launch(const LaunchInputs &in);

// -------------------------------------------------------------- open errors --

enum class OpenErrorKind {
  none,
  not_found,     ///< no such file
  not_a_file,    ///< a directory or other non-regular file
  permission,    ///< the OS refused to open it
  locked,        ///< another process holds the write lock past the timeout
  not_a_project, ///< not sqlite, or sqlite without the ReUseX tables
  corrupt,       ///< sqlite says the image is malformed
  other,
};

/// Map a ProjectDB / sqlite exception message to a kind.
OpenErrorKind classify_open_error(std::string_view what);

/// The headline shown for @p kind, in Danish (UTF-8). The raw exception text
/// is shown under it as detail.
std::string open_error_title_da(OpenErrorKind kind);

/// One sentence on what the user can do about it (UTF-8, Danish).
std::string open_error_hint_da(OpenErrorKind kind);

} // namespace rux::qt
