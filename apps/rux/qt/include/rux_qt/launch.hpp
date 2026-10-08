// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// What plain `rux` does, and how a failed project open is explained — the
// Qt-free half of the app shell's start-up, unit-tested without a display.

#include <functional>
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
  bool qt_client_built = true;      ///< false when no GUI launcher is linked
  std::string_view display;         ///< $DISPLAY
  std::string_view wayland_display; ///< $WAYLAND_DISPLAY
  std::string_view xdg_runtime_dir; ///< $XDG_RUNTIME_DIR
  std::string_view qpa_platform;    ///< $QT_QPA_PLATFORM
  /// Whether a socket path exists. Empty = assume every socket exists
  /// (callers that cannot check). rux passes a std::filesystem check.
  std::function<bool(const std::string &)> path_exists;
};

/// True when a window can be shown — i.e. Qt's platform plugin will connect
/// instead of aborting with "could not connect to display":
///  - a Wayland display needs its socket ($WAYLAND_DISPLAY absolute, or
///    under $XDG_RUNTIME_DIR) to exist — a stale variable in tmux/ssh does
///    not count;
///  - a local X display (":0", "unix:0.0") needs /tmp/.X11-unix/X<n>; a
///    remote one ("localhost:10.0", ssh -X forwarding) is trusted;
///  - QT_QPA_PLATFORM (options after ':' ignored) narrows it: "xcb" needs
///    X, "wayland*" needs Wayland, a list ("wayland;xcb") needs any member,
///    "offscreen" / "minimal" need nothing (tests, screenshots), and any
///    other platform (eglfs, vnc, …) is trusted.
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
  /// A read-only directory holding a non-empty -wal (an unfinished write):
  /// sqlite cannot read the WAL without creating -shm, and the main file
  /// alone is not the whole project.
  wal_read_only_dir,
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
