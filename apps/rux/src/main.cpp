// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The `rux` entry point, and deliberately nothing else (#249).
//
// Everything the CLI actually does lives in `rux_lib` (src/rux.cpp holds
// `rux::run`), so the app logic is linkable from `reusex_unit_tests_vision`.
// A translation unit that defines `main` cannot go into a test binary, which
// is the whole reason this file is one line long.

//
// It is also the one place the Qt client is linked (RUX_HAVE_QT_CLIENT): the
// launcher below is handed to rux::run, so rux_lib and the test binaries
// that link it never pull in Qt Widgets.

#include <rux_app.hpp>
#include <rux_qt/background.hpp>

#include <cstdio>
#include <cstdlib>

#ifdef RUX_HAVE_QT_CLIENT
#include <rux_qt/app.hpp>
#endif

int main(int argc, char **argv) {
#ifdef RUX_HAVE_QT_CLIENT
  const int rc = rux::run(
      argc, argv, [](int ac, char **av, const rux::GuiLaunch &request) {
        rux::qt::AppOptions o;
        if (request.project)
          o.project = QString::fromStdString(request.project->string());
        o.quit_after_ms = request.quit_after_ms;
        return rux::qt::run_app(ac, av, o);
      });
#else
  const int rc = rux::run(argc, argv);
#endif
  // A detached Qt-client thread outlived the exit wait (rux::run then left
  // spdlog up for it): end without running static destructors under it —
  // or, if it has finished since, tear the logger down after all.
  if (rux::logging_left_up()) {
    if (rux::qt::background_work_in_flight() > 0) {
      std::fflush(nullptr);
      std::quick_exit(rc);
    }
    rux::finish_logging();
  }
  return rc;
}
