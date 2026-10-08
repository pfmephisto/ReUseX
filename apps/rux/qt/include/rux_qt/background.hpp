// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Work the Qt client runs on DETACHED threads — a project open waiting out a
// lock or migrating, an ICP refine — that nothing joins, so the window can
// close while it runs. Such a thread may still log through spdlog and the
// ReUseX log handler, so process teardown must know about it:
//
//  - run_app waits a bounded time for it after the window has closed;
//  - if it is still running then, rux::run skips spdlog::shutdown() and the
//    log-handler reset, and main() ends the process with std::quick_exit, so
//    neither the logger nor any static the thread touches is destroyed under
//    it.
//
// Qt-free (rux_qt_core) because rux_lib and main() consult it.

namespace rux::qt {

/// RAII marker for one piece of detached work: counted while it lives.
class BackgroundWork {
    public:
  BackgroundWork();
  ~BackgroundWork();
  BackgroundWork(const BackgroundWork &) = delete;
  BackgroundWork &operator=(const BackgroundWork &) = delete;
};

/// How many BackgroundWork markers are alive, process-wide.
int background_work_in_flight();

/// Wait up to @p timeout_ms for every marker to end. True if none is left.
bool wait_for_background_work(int timeout_ms);

} // namespace rux::qt
