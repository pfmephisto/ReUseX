// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The page registry: every screen the gallery (and later the app) can show,
// built from a factory so a screenshot can instantiate one page with fixture
// data and no main window.
//
// Registration is explicit (register_page), not static-initialiser magic:
// rux_qt_lib is a static library and the linker would drop an unreferenced
// registrar object.

#include <QString>
#include <QWidget>

#include <functional>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace rux::qt {

struct PageContext {
  /// The open project (a copy), or nullptr: pages must show an empty state.
  const reusex::ProjectDB *db = nullptr;
  QString project_path;
  /// Use the real QVTKOpenGLNativeWidget for 3D panes (needs a display).
  bool interactive_3d = false;
};

struct PageInfo {
  QString name;        ///< --page value, e.g. "components"
  QString description; ///< one line for --list-pages
  std::function<QWidget *(const PageContext &)> make;
};

void register_page(PageInfo page);
const std::vector<PageInfo> &pages();
const PageInfo *find_page(const QString &name);

} // namespace rux::qt
