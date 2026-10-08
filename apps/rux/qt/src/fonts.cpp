// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/fonts.hpp>

#include <QDir>
#include <QFontDatabase>

#include <cstdio>

// Q_INIT_RESOURCE must be called at global scope. rux_qt_lib is a static
// library, so nothing else guarantees the linker keeps the resource object.
static void init_rux_qt_resources() { Q_INIT_RESOURCE(rux_qt); }

namespace rux::qt {

QStringList required_font_families() {
  return {"Archivo", "Oswald", "JetBrains Mono"};
}

QStringList ensure_bundled_fonts() {
  static QStringList missing = [] {
    init_rux_qt_resources();
    const QDir dir(":/rux_qt/fonts");
    for (const QString &f : dir.entryList({"*.ttf"}, QDir::Files)) {
      if (QFontDatabase::addApplicationFont(dir.filePath(f)) < 0)
        std::fprintf(stderr, "rux-qt: ERROR could not load font %s\n",
                     qPrintable(f));
    }
    QStringList absent;
    const QStringList known = QFontDatabase::families();
    for (const QString &family : required_font_families()) {
      if (!known.contains(family)) {
        std::fprintf(stderr, "rux-qt: ERROR font family '%s' not available\n",
                     qPrintable(family));
        absent << family;
      }
    }
    return absent;
  }();
  return missing;
}

} // namespace rux::qt
