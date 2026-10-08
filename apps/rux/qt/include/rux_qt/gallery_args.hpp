// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Command line of `rux-qt-gallery`, the design loop's screenshot tool.
//
// Parsed without Qt because two of the flags have to act before a
// QApplication exists: `--scale` becomes QT_SCALE_FACTOR, and a screenshot
// run selects the offscreen platform plugin.

#include <rux_qt/tokens.hpp>

#include <string>

namespace rux::qt {

struct GalleryArgs {
  std::string page = "components";
  ThemeMode theme = ThemeMode::dark;
  int width = 1440;
  int height = 900;
  double scale = 1.0;
  std::string project;    ///< a COPY of a .rux; empty = no project
  std::string screenshot; ///< PNG path; empty = interactive window
  std::string style_dir;  ///< read tokens.css + QSS from here (implies dev)
  bool dev = false;       ///< read styles from the source tree, hot reload
  bool gl = false;        ///< real QVTKOpenGLNativeWidget (needs a display)
  bool list_pages = false;
  bool help = false;
  std::string error; ///< non-empty when the command line is invalid
};

GalleryArgs parse_gallery_args(int argc, const char *const *argv);

/// The `--help` text.
std::string gallery_usage();

} // namespace rux::qt
