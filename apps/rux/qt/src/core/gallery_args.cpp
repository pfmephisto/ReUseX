// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/gallery_args.hpp>

#include <charconv>
#include <string_view>

namespace rux::qt {
namespace {

bool parse_int(std::string_view s, int &out) {
  auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
  return ec == std::errc() && p == s.data() + s.size() && out > 0;
}

} // namespace

GalleryArgs parse_gallery_args(int argc, const char *const *argv) {
  GalleryArgs a;
  for (int i = 1; i < argc; ++i) {
    const std::string_view flag = argv[i];
    auto value = [&](std::string &out) {
      if (i + 1 >= argc) {
        a.error = std::string(flag) + " needs a value";
        return false;
      }
      out = argv[++i];
      return true;
    };
    std::string v;
    if (flag == "--page") {
      value(a.page);
    } else if (flag == "--theme") {
      if (!value(v))
        break;
      if (v == "dark")
        a.theme = ThemeMode::dark;
      else if (v == "light")
        a.theme = ThemeMode::light;
      else
        a.error = "--theme must be dark or light, not '" + v + "'";
    } else if (flag == "--size") {
      if (!value(v))
        break;
      const auto x = v.find('x');
      if (x == std::string::npos ||
          !parse_int(std::string_view(v).substr(0, x), a.width) ||
          !parse_int(std::string_view(v).substr(x + 1), a.height))
        a.error = "--size must be WIDTHxHEIGHT, e.g. 1440x900, not '" + v + "'";
    } else if (flag == "--scale") {
      if (!value(v))
        break;
      // from_chars, not stod: locale-independent (see tokens.cpp).
      auto [p, ec] = std::from_chars(v.data(), v.data() + v.size(), a.scale);
      if (ec != std::errc() || p != v.data() + v.size())
        a.scale = 0;
      if (!(a.scale > 0 && a.scale <= 4))
        a.error = "--scale must be a number in (0, 4], not '" + v + "'";
    } else if (flag == "--project") {
      value(a.project);
    } else if (flag == "--screenshot") {
      value(a.screenshot);
    } else if (flag == "--style-dir") {
      if (value(a.style_dir))
        a.dev = true;
    } else if (flag == "--dev") {
      a.dev = true;
    } else if (flag == "--gl") {
      a.gl = true;
    } else if (flag == "--list-pages") {
      a.list_pages = true;
    } else if (flag == "--help" || flag == "-h") {
      a.help = true;
    } else {
      a.error = "unknown flag '" + std::string(flag) + "'";
    }
    if (!a.error.empty())
      break;
  }
  return a;
}

std::string gallery_usage() {
  return R"(rux-qt-gallery — render a page of the rux Qt client, headless or live

Usage: rux-qt-gallery [options]

  --page NAME          page to show (default: components; see --list-pages)
  --theme dark|light   token theme (default: dark)
  --size WxH           window size in logical pixels (default: 1440x900)
  --scale N            device pixel ratio, e.g. 2 for review shots (default: 1)
  --project FILE.rux   a COPY of a project to read fixture data from
  --screenshot OUT.png render once, save, exit. Missing tokens exit 3.
  --dev                read tokens.css and the QSS template from the source
                       tree and hot-reload them on change
  --style-dir DIR      like --dev, but read <DIR>/app.qss
  --gl                 use the real QVTKOpenGLNativeWidget for 3D panes
                       (needs a display; qt_shot.sh --gl runs it under xvfb)
  --list-pages         print the registered page names and exit

Screenshots normally run with QT_QPA_PLATFORM=offscreen (qt_shot.sh sets it);
3D panes then render through VTK's EGL offscreen window into an image.
)";
}

} // namespace rux::qt
