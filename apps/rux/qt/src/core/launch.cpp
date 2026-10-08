// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/launch.hpp>

namespace rux::qt {
namespace {

bool contains(std::string_view hay, std::string_view needle) {
  return hay.find(needle) != std::string_view::npos;
}

bool exists(const LaunchInputs &in, const std::string &path) {
  return !in.path_exists || in.path_exists(path);
}

bool wayland_ok(const LaunchInputs &in) {
  if (in.wayland_display.empty())
    return false;
  std::string sock(in.wayland_display);
  if (sock.front() != '/') {
    if (in.xdg_runtime_dir.empty())
      return false; // libwayland cannot resolve a relative name either
    sock = std::string(in.xdg_runtime_dir) + "/" + sock;
  }
  return exists(in, sock);
}

bool x11_ok(const LaunchInputs &in) {
  const std::string_view d = in.display;
  const auto colon = d.rfind(':');
  if (d.empty() || colon == std::string_view::npos)
    return false;
  const std::string_view host = d.substr(0, colon);
  if (!host.empty() && host != "unix")
    return true; // TCP / ssh -X forwarding: cannot check cheaply, trust it
  std::string_view num = d.substr(colon + 1);
  num = num.substr(0, num.find('.'));
  if (num.empty() ||
      num.find_first_not_of("0123456789") != std::string_view::npos)
    return false;
  return exists(in, "/tmp/.X11-unix/X" + std::string(num));
}

bool platform_ok(const LaunchInputs &in, std::string_view p) {
  p = p.substr(0, p.find(':')); // "offscreen:fontengine=freetype"
  if (p == "offscreen" || p == "minimal")
    return true;
  if (p == "xcb")
    return x11_ok(in);
  if (p.substr(0, 7) == "wayland") // wayland, wayland-egl, wayland-brcm
    return wayland_ok(in);
  return true; // eglfs, linuxfb, vnc, …: the user asked for it explicitly
}

} // namespace

bool has_display(const LaunchInputs &in) {
  std::string_view q = in.qpa_platform;
  if (q.empty())
    return x11_ok(in) || wayland_ok(in);
  // A ';'-separated list: Qt tries each in turn.
  while (true) {
    const auto semi = q.find(';');
    const std::string_view one = q.substr(0, semi);
    if (!one.empty() && platform_ok(in, one))
      return true;
    if (semi == std::string_view::npos)
      return false;
    q.remove_prefix(semi + 1);
  }
}

LaunchAction decide_launch(const LaunchInputs &in) {
  if (in.has_subcommand)
    return LaunchAction::run_subcommand;
  if (in.qt_client_built && has_display(in))
    return LaunchAction::open_app;
  return LaunchAction::print_help;
}

OpenErrorKind classify_open_error(std::string_view what) {
  // sqlite's own wording, as ProjectDB passes it through.
  if (contains(what, "is locked") || contains(what, "database is busy") ||
      contains(what, "SQLITE_BUSY"))
    return OpenErrorKind::locked;
  if (contains(what, "file is not a database") ||
      contains(what, "Required table"))
    return OpenErrorKind::not_a_project;
  if (contains(what, "malformed") || contains(what, "disk I/O error"))
    return OpenErrorKind::corrupt;
  if (contains(what, "unable to open database file") ||
      contains(what, "readonly database") ||
      contains(what, "read-only database") ||
      contains(what, "Permission denied") ||
      contains(what, "access permission denied"))
    return OpenErrorKind::permission;
  return OpenErrorKind::other;
}

std::string open_error_title_da(OpenErrorKind kind) {
  switch (kind) {
  case OpenErrorKind::none:
    return {};
  case OpenErrorKind::not_found:
    return "Filen findes ikke";
  case OpenErrorKind::not_a_file:
    return "Det er ikke en projektfil";
  case OpenErrorKind::permission:
    return "Ingen adgang til projektet";
  case OpenErrorKind::locked:
    return "Projektet er låst af en anden proces";
  case OpenErrorKind::not_a_project:
    return "Filen er ikke et ReUseX-projekt";
  case OpenErrorKind::corrupt:
    return "Projektfilen er beskadiget";
  case OpenErrorKind::other:
    return "Projektet kunne ikke åbnes";
  }
  return "Projektet kunne ikke åbnes";
}

std::string open_error_hint_da(OpenErrorKind kind) {
  switch (kind) {
  case OpenErrorKind::none:
    return {};
  case OpenErrorKind::not_found:
    return "Den er måske flyttet, slettet eller ligger på et drev, der ikke "
           "er tilsluttet.";
  case OpenErrorKind::not_a_file:
    return "Vælg en .rux-fil, ikke en mappe.";
  case OpenErrorKind::permission:
    return "Tjek at du har læseadgang til filen og mappen, den ligger i.";
  case OpenErrorKind::locked:
    return "En anden rux- eller ruxd-proces skriver til den lige nu. Vent til "
           "den er færdig, eller åbn projektet skrivebeskyttet.";
  case OpenErrorKind::not_a_project:
    return "Vælg en .rux-fil oprettet med rux import.";
  case OpenErrorKind::corrupt:
    return "Åbn en sikkerhedskopi, eller kør rux validate for detaljer.";
  case OpenErrorKind::other:
    return "Se detaljerne herunder. Kør rux -vv -p <fil> info for mere.";
  }
  return {};
}

} // namespace rux::qt
