// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/launch.hpp>

namespace rux::qt {
namespace {

bool contains(std::string_view hay, std::string_view needle) {
  return hay.find(needle) != std::string_view::npos;
}

} // namespace

bool has_display(const LaunchInputs &in) {
  return !in.display.empty() || !in.wayland_display.empty() ||
         in.qpa_platform == "offscreen";
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
