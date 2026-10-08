// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// What the command palette lists — the Qt-free table. AppShell maps each
// entry's `id` to the code that runs it; the ranking tests rank this exact
// table, so a change to a title or a keyword is caught by them.
//
// Ids: "page:<n>" (nav order, 0 = Start), "open", "reload", "close",
// "copy-path", "copy-cli", "inspector", "theme", "quit", "recent:<path>".

#include <rux_qt/fuzzy.hpp>
#include <rux_qt/recent.hpp>

#include <string>
#include <vector>

namespace rux::qt {

/// One nav page as the palette and the rail name it.
struct PageEntry {
  std::string name;     ///< "Database"
  std::string keywords; ///< aliases, Danish and English
};

/// The shell's pages in rail order (Start, Database, 3D, Posegraf,
/// Pipeline, Log). Alt+1 … Alt+6.
const std::vector<PageEntry> &page_entries();

struct PaletteEntry {
  std::string id;
  std::string group;    ///< "Sider" | "Handlinger" | "Seneste projekter"
  std::string title;    ///< ranked and highlighted
  std::string keywords; ///< ranked as one contiguous run, never shown
  std::string subtitle; ///< shown (a path, a command); NOT searchable
  std::string shortcut; ///< "Ctrl+O"; also the QAction's key sequence
  std::string badge;    ///< "Her" | "Åben" | "Mangler" | ""
  bool enabled = true;
};

struct PaletteState {
  bool project_open = false;
  bool project_failed = false;
  std::string project_path; ///< absolute; empty when none
  int current_page = 0;
  bool inspector_visible = true;
  bool dark_theme = true;
  std::vector<RecentEntry> recent;
};

/// Every page, the actions that apply in @p s, and the recent projects.
std::vector<PaletteEntry> build_palette(const PaletteState &s);

/// The ranking input for @p entries: title + keywords. Subtitles are not
/// searchable — full paths would make "home" or "tmp" match every row.
std::vector<PaletteCandidate>
palette_candidates(const std::vector<PaletteEntry> &entries);

/// The title the palette and the start page give a recent project: the
/// directory name for a generic "project.rux", else the file's base name.
std::string recent_project_title(const std::string &path);

/// The shortcut of an action id, as build_palette lists it ("" if none).
std::string action_shortcut(const std::string &id);

} // namespace rux::qt
