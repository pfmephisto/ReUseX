// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Fuzzy ranking for the command palette (Ctrl+K) — the Qt-free half.
//
// A query matches a candidate when its characters appear in order in the
// candidate's title (a subsequence, case-insensitive, spaces in the query
// ignored). Matches score higher when they start words, run consecutively
// and start early, so "db" ranks "Database" above "Ændr billede". A query
// that does not match the title may still match the candidate's keywords
// (aliases, a file path) at a lower score and without highlight positions.
//
// Danish letters fold: "o" in the query matches "ø", "a" matches "å" and
// "æ", so "korsel" finds "Kørselslog" on a keyboard without them.
//
// Positions are code-point indices into the title, which equal QString
// (UTF-16) indices for every character outside the astral planes.

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

struct PaletteCandidate {
  std::string title;    ///< UTF-8; matched and highlighted
  std::string keywords; ///< UTF-8; matched at a penalty, never highlighted
};

struct PaletteMatch {
  std::size_t index = 0;      ///< into the candidate list
  int score = 0;              ///< higher is better
  std::vector<int> positions; ///< matched title code points; empty for a
                              ///< keyword match or an empty query
};

/// Rank @p candidates for @p query, best first. An empty (or all-space)
/// query returns every candidate in its original order with score 0.
/// Candidates that do not match are left out. Ties keep the shorter title
/// first, then the original order — the result is deterministic.
std::vector<PaletteMatch>
rank_palette(std::string_view query,
             const std::vector<PaletteCandidate> &candidates);

/// Score one title, or -1 if @p query is not a subsequence of it. Exposed for
/// tests; @p positions receives the chosen alignment.
int fuzzy_score(std::string_view query, std::string_view text,
                std::vector<int> *positions = nullptr);

} // namespace rux::qt
