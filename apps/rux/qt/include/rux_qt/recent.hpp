// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The recent-projects list — the Qt-free half. The Qt side
// (RecentProjects) only loads and stores the list in QSettings.
//
// Most recent first, at most kMaxRecent entries, no duplicates. Paths are
// compared after lexical normalisation ("a/./b.rux" == "a/b.rux"), not by
// resolving symlinks: a project on an unmounted drive must stay in the list
// (marked missing) rather than vanish or throw.

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace rux::qt {

inline constexpr std::size_t kMaxRecent = 10;

/// Lexically normalised absolute form of @p path (relative paths are taken
/// against @p cwd). Never touches the file system.
std::string normalise_project_path(const std::string &path,
                                   const std::string &cwd);

/// @p list with @p path moved (or added) to the front, duplicates removed,
/// capped at @p max entries. Empty paths in @p list are dropped.
std::vector<std::string> recent_push(std::vector<std::string> list,
                                     const std::string &path,
                                     std::size_t max = kMaxRecent);

/// @p list without @p path.
std::vector<std::string> recent_remove(std::vector<std::string> list,
                                       const std::string &path);

/// A stored list cleaned for use: normalised, de-duplicated (first wins),
/// empty entries dropped, capped. Settings written by hand or by an older
/// build go through this on load.
std::vector<std::string> recent_sanitise(const std::vector<std::string> &list,
                                         const std::string &cwd,
                                         std::size_t max = kMaxRecent);

struct RecentEntry {
  std::string path;
  bool missing = false; ///< the file is gone (deleted, moved, unmounted)
};

/// Pair each path with whether it still exists, asked through @p exists so
/// tests need no files.
std::vector<RecentEntry>
recent_annotate(const std::vector<std::string> &list,
                const std::function<bool(const std::string &)> &exists);

} // namespace rux::qt
