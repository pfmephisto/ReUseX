// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/recent.hpp>

#include <algorithm>
#include <filesystem>

namespace rux::qt {

std::string normalise_project_path(const std::string &path,
                                   const std::string &cwd) {
  if (path.empty())
    return {};
  std::filesystem::path p(path);
  if (p.is_relative())
    p = std::filesystem::path(cwd) / p;
  std::string out = p.lexically_normal().string();
  // lexically_normal keeps a trailing separator ("a/b/" stays); drop it so a
  // path to a file and the same path with a slash compare equal.
  while (out.size() > 1 && out.back() == '/')
    out.pop_back();
  return out;
}

std::vector<std::string> recent_remove(std::vector<std::string> list,
                                       const std::string &path) {
  list.erase(std::remove(list.begin(), list.end(), path), list.end());
  return list;
}

std::vector<std::string> recent_push(std::vector<std::string> list,
                                     const std::string &path, std::size_t max) {
  list.erase(std::remove(list.begin(), list.end(), std::string()), list.end());
  if (!path.empty()) {
    list = recent_remove(std::move(list), path);
    list.insert(list.begin(), path);
  }
  if (list.size() > max)
    list.resize(max);
  return list;
}

std::vector<std::string> recent_sanitise(const std::vector<std::string> &list,
                                         const std::string &cwd,
                                         std::size_t max) {
  std::vector<std::string> out;
  for (const auto &raw : list) {
    const std::string p = normalise_project_path(raw, cwd);
    if (p.empty() || std::find(out.begin(), out.end(), p) != out.end())
      continue;
    out.push_back(p);
    if (out.size() == max)
      break;
  }
  return out;
}

std::vector<RecentEntry>
recent_annotate(const std::vector<std::string> &list,
                const std::function<bool(const std::string &)> &exists) {
  std::vector<RecentEntry> out;
  out.reserve(list.size());
  for (const auto &p : list)
    out.push_back({p, !exists(p)});
  return out;
}

} // namespace rux::qt
