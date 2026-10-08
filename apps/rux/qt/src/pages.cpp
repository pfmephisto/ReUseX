// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/pages.hpp>

namespace rux::qt {
namespace {
std::vector<PageInfo> &registry() {
  static std::vector<PageInfo> r;
  return r;
}
} // namespace

void register_page(PageInfo page) { registry().push_back(std::move(page)); }

const std::vector<PageInfo> &pages() { return registry(); }

const PageInfo *find_page(const QString &name) {
  for (const auto &p : registry())
    if (p.name == name)
      return &p;
  return nullptr;
}

} // namespace rux::qt
