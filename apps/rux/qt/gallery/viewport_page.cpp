// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The Q0 "viewport" page was a stand-in for the 3D workspace; since Q3 it is
// the real one (the shell on its 3D page), kept under its old name so older
// shot scripts still find it. Also the gallery's page registration.

#include "demo_pages.hpp"

namespace rux::qt::gallery {

QWidget *make_viewport_page(const PageContext &ctx) {
  return make_shell_3d(ctx);
}

void register_demo_pages() {
  register_page({"components", "Alle delte komponenter med projektdata",
                 make_components_page});
  register_page(
      {"viewport", "3D-arbejdsområdet (samme som 3d)", make_viewport_page});
  register_shell_pages();
}

} // namespace rux::qt::gallery
