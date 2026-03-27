// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export.hpp"
#include "export/materialepas.hpp"
#include "export/rhino.hpp"
#include "export/speckle.hpp"

void setup_subcommand_export(CLI::App &app) {
  auto *sub = app.add_subcommand(
      "export",
      "Export data to various formats (Rhino 3DM, Speckle, Materialepas JSON)");

  setup_subcommand_export_materialepas(*sub);
  setup_subcommand_export_rhino(*sub);
  setup_subcommand_export_speckle(*sub);

  sub->require_subcommand(1);
}
