// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "export.hpp"
#include "export/materialepas.hpp"
#include "export/rhino.hpp"
#include "export/speckle.hpp"

void setup_subcommand_export(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = app.add_subcommand(
      "export",
      "Export data to various formats (Rhino 3DM, Speckle, Materialepas JSON)");

  setup_subcommand_export_materialepas(*sub, global_opt);
  setup_subcommand_export_rhino(*sub, global_opt);
  setup_subcommand_export_speckle(*sub, global_opt);

  sub->require_subcommand(1);
}
