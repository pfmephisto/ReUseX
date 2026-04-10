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
      "Export data to external formats");

  sub->footer(R"(
DESCRIPTION:
  Parent command for exporting data from ReUseX project databases to
  external formats. Supports CAD (Rhino 3DM), web platforms (Speckle),
  and Danish material passport interchange format (JSON).

SUBCOMMANDS:
  rhino         Export labeled point cloud to Rhino 3DM format
  speckle       Export point cloud or mesh to Speckle platform
  materialepas  Export material passports to JSON file

EXAMPLES:
  rux export rhino -o model.3dm        # Export to Rhino CAD
  rux export speckle -p PROJECT_ID     # Upload to Speckle
  rux export materialepas -o data.json # Export passports

NOTES:
  - Use 'rux export <subcommand> --help' for detailed options
  - Export typically performed after processing pipeline completes
  - Multiple export formats can be used for different workflows
)");

  setup_subcommand_export_materialepas(*sub, global_opt);
  setup_subcommand_export_rhino(*sub, global_opt);
  setup_subcommand_export_speckle(*sub, global_opt);

  sub->require_subcommand(1);
}
