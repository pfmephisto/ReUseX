// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "edit.hpp"
#include "edit/downsample.hpp"

void setup_subcommand_edit(CLI::App &app,
                           std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = app.add_subcommand(
      "edit", "Edit clouds already stored in the project");

  sub->footer(R"(
DESCRIPTION:
  Parent command for cloud-level edits that transform an existing cloud
  in the project database into another cloud of the same type. Edits
  read from and write to the project specified by -p/--project.

SUBCOMMANDS:
  downsample    Voxel-grid downsample, with optional parallel clouds

EXAMPLES:
  rux edit downsample -r 0.05
  rux edit downsample -r 0.05 --with normals
  rux edit downsample -i cloud -o cloud_5cm -r 0.05 --with normals:normals_5cm

NOTES:
  - Use 'rux edit <subcommand> --help' for detailed options
  - Edits operate in place by default; pass -o/--output to write a new cloud
)");

  setup_subcommand_edit_downsample(*sub, global_opt);

  sub->require_subcommand(1);
}
