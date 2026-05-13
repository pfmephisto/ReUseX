// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "filter.hpp"
#include "filter/downsample.hpp"

void setup_subcommand_filter(CLI::App &app,
                             std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = app.add_subcommand(
      "filter", "Apply transformations to clouds already in the project");

  sub->footer(R"(
DESCRIPTION:
  Parent command for cloud-level filters that transform an existing cloud
  in the project database into another cloud of the same type. Filters
  read from and write to the project specified by -p/--project.

SUBCOMMANDS:
  downsample    Voxel-grid downsample, with optional parallel clouds

EXAMPLES:
  rux filter downsample -r 0.05
  rux filter downsample -r 0.05 --with normals
  rux filter downsample -i cloud -o cloud_5cm -r 0.05 --with normals:normals_5cm

NOTES:
  - Use 'rux filter <subcommand> --help' for detailed options
  - Filters operate in place by default; pass -o/--output to write a new cloud
)");

  setup_subcommand_filter_downsample(*sub, global_opt);

  sub->require_subcommand(1);
}
