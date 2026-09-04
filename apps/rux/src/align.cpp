// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "align.hpp"
#include "align/panorama.hpp"

void setup_subcommand_align(CLI::App &app,
                            std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = app.add_subcommand(
      "align", "Content-based alignment of auxiliary data to the scan");

  sub->footer(R"(
DESCRIPTION:
  Parent command for aligning auxiliary captures to the scan by image content
  rather than metadata. Refines placements that were seeded from timestamps.

SUBCOMMANDS:
  360    Align 360 panoramas to the scan and refine their 6-DoF pose

NOTES:
  - Use 'rux align <subcommand> --help' for detailed options
)");

  setup_subcommand_align_panorama(*sub, global_opt);
}
