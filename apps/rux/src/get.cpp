// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "get.hpp"
#include "get/clouds.hpp"

void setup_subcommand_get(CLI::App &app) {
  auto *sub = app.add_subcommand(
      "get", "Read and list project database contents");

  // Register all subcommands
  setup_subcommand_get_clouds(*sub);

  sub->require_subcommand(1);
}
