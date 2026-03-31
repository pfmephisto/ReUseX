// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "add.hpp"
#include "add/cloud.hpp"

void setup_subcommand_add(CLI::App &app) {
  auto *sub = app.add_subcommand(
      "add", "Add new data to project database");

  // Register all subcommands
  setup_subcommand_add_cloud(*sub);

  sub->require_subcommand(1);
}
