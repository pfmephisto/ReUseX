// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "remove.hpp"
#include "remove/cloud.hpp"

void setup_subcommand_remove(CLI::App &app) {
  auto *sub = app.add_subcommand(
      "remove", "Delete data from project database");

  // Register all subcommands
  setup_subcommand_remove_cloud(*sub);

  sub->require_subcommand(1);
}
