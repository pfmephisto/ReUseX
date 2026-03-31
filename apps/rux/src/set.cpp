// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "set.hpp"
#include "set/project.hpp"

void setup_subcommand_set(CLI::App &app) {
  auto *sub = app.add_subcommand(
      "set", "Update project database properties");

  // Register all subcommands
  setup_subcommand_set_project(*sub);

  sub->require_subcommand(1);
}
