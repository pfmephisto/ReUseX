// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create.hpp"
#include "create/clouds.hpp"

void setup_subcommand_create(CLI::App &app) {
  auto *sub = app.add_subcommand(
      "create", "Create derived data products from a ReUseX project.");

  setup_subcommand_create_clouds(*sub);

  sub->require_subcommand(1);
}
