// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create.hpp"
#include "create/annotate.hpp"
#include "create/clouds.hpp"
#include "create/instances.hpp"
#include "create/material.hpp"
#include "create/mesh.hpp"
#include "create/planes.hpp"
#include "create/project.hpp"
#include "create/rooms.hpp"
#include "create/texture.hpp"

void setup_subcommand_create(CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto *sub = app.add_subcommand(
      "create", "Create derived data products from a ReUseX project");

  // Register all subcommands (ordered by pipeline flow)
  setup_subcommand_create_clouds(*sub, global_opt);
  setup_subcommand_create_annotate(*sub, global_opt);
  setup_subcommand_create_material(*sub, global_opt);
  setup_subcommand_create_project(*sub, global_opt);
  setup_subcommand_create_planes(*sub, global_opt);
  setup_subcommand_create_rooms(*sub, global_opt);
  setup_subcommand_create_instances(*sub, global_opt);
  setup_subcommand_create_mesh(*sub, global_opt);
  setup_subcommand_create_texture(*sub, global_opt);

  sub->require_subcommand(1);
}
