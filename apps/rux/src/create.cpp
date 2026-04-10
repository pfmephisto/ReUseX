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
      "create", "Create derived data from project");

  sub->footer(R"(
DESCRIPTION:
  Parent command for creating derived data products from ReUseX project
  databases. Process sensor data through reconstruction, segmentation,
  annotation, and mesh generation pipelines.

SUBCOMMANDS:
  clouds       Reconstruct 3D point clouds from depth images
  annotate     Run ML inference on sensor frames
  planes       Detect and segment planar surfaces
  rooms        Segment rooms using Leiden clustering
  instances    Separate labels into spatial instances
  mesh         Generate watertight mesh from planes
  texture      Apply textures to mesh from sensor frames
  project      Project 2D labels onto 3D point cloud
  material     Create blank material passport
  windows      Create window building components

TYPICAL WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct geometry
  3. rux create planes                 # Segment surfaces
  4. rux create rooms                  # Segment rooms
  5. rux create mesh                   # Generate mesh

NOTES:
  - Use 'rux create <subcommand> --help' for detailed options
  - Commands read from and write to project database
  - Most commands depend on earlier pipeline stages
)");

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
