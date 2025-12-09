// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/texture.hpp"

#include <ReUseX/geometry/texture_mesh.hpp>
#include <pcl/io/auto_io.h>

#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/optimizer/OptimizerG2O.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <Eigen/Dense>

void setup_subcommand_texture(CLI::App &app) {
  auto opt = std::make_shared<SubcommandTextureOptions>();
  auto *sub = app.add_subcommand(
      "texture",
      "Apply textures to a mesh using views from an RTAB-Map database.");

  sub->add_option("mesh", opt->mesh_path_in, "Path to the input mesh file.")
      ->required()
      ->check(CLI::ExistingFile)
      ->default_val(opt->mesh_path_in);

  sub->add_option("db", opt->db_path_in, "Path to the RTAB-Map database file.")
      ->required()
      ->check(CLI::ExistingFile)
      ->default_val(opt->db_path_in);

  sub->add_option("output", opt->mesh_path_out, "Path to the output textured mesh file.")
      ->default_val(opt->mesh_path_out);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_texture");
    return run_subcommand_texture(*opt);
  });
}
int run_subcommand_texture(SubcommandTextureOptions const &opt) {

  using Poses = std::map<int, rtabmap::Transform>;
  using Nodes = std::map<int, rtabmap::Signature>;
  using Links = std::multimap<int, rtabmap::Link>;

  spdlog::info("Intializing RTAB-Map ...");
  spdlog::stopwatch timer;
  rtabmap::ParametersMap params;
  rtabmap::Rtabmap rtabmap;
  spdlog::debug("Database path: {}", opt.db_path_in.string());
  rtabmap.init(params, opt.db_path_in.c_str());
  rtabmap.setWorkingDirectory("./");
  spdlog::debug("RTAB-Map initialized in {:.3f}s", timer);

  // Save 3D map
  spdlog::info("Loading Graph");
  timer.reset();

  Poses poses;
  Links links;
  Nodes nodes;

  rtabmap.getGraph(poses /*poses*/, links /*constraints*/,
                   true
                   /*optimized*/,
                   true /*global*/, &nodes /*signatures*/,
                   true
                   /*withImages*/,
                   true /*withScan*/, true /*withUserData*/,
                   true /*withGrid*/ /*withWords*/
                   /*withGlobalDescriptors*/);
  spdlog::debug("Graph loaded in {:.3f}s", timer);

  auto mesh = std::make_shared<pcl::PolygonMesh>();
  pcl::io::load(opt.mesh_path_in.string(), *mesh);

  auto textured_mesh = ReUseX::geometry::texture_mesh(mesh, poses, nodes);

  spdlog::info("Saving textured mesh to {}", opt.mesh_path_out.string());
  pcl::io::save(opt.mesh_path_out.string(), *textured_mesh);

  rtabmap.close(false);
  return RuxError::SUCCESS;
}
