// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/surface_reconstruction.hpp"

#include "ReUseX/fmt_formatter.hpp"
#include "pcl/polygonal_surface_reconstruction.hpp"
#include "spdmon/spdmon.hpp"

#include <CLI/CLI.hpp>
#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/PolygonMesh.h>
#include <pcl/filters/filter.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>

namespace fs = std::filesystem;

using PointT = pcl::PointXYZRGBNormal;
using NormalT = pcl::PointXYZRGBNormal;
using LabelT = pcl::PointXYZRGBL;

using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = typename Cloud::Ptr;
using CloudConstPtr = typename Cloud::ConstPtr;

using CloudN = pcl::PointCloud<NormalT>;
using CloudNPtr = typename CloudN::Ptr;
using CloudNConstPtr = typename CloudN::ConstPtr;

using CloudL = pcl::PointCloud<LabelT>;
using CloudLPtr = typename CloudL::Ptr;
using CloudLConstPtr = typename CloudL::ConstPtr;

using Mesh = pcl::PolygonMesh;
using MeshPtr = typename Mesh::Ptr;
using MeshConstPtr = typename Mesh::ConstPtr;

void setup_subcommand_surface_reconstruction(CLI::App &app) {

  auto opt = std::make_shared<SubcommandSurfaceReconstructionOptions>();
  auto *sub =
      app.add_subcommand("surface_reconctruction",
                         "Run the polygonal surface reconstruction algorithm");

  sub->add_option("input", opt->path_in, "Path to the input point cloud file.")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("input-labels", opt->path_in_labels,
                  "Path to the input labels file")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("output", opt->path_out, "Path to the output mesh file")
      ->default_val(opt->path_out);

  sub->add_option("-f, --fitting", opt->fitting, "")
      ->default_val(opt->fitting)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-c, --coverage", opt->coverage, "")
      ->default_val(opt->coverage)
      ->check(CLI::Range(0.0, 1.0));

  sub->add_option("-x, --complexity", opt->complexity, "")
      ->default_val(opt->complexity)
      ->check(CLI::Range(0.0, 1.0));

  sub->callback([opt]() {
    spdlog::trace("calling surface_reconctruction subcommand");
    return run_subcommand_surface_reconstruction(*opt);
  });
}

int run_subcommand_surface_reconstruction(
    SubcommandSurfaceReconstructionOptions const &opt) {

  spdlog::trace("Load the point cloud from disk");
  CloudPtr cloud(new Cloud);
  pcl::io::load<PointT>(opt.path_in.string(), *cloud);

  spdlog::trace("Load the labels from disk");
  CloudLPtr labels(new CloudL);
  pcl::io::load<LabelT>(opt.path_in_labels.string(), *labels);

  if (cloud->empty() || labels->empty()) {
    spdlog::error("Input point cloud or labels are empty.");
    return 1;
  }

  if (cloud->size() != labels->size()) {
    spdlog::error("Input point cloud and labels must have the same size.");
    return 1;
  }

  spdlog::trace("Initialize the polygonal surface reconstruction algorithm");
  pcl::PolygonalSurfaceReconstruction<PointT, NormalT, LabelT> psr;
  psr.setInputCloud(cloud);
  psr.setInputNormals(cloud);
  psr.setInputLabels(labels);

  psr.setFittingParam(opt.fitting);
  psr.setCoverageParam(opt.coverage);
  psr.setComplexityParam(opt.complexity);

  spdlog::trace("Call the segmentation algorithm");
  MeshPtr mesh(new Mesh);
  psr.segment(*mesh);

  // spdlog::trace("Initialize labels and copy xyzrgb data to labels");
  // CloudLPtr labels(new CloudL);
  // pcl::copyPointCloud(*cloud, *labels);

  // spdlog::info("Found {} clusters", seg.getCentroids().size());

  spdlog::trace("Save the mesh to disk");
  pcl::io::save(opt.path_out, *mesh);

  return 0;
}
