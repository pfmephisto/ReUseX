#include "ReUseX/fmt_formatter.hh"
#include "pcl/polygonal_surface_reconstruction.hh"
#include "spdmon/spdmon.hh"

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

struct Params {
  fs::path path_in;
  fs::path path_in_labels;
  fs::path path_out = fs::current_path() / "mesh.ply";

  double fitting = 0.20;
  double coverage = 0.10;
  double complexity = 0.70;

  int verbosity = 0;
};

std::string shell;
CLI::App *comp = nullptr;

std::unique_ptr<CLI::App> initApp(Params &params) {

  auto app = std::make_unique<CLI::App>(
      "This tool exports a rtab-map database to a pcl point cloud for "
      "use in reusex.");

  app->get_formatter()->column_width(40);

  app->add_option("input", params.path_in,
                  "Path to the input point cloud file.")
      ->required()
      ->check(CLI::ExistingFile);

  app->add_option("input-labels", params.path_in_labels,
                  "Path to the input labels file")
      ->required()
      ->check(CLI::ExistingFile);

  app->add_option("output", params.path_out, "Path to the output mesh file")
      ->default_val(params.path_out);

  app->add_option("-f, --fitting", params.fitting, "")
      ->default_val(params.fitting)
      ->check(CLI::Range(0.0, 1.0));

  app->add_option("-c, --coverage", params.coverage, "")
      ->default_val(params.coverage)
      ->check(CLI::Range(0.0, 1.0));

  app->add_option("-x, --complexity", params.complexity, "")
      ->default_val(params.complexity)
      ->check(CLI::Range(0.0, 1.0));

  app->add_flag("-v,--verbose", params.verbosity,
                "Increase verbosity, use -vv & -vvv "
                "for more verbosity")
      ->multi_option_policy(CLI::MultiOptionPolicy::Sum)
      ->check(CLI::Range(0, 3));

  return app;
}

int main(int argc, char **argv) {

  Params config;
  auto app = initApp(config);
  argv = app->ensure_utf8(argv);
  CLI11_PARSE(*app, argc, argv);

  spdlog::set_level(
      static_cast<spdlog::level::level_enum>(3 - config.verbosity));
  spdlog::trace("reusex_polygonal_surface_reconstruction started");
  spdlog::info("Verbosity level: {}",
               spdlog::level::to_string_view(spdlog::get_level()));

  spdlog::trace("Load the point cloud from disk");
  CloudPtr cloud(new Cloud);
  pcl::io::load<PointT>(config.path_in.string(), *cloud);

  spdlog::trace("Load the labels from disk");
  CloudLPtr labels(new CloudL);
  pcl::io::load<LabelT>(config.path_in_labels.string(), *labels);

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

  psr.setFittingParam(config.fitting);
  psr.setCoverageParam(config.coverage);
  psr.setComplexityParam(config.complexity);

  spdlog::trace("Call the segmentation algorithm");
  MeshPtr mesh(new Mesh);
  psr.segment(*mesh);

  // spdlog::trace("Initialize labels and copy xyzrgb data to labels");
  // CloudLPtr labels(new CloudL);
  // pcl::copyPointCloud(*cloud, *labels);

  // spdlog::info("Found {} clusters", seg.getCentroids().size());

  spdlog::trace("Save the mesh to disk");
  pcl::io::save(config.path_out, *mesh);

  return 0;
}
