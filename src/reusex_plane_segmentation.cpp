#include "core/fmt_formatter.hh"
#include "core/planar_region_growing.hh"
#include "core/spdmon.hh"

#include <CLI/CLI.hpp>
#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

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

struct Params {
  fs::path path_in;
  fs::path path_out = fs::current_path() / "planes.pcd";

  float angle_threshold = 25.0f;
  float plane_dist_threshold = 0.07;
  int minInliers = 1000;
  // 2 * (1 / 0.02) * (1 / 0.02); // ca 2sqm in 2cm resolution of point cloud
  float radius = 0.5;
  float interval_0 = 16;
  float interval_factor = 1.5;

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

  app->add_option("output", params.path_out,
                  "Path to the output point cloud file")
      ->default_val(params.path_out);

  app->add_option("-a, --angle-threshold", params.angle_threshold,
                  "Angle threshold for plane fitting (default: 25° aka. "
                  "cos(25) or 0.96592583)")
      ->default_val(params.angle_threshold)
      ->check(CLI::Range(0.0, 365.0));

  app->add_option("-d, --plane-dist-threshold", params.plane_dist_threshold,
                  "Distance threshold for plane fitting (default: 0.1m)")
      ->default_val(params.plane_dist_threshold)
      ->check(CLI::Range(0.0, 1.0));

  app->add_option("-m, --min-cluster-size", params.minInliers,
                  "Minimum cluster size for plane fitting (default: 2sqm in "
                  "2cm resolution of point cloud)")
      ->default_val(params.minInliers)
      ->check(CLI::Range(3, 1000000));

  app->add_option("-r, --radius", params.radius, "Radius for region growing")
      ->default_val(params.radius)
      ->check(CLI::Range(0.0, 1.0));

  app->add_option("-i, --interval-0", params.interval_0,
                  "Initial interval for plane update")
      ->default_val(params.interval_0)
      ->check(CLI::Range(1.0, 10000.0));

  app->add_option("-f, --interval-factor", params.interval_factor,
                  "Factor for interval update")
      ->default_val(params.interval_factor)
      ->check(CLI::Range(1.0, 10.0));

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
  spdlog::trace("reusex_plane_segmentation started");
  spdlog::info("Verbosity level: {}",
               spdlog::level::to_string_view(spdlog::get_level()));

  spdlog::trace("Load the point cloud from disk");
  CloudPtr cloud(new Cloud);
  // pcl::io::loadPCDFile<PointT>(config.path_in.string(), *cloud);
  pcl::io::load<PointT>(config.path_in.string(), *cloud);

  // pcl::Indices indices;
  // pcl::removeNaNNormalsFromPointCloud(*cloud, *cloud, indices);

  spdlog::trace("Initialize the segmentation algorithm");
  pcl::PlanarRegionGrowing<PointT, NormalT, LabelT> seg;
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud);

  seg.setAngularThreshold(config.angle_threshold);
  seg.setDistanceThreshold(config.plane_dist_threshold);
  seg.setMinInliers(config.minInliers);

  seg.setRadiusSearch(config.radius);

  seg.setInitialInterval(config.interval_0);
  seg.setIntervalFactor(config.interval_factor);

  // pcl::IndicesPtr indices(new pcl::Indices);
  // for (size_t i = 0; i < 1000; ++i) {
  //   indices->push_back(static_cast<int>(i));
  // }
  // seg.setIndices(indices);

  spdlog::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*cloud, *labels);

  spdlog::trace("Call the segmentation algorithm");
  seg.segment(labels);
  spdlog::info("Found {} clusters", seg.getCentroids().size());

  spdlog::trace("Save the point cloud with planes");
  pcl::io::savePCDFileBinary(config.path_out, *labels);

  return 0;
}
