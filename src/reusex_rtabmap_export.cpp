#include "core/fmt_formatter.hh"
#include "core/spdmon.hh"
#include <CLI/CLI.hpp>
#include <filesystem>
#include <fmt/format.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/util3d.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;
using namespace rtabmap;

int main(int argc, char **argv) {
  spdlog::trace("reusex_rtabmap_export started");

  CLI::App app{"This tool exports a rtab-map database to a pcl point cloud for "
               "use in reusex."};

  argv = app.ensure_utf8(argv);
  app.get_formatter()->column_width(40);

  fs::path database_path;
  app.add_option("database", database_path,
                 "Path to the RTAB-Map database file.")
      ->required()
      ->check(CLI::ExistingFile);

  fs::path out_cloud_path;
  app.add_option("cloud", out_cloud_path,
                 "Path to the output point cloud file (default: cloud.pcd)")
      ->default_val(fs::current_path() / "cloud.pcd")
      ->check(CLI::NonexistentPath);

  fs::path out_normals_path;
  app.add_option("normals", out_normals_path,
                 "Path to the output normals file (default: normals.pcd)")
      ->default_val(fs::current_path() / "normals.pcd")
      ->check(CLI::NonexistentPath);

  fs::path out_trajectory_path;
  app.add_option("trajectory", out_trajectory_path,
                 "Path to the output trajectory file (default: trajectory.txt)")
      ->default_val(fs::current_path() / "trajectory.txt")
      ->check(CLI::NonexistentPath);

  float grid_size = 0.01f;
  app.add_option("-g,--grid", grid_size,
                 fmt::format("Voxel grid size for downsampling the point cloud "
                             "(default: {:.3f}",
                             grid_size)
                     .c_str())
      ->default_val(0.01f);

  int verbosity = 0;
  app.add_flag("-v,--verbose", verbosity,
               "Increase verbosity, use -vv & -vvv "
               "for more verbosity")
      ->multi_option_policy(CLI::MultiOptionPolicy::Sum);

  CLI11_PARSE(app, argc, argv);

  verbosity = std::max(verbosity, 0);
  verbosity = std::min(verbosity, 3);
  verbosity = 3 - verbosity; // 0=trace, 1=debug,
  // 0 > trace, 1 > debug, 2 > info, 3 > warn, 4 > err, 5 > critical, 6 > off

  spdlog::set_level(static_cast<spdlog::level::level_enum>(verbosity));

  spdlog::debug("Database path: {}", database_path);

  ParametersMap params;

  Rtabmap rtabmap;
  rtabmap.init(params, database_path.c_str());
  rtabmap.setWorkingDirectory("./");

  // Save 3D map
  spdlog::info("Processing database");
  std::map<int, Signature> nodes;
  std::map<int, Transform> optimizedPoses;
  std::multimap<int, Link> links;
  rtabmap.getGraph(optimizedPoses, links, true, true, &nodes, true, true, true,
                   true);

  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr normals(
      new pcl::PointCloud<pcl::PointXYZLNormal>);

  {
    auto logger = spdmon::LoggerProgress(
        "Assembling point clouds from signatures", nodes.size());

    for (std::map<int, Transform>::iterator iter = optimizedPoses.begin();
         iter != optimizedPoses.end(); ++iter) {
      Signature node = nodes.find(iter->first)->second;

      // uncompress data
      node.sensorData().uncompressData();

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp =
          util3d::cloudRGBFromSensorData(
              node.sensorData(),
              4,    // image decimation before creating the clouds
              4.0f, // maximum depth of the cloud
              0.0f);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      std::vector<int> index;
      pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, index);
      if (!tmpNoNaN->empty()) {

        tmpNoNaN = util3d::transformPointCloud(tmpNoNaN, iter->second);
        pcl::PointCloud<pcl::PointXYZRGBL> tmpCloud;
        pcl::copyPointCloud(*tmpNoNaN, tmpCloud);

        // Set sensor origion for the normal estimation
        float x, y, z;
        iter->second.getTranslation(x, y, z);
        tmpCloud.sensor_origin_ = Eigen::Vector4f(x, y, z, 1.0f);

        // Normal estimation
        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointXYZLNormal> ne;
        ne.setInputCloud(tmpNoNaN);
        ne.setRadiusSearch(0.1);
        pcl::PointCloud<pcl::PointXYZLNormal> normalsTmp;
        pcl::copyPointCloud(*tmpNoNaN, normalsTmp);
        ne.compute(normalsTmp);

        *cloud += tmpCloud;
        *normals += normalsTmp;

        ++logger;
      }
    }
  }

  if (cloud->size()) {
    spdlog::info(
        "Voxel grid filtering of the assembled cloud (voxel={}, {} points)",
        0.01f, (int)cloud->size());

    pcl::VoxelGrid<pcl::PointXYZRGBL> voxelGrid;
    voxelGrid.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelGrid.setInputCloud(cloud);
    voxelGrid.filter(*cloud);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices());
    voxelGrid.getRemovedIndices(*indices);

    spdlog::debug("Points after voxel grid filtering: {}", cloud->size());
    spdlog::debug("Removed {} points after voxel grid filtering",
                  indices->indices.size());

    pcl::ExtractIndices<pcl::PointXYZLNormal> extract;
    extract.setInputCloud(normals);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*normals);

    spdlog::debug("Normals size after filtering: {}", normals->size());

    // cloud = util3d::voxelize(cloud, 0.01f);

    spdlog::info("Saving {} ({})", out_cloud_path, cloud->size());
    pcl::io::savePCDFile(out_cloud_path, *cloud, true);

    spdlog::info("Saving {} ({})", out_normals_path, normals->size());
    pcl::io::savePCDFile(out_normals_path, *normals, true);
  } else {
    spdlog::error("Saving clouds failed! The clouds are empty.");
  }

  // Save trajectory
  spdlog::info("Saving {} ...", out_trajectory_path);
  if (optimizedPoses.size() &&
      graph::exportPoses(out_trajectory_path, 0, optimizedPoses, links)) {
  } else {
    spdlog::error("Saving {} ... failed!", out_trajectory_path);
  }

  rtabmap.close();

  return 0;
}
