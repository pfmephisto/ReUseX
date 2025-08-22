#include "core/fmt_formatter.hh"
#include "core/markov_clustering.hh"
#include "core/spdmon.hh"

#include <CLI/CLI.hpp>
#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/pca.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

using Indices = pcl::Indices;
using IndicesPtr = pcl::IndicesPtr;
using IndicesConstPtr = pcl::IndicesConstPtr;

using PointT = pcl::PointXYZRGBL;
using NormalT = pcl::Normal;
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
  fs::path path_out = fs::current_path() / "clusters.pcd";

  float inflation = 2.0f;
  float grid_size = 0.2f;

  int verbosity = 0;
};

std::string shell;
CLI::App *comp = nullptr;

std::unique_ptr<CLI::App> initApp(Params &params) {

  auto app = std::make_unique<CLI::App>(
      "This tool applies the Markov clustering algorithm to a point cloud "
      "based in the visual relation between the points.");

  app->get_formatter()->column_width(40);

  app->add_option("input", params.path_in,
                  "Path to the input point cloud file.")
      ->required()
      ->check(CLI::ExistingFile);

  app->add_option("output", params.path_out,
                  "Path to the output point cloud file")
      ->default_val(params.path_out);

  app->add_option("-i, --inflation", params.inflation,
                  "The inflation factor for the MCL algorithm.")
      ->default_val(params.inflation)
      ->check(CLI::Range(0.0, 10.0));

  app->add_option("-g, --grid-size", params.grid_size,
                  "The grid size for the MCL algorithm.")
      ->default_val(params.grid_size)
      ->check(CLI::Range(0.01, 10.0));

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
  spdlog::trace("reusex_markov_clustering started");
  spdlog::info("Verbosity level: {}",
               spdlog::level::to_string_view(spdlog::get_level()));

  spdlog::trace("Load the point cloud from disk");
  CloudPtr cloud(new Cloud);
  pcl::io::load<PointT>(config.path_in.string(), *cloud);

  std::map<size_t, IndicesPtr> inlier_indices_map;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (cloud->points[i].label == 0)
      continue; // Skip points without label
    if (inlier_indices_map.find(cloud->points[i].label) ==
        inlier_indices_map.end())
      inlier_indices_map[cloud->points[i].label] = IndicesPtr(new Indices);
    inlier_indices_map[cloud->points[i].label]->push_back(i);
  }

  size_t number_of_segments = inlier_indices_map.size();
  spdlog::info("Number of segments found: {}", number_of_segments);

  spdlog::trace("Compute plane normals using PCA");
  CloudNPtr normals(new CloudN);
  normals->resize(cloud->size());
  pcl::PCA<PointT> pca;
  pca.setInputCloud(cloud);

  for (const auto &pair : inlier_indices_map) {

    pca.setIndices(pair.second);
    const Eigen::Vector3f &normal = pca.getEigenVectors().col(2).normalized();

    Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();

    for (const auto &index : *pair.second)
      avg_normal += cloud->points[index].getVector3fMap();
    avg_normal /= static_cast<float>(pair.second->size());

    const int flip = (avg_normal.dot(normal) < 0.0f) ? -1 : 1;

    for (const auto &index : *pair.second) {
      normals->points[index].normal_x = normal[0] * flip;
      normals->points[index].normal_y = normal[1] * flip;
      normals->points[index].normal_z = normal[2] * flip;
    }
  }

  // TODO: Filter the point cloud select only a few indices
  IndicesPtr indices(new Indices);
  pcl::UniformSampling<PointT> us;
  us.setInputCloud(cloud);
  us.setRadiusSearch(config.grid_size);
  us.filter(*indices);

  spdlog::trace("Initialize the Markov Clustering algorithm");
  pcl::MarkovClustering<PointT, NormalT, LabelT> mcl;
  mcl.setInflationFactor(config.inflation);
  mcl.setGridSize(config.grid_size);
  mcl.setInputCloud(cloud);
  mcl.setInputNormals(normals);
  mcl.setIndices(indices);

  spdlog::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*cloud, *labels);

  mcl.cluster(*labels);

  spdlog::info("Number of clusters found: {}", mcl.getNumClusters());

  spdlog::trace("Save the point cloud with planes");
  pcl::io::savePCDFileBinary(config.path_out, *labels);

  return 0;
}
