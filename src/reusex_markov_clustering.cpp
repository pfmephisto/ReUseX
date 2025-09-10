#include "core/fmt_formatter.hh"
#include "core/markov_clustering.hh"
#include "core/spdmon.hh"

// GraphBLAS imports complex.h which defines a macro named 'I' that conflicts
// with the type in CLI11
#ifdef I
#undef I
#endif

#include <CLI/CLI.hpp>
#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/pca.h>
#include <pcl/correspondence.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "core/io.hh"

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

  int expansion = 2;
  double inflation = 2;
  double pruning_threshold = 0.0001;
  double convergence_threshold = 1e-8;
  int max_iter = 100;

  float grid_size = 0.2f;

  int verbosity = 0;
  bool visualize = false;
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

  app->add_option("-e, --expansion", params.expansion,
                  "The expansion factor for the MCL algorithm.")
      ->default_val(params.expansion)
      ->check(CLI::Range(1, 10));

  app->add_option("-p, --pruning-threshold", params.pruning_threshold,
                  "The pruning threshold for the MCL algorithm.")
      ->default_val(params.pruning_threshold)
      ->check(CLI::Range(0.0, 1.0));

  app->add_option("-c, --convergence-threshold", params.convergence_threshold,
                  "The convergence threshold for the MCL algorithm.")
      ->default_val(params.convergence_threshold)
      ->check(CLI::Range(0.0, 1.0));

  app->add_option("-m, --max-iter", params.max_iter,
                  "The maximum number of iterations for the MCL algorithm.")
      ->default_val(params.max_iter)
      ->check(CLI::Range(1, 1000));

  app->add_option("-g, --grid-size", params.grid_size,
                  "The grid size for the MCL algorithm.")
      ->default_val(params.grid_size)
      ->check(CLI::Range(0.01, 10.0));

  app->add_flag("-d, --visualize", params.visualize,
                "Visualize the clustering process.")
      ->default_val(params.visualize);

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

  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
      centroids;
  std::vector<IndicesPtr> inlier_indices;

  fs::path planes_path = config.path_in.replace_extension(".planes");
  if (!ReUseX::read(planes_path, model_coefficients, centroids, inlier_indices))
    spdlog::warn("Loading planes form {} failed.", planes_path.string());

  assert(model_coefficients.size() == inlier_indices.size());
  assert(model_coefficients.size() == centroids.size());

  IndicesPtr indices(new Indices);
  pcl::UniformSampling<PointT> us;
  us.setInputCloud(cloud);
  us.setRadiusSearch(config.grid_size);
  for (const auto &idx : inlier_indices) {
    us.setIndices(idx);
    IndicesPtr local_indices(new Indices);
    us.filter(*local_indices);
    indices->insert(indices->end(), local_indices->begin(),
                    local_indices->end());
  }

  CloudNPtr normals(new CloudN);
  normals->resize(cloud->size());

  for (size_t i = 0; i < model_coefficients.size(); ++i) {
    const auto &coeff = model_coefficients[i];

    Eigen::Vector3f normal(coeff.values[0], coeff.values[1], coeff.values[2]);
    normal.normalize();
    for (const auto &index : *inlier_indices[i]) {
      normals->points[index].normal_x = normal[0];
      normals->points[index].normal_y = normal[1];
      normals->points[index].normal_z = normal[2];
    }
  }

  spdlog::trace("Initialize the Markov Clustering algorithm");
  pcl::MarkovClustering<PointT, NormalT, LabelT> mcl;
  mcl.setInflationFactor(config.inflation);
  mcl.setExpansionFactor(config.expansion);
  mcl.setPruningThreshold(config.pruning_threshold);
  mcl.setConvergenceThreshold(config.convergence_threshold);
  mcl.setMaxIterations(config.max_iter);

  mcl.setGridSize(config.grid_size);
  mcl.setInputCloud(cloud);
  mcl.setInputNormals(normals);
  mcl.setIndices(indices);

  // Set up PCL Visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (config.visualize) {
    spdlog::warn("Visualization is an experimental feature.");
    viewer = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("MCL Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    mcl.registerVisualizationCallback(
        [&viewer](typename pcl::PointCloud<pcl::PointXYZ>::Ptr points,
                  std::shared_ptr<std::vector<pcl::Vertices>> vertices,
                  pcl::CorrespondencesPtr correspondences) {
          spdlog::trace("Updating visualization context with {} points and "
                        "{} polygons",
                        points->size(), vertices->size());

          const std::string polygon_id = "disk_polygon";
          viewer->addPolygonMesh<pcl::PointXYZ>(points, *vertices, polygon_id);
          viewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,
              polygon_id);
          viewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, polygon_id);

          viewer->addCorrespondences<pcl::PointXYZ>(
              points, points, *correspondences, "correspondences");
        });

    viewer->addPointCloud<PointT>(cloud, "cloud");
  }

  spdlog::trace("Initialize labels and copy xyzrgb data to labels");
  CloudLPtr labels(new CloudL);
  pcl::copyPointCloud(*cloud, *labels);
  for (size_t i = 0; i < labels->points.size(); ++i)
    labels->points[i].label = -1;

  mcl.cluster(*labels);
  spdlog::trace("Done clustering");

  if (viewer)
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  // Assign the label to all points
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud, indices);
  IndicesPtr missing_indices(new Indices);
  spdlog::trace("Resizing missing indices to size {}, cloud size: {}, "
                "indices size: {}",
                cloud->points.size() - indices->size(), cloud->points.size(),
                indices->size());
  missing_indices->reserve(cloud->points.size() - indices->size());

  std::sort(indices->begin(), indices->end());

  int j = 0;
  for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
    if (j < static_cast<int>(indices->size()) && indices->at(j) == i) {
      ++j; // skip
    } else {
      missing_indices->push_back(i);
    }
  }

  for (size_t i = 0; i < missing_indices->size(); ++i) {
    const size_t idx = missing_indices->at(i);
    std::vector<int> nn_indices(1);
    std::vector<float> nn_sqr_dists(1);
    if (kdtree.nearestKSearch(cloud->points[idx], 1, nn_indices, nn_sqr_dists) >
        0) {
      labels->points[idx].label = labels->points[nn_indices[0]].label;
    }
  }

  spdlog::info("Number of clusters found: {}", mcl.getNumClusters());

  spdlog::trace("Save the point cloud with planes to dist at: {}",
                config.path_out.string());
  pcl::io::savePCDFileBinary(config.path_out, *labels);

  return 0;
}
