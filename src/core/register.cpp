#include "register.hh"
#include "compute_normals.hh"
#include "fmt_formatter.hh"
#include "icp.hh"
#include "parse_input_files.hh"
#include "spdmon.hh"
#include "types/Filters.hh"
#include "visualizer/visualizer.hh"

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/elch.h>
#include <pcl/registration/lum.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/eigen_types.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Core/util/Memory.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>

#include <omp.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using namespace ReUseX;

/**
 * @brief Converts an odometry matrix to an isometry transformation.
 *
 * This function takes an odometry matrix as input and converts it into a
 * g2o::Isometry3 transformation, which represents a rigid body transformation
 * in 3D space (rotation and translation).
 *
 * @param odometry An Eigen::MatrixXd representing the odometry matrix.
 *                 This matrix typically contains the transformation data
 *                 (e.g., rotation and translation) from one pose to another.
 *
 * @return g2o::Isometry3 The corresponding isometry transformation.
 *
 * @details The function assumes that the input odometry matrix is valid and
 *          contains the necessary data for constructing the isometry. It is
 *          commonly used in SLAM or robotics applications where odometry data
 *          needs to be converted into a format suitable for graph optimization.
 *
 * @note Ensure that the Eigen and g2o libraries are properly included and
 * linked. The input matrix must conform to the expected structure for
 * conversion.
 */
g2o::Isometry3 odometryToIsometry(Eigen::MatrixXd odometry) {

  // Create an Eigen::Transform object
  g2o::Isometry3 transform = g2o::Isometry3::Identity();

  // Extract position (first 3 elements)
  Eigen::Vector3d p = odometry.block<1, 3>(0, 2).cast<double>();
  Eigen::Vector4d q = odometry.block<1, 4>(0, 5).cast<double>();

  // Extract quaternion (last 4 elements)
  Eigen::Quaterniond quat(q[3], q[0], q[1], q[2]); // w, x, y, z

  // Normalize quaternion to avoid numerical instability
  quat.normalize();

  // Set transformation matrix
  transform.linear() = quat.toRotationMatrix();
  transform.translation() = p;

  return g2o::Isometry3(transform);
}

/**
 * @brief Registers nodes into a graph optimization problem.
 *
 * This function adds nodes to a graph managed by a g2o::SparseOptimizer
 * instance. It processes a dataset and a set of indices to register nodes into
 * the optimizer.
 *
 * @param optimizer A unique pointer to the g2o::SparseOptimizer instance.
 *                  This optimizer manages the graph structure.
 * @param dataset The dataset containing the nodes to be registered.
 * @param indices A vector of indices specifying the nodes to be added to the
 * graph.
 *
 * @note Ensure that the optimizer is properly initialized before calling this
 *       function. The dataset must contain valid nodes corresponding to the
 *       provided indices.
 */
void register_nodes(std::unique_ptr<g2o::SparseOptimizer> &optimizer,
                    Dataset &dataset, std::vector<size_t> &indices) {
  spdlog::info("Entering register_nodes");

  std::vector<g2o::VertexSE3 *> verticies{};
  verticies.resize(indices.size());

  spdlog::trace("Starting parallel for loop");

  {
    spdmon::LoggerProgress monitor("Add Verticies", indices.size());
#pragma omp parallel for shared(dataset, verticies, monitor)
    for (int i = 0; i < indices.size(); i++) {
      size_t index = indices[i];

      verticies[i] = new g2o::VertexSE3();

      verticies[i]->setId(index);
      auto odometry = dataset[index].get<Field::ODOMETRY>();

      g2o::Isometry3 pose = odometryToIsometry(odometry);

      verticies[i]->setEstimate(pose);

      ++monitor;
    }
  }

  // Fix the location of the first frame
  verticies[0]->setFixed(true);
  // verticies[0]->setEstimate(g2o::Isometry3::Identity());

  spdlog::trace("Setting verticies");
  for (auto &v : verticies)
    optimizer->addVertex(v);
}

/**
 * @brief Registers consecutive edges in a graph optimization problem.
 *
 * This function adds edges between consecutive nodes in a graph, using the
 * specified kernel and optimization parameters. It is designed to work with
 * a sparse optimizer and a dataset containing graph nodes.
 *
 * @param optimizer A unique pointer to the g2o::SparseOptimizer instance.
 *                  This optimizer is used to manage the graph structure.
 * @param dataset The dataset containing the graph nodes to be connected.
 * @param indices A vector of indices specifying the nodes to connect.
 * @param groupSize The size of the groups of nodes to be connected
 * consecutively.
 * @param kernelName The name of the robust kernel to be used for edge
 * weighting. Supported kernels include:
 *                   - Huber
 *                   - Cauchy
 *                   - Welsch
 *                   - Tukey
 *                   - Fair
 *                   - DCS
 *                   - GemanMcClure
 *                   - ScaleDelta
 *                   - Saturated
 *                   - PseudoHuber
 * @param deltaValue The delta value used for the robust kernel.
 * @param maxCorrespondence The maximum correspondence threshold for edge
 * creation.
 * @param information_matrix The 6x6 information matrix used for edge weighting.
 *
 * @note Ensure that the optimizer is properly initialized before calling this
 *       function. The kernel name must match one of the supported kernels.
 */
void register_consecutive_edges(
    std::unique_ptr<g2o::SparseOptimizer> &optimizer, Dataset &dataset,
    std::vector<size_t> &indices, const size_t groupSize,
    const std::string kernelName, const double deltaValue,
    const double maxCorrespondence,
    const Eigen::Matrix<double, 6, 6> information_matrix) {
  spdlog::info("Entering register_consecutive_edges");
  assert(groupSize > 0);
  // spdlog::trace("Parameters=> groupSize:{} maxCorrespondence:{}",groupSize,
  // maxCorrespondence);
  spdlog::debug("Parameters:");
  spdlog::debug("groupSize = {}", groupSize);
  spdlog::debug("kernelName = {}", kernelName);
  spdlog::debug("deltaValue = {}", deltaValue);
  spdlog::debug("maxCorrespondence = {}", maxCorrespondence);
  spdlog::debug("information_matrix = {}",
                information_matrix.format(OctaveFmt));

  using PointT = pcl::PointXYZRGBA;
  using Cloud = typename pcl::PointCloud<PointT>::Ptr;
  using FilterCollection = std::vector<typename pcl::Filter<PointT>::Ptr>;
  using Isometry = g2o::Isometry3;

  spdlog::trace("Set up result vector");
  size_t const num_edges = (indices.size() - groupSize) * groupSize;

  spdlog::trace("Creating edge pairs");
  std::vector<std::pair<size_t, size_t>> edge_pairs;
  edge_pairs.reserve(num_edges);
  for (size_t i = groupSize; i < indices.size(); i++) {
    for (size_t j = groupSize; j > 0; j--) {
      const size_t idx = (i - groupSize) * (groupSize - 1) + (i - j);
      const size_t target_index = indices[i - j];
      const size_t source_index = indices[i];
      edge_pairs.push_back(std::make_pair(source_index, target_index));
    }
  }

  spdlog::trace("Creating edges");
  std::vector<g2o::EdgeSE3 *> edges{};
  edges.resize(num_edges);

  const g2o::RobustKernelFactory *kernel_factory =
      g2o::RobustKernelFactory::instance();

  std::shared_ptr<spdmon::LoggerProgress> monitor =
      std::make_shared<spdmon::LoggerProgress>("Adding edges",
                                               edge_pairs.size());
  spdlog::trace("Starting parallel for loop");
#pragma omp parallel for shared(dataset, edge_pairs, edges, kernel_factory)    \
    firstprivate(kernelName, deltaValue, information_matrix)
  for (size_t i = 0; i < edge_pairs.size(); i++) {
    // Get the two indices
    const auto [source_index, target_index] = edge_pairs[i];

    const auto source =
        static_cast<g2o::VertexSE3 *>(optimizer->vertex(source_index));
    const auto target =
        static_cast<g2o::VertexSE3 *>(optimizer->vertex(target_index));

    const auto source_pose = source->estimate();
    const auto target_pose = target->estimate();

    // Create Edge
    edges[i] = new g2o::EdgeSE3();
    edges[i]->setId(i);
    edges[i]->setVertex(0, source);
    edges[i]->setVertex(1, target);

    edges[i]->setMeasurement(source_pose.inverse() * target_pose);
    edges[i]->setInformation(information_matrix);

    // Set Robust Kernel
    g2o::RobustKernel *kernel = kernel_factory->construct(kernelName);
    kernel->setDelta(deltaValue); // 1.5 to 2.5
    edges[i]->setRobustKernel(kernel);

    ++(*monitor);
  }

  std::vector<double> fitness_scores;
#if 0 // ICP Alignment
  fitness_scores.resize(edges.size());
  for (int i = 0; i < edges.size(); i++) {

    size_t source_index = edge_pairs[i].first;
    size_t target_index = edge_pairs[i].second;
    auto source_cloud = dataset[source_index].get<Field::CLOUD>();
    auto target_cloud = dataset[target_index].get<Field::CLOUD>();

    // Set up filters
    FilterCollection filters{
        Filters::HighConfidenceFilter<PointT>(),
        Filters::GridFilter<PointT>(0.1),
    };

    auto [matrix, fitness_score] =
        icp<PointT>(source_cloud, target_cloud, filters, maxCorrespondence);
    fitness_scores[i] = fitness_score;

    auto source_pose =
        static_cast<g2o::VertexSE3 *>(optimizer->vertex(source_index))
            ->estimate();
    auto const target_pose =
        static_cast<g2o::VertexSE3 *>(optimizer->vertex(target_index))
            ->estimate();

    // Update the source pose
    source_pose = source_pose * g2o::Isometry3(matrix.cast<double>());
    edges[i]->setMeasurement(source_pose.inverse() * target_pose);
    edges[i]->setInformation(information_matrix / fitness_score);
  }
#endif

  if (!ReUseX::Visualizer::isInitialised()) {
    monitor.reset();
    monitor.reset(
        new spdmon::LoggerProgress("Adding edges to viewer", edges.size()));
    spdlog::trace("Visualize edges");
    const auto viewer = ReUseX::Visualizer::getInstance()
                            ->getViewer<pcl::visualization::PCLVisualizer>();
    for (size_t i = 0; i < edges.size(); i++) {

      const auto edge = edges[i];

      auto const source = static_cast<g2o::VertexSE3 *>(
                              optimizer->vertex(edge->vertex(0)->id()))
                              ->estimate()
                              .translation()
                              .cast<float>();
      auto const target = static_cast<g2o::VertexSE3 *>(
                              optimizer->vertex(edge->vertex(1)->id()))
                              ->estimate()
                              .translation()
                              .cast<float>();

      pcl::PointXYZ p_start, p_end;
      p_start.x = source.x();
      p_start.y = source.y();
      p_start.z = source.z();
      p_end.x = target.x();
      p_end.y = target.y();
      p_end.z = target.z();

      pcl::RGB color = pcl::RGB(0, 255, 0);

      if (fitness_scores.size() > 0) {
        double const score =
            std::max<double>(0.0, std::min<double>(1.0, fitness_scores[i]));
        auto lut = pcl::ColorLUT<pcl::LUT_VIRIDIS>();
        color = lut.at(static_cast<size_t>(score * lut.size()));
      }

      const std::string name =
          fmt::format("edge_{}-{}_orig", edge->vertices()[0]->id(),
                      edge->vertices()[1]->id());

      viewer->addLine(p_start, p_end, color.r / 255, color.g / 255,
                      color.b / 255, name);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, name);

      ++(*monitor);
    }
  }
  // g2o::RobustKernelFactory *robust_kernel_factory =
  //     g2o::RobustKernelFactory::instance();

  // double max_fitness_score = std::numeric_limits<double>::min();
  // double min_fitness_score = std::numeric_limits<double>::max();

  // spdlog::trace("Starting parrallel for loop");
  // spdmon::LoggerProgress monitor("Adding edges", num_edges);
  // monitor.GetLogger()->set_level(spdlog::level::trace);

  // #pragma omp parallel for shared(dataset, edge_pairs, edges, monitor)
  //   for (int i = 0; i < edge_pairs.size(); ++i) {
  //
  //     auto [source_index, target_index] = edge_pairs[i];
  //
  //     spdlog::trace("Edge: {:06}-{:06}", target_index, source_index);
  //
  //     // Set up target
  //     Cloud target = CreateCloud(dataset[target_index]);
  //     g2o::Isometry3 pose_target = g2o::Isometry3::Identity();
  //     pose_target.translation() =
  //     target->sensor_origin_.head<3>().cast<double>(); pose_target.linear()
  //     =
  //         target->sensor_orientation_.toRotationMatrix().cast<double>();
  //
  //     // Set up source
  //     Cloud source = CreateCloud(dataset[source_index]);
  //     g2o::Isometry3 pose_source = g2o::Isometry3::Identity();
  //     pose_source.translation() =
  //     source->sensor_origin_.head<3>().cast<double>(); pose_source.linear()
  //     =
  //         source->sensor_orientation_.toRotationMatrix().cast<double>();
  //
  //     pcl::PointXYZ p_source, p_target;
  //     p_source.x = pose_source.translation().x();
  //     p_source.y = pose_source.translation().y();
  //     p_source.z = pose_source.translation().z();
  //     p_target.x = pose_target.translation().x();
  //     p_target.y = pose_target.translation().y();
  //     p_target.z = pose_target.translation().z();
  //
  //     // Set up filters
  //     FilterCollection filters{
  //         Filters::HighConfidenceFilter<pcl::PointXYZRGBA>(), // vg,
  //         Filters::GridFilter<pcl::PointXYZRGBA>(0.1),
  //     };
  //
  //     // Compute ICP
  //     spdlog::debug("Number of filters pre icp: {}", filters.size());
  //     // auto [matrix, fitness_score] =
  //     //     icp<PointT>(source, target, filters, maxCorrespondence);
  //     Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  //     double fitness_score = 0.001;
  //
  //     // Collect statitics on the fitness_score
  // #pragma omp critical
  //     {
  //       min_fitness_score = std::min(min_fitness_score, fitness_score);
  //       max_fitness_score = std::max(max_fitness_score, fitness_score);
  //     }
  //
  //     // TODO: Fix
  //
  //     if (fitness_score > 0.04) {
  //       matrix = Eigen::Matrix4f::Identity();
  //       // spdlog::warn("Fitness score is too large: {:.6f}",
  //       fitness_score);
  //     }
  //
  //     g2o::Isometry3 xform = g2o::Isometry3(matrix.cast<double>());
  //
  //     // Update the soruce
  //     pose_source = pose_source * xform;
  //
  //     pcl::PointXYZ p_source_new;
  //     p_source_new.x = pose_source.translation().x();
  //     p_source_new.y = pose_source.translation().y();
  //     p_source_new.z = pose_source.translation().z();
  //
  //     const std::string name_1 =
  //         fmt::format("edge_{}-{}", target_index, source_index);
  //     const std::string name_2 = fmt::format("edge_{}-move", source_index);
  //
  //     if (!ReUseX::Visualizer::isInitialised()) {
  //       auto viewer = ReUseX::Visualizer::getInstance()
  //                         ->getViewer<pcl::visualization::PCLVisualizer>();
  //
  //       // Clamp the input to [0.0, 1.0] to avoid overflow/underflow
  //       double value =
  //           std::max<double>(0.0, std::min<double>(1.0, fitness_score));
  //       auto lut = pcl::ColorLUT<pcl::LUT_VIRIDIS>();
  //       pcl::RGB color = lut.at(static_cast<size_t>(value * lut.size()));
  //       double r, g, b;
  //       r = color.r / 255.0;
  //       g = color.g / 255.0;
  //       b = color.b / 255.0;
  //
  //       viewer->addLine(p_source, p_target, 1, 0, 0, name_1);
  //       viewer->addLine(p_source, p_source_new, r, g, b, name_2);
  //
  //       viewer->setShapeRenderingProperties(
  //           pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, name_1);
  //       viewer->setShapeRenderingProperties(
  //           pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, name_2);
  //     }
  //
  //     // Create Edge
  //     edges[i] = new g2o::EdgeSE3();
  //     edges[i]->setId(i);
  //     edges[i]->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex *>(
  //                                optimizer->vertex(target_index)));
  //     edges[i]->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex *>(
  //                                optimizer->vertex(source_index)));
  //
  //     // TODO: Why is this not inversed but is seems to be correnct?
  //     edges[i]->setMeasurement(pose_target.inverse() * pose_source);
  //     edges[i]->setInformation(information_matrix / fitness_score);
  //
  //     // Set Robust Kernel
  //     g2o::RobustKernel *kernel =
  //     robust_kernel_factory->construct(kernelName); if (kernel == nullptr)
  //     {
  //       spdlog::error("Kernel {} not found!", kernelName);
  //       continue;
  //     }
  //
  //     kernel->setDelta(deltaValue); // 1.5 to 2.5
  //     edges[i]->setRobustKernel(kernel);
  //
  //     ++monitor;
  //   }

  spdlog::trace("Setting edges");
  for (auto &edge : edges) // Add edges to optimizer
    optimizer->addEdge(edge);

  // spdlog::info("Min fitness score: {:.6f}", min_fitness_score);
  // spdlog::info("Max fitness score: {:.6f}", max_fitness_score);
}

/**
 * @brief Visualizes a point cloud with optional custom processing.
 *
 * This function visualizes a subset of a point cloud dataset using the
 * specified indices. It optionally applies a custom processing function to the
 * point cloud before visualization.
 *
 * @param dataset The dataset containing the point cloud data.
 * @param indices A vector of indices specifying the points to visualize.
 * @param name A string representing the name of the visualization window.
 * @param func An optional custom function to process the point cloud before
 *             visualization. The function takes a shared pointer to a
 *             pcl::PointCloud<pcl::PointXYZRGBA> and an integer as arguments.
 *
 * @details The function extracts the specified points from the dataset and
 *          visualizes them in a window. If a custom function is provided, it
 *          is applied to the point cloud before visualization. This is useful
 *          for tasks such as filtering, coloring, or transforming the point
 * cloud.
 *
 * @note Ensure that the dataset contains valid point cloud data and that the
 *       indices are within bounds. The visualization requires the PCL (Point
 *       Cloud Library) to be properly installed and configured.
 */
void visualize_cloud(
    Dataset &dataset, std::vector<size_t> indices, std::string name,
    std::optional<std::function<
        void(typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, int)>>
        func = {},
    std::function<
        typename pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGBA>::
            Ptr(typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)>
        getColor = [](pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
        -> std::shared_ptr<
            pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGBA>> {
      return std::make_shared<
          pcl::visualization::PointCloudColorHandlerRGBField<
              pcl::PointXYZRGBA>>(cloud);
    }) {

  // Check if we are in a visualization context
  if (!ReUseX::Visualizer::isInitialised())
    return;

  auto viewer = ReUseX::Visualizer::getInstance();
  auto pcl_viewer = viewer->getViewer<pcl::visualization::PCLVisualizer>();

  spdmon::LoggerProgress monitor("Adding Clouds to Viewer", indices.size());

  // Merge clouds
  typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr merged(
      new pcl::PointCloud<pcl::PointXYZRGBA>());
#pragma omp parallel for
  for (int i = 0; i < indices.size(); i++) {
    auto idx = indices[i];
    auto cloud = CreateCloud(dataset[idx]);

    // Filter point cloud to only include hight confidence points
    auto filter = Filters::HighConfidenceFilter<pcl::PointXYZRGBA>();
    filter->setInputCloud(cloud);
    filter->filter(*cloud);

    if (func.has_value())
      func.value()(cloud, idx);

    // Get the pose
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.linear() = cloud->sensor_orientation_.toRotationMatrix();
    pose.translation() = cloud->sensor_origin_.head<3>();
#pragma omp critical
    {
      *merged += *cloud;
      // pcl_viewer->addCoordinateSystem(0.5, pose,
      //                                 fmt::format("{}_{}_pose", name, i));
    }
    ++monitor;
  }

  // Filter if there are a lot of points
  if (indices.size() > 300) {
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setLeafSize(0.05, 0.05, 0.05);
    vg.setInputCloud(merged);
    vg.filter(*merged);
  }
  auto color = getColor(merged);

  // Set orientation adn origin to identity
  merged->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 1);
  merged->sensor_orientation_ = Eigen::Quaternionf::Identity();

  pcl_viewer->addPointCloud<pcl::PointXYZRGBA>(merged, *color, name);
  pcl_viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
  // pcl_viewer->setPointCloudRenderingProperties(
  //     pcl::visualization::RenderingProperties::PCL_VISUALIZER_SHADING,
  //     pcl::visualization::ShadingRepresentationProperties::
  //         PCL_VISUALIZER_SHADING_PHONG,
  //     name);
  viewer->step();
}

inline void merge_edges(std::vector<std::pair<size_t, size_t>> lhs,
                        const std::vector<std::pair<size_t, size_t>> rhs) {
  std::copy(rhs.begin(), rhs.end(), std::back_inserter(lhs));
}
#pragma omp declare reduction(                                                 \
        + : std::vector<std::pair<size_t, size_t>> : merge_edges(omp_out,      \
                                                                     omp_in))  \
    initializer(omp_priv = std::vector<std::pair<size_t, size_t>>())

// TODO: Add function doc string
void loop_detection(std::unique_ptr<g2o::SparseOptimizer> &optimizer,
                    Dataset &dataset, std::vector<size_t> &indices,
                    const std::string kernelName, const double deltaValue,
                    const Eigen::Matrix<double, 6, 6> information_matrix) {

  using PointT = pcl::PointXYZRGBA;
  using Cloud = typename pcl::PointCloud<PointT>::Ptr;
  using FilterCollection = std::vector<typename pcl::Filter<PointT>::Ptr>;

  // const float LIMIT_LOWER = 0.3f;
  const float LIMIT_UPPER = 2.0f;
  const size_t MAX_FRAMES_DIST = 500;
  const size_t NUM_SAMPLES = 100;

  spdlog::trace("Entering loop detection");

  size_t num_edges = (indices.size() * (indices.size() - 1)) / 2;

  std::vector<std::pair<size_t, size_t>> edges;
  edges.reserve(num_edges);

  // Set up logger
  std::shared_ptr<spdmon::LoggerProgress> monitor;
  monitor =
      std::make_shared<spdmon::LoggerProgress>("Collect all edges", num_edges);
  monitor->GetLogger()->set_level(spdlog::level::warn);

#pragma omp parrallel for collapse(2) reduction(+ : edges)                     \
    shared(dataset, indices, edges, LIMIT_UPPER)
  for (int i = 0; i < indices.size(); ++i) { // Target
    for (int j = 0; j < i; ++j) {            // Source

      if (i == j) {
        ++(*monitor);
        continue;
      }

      const std::size_t idx_i = indices[i];
      const std::size_t idx_j = indices[j];

      // Get the two verticies
      const g2o::Isometry3 pose1 =
          static_cast<g2o::VertexSE3 *>(optimizer->vertex(idx_i))->estimate();
      const g2o::Isometry3 pose2 =
          static_cast<g2o::VertexSE3 *>(optimizer->vertex(idx_j))->estimate();

      const double dist = (pose1.translation() - pose2.translation()).norm();
      const size_t num_frames_appart =
          std::abs(static_cast<int>(idx_i) - static_cast<int>(idx_j));

      if (dist > LIMIT_UPPER || num_frames_appart < MAX_FRAMES_DIST) {
        edges.emplace_back(idx_j, idx_i);
      }

      ++(*monitor);
    }
  }
  spdlog::info("Found {} edges", edges.size());

  spdlog::info("Subsampling edges");
  std::vector<std::pair<size_t, size_t>> subsampled_edges;
  subsampled_edges.reserve(NUM_SAMPLES);

  // Sort edges by largest loops
  std::sort(edges.begin(), edges.end(), [](const auto &lhs, const auto &rhs) {
    return std::abs(static_cast<int>(lhs.first) -
                    static_cast<int>(lhs.second)) <
           std::abs(static_cast<int>(rhs.first) - static_cast<int>(rhs.second));
  });
  // Copy the first NUM_SAMPLES edges
  std::copy(edges.begin(), edges.begin() + NUM_SAMPLES,
            std::back_inserter(subsampled_edges));

  // std::sample(edges.begin(), edges.end(),
  // std::back_inserter(subsampled_edges),
  //             NUM_SAMPLES, std::mt19937{std::random_device{}()});

  monitor.reset();
  monitor.reset(
      new spdmon::LoggerProgress("Compute alignment", subsampled_edges.size()));
  monitor->GetLogger()->set_level(spdlog::level::debug);

  size_t num_edges_found = 0;
  double min_fitness_score = std::numeric_limits<double>::max();
  double max_fitness_score = std::numeric_limits<double>::min();

#pragma omp parallel for shared(dataset, subsampled_edges)
  for (int i = 0; i < subsampled_edges.size(); ++i) {

    auto [target_index, source_index] = subsampled_edges[i];

    // Set up target
    spdlog::trace("Create target pose");
    Cloud target = CreateCloud(dataset[target_index]);
    g2o::Isometry3 pose_target = g2o::Isometry3::Identity();
    pose_target.translation() = target->sensor_origin_.head<3>().cast<double>();
    pose_target.linear() =
        target->sensor_orientation_.toRotationMatrix().cast<double>();

    // Set up source
    spdlog::trace("Create source pose");
    Cloud source = CreateCloud(dataset[source_index]);
    g2o::Isometry3 pose_source = g2o::Isometry3::Identity();
    pose_source.translation() = source->sensor_origin_.head<3>().cast<double>();
    pose_source.linear() =
        source->sensor_orientation_.toRotationMatrix().cast<double>();

    // Set up filters
    spdlog::trace("Setting up filters");
    FilterCollection filters{
        Filters::HighConfidenceFilter<pcl::PointXYZRGBA>(), // vg,
        Filters::GridFilter<pcl::PointXYZRGBA>(0.1),
    };

    // Compute ICP
    spdlog::trace("Computing ICP");
    auto [matrix, fitness_score] =
        icp<PointT>(source, target, filters, LIMIT_UPPER);

#pragma omp critical
    {
      min_fitness_score = std::min(min_fitness_score, fitness_score);
      max_fitness_score = std::max(max_fitness_score, fitness_score);
    }

    if (fitness_score > 0.04) {
      spdlog::warn("Fitness score is too large: {:.6f}", fitness_score);
      matrix = Eigen::Matrix4f::Identity();
      // continue;
    }

    //// Check if the distance it too large
    // if (xform.linear().norm() > 2.0) {
    //   spdlog::warn("Distance between {} and {} is too large: {}", i, j,
    //                xform.linear().norm());
    //   ++monitor;
    //   continue;
    // } else {
    //   spdlog::error("Distance between {} and {} is: {}", i, j,
    //                 xform.linear().norm());
    // }

    // TODO: Fix loop detection
    // Update the soruce
    spdlog::trace("Updating pose");
    pose_source = pose_source * g2o::Isometry3(matrix.cast<double>());

    // Create edge
    spdlog::trace("Creating edge");
    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
    edge->setVertex(0, optimizer->vertex(target_index)); // Target
    edge->setVertex(1, optimizer->vertex(source_index)); // Source
    edge->setMeasurement(pose_target.inverse() * pose_source);

    // Set information matrix
    edge->setInformation(information_matrix / (fitness_score * fitness_score));

    // Set robust kernel
    g2o::RobustKernel *kernel =
        g2o::RobustKernelFactory::instance()->construct(kernelName);
    if (kernel == nullptr) {
      spdlog::error("Kernel {} not found!", kernelName);
      ++(*monitor);
      continue;
    }
    kernel->setDelta(deltaValue);
    edge->setRobustKernel(kernel);

#pragma omp critical
    {
      // Add debug Geometry
#if 1 // TODO: Remove debugging code
      auto viewer = ReUseX::Visualizer::getInstance()
                        ->getViewer<pcl::visualization::PCLVisualizer>();
      // viewer->addCoordinateSystem(
      //     0.2, Eigen::Affine3f(matrix),
      //     fmt::format("xform_{}-{}", target_index, source_index));

      pcl::PointXYZ p1, p2;
      p1.x = pose_source.translation().x();
      p1.y = pose_source.translation().y();
      p1.z = pose_source.translation().z();

      p2.x = pose_target.translation().x();
      p2.y = pose_target.translation().y();
      p2.z = pose_target.translation().z();

      // Clamp the input to [0.0, 1.0] to avoid overflow/underflow
      auto value = std::max<double>(0.0, std::min<double>(fitness_score, 1.0));
      auto lut = pcl::ColorLUT<pcl::LUT_VIRIDIS>();
      pcl::RGB color = lut.at(static_cast<size_t>(value * lut.size()));
      double r, g, b;
      r = color.r / 255.0;
      g = color.g / 255.0;
      b = color.b / 255.0;

      viewer->addLine(p1, p2, r, g, b,
                      fmt::format("edge_{}-{}", target_index, source_index));

      pcl::PointXYZ center;
      center.x = (p1.x + p2.x) / 2;
      center.y = (p1.y + p2.y) / 2;
      center.z = (p1.z + p2.z) / 2;

      // If disntance larger 20cm
      double distance = (p1.getVector3fMap() - p2.getVector3fMap()).norm();
      if (distance > 0.2) {
        viewer->addText3D(
            fmt::format("edge {}-{}", target_index, source_index), center, 0.01,
            0.0, 0.0, 0.0,
            fmt::format("edge_text_{}-{}", target_index, source_index));
      }
#endif

      // Add Add edge
      spdlog::trace("Adding edge");
      edge->setId(optimizer->edges().size());
      optimizer->addEdge(edge);

      // Update min and max fitness score
      ++num_edges_found;
    }
    // spdlog::debug("Added edge between {} and {}", i, j);
    ++(*monitor);
  }
  spdlog::info("Found {} of {} edges", num_edges_found, num_edges);
  spdlog::info("Min fitness score: {}", min_fitness_score);
  spdlog::info("Max fitness score: {}", max_fitness_score);
}

/**
 * @brief Writes a graph structure to a specified file path.
 *
 * This function processes a dataset and writes a graph representation to the
 * specified file path. It supports configurable solver and kernel options,
 * as well as parameters for controlling the graph generation process.
 *
 * @param path The file path where the graph will be written.
 * @param dataset The dataset containing the graph data.
 * @param indices A vector of indices specifying the subset of the dataset to
 * process.
 * @param groupSize The size of the groups to be processed in the graph.
 * @param solverName The name of the solver to be used. Format:
 *                   [algorithm]_[fix|var][pose_dim]_[landmark_dim]_[linear_solver].
 *                   Examples:
 *                   - "lm_fix6_3_csparse" (Levenberg-Marquardt, fixed pose
 * 6D, landmark 3D, CSparse solver)
 *                   - "gn_var_dense" (Gauss-Newton, variable size, Dense
 * solver)
 * 		     Common Algorithm Prefixes:
 * 		     "lm_" → Levenberg-Marquardt
 * 		     "gn_" → Gauss-Newton
 * 		     "dl_" → Dogleg
 * 		     Pose & Landmark Dimension:
 * 		     "fix6_3" → Fixed size pose (6D) and landmark (3D) (used in
 * 3 Bundle A justment) "var" → Variable block size Linear Solver Types:
 * 		     "csparse" → Uses CSparse
 * 		     "cholmod" → Uses CHOLMOD (SuiteSparse)
 * 		     "pcg" → Uses Preconditioned Conjugate Gradient
 * 		     "dense" → Uses Dense Solver (not recommended for large p
 * oblems
 * @param kernelName The name of the kernel to be used for optimization.
 * @param maxIterations The maximum number of iterations for the solver.
 * @param maxCorrespondence The maximum correspondence threshold for graph
 * edges.
 * @param deltaValue The delta value used for convergence criteria.
 * @param information_matrix The 6x6 information matrix used for edge
 * weighting.
 *
 * @return fs::path The path to the written graph file.
 *
 * @note Ensure that the provided `path` is writable and the dataset is valid.
 *       The solver and kernel names must conform to the expected formats.
 *       This function logs its progress using `spdlog`.
 */
fs::path
ReUseX::write_graph(fs::path path, Dataset &dataset,
                    std::vector<size_t> &indices, const size_t groupSize,
                    const std::string solverName, const std::string kernelName,
                    const int maxIterations, const double maxCorrespondence,
                    const double deltaValue,
                    const Eigen::Matrix<double, 6, 6> information_matrix) {
  spdlog::info("Entering Write Graph Function");

  spdlog::debug("Indecies: first: {} last: {}, count: {}", indices.front(),
                indices.back(), indices.size());

  if (ReUseX::Visualizer::isInitialised())
    spdlog::info("Using Visualizer");
  else
    spdlog::trace("Not using Visualizer");

  auto data = dataset[0];
  spdlog::debug("Field: {}", data.fields());

  visualize_cloud(dataset, indices, "Pre Align");

  // Set up sover
  g2o::OptimizationAlgorithmFactory *solver_factory =
      g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;

  g2o::OptimizationAlgorithm *solver =
      solver_factory->construct(solverName, solver_property);
  if (!solver) {
    spdlog::error("Failed to create solver: {}", solverName);
  }
  spdlog::info("Solver {} initialized successfully!", solverName);

  spdlog::trace("Initialise Optimizer");
  std::unique_ptr<g2o::SparseOptimizer> optimizer =
      std::unique_ptr<g2o::SparseOptimizer>(new g2o::SparseOptimizer());
  // optimizer->setVerbose(true);
  optimizer->setVerbose(false);
  optimizer->setAlgorithm(solver);

  spdlog::trace("Add Verticies");
  register_nodes(optimizer, dataset, indices);

  spdlog::trace("Add Edges");
  register_consecutive_edges(optimizer, dataset, indices, groupSize, kernelName,
                             deltaValue, maxCorrespondence, information_matrix);

  spdlog::trace("Finding loop closures");
  loop_detection(optimizer, dataset, indices, kernelName, deltaValue,
                 information_matrix);

  spdlog::trace("Optimize");
  spdlog::stopwatch sw;
  optimizer->initializeOptimization();
  optimizer->optimize(maxIterations);
  spdlog::debug("Optimize ran in {:.3f}s", sw);

  spdlog::trace("Save graph to file");
  optimizer->save(path.c_str());

  auto move_cloud = [&optimizer](
                        typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                        int idx) {
    // Construct original pose
    g2o::Isometry3 pose_orig = g2o::Isometry3::Identity();
    pose_orig.translation() = cloud->sensor_origin_.head<3>().cast<double>();
    pose_orig.linear() =
        cloud->sensor_orientation_.toRotationMatrix().cast<double>();

    // Get estimated location
    g2o::Isometry3 estimate =
        static_cast<g2o::VertexSE3 *>(optimizer->vertex(idx))->estimate();

    // Compute relative transform and aplly it
    g2o::Isometry3 xform = pose_orig.inverse() * estimate;

    pcl::transformPointCloud(*cloud, *cloud, xform.matrix());
  };

  visualize_cloud(dataset, indices, "Post Align", move_cloud,
                  [](pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
                    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgb(
                        new pcl::PointCloud<pcl::PointXYZRGBA>());
                    cloud_rgb->resize(cloud->size());

#pragma omp parallel for
                    for (int i = 0; i < cloud->size(); ++i) {
                      cloud_rgb->points[i].r = cloud->points[i].r;
                      cloud_rgb->points[i].g =
                          std::min(cloud->points[i].g + 50, 255);
                      cloud_rgb->points[i].b = cloud->points[i].b;
                    }
                    return std::make_shared<
                        pcl::visualization::PointCloudColorHandlerRGBField<
                            pcl::PointXYZRGBA>>(cloud_rgb);
                  });

  // Visualize new path
  if (ReUseX::Visualizer::isInitialised()) {
    auto viewer = ReUseX::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();

    for (size_t i = 1; i < indices.size(); i++) {

      size_t idx_start = indices[i - 1];
      size_t idx_end = indices[i];

      auto v_start = static_cast<g2o::VertexSE3 *>(optimizer->vertex(idx_start))
                         ->estimate()
                         .translation()
                         .cast<float>();
      auto v_end = static_cast<g2o::VertexSE3 *>(optimizer->vertex(idx_end))
                       ->estimate()
                       .translation()
                       .cast<float>();

      pcl::PointXYZ p_start, p_end;
      p_start.x = v_start.x();
      p_start.y = v_start.y();
      p_start.z = v_start.z();
      p_end.x = v_end.x();
      p_end.y = v_end.y();
      p_end.z = v_end.z();

      viewer->addLine(p_start, p_end, 0, 1, 0,
                      fmt::format("edge_new{}-{}", idx_start, idx_end));
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3,
          fmt::format("edge_new{}-{}", idx_start, idx_end));
    }
  } // End if visualizer

  spdlog::trace("Clear");
  optimizer->clear();

  if (ReUseX::Visualizer::isInitialised())
    ReUseX::Visualizer::getInstance()->wait();

  return path;
}
