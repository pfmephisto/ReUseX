#include "functions/register.hh"
#include "functions/compute_normals.hh"
#include "functions/fmt_formatter.hh"
#include "functions/icp.hh"
#include "functions/parse_input_files.hh"
#include "functions/progress_bar.hh"
#include "functions/spdmon.hh"
#include "types/Filters.hh"
#include "visualizer/visualizer.hh"

#include <eigen3/Eigen/src/Core/util/Memory.h>
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

// #include <robust_kernel.h>
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
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>

// #include <opencv4/opencv2/features2d.hpp>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <memory>
#include <omp.h>
#include <optional>
#include <string>
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
  std::vector<g2o::EdgeSE3 *> edges{};
  edges.resize((indices.size() - groupSize) * groupSize);

  spdlog::trace("Starting parrallel for loop");

  g2o::RobustKernelFactory *robust_kernel_factory =
      g2o::RobustKernelFactory::instance();

  {
    spdmon::LoggerProgress monitor("Adding edges",
                                   (indices.size() - groupSize) * groupSize);
    // monitor.GetLogger()->set_level(spdlog::level::trace);

#pragma omp parallel for firstprivate(groupSize, maxCorrespondence,            \
                                          deltaValue)                          \
    shared(dataset, edges, monitor, robust_kernel_factory)
    for (int i = groupSize; i < indices.size();
         i++) { // Loop over all nodes and groups

      for (int j = groupSize; j > 0; j--) {
        size_t idx = (i - groupSize) * (groupSize - 1) + (i - j);
        // spdlog::trace("Start itterataion {} ({}-{})", idx, i, j);
        //  Set up indices
        size_t target_index = indices[i - j];
        size_t source_index = indices[i];

        spdlog::trace("Edge: {:06}-{:06}", target_index, source_index);

        // Set up target
        Cloud target = CreateCloud(dataset[target_index]);
        g2o::Isometry3 pose_target = g2o::Isometry3::Identity();
        pose_target.translation() =
            target->sensor_origin_.head<3>().cast<double>();
        pose_target.linear() =
            target->sensor_orientation_.toRotationMatrix().cast<double>();

        // Set up source
        Cloud source = CreateCloud(dataset[source_index]);
        g2o::Isometry3 pose_source = g2o::Isometry3::Identity();
        pose_source.translation() =
            source->sensor_origin_.head<3>().cast<double>();
        pose_source.linear() =
            source->sensor_orientation_.toRotationMatrix().cast<double>();

        auto vg = std::shared_ptr<pcl::VoxelGrid<PointT>>(
            new pcl::VoxelGrid<PointT>());
        vg->setLeafSize(0.1, 0.1, 0.1);

        // Set up filters
        FilterCollection filters{
            Filters::HighConfidenceFilter<pcl::PointXYZRGBA>(), vg,
            Filters::GridFilter<pcl::PointXYZRGBA>(0.1),
            // Filters::SIVFeatures<pcl::PointXYZRGBA>()
            // Filters::SIVFeatures<PointT>());
        };

        // Compute ICP
        spdlog::debug("Number of filters pre icp: {}", filters.size());
        g2o::Isometry3 xform = g2o::Isometry3(
            icp<PointT>(source, target, filters, maxCorrespondence)
                .cast<double>());

        // Update the soruce
        // pose_source = xform.inverse() * pose_source;
        // pose_source = xform * pose_source;
        pose_source = pose_source * xform;

        // Create Edge
        edges[idx] = new g2o::EdgeSE3();
        edges[idx]->setId(idx);

        edges[idx]->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer->vertex(target_index)));
        edges[idx]->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer->vertex(source_index)));

        // edges[idx]->setMeasurement(pose_source.inverse() * pose_target);
        edges[idx]->setMeasurement(pose_target * pose_source);
        // edges[idx]->setInformation(information_matrix);

        // Set Robust Kernel
        g2o::RobustKernel *kernel =
            robust_kernel_factory->construct(kernelName);
        if (kernel == nullptr) {
          spdlog::error("Kernel {} not found!", kernelName);
          continue;
        }

        kernel->setDelta(deltaValue); // 1.5 to 2.5
        edges[idx]->setRobustKernel(kernel);

        ++monitor;
      }
    }
  }

  spdlog::trace("Setting edges");
  for (auto &edge : edges) // Add edges to optimizer
    optimizer->addEdge(edge);
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
      pcl_viewer->addCoordinateSystem(0.5, pose,
                                      fmt::format("{}_{}_pose", name, i));
    }
    ++monitor;
  }

  // Filter if there are a lot of points
  if (indices.size() > 500) {
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
    vg.setLeafSize(0.3, 0.3, 0.3);
    vg.setInputCloud(merged);
    vg.filter(*merged);
  }
  auto color = getColor(merged);
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

// TODO: Add function doc string
void loop_detection(std::unique_ptr<g2o::SparseOptimizer> &optimizer,
                    Dataset &dataset, std::vector<size_t> &indices,
                    const std::string kernelName, const double deltaValue,
                    const Eigen::Matrix<double, 6, 6> information_matrix) {

  spdlog::trace("Entering loop detection");

  auto monitor = spdmon::LoggerProgress(
      "Checking edges", (indices.size() * (indices.size() - 1)) / 2);
  monitor.GetLogger()->set_level(spdlog::level::debug);

#pragma omp parallel for collapse(2) shared(dataset, indices)
  for (int i = 0; i < indices.size(); ++i) {
    for (int j = 0; j < i; ++j) {

      if (i == j) {
        ++monitor;
        continue;
      }

      spdlog::trace("Checking edge {}-{}", i, j);
      std::size_t idx_i = indices[i];
      std::size_t idx_j = indices[j];

      // Get the two verticies
      spdlog::trace("Gettig poses: {}-{}", idx_i, idx_j);
      g2o::Isometry3 pose1 =
          static_cast<g2o::VertexSE3 *>(optimizer->vertex(idx_i))->estimate();
      g2o::Isometry3 pose2 =
          static_cast<g2o::VertexSE3 *>(optimizer->vertex(idx_j))->estimate();

      spdlog::trace("Compute distance");
      double dist = (pose1.translation() - pose2.translation()).norm();

      if (dist > 5.0) {
        spdlog::debug("Distance between {} and {} is too large: {}", i, j,
                      dist);
        ++monitor;
        continue;
      }

      // ICP
      spdlog::trace("Creating clouds");
      auto cloud1 = CreateCloud(dataset[indices[i]]);
      auto cloud2 = CreateCloud(dataset[indices[j]]);

      // Set up filters
      spdlog::trace("Setting up filters");
      std::vector<typename pcl::Filter<pcl::PointXYZRGBA>::Ptr> filters{
          Filters::HighConfidenceFilter<pcl::PointXYZRGBA>(),
          Filters::GridFilter<pcl::PointXYZRGBA>(0.1),
      };
      spdlog::trace("Computing ICP");
      auto xform = Eigen::Matrix4f::Identity();
      // auto xform = icp<pcl::PointXYZRGBA>(cloud2, cloud1, filters, dist);

      // Update the pose
      spdlog::trace("Updating pose");
      pose1 = pose1 * g2o::Isometry3(xform.matrix().cast<double>());

      // Compute the relative transform
      spdlog::trace("Computing relative transform");
      g2o::Isometry3 measurment = pose1.inverse() * pose2;

      // Create edge
      spdlog::trace("Creating edge");
      g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
      edge->setVertex(1, optimizer->vertex(idx_i));
      edge->setVertex(0, optimizer->vertex(idx_j));
      edge->setMeasurement(measurment);

      // Set information matrix
      edge->setInformation(information_matrix);

      // Set robust kernel
      g2o::RobustKernel *kernel =
          g2o::RobustKernelFactory::instance()->construct(kernelName);
      if (kernel == nullptr) {
        spdlog::error("Kernel {} not found!", kernelName);
        ++monitor;
        continue;
      }
      kernel->setDelta(deltaValue);
      edge->setRobustKernel(kernel);

#pragma omp critical
      {
        spdlog::trace("Adding edge");
        edge->setId(optimizer->edges().size());
        optimizer->addEdge(edge);
      }
      spdlog::debug("Added edge between {} and {}", i, j);
      ++monitor;
    }
  }
  // pcl::registration::ELCH<pcl::PointXYZRGBA>::Ptr elch(
  //     new pcl::registration::ELCH<pcl::PointXYZRGBA>());
  // pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>::Ptr icp(
  //     new pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>);
  // icp->setMaximumIterations(100);
  // icp->setMaxCorrespondenceDistance(0.1);
  // icp->setRANSACOutlierRejectionThreshold(0.1);
  // elch->setReg(icp);

  // std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
  //             Eigen::aligned_allocator<pcl::PointXYZRGBA>>
  //     clouds;
  // clouds.resize(indices.size());

  // std::shared_ptr<spdmon::LoggerProgress> monitor;
  // monitor = std::make_shared<spdmon::LoggerProgress>("Createing clouds",
  //                                                    indices.size());

  // #pragma omp parallel for shared(dataset, indices, monitor)
  // for (int i = 0; i < indices.size(); ++i) {
  //   size_t idx = indices[i];
  //   clouds[i] = CreateCloud(dataset[idx]);
  //   auto filter = Filters::HighConfidenceFilter<pcl::PointXYZRGBA>();
  //   filter->setInputCloud(clouds[i]);
  //   filter->filter(*clouds[i]);
  //   ++(*monitor);
  // }

  // monitor.reset();
  // monitor =
  //     std::make_shared<spdmon::LoggerProgress>("Adding clouds",
  //     indices.size());
  // for (int i = 0; i < indices.size(); ++i) {
  //   elch->addPointCloud(clouds[i]);
  //   ++(*monitor);
  // }

  //  auto monitor = spdmon::LoggerProgress("Feature detection",
  //  indices.size());
  //
  // #pragma omp parrallel for shared(dataset, indices, monitor)
  //  for (int i = 0; i < indices.size(); ++i) {
  //    size_t idx = indices[i];
  //
  //    spdlog::trace("Feature detection for index {}", idx);
  //    auto cloud = CreateCloud(dataset[idx]);
  //
  //    spdlog::trace("Creating normals");
  //    pcl::PointCloud<PointXYZRGBANormal>::Ptr cloud_with_normals =
  //        compute_normals<pcl::PointXYZRGBA, PointXYZRGBANormal>(cloud);
  //
  //    spdlog::trace("Creating FPFH");
  //    pcl::FPFHEstimationOMP<PointXYZRGBANormal, PointXYZRGBANormal,
  //                           pcl::FPFHSignature33>
  //        fpfh;
  //    fpfh.setInputCloud(cloud_with_normals);
  //    fpfh.setInputNormals(cloud_with_normals);
  //    pcl::search::KdTree<PointXYZRGBANormal>::Ptr tree(
  //        new pcl::search::KdTree<PointXYZRGBANormal>());
  //    fpfh.setSearchMethod(tree);
  //
  //    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(
  //        new pcl::PointCloud<pcl::FPFHSignature33>());
  //
  //    // IMPORTANT: the radius used here has to be larger than the radius
  //    used to
  //    // estimate the surface normals!!!
  //    fpfh.setRadiusSearch(0.15);
  //    fpfh.compute(*fpfhs);
  //
  //    spdlog::debug("FPFH Size: {}", fpfhs->size());
  //    spdlog::debug("FPFH Point at 0: {}", fpfhs->points.at(0));
  //
  //    ++monitor;
  //  }
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
 * 		     "fix6_3" → Fixed size pose (6D) and landmark (3D) (used in 3
 * Bundle A justment) "var" → Variable block size Linear Solver Types:
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

  if (ReUseX::Visualizer::isInitialised())
    spdlog::info("Using Visualizer");
  else
    spdlog::trace("Not using Visualizer");

  auto data = dataset[0];
  for (auto field : data.fields()) {
    spdlog::debug("Field: {}", field);
  }

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
    // g2o::Isometry3 xform = pose_orig * estimate;
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

  spdlog::trace("Clear");
  optimizer->clear();

  if (ReUseX::Visualizer::isInitialised())
    ReUseX::Visualizer::getInstance()->wait();

  return path;
}
