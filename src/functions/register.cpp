#include "functions/register.hh"
#include "functions/icp.hh"
#include "functions/parse_input_files.hh"
#include "functions/progress_bar.hh"
#include "functions/spdmon.hh"
#include "types/Filters.hh"

#include "visualizer/visualizer.hh"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <robust_kernel.h>
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
// #include <g2o/types/slam3d/edge_se3.h>
// #include <g2o/types/slam3d/types_slam3d.h>
// #include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <omp.h>

#include <cassert>
#include <filesystem>
#include <string>
#include <vector>

const Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "",
                                "\n[", "]");

template <typename T>
struct fmt::formatter<Eigen::WithFormat<T>> : fmt::ostream_formatter {};

namespace fs = std::filesystem;
using namespace ReUseX;

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

// Available kernels:
// Huber
// Cauchy
// Welsch
// Tukey
// Fair
// DCS
// GemanMcClure
// ScaleDelta
// Saturated
// PseudoHuber
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

        // Set up filters
        FilterCollection filters{
            // Filters::HighConfidenceFilter<pcl::PointXYZRGBA>(),
            // Filters::GridFilter<pcl::PointXYZRGBA>(0.1),
            Filters::SIVFeatures<pcl::PointXYZRGBA>()};

        // Compute ICP
        g2o::Isometry3 xform = g2o::Isometry3(
            icp<PointT>(source, target, filters, maxCorrespondence)
                .cast<double>());

        // Update the soruce
        // pose_source = pose_source * xform;
        pose_source = pose_source * xform.inverse();
        // pose_source = xform * pose_source;
        // pose_source = xform.inverse() * pose_source;

        // Isometry t = Isometry(pose_target);
        // t = t * xform;
        // t = t * xform.inverse();
        // t = xform * t;
        // t = xform.inverse() * t;

        // Create Edge
        edges[idx] = new g2o::EdgeSE3();
        edges[idx]->setId(idx);

        edges[idx]->setVertex(0, static_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer->vertex(target_index)));
        edges[idx]->setVertex(1, static_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer->vertex(source_index)));

        // edges[idx]->setMeasurement(pose_source.inverse() * pose_target);
        edges[idx]->setMeasurement(pose_target.inverse() * pose_source);
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

// solverName = "lm_fix6_3_csparse"
// [algorithm]_[fix|var][pose_dim]_[landmark_dim]_[linear_solver]
// Common Algorithm Prefixes:
// "lm_" → Levenberg-Marquardt
// "gn_" → Gauss-Newton
// "dl_" → Dogleg
// Pose & Landmark Dimension:
// "fix6_3" → Fixed size pose (6D) and landmark (3D) (used in 3D Bundle A
// justment) "var" → Variable block size
// Linear Solver Types:
// "csparse" → Uses CSparse
// "cholmod" → Uses CHOLMOD (SuiteSparse)
// "pcg" → Uses Preconditioned Conjugate Gradient
// "dense" → Uses Dense Solver (not recommended for large problems)
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

  if (ReUseX::Visualizer::isInitialised()) {
    auto viewer = ReUseX::Visualizer::getInstance();
    auto pcl_viewer = viewer->getViewer<pcl::visualization::PCLVisualizer>();

    spdmon::LoggerProgress monitor("Adding Source Point Clouds",
                                   indices.size());

    // monitor.GetLogger()->set_level(spdlog::level::debug);
#pragma omp parallel for
    for (int i = 0; i < indices.size(); i++) {
      auto idx = indices[i];
      // for (auto [logger, idx] : spdmon::LogProgress(indices)) {

      // Create Point Cloud
      auto cloud = CreateCloud(dataset[idx]);

      // Filter point cloud to only include hight confidence points
      auto filter = Filters::HighConfidenceFilter<pcl::PointXYZRGBA>();
      filter->setInputCloud(cloud);
      filter->filter(*cloud);

      // Set name for point cloud in viewer
      std::string name = cloud->header.frame_id + "_pre";

      // Set the point cloud colore to be rea
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> red(
          cloud, 255, 0, 0);

#pragma omp critical
      {
        // Add point cloud to viewer
        pcl_viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, red, name);

        // Set point cloud display properties
        // pcl_viewer->setPointCloudRenderingProperties(
        //     pcl::visualization::PCL_VISUALIZER_SHADING,
        //     pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, name);
        pcl_viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
      }
      ++monitor;
    }
    viewer->step();
  }

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

  //// Visualise incremental transforms
  //// Temporary code
  // if (ReUseX::Visualizer::isInitialised() && groupSize == 1) {
  //   auto viewer = ReUseX::Visualizer::getInstance();
  //   auto pcl_viewer = viewer->getViewer<pcl::visualization::PCLVisualizer>();

  //  std::vector<g2o::Isometry3> transforms = std::vector<g2o::Isometry3>();
  //  transforms.resize(optimizer->edges()->size());

  //  for (size_t i = 0; i < optimizer->edges().size(); ++i) {
  //    transforms[i] = optimizer->Edge(i).estimate();
  //  }
  //}

  spdlog::trace("Optimize");
  spdlog::stopwatch sw;
  optimizer->initializeOptimization();
  optimizer->optimize(maxIterations);
  spdlog::debug("Optimize ran in {:.3f}s", sw);

  spdlog::trace("Save graph to file");
  optimizer->save(path.c_str());

  if (ReUseX::Visualizer::isInitialised()) {
    auto viewer = ReUseX::Visualizer::getInstance();
    auto pcl_viewer = viewer->getViewer<pcl::visualization::PCLVisualizer>();

    spdmon::LoggerProgress monitor("Adding Target Point Clouds",
                                   indices.size());

    // monitor.GetLogger()->set_level(spdlog::level::debug);
#pragma omp parallel for
    for (int i = 0; i < indices.size(); i++) {
      auto idx = indices[i];
      // for (auto [logger, idx] : spdmon::LogProgress(indices)) {

      // Create a point cloud
      auto cloud = CreateCloud(dataset[idx]);

      // Filter the point cloud
      auto filter = Filters::HighConfidenceFilter<pcl::PointXYZRGBA>();
      filter->setInputCloud(cloud);
      filter->filter(*cloud);

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

      // Set the name for the point cloud in the viewer
      std::string name = cloud->header.frame_id + "_post";

      // The the point cloud color to be green
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> green(
          cloud, 0, 255, 0);

#pragma omp critical
      {
        // Add point cloud to the viewer
        pcl_viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, green, name);

        // Set point cloud display properties
        // pcl_viewer->setPointCloudRenderingProperties(
        //     pcl::visualization::PCL_VISUALIZER_SHADING,
        //     pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, name);
        pcl_viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
        pcl_viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, name);
      }

      ++monitor;
    }
  }

  spdlog::trace("Clear");
  optimizer->clear();

  if (ReUseX::Visualizer::isInitialised())
    ReUseX::Visualizer::getInstance()->wait();

  return path;
}
