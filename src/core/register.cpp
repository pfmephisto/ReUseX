#include "register.hh"
#include "compute_normals.hh"
#include "fmt_formatter.hh"
#include "icp.hh"
#include "parse_input_files.hh"
#include "spdmon.hh"
#include "types/Filters.hh"
#include "visualizer/visualizer.hh"

#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
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

#define VISUALIZE 1

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

  Eigen::Matrix4d xform{};
  xform << 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1;

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
  // transform.translation() = p.head<3>(); // Eigen::Vector3d(p[0], p[1],
  // p[1]);
  transform.translation() = Eigen::Vector3d(p[0], p[1], p[2]);

  return transform; //* g2o::Isometry3(xform);
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
 * 		     "fix6_3" → Fixed size pose (6D) and landmark (3D) (used i
 *  3 Bundle A justment) "var" → Variable block size Linear Solver Types:
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

  spdlog::debug("Indecies: first: {} last: {}, count: {}, step: ~{}",
                indices.front(), indices.back(), indices.size(),
                (indices.back() - indices.front()) / indices.size());

  spdlog::debug("Field: {}", dataset[0].fields());

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

  using PointT = pcl::PointXYZRGBA;
  using Cloud = typename pcl::PointCloud<PointT>;
  using CloudPtr = typename Cloud::Ptr;
  using Filter = typename pcl::Filter<PointT>::Ptr;
  using FilterCollection = std::vector<Filter>;
  using RGBHandler =
      typename pcl::visualization::PointCloudColorHandlerRGBField<PointT>;
  using CustomColorHander =
      pcl::visualization::PointCloudColorHandlerCustom<PointT>;

  // Initialise point cloud for icp alignment
  CloudPtr cloud(new Cloud());
  //*cloud += *CreateCloud(dataset[indices[0]]);
  Filter filter = ReUseX::Filters::HighConfidenceFilter<PointT>();
  filter->setInputCloud(cloud);
  filter->filter(*cloud);

#if VISUALIZE
  if (ReUseX::Visualizer::isInitialised()) {
    auto viewer = ReUseX::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();
    viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, RGBHandler(cloud), "cloud");
    viewer->resetCamera();
    viewer->spinOnce(100);
  }
#endif

  // All point clouds
  std::vector<CloudPtr, Eigen::aligned_allocator<CloudPtr>> clouds(
      indices.size());
  auto monitor = std::make_shared<spdmon::LoggerProgress>(
      "Loading Point clouds", indices.size());
#pragma omp parallel for shared(dataset, indices, clouds)
  for (int i = 0; i < indices.size(); ++i) {
    clouds[i] = CreateCloud(dataset[indices[i]]);
    auto filter = Filters::HighConfidenceFilter<pcl::PointXYZRGBA>();
    filter->setInputCloud(clouds[i]);
    filter->filter(*clouds[i]);

    clouds[i]->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 1);
    clouds[i]->sensor_orientation_ = Eigen::Quaternionf::Identity();

    ++(*monitor);
  }

  monitor =
      std::make_shared<spdmon::LoggerProgress>("Set vertecies", indices.size());
#pragma omp parallel for shared(dataset, indices, optimizer)
  for (int i = 0; i < indices.size(); ++i) {
    // Get the pose
    g2o::Isometry3 pose =
        odometryToIsometry(dataset[indices[i]].get<Field::ODOMETRY>());

    // Add vertex
    g2o::VertexSE3 *vert = new g2o::VertexSE3();
    vert->setId(i);
    vert->setFixed(i == 0); // Fix the first vertex
    vert->setEstimate(pose);

#pragma omp critical
    optimizer->addVertex(vert);

    ++(*monitor);
  }

#if 0
  if (ReUseX::Visualizer::isInitialised()) {
    auto viewer = ReUseX::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();
    for (size_t i = 0; i < indices.size(); ++i) {

      auto pose =
          static_cast<g2o::VertexSE3 *>(optimizer->vertex(i))->estimate();

      Eigen::Affine3f af_pose = Eigen::Affine3f::Identity();
      af_pose.linear() = pose.rotation().cast<float>();
      af_pose.translation() = pose.translation().cast<float>();

      auto name = fmt::format("pose_{}", indices[i]);

      viewer->addCoordinateSystem(0.2, af_pose,
                                  fmt::format("pose_{}", indices[i]));
    }
  }
#endif

  // std::getchar();

  monitor = std::make_shared<spdmon::LoggerProgress>("Creating splits",
                                                     indices.size() - 1);
  std::vector<std::pair<size_t, size_t>> sections;
  size_t start, end;
  start = 0;

  for (size_t i = 1; i < indices.size(); ++i) {
    size_t prev_index = i - 1;

    g2o::Isometry3 pose, pose_prev;
    pose = static_cast<g2o::VertexSE3 *>(optimizer->vertex(i))->estimate();
    pose_prev = static_cast<g2o::VertexSE3 *>(optimizer->vertex(prev_index))
                    ->estimate();

    double dist = (pose_prev.translation() - pose.translation()).norm();
    if (dist > 0.2) {

      sections.emplace_back(start, i - 1);
      start = i;
    }
    ++(*monitor);
  }
  sections.emplace_back(start, indices.size() - 1);

  std::vector<std::pair<size_t, size_t>> unrolled_pairs;
  unrolled_pairs = std::vector<std::pair<size_t, size_t>>();
  for (int i = 0; i < sections.size(); ++i)
    for (int j = sections[i].first + 1; j <= sections[i].second; ++j)
      unrolled_pairs.emplace_back(j - 1, j);

#if VISUALIZE
  if (ReUseX::Visualizer::isInitialised()) {
    auto viewer = ReUseX::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();
    for (size_t i = 0; i < unrolled_pairs.size(); ++i) {
      auto name = fmt::format("edge_{}-{}", unrolled_pairs[i].first,
                              unrolled_pairs[i].second);

      g2o::Isometry3 pose1, pose2;
      pose1 = static_cast<g2o::VertexSE3 *>(
                  optimizer->vertex(unrolled_pairs[i].first))
                  ->estimate();
      pose2 = static_cast<g2o::VertexSE3 *>(
                  optimizer->vertex(unrolled_pairs[i].second))
                  ->estimate();

      pcl::PointXYZ p1, p2;
      p1.x = pose1.translation()(0);
      p1.y = pose1.translation()(1);
      p1.z = pose1.translation()(2);
      p2.x = pose2.translation()(0);
      p2.y = pose2.translation()(1);
      p2.z = pose2.translation()(2);

      viewer->addLine(p1, p2, 0, 0, 0, name);
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, name);
    }
  }
#endif

  monitor = std::make_shared<spdmon::LoggerProgress>("Alining segments",
                                                     unrolled_pairs.size());
#pragma omp parallel for shared(clouds, optimizer, sections)                   \
    firstprivate(maxCorrespondence)
  for (int i = 0; i < unrolled_pairs.size(); ++i) {

    const size_t idx_tgt = unrolled_pairs[i].first;
    const size_t idx_src = unrolled_pairs[i].second;

    const FilterCollection filters = {
        Filters::HighConfidenceFilter<pcl::PointXYZRGBA>(),
        Filters::GridFilter<pcl::PointXYZRGBA>(0.15)};

    auto [matrix, fitness_score] = icp<pcl::PointXYZRGBA>(
        clouds[idx_src], clouds[idx_tgt], filters, maxCorrespondence);

    if (fitness_score > 0.02) {
      ++(*monitor);
      continue;
    }

    pcl::transformPointCloud(*clouds[idx_src], *clouds[idx_src], matrix);

    // Update pose
    g2o::Isometry3 pose =
        static_cast<g2o::VertexSE3 *>(optimizer->vertex(idx_src))->estimate();
    pose = pose * matrix.cast<double>();
    static_cast<g2o::VertexSE3 *>(optimizer->vertex(idx_src))
        ->setEstimate(pose);

    // Add edge

    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
    edge->setVertex(0, optimizer->vertex(idx_tgt));
    edge->setVertex(1, optimizer->vertex(idx_src));
    edge->setMeasurement(g2o::Isometry3(matrix.cast<double>()));
    edge->setInformation(information_matrix / fitness_score);

    auto kernel = g2o::RobustKernelFactory::instance()->construct(kernelName);
    kernel->setDelta(deltaValue);
    edge->setRobustKernel(kernel);
    // edge->setLevel(1);

#pragma omp critical
    optimizer->addEdge(edge);

    ++(*monitor);
  }

  monitor = std::make_shared<spdmon::LoggerProgress>("Merge Sections",
                                                     sections.size());
  std::vector<CloudPtr> clouds_sections(sections.size());
  for (size_t i = 0; i < sections.size(); ++i) {
    clouds_sections[i] = CloudPtr(new Cloud());
    for (size_t j = sections[i].first; j <= sections[i].second; ++j) {
      *clouds_sections[i] += *clouds[j];
    }

    auto filter = Filters::GridFilter<pcl::PointXYZRGBA>(0.05);
    filter->setInputCloud(clouds_sections[i]);
    filter->filter(*clouds_sections[i]);
  }

#if 0
  if (ReUseX::Visualizer::isInitialised()) {
    auto viewer = ReUseX::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();
    viewer->removeAllPointClouds();
    for (size_t i = 0; i < sections.size(); ++i) {
      pcl::RGB color = pcl::GlasbeyLUT::at(i % pcl::GlasbeyLUT::size());
      auto ch =
          CustomColorHander(clouds_sections[i], color.r, color.g, color.b);
      auto name =
          fmt::format("cloud_{}-{}", sections[i].first, sections[i].second);
      viewer->addPointCloud<pcl::PointXYZRGBA>(clouds_sections[i], ch, name);
      viewer->spinOnce(100);
    }
  }
#endif

  monitor = std::make_shared<spdmon::LoggerProgress>(
      "Reorient Sections", clouds_sections.size() - 1);
  g2o::Isometry3 matrix = g2o::Isometry3::Identity();
  for (size_t i = 1; i < clouds_sections.size(); ++i) {

    const auto s_idx = sections[i - 1].second;
    const auto e_idx = sections[i].first;

    g2o::Isometry3 s_pose, e_pose;
    s_pose =
        static_cast<g2o::VertexSE3 *>(optimizer->vertex(s_idx))->estimate();
    e_pose =
        static_cast<g2o::VertexSE3 *>(optimizer->vertex(e_idx))->estimate();

    // matrix = g2o::Isometry3::Identity();
    matrix = matrix * (e_pose.inverse() * s_pose);
    // TODO: This is disableing the reorientation of the point clouds
    // matrix = matrix * g2o::Isometry3::Identity(); // Don't change anything

#if 0
    if (ReUseX::Visualizer::isInitialised()) {
      auto viewer = ReUseX::Visualizer::getInstance()
                        ->getViewer<pcl::visualization::PCLVisualizer>();
      auto name =
          fmt::format("cloud_{}-{}_pre", sections[i].first, sections[i].second);

      pcl::RGB color = pcl::GlasbeyLUT::at(i % pcl::GlasbeyLUT::size());
      auto ch =
          CustomColorHander(clouds_sections[i], color.r, color.g, color.b);
      viewer->addPointCloud<pcl::PointXYZRGBA>(clouds_sections[i], ch, name);

      Eigen::Affine3f s_pose_a, e_pose_a;
      s_pose_a = Eigen::Affine3f::Identity();
      s_pose_a.linear() = s_pose.rotation().cast<float>();
      s_pose_a.translation() = s_pose.translation().cast<float>();

      e_pose_a = Eigen::Affine3f::Identity();
      e_pose_a.linear() = e_pose.rotation().cast<float>();
      e_pose_a.translation() = e_pose.translation().cast<float>();

      pcl::PointXYZ p1, p2;
      p1.x = s_pose_a.translation()(0);
      p1.y = s_pose_a.translation()(1);
      p1.z = s_pose_a.translation()(2);
      p2.x = e_pose_a.translation()(0);
      p2.y = e_pose_a.translation()(1);
      p2.z = e_pose_a.translation()(2);

      pcl::RGB color_line = pcl::GlasbeyLUT::at(i % pcl::GlasbeyLUT::size());

      viewer->addLine(p1, p2, color_line.r / 255.0, color_line.g / 255.0,
                      color_line.b / 255.0,
                      fmt::format("line_{}-{}_pre", s_idx, e_idx));
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3,
          fmt::format("line_{}-{}_pre", s_idx, e_idx));

      viewer->addCoordinateSystem(0.2, s_pose_a,
                                  fmt::format("s_pose_{}_pre", s_idx));
      viewer->addCoordinateSystem(0.2, e_pose_a,
                                  fmt::format("e_pose_{}_pre", e_idx));
      viewer->spinOnce(100);
    }
#endif
    pcl::transformPointCloud(*clouds_sections[i], *clouds_sections[i],
                             matrix.matrix());

#if VISUALIZE
    if (ReUseX::Visualizer::isInitialised()) {
      auto viewer = ReUseX::Visualizer::getInstance()
                        ->getViewer<pcl::visualization::PCLVisualizer>();
      auto name = fmt::format("cloud_{}-{}_post", sections[i].first,
                              sections[i].second);

      pcl::RGB color = pcl::GlasbeyLUT::at(i % pcl::GlasbeyLUT::size());
      auto ch =
          CustomColorHander(clouds_sections[i], color.r, color.g, color.b);
      viewer->addPointCloud<pcl::PointXYZRGBA>(clouds_sections[i], ch, name);

      s_pose = s_pose * matrix;
      e_pose = e_pose * matrix;

      Eigen::Affine3f s_pose_a, e_pose_a;
      s_pose_a = Eigen::Affine3f::Identity();
      s_pose_a.linear() = s_pose.rotation().cast<float>();
      s_pose_a.translation() = s_pose.translation().cast<float>();

      e_pose_a = Eigen::Affine3f::Identity();
      e_pose_a.linear() = e_pose.rotation().cast<float>();
      e_pose_a.translation() = e_pose.translation().cast<float>();

      pcl::PointXYZ p1, p2;
      p1.x = s_pose_a.translation()(0);
      p1.y = s_pose_a.translation()(1);
      p1.z = s_pose_a.translation()(2);
      p2.x = e_pose_a.translation()(0);
      p2.y = e_pose_a.translation()(1);
      p2.z = e_pose_a.translation()(2);

      pcl::RGB color_line = pcl::GlasbeyLUT::at(i % pcl::GlasbeyLUT::size());

      viewer->addLine(p1, p2, color_line.r / 255.0, color_line.g / 255.0,
                      color_line.b / 255.0,
                      fmt::format("line_{}-{}_post", s_idx, e_idx));
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3,
          fmt::format("line_{}-{}_post", s_idx, e_idx));

      viewer->addCoordinateSystem(0.2, s_pose_a,
                                  fmt::format("s_pose_{}_post", s_idx));
      viewer->addCoordinateSystem(0.2, e_pose_a,
                                  fmt::format("e_pose_{}_post", e_idx));
      viewer->spinOnce(100);
    }

#endif
    ++(*monitor);
  }

  // // Sort by size
  // std::vector<size_t> selected_indices(sections.size());
  // std::iota(selected_indices.begin(), selected_indices.end(), 0);
  // std::sort(selected_indices.begin(), selected_indices.end(),
  //           [&clouds_sections](size_t a, size_t b) {
  //             return clouds_sections[a]->size() >
  //             clouds_sections[b]->size();
  //           });

  // monitor = std::make_shared<spdmon::LoggerProgress>(
  //     "Alining sections", clouds_sections.size() - 1);
  // for (size_t i = 1; i < clouds_sections.size(); ++i) {
  //   auto tgt_idx = selected_indices[i - 1];
  //   auto src_idx = selected_indices[i];

  //   FilterCollection filters = {
  //       Filters::HighConfidenceFilter<pcl::PointXYZRGBA>(),
  //       Filters::GridFilter<pcl::PointXYZRGBA>(0.30)};

  //   auto [matrix, fitness_score] = icp<pcl::PointXYZRGBA>(
  //       clouds_sections[src_idx], clouds_sections[tgt_idx], filters, 5);

  //   pcl::transformPointCloud(*clouds_sections[src_idx],
  //                            *clouds_sections[src_idx], matrix);
  //   ++(*monitor);
  // }

#if VISUALIZE
  if (ReUseX::Visualizer::isInitialised()) {
    auto viewer = ReUseX::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();
    // viewer->removeAllPointClouds();
    for (size_t i = 0; i < clouds_sections.size(); ++i) {
      pcl::RGB color = pcl::GlasbeyLUT::at(i % pcl::GlasbeyLUT::size());
      auto ch =
          CustomColorHander(clouds_sections[i], color.r, color.g, color.b);
      auto name =
          fmt::format("cloud_{}-{}", sections[i].first, sections[i].second);
      viewer->addPointCloud<pcl::PointXYZRGBA>(clouds_sections[i], ch, name);
      viewer->spinOnce(100);
    }
  }
#endif

  if (ReUseX::Visualizer::isInitialised())
    ReUseX::Visualizer::getInstance()->wait();

  // Save start and end pose save the point clouds

  // for (size_t i = 0; i < sections.size(); ++i) {
  //   auto start_idx = sections[i].first;
  //   auto end_idx = sections[i].second;

  //  auto start_pose =
  //      static_cast<g2o::VertexSE3
  //      *>(optimizer->vertex(start_idx))->estimate();
  //  auto end_pose =
  //      static_cast<g2o::VertexSE3 *>(optimizer->vertex(end_idx))->estimate();

  //  std::string file_name = fmt::format("cloud_{}-{}.ply", start_idx,
  //  end_idx); fs::path file_path = fs::path("./") / file_name;

  //  spdlog::debug("Start Pose {}-{}: {}", start_idx, end_idx,
  //                start_pose.matrix().format(OctaveFmt));
  //  spdlog::debug("End Pose {}-{}: {}", start_idx, end_idx,
  //                end_pose.matrix().format(OctaveFmt));

  //  // save<pcl::PointXYZRGBA>(file_path, clouds_sections[i]);
  //  pcl::PLYWriter writer;
  //  writer.write(file_path, *clouds_sections[i], true, false);
  //}

  return path;
}
