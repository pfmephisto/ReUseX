#include "register.hh"
#include "fmt_formatter.hh"
#include "parse_input_files.hh"
#include "spdmon.hh"
#include "types/Filters.hh"
#include "visualizer/visualizer.hh"

#include <pcl/common/colors.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/registration/elch.h>
#include <pcl/registration/lum.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

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

constexpr auto mkLogger = [](const std::string name, const int size) {
  return std::make_shared<spdmon::LoggerProgress>(name, size);
};

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
static const g2o::Isometry3 odometryToIsometry(Eigen::MatrixXd odometry) {

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

class custom_mapping : public pcl::PointRepresentation<pcl::PointXYZRGBA> {
  using pcl::PointRepresentation<pcl::PointXYZRGBA>::nr_dimensions_;

    private:
  const Eigen::Vector3f rgb_to_intensity_weight_ =
      Eigen::Vector3f(0.299, 0.587, 0.114); // YIQ, YUV and NTSC
  // static const Eigen::Vector3f rgb_to_intensity_weight_ =
  //   Eigen::Vector3f (0.2126, 0.7152, 0.0722); // sRGB (and Rec709)

    public:
  custom_mapping() { nr_dimensions_ = 4; };

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray(const pcl::PointXYZRGBA &p, float *out) const {
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] =
        p.getRGBVector3i().cast<float>().dot(rgb_to_intensity_weight_) / 255.0;
  }
};

bool getKeypoints(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud,
                  pcl::PointCloud<pcl::PointXYZRGBA> &keypoints,
                  const Eigen::Affine3f &pose = Eigen::Affine3f::Identity()) {

  using PointT = pcl::PointXYZRGBA;
  using Cloud = pcl::PointCloud<PointT>;
  using CloudPtr = typename Cloud::Ptr;

  using Image = pcl::RangeImagePlanar;
  using ImagePtr = pcl::RangeImagePlanar::Ptr;

  using Extractor = pcl::ExtractIndices<pcl::PointXYZRGBA>;
  using ExtractorPtr = typename Extractor::Ptr;

  ImagePtr range_image_ptr(new Image);
  Image range_image = *range_image_ptr;
  range_image.createFromPointCloudWithFixedSize(
      *cloud, 256 /*width*/, 192 /*height*/, 127.12994667 /*center x*/,
      95.74299333 /*center y*/, 212.14793333 /*focal length x*/,
      212.14793333 /*focal length y*/, pose /*sensor pose*/,
      pcl::RangeImage::CAMERA_FRAME /*coordinate frame*/, 0.0f /*noise level*/,
      0.0f /*min range*/);
  // range_image.setUnseenToMaxRange();

  // range_image_widget_src.showRangeImage(range_image_src);
  // range_image_widget_tgt.showRangeImage(range_image_tgt);

  spdlog::trace("Keypoint detection");
  pcl::RangeImageBorderExtractor b_ext;
  pcl::NarfKeypoint detect(&b_ext);
  detect.getParameters().support_size = 0.1f;

  spdlog::trace("Set range image");
  detect.setRangeImage(&range_image);
  pcl::PointCloud<int> keypoint_idx;
  detect.compute(keypoint_idx);

  spdlog::trace("Convert keypoint indices to PointIndices");
  pcl::PointIndices::Ptr sel(new pcl::PointIndices());
  sel->indices.reserve(keypoint_idx.size());
  for (size_t idx = 0; idx < keypoint_idx.size(); ++idx)
    sel->indices.push_back(keypoint_idx[idx]);

  if (sel->indices.size() < 3) {
    spdlog::warn("No keypoints found");
    return false;
  }

  spdlog::trace("Extract keypoints");
  ExtractorPtr ext(new Extractor(true));
  ext->setInputCloud(cloud);
  ext->setIndices(sel);
  ext->filter(keypoints);

  return true;
};

class Buffer {
  using Affine3f = Eigen::Affine3f;
  using Matrix4f = Eigen::Matrix4f;
  using Vector3f = Eigen::Vector3f;

    private:
  const size_t capacity_;
  std::vector<Vector3f> buffer_;
  Affine3f last_;
  size_t end_ = 0;

    public:
  Buffer(const size_t capacity) : capacity_(capacity) {
    buffer_ = std::vector<Vector3f>(capacity, Vector3f::Zero());
    last_ = Affine3f::Identity();
  }

  void push_back(const Matrix4f &value) {
    Affine3f last_(value);
    buffer_[end_] = last_.translation();
    end_ = (end_ + 1) % capacity_;
  }

  Affine3f average() const {

    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    for (const auto &vec : buffer_)
      t += vec;
    t /= capacity_;

    Affine3f avg = Affine3f::Identity();
    avg.translation() = t;
    avg.linear() = last_.linear();

    return avg;
  }
};

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

  using PointT = pcl::PointXYZRGBA;
  // using PointT = PointXYZRGBANormal;
  using Cloud = typename pcl::PointCloud<PointT>;
  using CloudPtr = typename Cloud::Ptr;

  using Clouds = std::vector<CloudPtr, Eigen::aligned_allocator<CloudPtr>>;
  using CloudsWithNormals =
      std::vector<CloudPtr, Eigen::aligned_allocator<CloudPtr>>;

  using NormalT = pcl::Normal;
  using CloudNormal = typename pcl::PointCloud<NormalT>;
  using CloudNormalPtr = typename CloudNormal::Ptr;

  using Filter = typename pcl::Filter<PointT>::Ptr;
  using FilterCollection = std::vector<Filter>;

  using RGBHandler =
      typename pcl::visualization::PointCloudColorHandlerRGBField<PointT>;
  using CustomColorHander =
      pcl::visualization::PointCloudColorHandlerCustom<PointT>;

  using ICP = typename pcl::IterativeClosestPoint<PointT, PointT>;

  spdlog::info("Entering Write Graph Function");
  spdlog::debug("Indecies: first: {} last: {}, count: {}, step: ~{}",
                indices.front(), indices.back(), indices.size(),
                ((indices.back() - indices.front()) / indices.size()) + 1);

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

  // Initialise point cloud for icp alignment
  Clouds clouds(indices.size());
  Clouds keypoints(indices.size());

  pcl::PointCloud<pcl::PointXYZ> path1 = pcl::PointCloud<pcl::PointXYZ>();
  pcl::PointCloud<pcl::PointXYZ> path2 = pcl::PointCloud<pcl::PointXYZ>();

  // All point clouds
  // CloudsWithNormals clouds_with_normals(indices.size(),
  //                                       CloudNormalPtr(new CloudNormal()));

  CloudPtr cloud(new Cloud());
  CloudPtr keypoint_cloud(new Cloud());
  if constexpr (VISUALIZE) {
    if (ReUseX::Visualizer::isInitialised()) {
      auto viewer = ReUseX::Visualizer::getInstance()
                        ->getViewer<pcl::visualization::PCLVisualizer>();
      viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, RGBHandler(cloud),
                                               "cloud");
      viewer->addPointCloud<pcl::PointXYZRGBA>(
          keypoint_cloud, CustomColorHander(cloud, 0, 255, 0), "keypoints");
      viewer->resetCamera();
      viewer->spinOnce(100);
    }
  }

  auto monitor = mkLogger("Loading Point clouds", indices.size());
#pragma omp parallel for shared(dataset, indices, clouds)
  for (int i = 0; i < indices.size(); ++i) {

    using Image = pcl::RangeImagePlanar;
    using ImagePtr = typename Image::Ptr;

    using Extractor = pcl::ExtractIndices<pcl::PointXYZRGBA>;
    using ExtractorPtr = typename Extractor::Ptr;

    spdlog::trace("Loading point cloud: {}", i);
    clouds[i] = CreateCloud(dataset[indices[i]]);

    spdlog::trace("Filter point cloud: {}", i);
    auto hf = ReUseX::Filters::HighConfidenceFilter<pcl::PointXYZRGBA>();
    hf->setInputCloud(clouds[i]);
    hf->filter(*clouds[i]);

    spdlog::trace("Creating planar image: {}", i);
    ImagePtr range_image_ptr(new Image);
    Image range_image = *range_image_ptr;
    range_image.createFromPointCloudWithFixedSize(
        *clouds[i], 256 /*width*/, 192 /*height*/, 127.12994667 /*center x*/,
        95.74299333 /*center y*/, 212.14793333 /*focal length x*/,
        212.14793333 /*focal length y*/,
        Eigen::Affine3f::Identity() /*sensor pose*/,
        pcl::RangeImage::CAMERA_FRAME /*coordinate frame*/,
        0.0f /*noise level*/, 0.0f /*min range*/);

    spdlog::trace("Keypoint detection");
    pcl::PointCloud<int> keypoint_idx;
    pcl::RangeImageBorderExtractor b_ext;
    pcl::NarfKeypoint detect(&b_ext);
    detect.getParameters().support_size = 0.1f;
    detect.setRangeImage(&range_image);
    detect.compute(keypoint_idx);

    spdlog::trace("Add point the keypoint cloud");
    keypoints[i] = CloudPtr(new Cloud());
    for (size_t idx = 0; idx < keypoint_idx.size(); ++idx)
      keypoints[i]->points.push_back(clouds[i]->points[keypoint_idx[idx]]);

    // spdlog::trace("Compute normals: {}", i);
    // pcl::NormalEstimation<PointT, NormalT> ne;
    // ne.setInputCloud(clouds[i]);
    // pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    // ne.setSearchMethod(tree);
    // ne.setRadiusSearch(0.03);
    // ne.compute(*normals[i]);

    // spdlog::trace("Pre align point cloud: {}", i);
    // Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    // pose.linear() = clouds[i]->sensor_orientation_.toRotationMatrix();
    // pose.translation() = clouds[i]->sensor_origin_.template head<3>();
    // pcl::transformPointCloud(*clouds[i], *clouds[i], pose);

    spdlog::trace("Clear point cloud header: {}", i);
    clouds[i]->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 1);
    clouds[i]->sensor_orientation_ = Eigen::Quaternionf::Identity();

    ++(*monitor);

    if constexpr (VISUALIZE) {
      if (ReUseX::Visualizer::isInitialised()) {
        auto viewer = ReUseX::Visualizer::getInstance()
                          ->getViewer<pcl::visualization::PCLVisualizer>();
#pragma omp critical
        viewer->spinOnce(100);
      }
    }
  }

  const Eigen::Matrix4d FLIP =
      (Eigen::Matrix4d() << 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1)
          .finished();

  std::vector<g2o::VertexSE3 *> vertices =
      std::vector<g2o::VertexSE3 *>(indices.size());

  monitor = mkLogger("Set vertecies", indices.size());
  const auto START =
      odometryToIsometry(dataset[indices[0]].get<Field::ODOMETRY>()).inverse();
#pragma omp parallel for shared(dataset, indices, optimizer, vertices)
  for (int i = 0; i < indices.size(); ++i) {
    // Get the pose
    g2o::Isometry3 pose =
        odometryToIsometry(dataset[indices[i]].get<Field::ODOMETRY>());

    pose = START * pose;

    // pose *= FLIP;
    // pose.translation()[1] *= -1;
    // pose.translation()[2] *= -1;

    // Add vertex
    g2o::VertexSE3 *vert = new g2o::VertexSE3();
    vert->setId(i);
    vert->setFixed(i == 0); // Fix the first vertex
    vert->setEstimate(pose);
    vertices[i] = vert;

    // #pragma omp critical
    //     optimizer->addVertex(vert);

    ++(*monitor);
  }

  monitor = mkLogger("Aligning frames", indices.size() - 1);
  // monitor->GetLogger()->set_level(spdlog::level::trace);

  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();

  *cloud += *clouds[0];
  *keypoint_cloud += *keypoints[0];

  pcl::PointXYZ p;
  p.getVector3fMap() =
      vertices[0]->estimate().translation().head<3>().cast<float>();
  path1.points.push_back(p);
  path2.points.push_back(p);

  // pcl::visualization::RangeImageVisualizer range_image_widget_src(
  //     "Range image Source");
  // pcl::visualization::RangeImageVisualizer range_image_widget_tgt(
  //     "Range image Target");

  const Eigen::Matrix4f FRUSTUM_CONVERSIO =
      (Eigen::Matrix4f() << 0, 0, 1, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1)
          .finished();

  const float alpha[4] = {1.0, 1.0, 1.0, 0.0};
  custom_mapping point_representation{};
  point_representation.setRescaleValues(alpha);
  auto point_representation_ptr =
      pcl::make_shared<const custom_mapping>(point_representation);

  for (size_t i = 1; i < indices.size() - 1; ++i) {
    // Intiialize Filters
    using FrustomCulling = pcl::FrustumCulling<PointT>::Ptr;
    using GridFilter = pcl::VoxelGrid<PointT>::Ptr;

    FrustomCulling fc = FrustomCulling(new pcl::FrustumCulling<PointT>());
    fc->setVerticalFOV(80);   // 60 degrees
    fc->setHorizontalFOV(60); // 45 degrees
    fc->setNearPlaneDistance(1.0);
    fc->setFarPlaneDistance(7);
    fc->setCameraPose(vertices[i]->estimate().matrix().cast<float>() *
                      FRUSTUM_CONVERSIO);

    GridFilter gf = ReUseX::Filters::GridFilter<PointT>(0.10);

    CloudPtr ft_src(new Cloud());
    CloudPtr ft_tgt(new Cloud());

    fc->setInputCloud(keypoint_cloud);
    fc->filter(*ft_tgt);

    bool use_keypoints = keypoints[i]->size() > 3 && ft_tgt->size() > 3;

    if (use_keypoints) {
      pcl::copyPointCloud(*keypoints[i], *ft_src);
    } else {
      gf->setInputCloud(clouds[i]);
      gf->filter(*ft_src);

      fc->setInputCloud(cloud);
      fc->filter(*ft_tgt);

      gf->setInputCloud(ft_tgt);
      gf->filter(*ft_tgt);
    }

    // TODO: List of things to implement
    //   Add compute_normals

    assert(ft_src->size() > 0 && "ft_src can not be empty");
    assert(ft_tgt->size() > 0 && "ft_tgt can not be empty");

    optimizer->addVertex(vertices[i]);

    ICP reg;
    reg.setMaximumIterations(50);
    reg.setMaxCorrespondenceDistance(0.5);
    reg.setTransformationEpsilon(1e-8);
    reg.setEuclideanFitnessEpsilon(1e-8);
    reg.setRANSACIterations(1000);
    reg.setRANSACOutlierRejectionThreshold(0.02);
    reg.setPointRepresentation(point_representation_ptr);

    reg.setInputSource(ft_src);
    reg.setInputTarget(ft_tgt);

    // Add icp visualization callback
    if constexpr (VISUALIZE) {
      if (ReUseX::Visualizer::isInitialised()) {
        auto viewer = ReUseX::Visualizer::getInstance()
                          ->getViewer<pcl::visualization::PCLVisualizer>();
        // Construct the callback function to visualize the
        // registration process
        std::function<typename pcl::Registration<
            PointT, PointT>::UpdateVisualizerCallbackSignature>
        callback([viewer](const pcl::PointCloud<PointT> &c1,
                          const pcl::Indices &idxs1,
                          const pcl::PointCloud<PointT> &c2,
                          const pcl::Indices &idxs2) -> void {
          CloudPtr c1_ptr = c1.makeShared();
          CloudPtr c2_ptr = c2.makeShared();

          viewer->addPointCloud<PointT>(
              c1_ptr, CustomColorHander(c1.makeShared(), 255, 0, 0),
              "icp_source");

          viewer->addPointCloud<PointT>(
              c2_ptr, CustomColorHander(c2.makeShared(), 0, 0, 255),
              "icp_target");

          pcl::Correspondences correspondences;
          for (size_t i = 0; i < idxs1.size(); ++i)
            correspondences.push_back(
                pcl::Correspondence(idxs1[i], idxs2[i], 1));

          viewer->addCorrespondences<PointT>(c1_ptr, c2_ptr, correspondences,
                                             "correspondences");

          viewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "icp_source");

          viewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "icp_target");

          viewer->spinOnce(1);

          viewer->removePointCloud("icp_source");
          viewer->removePointCloud("icp_target");
          viewer->removeShape("correspondences");
        });
        // Register the callback
        reg.registerVisualizationCallback(callback);
      }
    };

    reg.align(*ft_src, matrix);

    auto dist = reg.getFinalTransformation().block<3, 1>(0, 3).norm();

    bool hasConverged = false;
    if (reg.hasConverged() && reg.getFitnessScore() < 0.01 && dist < 0.5) {
      *keypoint_cloud += *keypoints[i];
      *cloud += *clouds[i];
      hasConverged = true;
    }

    auto xform = hasConverged ? reg.getFinalTransformation()
                              : Eigen::Matrix4f::Identity();

    if (hasConverged) {
      pcl::transformPointCloud(*clouds[i], *clouds[i], xform);
      pcl::transformPointCloud(*keypoints[i], *keypoints[i], xform);

      *keypoint_cloud += *keypoints[i];
      *cloud += *clouds[i];
    } else {
      if (!reg.hasConverged())
        spdlog::warn("ICP failed to converge at {}", indices[i]);
      if (reg.getFitnessScore() > 0.01)
        spdlog::warn("ICP fitness score too high at {}", indices[i]);
      if (dist > 0.5)
        spdlog::warn("ICP distance too high at {}", indices[i]);

      // std::getchar();
    }

    ++(*monitor);

    if constexpr (VISUALIZE) {

      if (i % 5 == 0) {
        spdlog::trace("Grid filter point cloud: {}", i);
        auto gf = ReUseX::Filters::GridFilter<PointT>(0.05);
        gf->setInputCloud(cloud);
        gf->filter(*cloud);

        spdlog::trace("Staticical outlier filter point cloud: {}", i);
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        // sor.filter(*cloud);
      }

      p.getVector3fMap() =
          vertices[i]->estimate().translation().head<3>().cast<float>();
      path1.points.push_back(p);
      path2.points.emplace_back(xform(0, 3), xform(1, 3), xform(2, 3));

      pcl::PolygonMesh mesh1, mesh2;
      pcl::toPCLPointCloud2(path1, mesh1.cloud);
      pcl::toPCLPointCloud2(path2, mesh2.cloud);

      mesh1.polygons.emplace_back();
      mesh1.polygons[0].vertices.resize(path1.points.size());
      std::iota(mesh1.polygons[0].vertices.begin(),
                mesh1.polygons[0].vertices.end(), 0);

      mesh2.polygons.emplace_back();
      mesh2.polygons[0].vertices.resize(path2.points.size());
      std::iota(mesh2.polygons[0].vertices.begin(),
                mesh2.polygons[0].vertices.end(), 0);

      if (ReUseX::Visualizer::isInitialised()) {
        auto viewer = ReUseX::Visualizer::getInstance()
                          ->getViewer<pcl::visualization::PCLVisualizer>();

        // Draw frustum
        if (i != 1) {
          viewer->removeShape(fmt::format("line_orig-{}", i));
          viewer->removeShape(fmt::format("line_icp-{}", i));
        }
        viewer->addPolylineFromPolygonMesh(mesh1,
                                           fmt::format("line_orig-{}", i));
        viewer->addPolylineFromPolygonMesh(mesh2,
                                           fmt::format("line_icp-{}", i));
        viewer->updatePointCloud<PointT>(cloud, RGBHandler(cloud), "cloud");
        viewer->updatePointCloud<PointT>(
            keypoint_cloud, CustomColorHander(keypoint_cloud, 0, 255, 0),
            "keypoints");

        viewer->addCoordinateSystem(
            0.2,
            Eigen::Affine3f(vertices[i]->estimate().matrix().cast<float>()),
            fmt::format("pose_icp_{}", i));
        if (i == 1)
          viewer->addCoordinateSystem(0.5, Eigen::Affine3f(FRUSTUM_CONVERSIO),
                                      fmt::format("Frustom", i));

        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2,
            fmt::format("line_orig-{}", i));
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2,
            fmt::format("line_icp-{}", i));
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 0,
            fmt::format("line_orig-{}", i));
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0,
            fmt::format("line_icp-{}", i));
        viewer->spinOnce(100);
      }
    }

    // if (!reg.hasConverged()) {
    //   spdlog::warn("ICP failed to converge at {}",
    //   indices[i]); if (ReUseX::Visualizer::isInitialised())
    //     ReUseX::Visualizer::getInstance()->wait();
    //   return path;
    // }
  }

  if (ReUseX::Visualizer::isInitialised())
    ReUseX::Visualizer::getInstance()->wait();

  return path;
}
