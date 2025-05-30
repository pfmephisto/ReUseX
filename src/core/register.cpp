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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/auto_io.h>
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
#define TEMP_EXTENSION "pcd"

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

  virtual bool isValid(const pcl::PointXYZRGBA &) const { return true; }
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

template <class T, size_t N> constexpr size_t size(T (&)[N]) { return N; }

template <
    typename PointT, typename NormalT = pcl::Normal,
    typename FPFHSignature33T = pcl::FPFHSignature33,

    typename CloudNormal = typename pcl::PointCloud<NormalT>,
    typename FPFHSignature33 = typename pcl::PointCloud<FPFHSignature33T>,

    typename CloudNormalPtr = typename CloudNormal::Ptr,
    typename FPFHSignature33Ptr =
        typename pcl::PointCloud<FPFHSignature33T>::Ptr,

    typename KDTree = pcl::search::KdTree<PointT>,
    typename KDTreePtr = typename pcl::search::KdTree<PointT>::Ptr,

    typename NormalEstimation = pcl::NormalEstimationOMP<PointT, pcl::Normal>,
    typename FPFHEstimation =
        pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33>>
FPFHSignature33T
getHistogram(typename pcl::PointCloud<PointT>::ConstPtr cloud) {

  spdlog::trace("Starting FPFH histogram computation");

  if (cloud->empty()) {
    spdlog::warn("Input cloud is empty, cannot compute FPFH histogram");
    return FPFHSignature33T();
  }

  NormalEstimation ne;
  ne.setInputCloud(cloud);
  KDTreePtr tree(new KDTree());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.05);
  ne.useSensorOriginAsViewPoint();

  spdlog::trace("Computing normals");
  CloudNormalPtr normals(new CloudNormal);
  ne.compute(*normals);

  // Create the FPFH estimation class, and pass the input dataset+normals to it
  spdlog::trace("Creating FPFH estimation object");
  FPFHEstimation fpfh;
  fpfh.setInputCloud(cloud);
  fpfh.setInputNormals(normals);
  fpfh.setSearchMethod(tree);
  // IMPORTANT: the radius used here has to be larger than the radius used to
  // estimate the surface normals!!!
  fpfh.setRadiusSearch(0.15);

  spdlog::trace("Computing FPFH features");
  FPFHSignature33Ptr fpfhs(new FPFHSignature33());
  fpfh.compute(*fpfhs);

  spdlog::trace("Aggregating FPFH features into a global histogram");
  FPFHSignature33T global;
  std::fill(std::begin(global.histogram), std::end(global.histogram), 0.0f);

  size_t histogram_size = size(global.histogram);

  for (const auto &f : fpfhs->points)
    for (int i = 0; i < histogram_size; ++i)
      global.histogram[i] += f.histogram[i];

  for (int i = 0; i < histogram_size; ++i)
    global.histogram[i] /= static_cast<float>(fpfhs->size());

  return global;
}

template <typename PointT> struct Submap {
  using Cloud = typename pcl::PointCloud<PointT>;
  using CloudPtr = typename Cloud::Ptr;
  CloudPtr map = CloudPtr(new Cloud());
  CloudPtr keypoints = CloudPtr(new Cloud());
  std::vector<size_t> indices = std::vector<size_t>();
};

bool spliteSubmap(const Eigen::Affine3f &prev, const Eigen::Affine3f &current,
                  const float translation_threshold = 0.07,
                  const float angle_threshold = pcl::deg2rad(5.0)) {

  const float distance = (current.translation() - prev.translation()).norm();
  const Eigen::Quaternionf q_prev(prev.linear());
  const Eigen::Quaternionf q_current(current.linear());
  const float angle = q_prev.angularDistance(q_current);

  const bool split = (distance > translation_threshold ||
                      angle > angle_threshold); // 0.05m, 2 degrees

  spdlog::debug("Distance: {:.3f}, Angle: {:.3f} -> Split: {}", distance,
                pcl::rad2deg(angle), split);

  return split;
}

template <typename PointT>
std::function<typename pcl::Registration<
    PointT, PointT>::UpdateVisualizerCallbackSignature>
reg_visualization_callback([](const pcl::PointCloud<PointT> &c1,
                              const pcl::Indices &idxs1,
                              const pcl::PointCloud<PointT> &c2,
                              const pcl::Indices &idxs2) -> void {
  if constexpr (VISUALIZE) {
    using CloudPtr = typename pcl::PointCloud<PointT>::Ptr;
    using RGBHandler =
        typename pcl::visualization::PointCloudColorHandlerRGBField<PointT>;
    using CustomColorHander =
        pcl::visualization::PointCloudColorHandlerCustom<PointT>;

    if (c1.empty() || c2.empty()) {
      spdlog::warn("One of the point clouds is empty, skipping visualization");
      return;
    }

    if (!ReUseX::Visualizer::isInitialised()) {
      spdlog::warn("Visualizer is not initialized, skipping visualization");
      return;
    }

    // Create shared pointers for the point clouds
    spdlog::trace("Creating shared pointers for point clouds");
    CloudPtr c1_ptr = c1.makeShared();
    CloudPtr c2_ptr = c2.makeShared();

    // Get the viewer instance
    auto viewer = ReUseX::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();

    spdlog::trace("Adding point clouds to viewer");
    viewer->addPointCloud<PointT>(
        c1_ptr, CustomColorHander(c1.makeShared(), 255, 0, 0), "icp_source");

    viewer->addPointCloud<PointT>(
        c2_ptr, CustomColorHander(c2.makeShared(), 0, 0, 255), "icp_target");

    spdlog::trace("Adding correspondences to viewer");
    pcl::Correspondences correspondences;
    for (size_t i = 0; i < idxs1.size(); ++i)
      correspondences.push_back(pcl::Correspondence(idxs1[i], idxs2[i], 1));

    viewer->addCorrespondences<PointT>(c1_ptr, c2_ptr, correspondences,
                                       "correspondences");

    spdlog::trace("Setting point cloud rendering properties");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "icp_source");

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "icp_target");

    viewer->spinOnce(1);

    spdlog::trace("Removing point clouds and correspondences from viewer");
    viewer->removePointCloud("icp_source");
    viewer->removePointCloud("icp_target");
    viewer->removeShape("correspondences");
    spdlog::trace("Visualization callback completed");
  };
});

/**
 * @brief Writes a graph structure to a specified file path.
 *
 * This function processes a dataset and writes a graph representation to
 * the specified file path. It supports configurable solver and kernel
 * options, as well as parameters for controlling the graph generation
 * process.
 *
 * @param path The file path where the graph will be written.
 * @param dataset The dataset containing the graph data.
 * @param indices A vector of indices specifying the subset of the dataset
 * to process.
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
 * @note Ensure that the provided `path` is writable and the dataset is
 * valid. The solver and kernel names must conform to the expected formats.
 *       This function logs its progress using `spdlog`.
 */

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
extract_key_points(typename pcl::PointCloud<PointT>::Ptr cloud,
                   const Eigen::Affine3f pose = Eigen::Affine3f::Identity()) {

  using Cloud = pcl::PointCloud<PointT>;
  using CloudPtr = typename Cloud::Ptr;

  using Image = pcl::RangeImagePlanar;
  using ImagePtr = pcl::RangeImagePlanar::Ptr;

  using Extractor = pcl::ExtractIndices<pcl::PointXYZRGBA>;
  using ExtractorPtr = typename Extractor::Ptr;

  spdlog::trace("Get planar image");
  ImagePtr range_image_ptr(new Image);
  Image range_image = *range_image_ptr;
  range_image.createFromPointCloudWithFixedSize(
      *cloud, 256 /*width*/, 192 /*height*/, 127.12994667 /*center x*/,
      95.74299333 /*center y*/, 212.14793333 /*focal length x*/,
      212.14793333 /*focal length y*/, pose /*sensor pose*/,
      pcl::RangeImage::CAMERA_FRAME /*coordinate frame*/, 0.0f /*noise level*/,
      0.0f /*min range*/);
  // range_image.setUnseenToMaxRange();

  spdlog::trace("Keypoint detection");
  pcl::RangeImageBorderExtractor b_ext;
  pcl::NarfKeypoint detect(&b_ext);
  detect.getParameters().support_size = 0.1f;

  spdlog::trace("Set range image");
  detect.setRangeImage(&range_image);
  pcl::PointCloud<int> keypoint_idx;
  detect.compute(keypoint_idx);

  spdlog::trace("Set keypoints");
  CloudPtr keypoints = CloudPtr(new Cloud());
  for (size_t i = 0; i < keypoint_idx.size(); ++i)
    keypoints->points.push_back(cloud->points[keypoint_idx[i]]);

  return keypoints;
}

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
  using ConditionalRemoval = pcl::ConditionalRemoval<PointT>::Ptr;
  using GridFilter = pcl::VoxelGrid<PointT>::Ptr;

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

  // Save submaps
  const fs::path temp_path = fs::path(fmt::format("./{}_reg", dataset.name()));

  spdlog::trace("Precompute submaps");
  std::vector<Submap<PointT>> submaps = std::vector<Submap<PointT>>();
  std::vector<Eigen::Affine3f> poses(indices.size());

  // Skip preprocessing if submaps already exist
  std::set<size_t> submap_indices = std::set<size_t>();
  if (fs::exists(temp_path))
    goto load_submaps;

  submaps.push_back(Submap<PointT>());
  for (size_t i = 0; i < indices.size(); ++i) {

    spdlog::trace("Load data");
    const size_t index = indices[i];
    const auto data = dataset[index];

    spdlog::trace("Get pose");
    g2o::Isometry3 pose = odometryToIsometry(data.get<Field::ODOMETRY>());

    spdlog::trace("Get point cloud");
    poses[i] = Eigen::Affine3f::Identity();
    poses[i].linear() = pose.linear().cast<float>();
    poses[i].translation() = pose.translation().cast<float>();

    spdlog::trace("Check if we need to split the submap");
    if (spliteSubmap(poses[i - 1], poses[i]))
      submaps.push_back(Submap<PointT>());

    submaps.back().indices.push_back(i);
  }

  spdlog::debug("Submaps: {}", submaps.size());
  {
    auto logger = spdmon::LoggerProgress("Submap", submaps.size());
#pragma omp parallel for shared(submaps, dataset, indices, poses)              \
    schedule(dynamic, 1)
    for (int i = 0; i < submaps.size(); i++) {

      // Submap
      spdlog::trace("Processing submap {}", i);
      CloudPtr submap = submaps[i].map;
      CloudPtr kp_submap = submaps[i].keypoints;

      spdlog::trace("Submap {}: indices: {}", i, submaps[i].indices.size());

      if (submaps[i].indices.size() == 0) {
        spdlog::warn("Submap {} is empty, skipping", i);
        ++logger;
        continue;
      }

      // Index of the first point cloud in the map
      const size_t index_0 = submaps[i].indices[0];
      const Eigen::Affine3f pose_0 = poses[index_0];
      spdlog::debug("Submap {}: first index: {}", i, index_0);

      const size_t index_0_mapped = indices[index_0];
      spdlog::debug("Submap {}: first index mapped: {}", i, index_0_mapped);
      CloudPtr first_cloud = CreateCloud(dataset[indices[index_0]]);

      ConditionalRemoval hf =
          ReUseX::Filters::HighConfidenceFilter<pcl::PointXYZRGBA>();
      hf->setInputCloud(first_cloud);
      hf->filter(*first_cloud);

      *submap += *first_cloud;
      *kp_submap += *extract_key_points<PointT>(first_cloud);

      pcl::transformPointCloud(*submap, *submap, pose_0);
      pcl::transformPointCloud(*kp_submap, *kp_submap, pose_0);

      for (size_t j = 1; j < submaps[i].indices.size(); ++j) {

        // Get the point cloud
        const size_t index_j = submaps[i].indices[j];
        const size_t index_j_mapped = indices[index_j];
        spdlog::trace("Load point cloud {} at index "
                      "{} in dataset",
                      index_j, index_j_mapped);
        CloudPtr cloud = CreateCloud(dataset[index_j_mapped]);
        CloudPtr keypoints = extract_key_points<PointT>(cloud);

        spdlog::trace("Transform point cloud and keypoints");
        pcl::transformPointCloud(*cloud, *cloud, poses[index_j]);
        pcl::transformPointCloud(*keypoints, *keypoints, poses[index_j]);

        cloud->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 1);
        cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();

        // Cloud for registration

        spdlog::trace("Filter point cloud");
        hf->setInputCloud(cloud);
        hf->filter(*cloud);

        CloudPtr src_cloud(new Cloud());

        auto gf = ReUseX::Filters::GridFilter<PointT>(0.05);
        gf->setInputCloud(cloud);
        gf->filter(*src_cloud);

        if (src_cloud->size() < 3) {
          spdlog::warn("Not enough points in cloud "
                       "at index {}",
                       indices[index_j]);
          ++logger;
          continue;
        }

        spdlog::trace("ICP registration");
        ICP reg;
        reg.setMaximumIterations(50);
        reg.setMaxCorrespondenceDistance(0.5);
        reg.setTransformationEpsilon(1e-8);
        reg.setEuclideanFitnessEpsilon(1e-8);
        reg.setRANSACIterations(1000);
        reg.setMaxCorrespondenceDistance(0.05);
        reg.setRANSACOutlierRejectionThreshold(0.02);

        // Set point representation
        const float alpha[4] = {1, 1, 1, 1};
        custom_mapping point_representation{};
        auto point_representation_ptr =
            pcl::make_shared<const custom_mapping>(point_representation);
        reg.setPointRepresentation(point_representation_ptr);

        spdlog::debug("Clouds: src: {}, submap: {}", fmt::ptr(src_cloud.get()),
                      fmt::ptr(submap.get()));

        reg.setInputSource(src_cloud);
        reg.setInputTarget(submap);

        // reg.registerVisualizationCallback(reg_visualization_callback<PointT>);

        reg.align(*src_cloud);

        if (reg.hasConverged()) {

          spdlog::trace("Get transformation");
          Eigen::Matrix4f xform = reg.getFinalTransformation();

          spdlog::trace("Transform cloud and keypoints");
          pcl::transformPointCloud(*cloud, *cloud, xform);
          pcl::transformPointCloud(*keypoints, *keypoints, xform);

          spdlog::trace("Add cloud and keypoints to submap");
          *submap += *cloud;
          *kp_submap += *keypoints;

        } else
          spdlog::warn("ICP failed to converge at {}", indices[index_j]);

        if (i % 5 == 0) {
          spdlog::trace("Grid filter point cloud");
          auto gf = ReUseX::Filters::GridFilter<PointT>(0.05);
          gf->setInputCloud(submap);
          gf->filter(*submap);
        }
      }

      spdlog::trace("Grid filter point cloud");
      auto gf = ReUseX::Filters::GridFilter<PointT>(0.05);
      gf->setInputCloud(submap);
      gf->filter(*submap);

      spdlog::trace("Staticical outlier filter point cloud");
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(submap);
      sor.setMeanK(50);
      sor.setStddevMulThresh(1.0);
      // sor.filter(*submap);

      ++logger;
    }
  }

save_submaps:
  // Create temp directory
  if (!fs::exists(temp_path))
    std::filesystem::create_directories(temp_path);
  else if (!fs::is_empty(temp_path)) {
    spdlog::warn("Temp directory {} is not empty, clearing it",
                 temp_path.string());
    fs::remove_all(temp_path);
    std::filesystem::create_directories(temp_path);
  }

  spdlog::trace("Save submaps");
  for (size_t i = 0; i < submaps.size(); ++i) {

    const std::string ext = TEMP_EXTENSION;
    const auto &submap = submaps[i];

    const fs::path submap_path =
        temp_path / fmt::format("submap_{}.{}", i, ext);
    spdlog::debug("Saving submap {} to {}", i, submap_path.string());
    pcl::io::savePCDFile(submap_path.string(), *submap.map);

    const fs::path keypoints_path =
        temp_path / fmt::format("keypoints_{}.{}", i, ext);
    spdlog::debug("Saving keypoints {} to {}", i, keypoints_path.string());
    pcl::io::savePCDFile(keypoints_path.string(), *submap.keypoints);

    // Save indices
    const fs::path indices_path = temp_path / fmt::format("indices_{}.txt", i);
    spdlog::debug("Saving indices {} to {}", i, indices_path.string());
    std::ofstream indices_file(indices_path);
    if (!indices_file.is_open()) {
      spdlog::error("Failed to open indices file: {}", indices_path.string());
      continue;
    }
    for (const auto &index : submap.indices) {
      indices_file << index << "\n";
    }
    indices_file.close();
  }
  goto align_submaps;

load_submaps:
  // Load submaps
  spdlog::trace("Load submaps");

  for (auto &entry : fs::directory_iterator(temp_path)) {
    if (entry.is_regular_file() &&
        entry.path().extension() ==
            fmt::format(".{}", TEMP_EXTENSION).c_str()) {
      const std::string filename = entry.path().filename().string();
      const std::string index_str =
          filename.substr(filename.find('_') + 1, filename.find('.'));

      size_t submap_index = std::atoi(index_str.c_str());
      spdlog::debug("Found submap index: {}", submap_index);

      submap_indices.insert(submap_index);
    }
  }

  spdlog::debug("Found {} submaps", submap_indices.size());
  submaps.clear();
  submaps.resize(submap_indices.size());

  {
    auto logger = spdmon::LoggerProgress("Load Submaps", submap_indices.size());

    for (const auto &index : submap_indices) {

      submaps[index] = Submap<PointT>();

      const std::string ext = TEMP_EXTENSION;

      const fs::path submap_path =
          temp_path / fmt::format("submap_{}.{}", index, ext);
      const fs::path keypoints_path =
          temp_path / fmt::format("keypoints_{}.{}", index, ext);

      spdlog::debug("Loading submap {} from {}", index, submap_path.string());
      pcl::io::loadPCDFile(submap_path.string(), *submaps[index].map);

      spdlog::debug("Loading keypoints {} from {}", index,
                    keypoints_path.string());
      pcl::io::loadPCDFile(keypoints_path.string(), *submaps[index].keypoints);

      // spdlog::trace("Remove NaN points from submap and keypoints");
      // pcl::PassThrough<PointT> pass;
      // for (CloudPtr m : std::array<CloudPtr, 2>{submaps[index].map,
      //                                           submaps[index].keypoints}) {
      //   pcl::Indices indices_map;
      //   pcl::removeNaNFromPointCloud(*m, *m, indices_map);
      //   for (auto &f : {"z", "y", "x"}) {
      //     pass.setInputCloud(m);
      //     pass.setFilterFieldName(f);
      //     pass.setFilterLimits(std::numeric_limits<float>::min(),
      //                          std::numeric_limits<float>::max());
      //     pass.filter(*m);
      //   }
      // }

      // Load indices
      const fs::path indices_path =
          temp_path / fmt::format("indices_{}.txt", index);
      spdlog::debug("Loading indices {} from {}", index, indices_path.string());
      std::ifstream indices_file(indices_path);
      if (!indices_file.is_open()) {
        spdlog::error("Failed to open indices file: {}", indices_path.string());
        continue;
      }
      std::string line;
      while (std::getline(indices_file, line)) {
        if (!line.empty()) {
          size_t idx = std::stoul(line);
          submaps[index].indices.push_back(idx);
        }
      }
      indices_file.close();

      spdlog::debug("Loaded submap {} with {} points and {} keypoints", index,
                    submaps[index].map->size(),
                    submaps[index].keypoints->size());
      ++logger;
    }
  }

align_submaps:
  spdlog::trace("Align submaps");

  if (false) {

    if constexpr (VISUALIZE) {
      spdlog::trace("Visualize submaps before alignment");
      if (ReUseX::Visualizer::isInitialised()) {
        auto viewer = ReUseX::Visualizer ::getInstance()
                          ->getViewer<pcl::visualization::PCLVisualizer>();
        CloudPtr all_submaps(new Cloud());
        for (size_t i = 0; i < submaps.size(); ++i)
          *all_submaps += *submaps[i].map;

        auto gf = ReUseX::Filters::GridFilter<PointT>(0.05);
        gf->setInputCloud(all_submaps);
        gf->filter(*all_submaps);

        viewer->addPointCloud<PointT>(all_submaps, RGBHandler(all_submaps),
                                      "all_submaps_before_alignment");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
            "all_submaps_before_alignment");
        viewer->resetCamera();
        viewer->spinOnce(1);
      }
    }

    auto logger = spdmon::LoggerProgress("Align Submaps", submaps.size() - 1);

    int prev_index = 0;
    for (size_t i = 1; i < submaps.size(); ++i) {

      // if (submaps[prev_index].map->empty()) {
      if (submaps[prev_index].keypoints->empty()) {
        prev_index = i;
        spdlog::warn(
            "Previous submap is empty, skipping registration for index {}", i);
        ++logger;
        continue;
      }

      // if (submaps[i].map->empty()) {
      if (submaps[i].keypoints->empty()) {
        spdlog::warn(
            "Current submap is empty, skipping registration for index {}", i);
        ++logger;
        continue;
      }

      spdlog::trace("ICP registration {} -> {}", prev_index, i);
      ICP reg;
      reg.setMaximumIterations(50);
      reg.setMaxCorrespondenceDistance(0.5);
      reg.setTransformationEpsilon(1e-8);
      reg.setEuclideanFitnessEpsilon(1e-8);
      reg.setRANSACIterations(1000);
      reg.setMaxCorrespondenceDistance(1.00);
      reg.setRANSACOutlierRejectionThreshold(0.02);

      // Set point representation
      const float alpha[4] = {1, 1, 1, 0};
      custom_mapping point_representation{};
      auto point_representation_ptr =
          pcl::make_shared<const custom_mapping>(point_representation);
      reg.setPointRepresentation(point_representation_ptr);

      // reg.setInputSource(submaps[i].map);
      // reg.setInputTarget(submaps[prev_index].map);
      reg.setInputSource(submaps[i].keypoints);
      reg.setInputTarget(submaps[prev_index].keypoints);

      reg.registerVisualizationCallback(reg_visualization_callback<PointT>);

      auto pose = poses[submaps[i].indices[0]].inverse() *
                  poses[submaps[i - 1].indices[0]];

      CloudPtr dummy(new Cloud());
      reg.align(*dummy, pose.matrix());

      if (reg.hasConverged()) {

        spdlog::trace("Get transformation");
        Eigen::Matrix4f xform = reg.getFinalTransformation();

        spdlog::trace("Transform cloud and keypoints");
        pcl::transformPointCloud(*submaps[i].map, *submaps[i].map, xform);
        pcl::transformPointCloud(*submaps[i].keypoints, *submaps[i].keypoints,
                                 xform);

        for (size_t j = 0; j < submaps[i].indices.size(); ++j) {
          const size_t index = submaps[i].indices[j];
          poses[index] = poses[index] * Eigen::Affine3f(xform);
        }

        prev_index = i;
      } else
        spdlog::warn("ICP failed to converge at {}", i);

      ++logger;
    }
  }

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr vocabulary(
      new pcl::PointCloud<pcl::FPFHSignature33>());

  static std::string VOCABULARY_NAME =
      fmt::format("{}_fpfh_vocab.pcd", dataset.name());

  if (fs::exists(VOCABULARY_NAME)) {
    spdlog::trace("Load vocabulary from file");
    pcl::io::loadPCDFile(VOCABULARY_NAME, *vocabulary);
    spdlog::debug("Vocabulary loaded with {} points", vocabulary->size());
  } else {
    spdlog::trace("Vocabulary file not found, creating new vocabulary");

    vocabulary->points.resize(submaps.size());
    vocabulary->width = submaps.size();
    vocabulary->height = 1;

#pragma omp parallel for shared(submaps, vocabulary)
    for (int i = 0; i < submaps.size(); ++i) {
      vocabulary->points[i] = getHistogram<PointT>(submaps[i].map);
    }
    spdlog::debug("Vocabulary created with {} points", vocabulary->size());
    pcl::io::savePCDFile(VOCABULARY_NAME, *vocabulary);
  }

  pcl::KdTreeFLANN<pcl::FPFHSignature33> kdtree;
  kdtree.setInputCloud(vocabulary);

  for (size_t i = 0; i < submaps.size(); ++i) {
    spdlog::trace("Get FPFH histogram for submap {}", i);

    if (submaps[i].map->empty()) {
      spdlog::warn("Submap {} is empty, skipping FPFH computation", i);
      continue;
    }

    pcl::FPFHSignature33 fpfh = getHistogram<PointT>(submaps[i].map);

    // Find the nearest neighbor in the vocabulary
    std::vector<int> search_indices(2);
    std::vector<float> search_distances(2);

    if (!kdtree.nearestKSearch(fpfh, 2, search_indices, search_distances)) {
      spdlog::warn("No nearest neighbors found for submap {}", i);
      continue;
    }

    spdlog::debug("Nearest neighbor for submap {}: index {}, distance {}", i,
                  search_indices[1], search_distances[1]);

    // Visualize the edge between submaps
    if constexpr (VISUALIZE) {
      if (ReUseX::Visualizer::isInitialised()) {
        auto viewer = ReUseX::Visualizer::getInstance()
                          ->getViewer<pcl::visualization::PCLVisualizer>();

        const pcl::RGB c = pcl::GlasbeyLUT::at(i);
        const size_t idx_1 = indices[submaps[i].indices[0]];
        const size_t idx_2 = indices[submaps[search_indices[1]].indices[0]];
        const std::string name = fmt::format("edge_{}_{}", idx_1, idx_2);

        pcl::PointXYZ p1, p2;
        p1.getVector3fMap() = dataset[idx_1]
                                  .get<Field::ODOMETRY>()
                                  .block<1, 3>(0, 2)
                                  .cast<float>();
        p2.getVector3fMap() = dataset[idx_2]
                                  .get<Field::ODOMETRY>()
                                  .block<1, 3>(0, 2)
                                  .cast<float>();

        viewer->addLine<pcl::PointXYZ>(p1, p2, c.r, c.g, c.b, name);
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, name);
      }
    }

    // // Create a vertex for the submap
    // g2o::VertexSE3 *v = new g2o::VertexSE3();
    // v->setId(i);
    // v->setEstimate(g2o::Isometry3(poses[submaps[i].indices[0]]));
    // v->setFixed(i == 0); // Fix the first vertex
    // optimizer->addVertex(v);

    // // Add a feature vector to the vertex
    // g2o::Vector7D feature_vector;
    // feature_vector.head<6>() =
    //     poses[submaps[i].indices[0]].matrix().cast<double>();
    // feature_vector[6] = static_cast<double>(indices[0]);
    // v->setUserData(new g2o::Vector7D(feature_vector));

    // // Add edges between submaps based on nearest neighbors
    // if (i > 0) {
    //   g2o::EdgeSE3 *e = new g2o::EdgeSE3();
    //   e->vertices()[0] = optimizer->vertex(i - 1);
    //   e->vertices()[1] = optimizer->vertex(i);
    //   e->setInformation(information_matrix.cast<double>());
    //   e->setMeasurement(g2o::Isometry3(poses[submaps[i].indices[0]]).inverse()
    //   *
    //                     g2o::Isometry3(poses[submaps[i - 1].indices[0]]));
    //   optimizer->addEdge(e);
    // }
  }

  if constexpr (VISUALIZE) {
    spdlog::trace("Visualize submaps");
    if (ReUseX::Visualizer::isInitialised()) {
      auto viewer = ReUseX::Visualizer::getInstance()
                        ->getViewer<pcl::visualization::PCLVisualizer>();

      for (size_t i = 0; i < submaps.size(); ++i) {
        CloudPtr map = submaps[i].map;
        CloudPtr keypoints = submaps[i].keypoints;

        const pcl::RGB c = pcl::GlasbeyLUT::at(i);

        viewer->addPointCloud<PointT>(map, RGBHandler(map),
                                      fmt::format("submap_cloud_{}", i));
        viewer->addPointCloud<PointT>(
            keypoints, CustomColorHander(keypoints, c.r, c.g, c.b),
            fmt::format("submap_keypoints_{}", i));

        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
            fmt::format("submap_keypoints_{}", i));
      }
    }

    ReUseX::Visualizer::getInstance()->wait();
  }
  return path;
}
