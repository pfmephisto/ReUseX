#include "register.hh"
#include "fmt_formatter.hh"
#include "parse_input_files.hh"
#include "report.hh"
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
#include <pcl/features/shot.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/auto_io.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/elch.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/transformation_estimation_svd.h>
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

template <typename PointT> struct Submap {
  using Cloud = typename pcl::PointCloud<PointT>;
  using CloudPtr = typename Cloud::Ptr;
  CloudPtr map = CloudPtr(new Cloud());
  CloudPtr keypoints = CloudPtr(new Cloud());
  std::vector<size_t> indices = std::vector<size_t>();

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features =
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr(
          new pcl::PointCloud<pcl::FPFHSignature33>());

  std::vector<std::pair<size_t, double>> loop_closure_candidates =
      std::vector<std::pair<size_t, double>>();
};

template <
    typename PointT, typename NormalT = pcl::Normal,
    typename PointOutT = pcl::FPFHSignature33,

    typename CloudNormal = typename pcl::PointCloud<NormalT>,
    typename CloudOut = typename pcl::PointCloud<PointOutT>,

    typename CloudNormalPtr = typename CloudNormal::Ptr,
    typename CloudOutPtr = typename pcl::PointCloud<PointOutT>::Ptr,

    typename KDTree = pcl::search::KdTree<PointT>,
    typename KDTreePtr = typename pcl::search::KdTree<PointT>::Ptr,

    typename NormalEstimation = pcl::NormalEstimationOMP<PointT, pcl::Normal>,
    typename FPFHEstimation =
        pcl::FPFHEstimationOMP<PointT, NormalT, PointOutT>>
void setHistogram(Submap<PointT> &submap) {

  spdlog::trace("Starting FPFH histogram computation");

  if (submap.map->empty()) {
    spdlog::warn("Input cloud is empty, cannot compute FPFH histogram");
    return;
  }

  NormalEstimation ne;
  ne.setInputCloud(submap.map);
  KDTreePtr tree(new KDTree());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.05);
  ne.useSensorOriginAsViewPoint();

  spdlog::trace("Computing normals");
  CloudNormalPtr normals(new CloudNormal);
  ne.compute(*normals);

  // TODO: Add sensor origin

  const float model_resolution =
      0.05f; // Adjust this value based on VoxelGrid downsampling.

  pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
  iss_detector.setSearchMethod(tree);

  iss_detector.setSalientRadius(2.5 * model_resolution); // Lowered from 10
  iss_detector.setNonMaxRadius(2 * model_resolution);    // Lowered from 8
  iss_detector.setThreshold21(0.6);                      // Lowered from 0.2
  iss_detector.setThreshold32(0.6);                      // Lowered from 0.2
  iss_detector.setMinNeighbors(10);                      // Lowered from 10

  iss_detector.setNumberOfThreads(10);

  iss_detector.setInputCloud(submap.map);

  iss_detector.compute(*submap.keypoints);
  pcl::PointIndicesConstPtr keypoints_indices =
      iss_detector.getKeypointsIndices();

  // Create the FPFH estimation class, and pass the input dataset+normals to
  // it
  spdlog::trace("Creating FPFH estimation object");
  FPFHEstimation fpfh;
  fpfh.setInputCloud(submap.map);
  fpfh.setIndices(keypoints_indices);
  fpfh.setInputNormals(normals);
  fpfh.setSearchMethod(tree);
  // IMPORTANT: the radius used here has to be larger than the radius used to
  // estimate the surface normals!!!
  fpfh.setRadiusSearch(0.15);

  spdlog::trace("Computing FPFH features");
  fpfh.compute(*submap.features);

  return;
}

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

g2o::Isometry3
getMeasurment(const Eigen::Affine3f &pose1, Eigen::Affine3f pose2,
              const Eigen::Matrix4f &xform = Eigen::Matrix4f::Identity()) {
  pose2 = pose2 * Eigen::Affine3f(xform);
  return g2o::Isometry3((pose1.inverse() * pose2).matrix().cast<double>());
}

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
  optimizer->setVerbose(false);
  optimizer->setAlgorithm(solver);

  Report report("Registration Report");

  // Save submaps
  const fs::path temp_path = fs::path(fmt::format("./{}_reg", dataset.name()));

  spdlog::trace("Precompute submaps");
  std::vector<Submap<PointT>> submaps = std::vector<Submap<PointT>>();
  std::vector<Eigen::Affine3f> poses(indices.size());

  // INFO: Point cloud for visualizing the path
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
  pcl::PointCloud<pcl::PointXYZ>::Ptr path_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  spdlog::info("Get poses");
  for (size_t i = 0; i < indices.size(); ++i) {

    spdlog::trace("Load data");
    const size_t index = indices[i];
    const auto data = dataset[index];

    spdlog::trace("Get poses");
    g2o::Isometry3 pose = odometryToIsometry(data.get<Field::ODOMETRY>());

    spdlog::trace("Get point cloud");
    poses[i] = Eigen::Affine3f::Identity();
    poses[i].linear() = pose.linear().cast<float>();
    poses[i].translation() = pose.translation().cast<float>();
  }

  // Skip preprocessing if submaps already exist
  std::set<size_t> submap_indices = std::set<size_t>();
  if (fs::exists(temp_path))
    goto load_submaps;

  // submaps.push_back(Submap<PointT>());
  for (size_t i = 0; i < indices.size(); ++i) {

    spdlog::trace("Check if we need to split the submap");
    if (spliteSubmap(poses[i - 1], poses[i]))
      submaps.push_back(Submap<PointT>());

    submaps.back().indices.push_back(i);
  }

  spdlog::debug("Submaps: {}", submaps.size());
  {
    auto logger = spdmon::LoggerProgress("Submap", submaps.size());
    spdlog::stopwatch sw;
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
    report.addLine(logger.GetLogger()->name(), sw.elapsed().count());
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
    spdlog::stopwatch sw;

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
    report.addLine(logger.GetLogger()->name(), sw.elapsed().count());
  }

  if constexpr (VISUALIZE) {
    spdlog::info("Visualize submaps after loading");
    if (ReUseX::Visualizer::isInitialised()) {
      spdlog::trace("Visualize submaps after loading");
      auto viewer = ReUseX::Visualizer::getInstance()
                        ->getViewer<pcl::visualization::PCLVisualizer>();

      viewer->removeAllPointClouds();

      for (size_t i = 0; i < submaps.size(); ++i) {
        viewer->addPointCloud<PointT>(submaps[i].map,
                                      RGBHandler(submaps[i].map),
                                      fmt::format("submap_{}", i));

        viewer->spinOnce(1);
      }
    }
  }

align_submaps:
  spdlog::trace("Align submaps");

  {
    auto logger =
        spdmon::LoggerProgress("Add vertecies to Optimizer", submaps.size());
    spdlog::stopwatch sw;

    for (size_t i = 0; i < submaps.size(); ++i) {
      spdlog::trace("Creating vertex for submap {}", i);
      g2o::VertexSE3 *v = new g2o::VertexSE3();
      v->setId(i);
      v->setEstimate(
          g2o::Isometry3(poses[submaps[i].indices[0]].matrix().cast<double>()));
      v->setFixed(i == 0); // Fix the first vertex
      optimizer->addVertex(v);
    }
    report.addLine(logger.GetLogger()->name(), sw.elapsed().count());
  }

  {
    auto logger = spdmon::LoggerProgress("Compute Keypoints and Histogram",
                                         submaps.size());
    spdlog::stopwatch sw;
#pragma omp parallel for shared(submaps)
    for (int i = 0; i < submaps.size(); ++i) {
      setHistogram<PointT>(submaps[i]);
      spdlog::debug("Submap {}: keypoints: {}, features: {}", i,
                    submaps[i].keypoints->size(), submaps[i].features->size());
      ++logger;
    }
    report.addLine(logger.GetLogger()->name(), sw.elapsed().count());
  }

  {
    auto logger =
        spdmon::LoggerProgress("Consecutive Edges", submaps.size() - 1);
    spdlog::stopwatch sw;

    mesh->polygons.push_back(pcl::Vertices());

    for (size_t i = 1; i < submaps.size(); ++i) {
      size_t prev_index = submaps[i - 1].indices.back();
      size_t current_index = submaps[i].indices.front();

      auto prev_pose = poses[prev_index];
      auto current_pose = poses[current_index];

      // INFO: Add path to path cloud
      for (size_t j = 0; j < submaps[i - 1].indices.size(); ++j) {
        size_t idx = submaps[i - 1].indices[j];
        auto pose = poses[idx];
        path_cloud->points.push_back(pcl::PointXYZ(pose.translation().x(),
                                                   pose.translation().y(),
                                                   pose.translation().z()));
        mesh->polygons.back().vertices.push_back(path_cloud->points.size() - 1);
      }
      path_cloud->points.push_back(pcl::PointXYZ(
          current_pose.translation().x(), current_pose.translation().y(),
          current_pose.translation().z()));
      mesh->polygons.back().vertices.push_back(path_cloud->points.size() - 1);

      ICP reg;
      reg.setMaximumIterations(50);
      reg.setTransformationEpsilon(1e-8);
      reg.setEuclideanFitnessEpsilon(1e-8);
      reg.setRANSACIterations(1000);
      reg.setMaxCorrespondenceDistance(0.05);
      reg.setRANSACOutlierRejectionThreshold(0.02);
      // Set point representation
      const float alpha[4] = {1, 1, 1, 0.5};
      custom_mapping point_representation{};
      auto point_representation_ptr =
          pcl::make_shared<const custom_mapping>(point_representation);
      reg.setPointRepresentation(point_representation_ptr);

      reg.setInputSource(submaps[i].map);
      reg.setInputTarget(submaps[i - 1].map);
      reg.registerVisualizationCallback(reg_visualization_callback<PointT>);

      CloudPtr dummy(new Cloud());
      // FIXME: The reorientation seems wrong,
      reg.align(*dummy);
      // reg.align(*dummy, (current_pose.inverse() * prev_pose).matrix());
      // reg.align(*dummy, (prev_pose.inverse() * current_pose).matrix());
      // reg.align(*dummy,
      //           (prev_pose.inverse() * current_pose).inverse().matrix());
      // reg.align(*dummy,
      //           (current_pose.inverse() * prev_pose).inverse().matrix());

      if (!reg.hasConverged()) {
        spdlog::warn("ICP failed to converge for submaps {} and {}", i - 1, i);
        ++logger;
        continue;
      }

      auto xform = reg.getFinalTransformation();
      auto score = reg.getFitnessScore();

      // TODO: Add edges for concecutive submaps
      g2o::EdgeSE3 *e = new g2o::EdgeSE3();
      spdlog::debug("Vertex {}: {}, Vertex {}: {}", i - 1,
                    optimizer->vertex(i - 1)->id(), i,
                    optimizer->vertex(i)->id());
      e->setVertex(0, optimizer->vertex(i - 1));
      e->setVertex(1, optimizer->vertex(i));
      e->setMeasurement(getMeasurment(poses[submaps[i - 1].indices[0]],
                                      poses[submaps[i].indices[0]], xform));
      e->setInformation(information_matrix.cast<double>() * score);

      e->setRobustKernel(
          g2o::RobustKernelFactory::instance()->construct(kernelName));

      optimizer->addEdge(e);

      // INFO: Visualize the
      if constexpr (VISUALIZE) {
        if (ReUseX::Visualizer::isInitialised()) {
          auto viewer = ReUseX::Visualizer::getInstance()
                            ->getViewer<pcl::visualization::PCLVisualizer>();

          // viewer->addCoordinateSystem(0.1, prev_pose,
          //                             fmt::format("pose_{}_prev", i - 1));
          // viewer->addCoordinateSystem(0.1, current_pose,
          //                             fmt::format("pose_{}_curr", i));

          viewer->addPointCloud<PointT>(dummy, RGBHandler(dummy),
                                        fmt::format("submap_dummy_{}", i));

          viewer->spinOnce(1);
        }
      }

      ++logger;
    }
    report.addLine(logger.GetLogger()->name(), sw.elapsed().count());
  }

  if (false) {

    auto logger = spdmon::LoggerProgress("Compute Loop Closure Candidates",
                                         submaps.size() - 1);
    spdlog::stopwatch sw;

    for (size_t i = 1; i < submaps.size(); ++i) {

      if constexpr (VISUALIZE) {
        if (ReUseX::Visualizer::isInitialised()) {
          auto viewer = ReUseX::Visualizer::getInstance()
                            ->getViewer<pcl::visualization::PCLVisualizer>();

          auto color = pcl::GlasbeyLUT::at(i);

          // Visualize the submap
          viewer->addPointCloud<PointT>(
              submaps[i].map,
              pcl::visualization::PointCloudColorHandlerCustom<PointT>(
                  submaps[i].map, color.r, color.g, color.b),
              fmt::format("submap_{}", i));

          //// Visualize the keypoints
          // viewer->addPointCloud<PointT>(
          //     submaps[i].keypoints,
          //     pcl::visualization::PointCloudColorHandlerCustom<PointT>(
          //         submaps[i].keypoints, 255, 0, 0),
          //     fmt::format("keypoints_{}", i));

          viewer->spinOnce(100);
        }
      }

      for (size_t j = i - 1; j > 0 && i - j < 10; --j) {

        pcl::KdTreeFLANN<pcl::FPFHSignature33> kdtree;
        kdtree.setInputCloud(submaps[j].features);

        size_t good_matches = 0;

        for (size_t k = 0; k < submaps[i].features->size(); ++k) {
          const auto &fpfh = submaps[i].features->points[k];

          const size_t K = 2;
          std::vector<int> indices(K);
          std::vector<float> sqr_dists(K);

          if (kdtree.nearestKSearch(fpfh, K, indices, sqr_dists)) {
            float ratio = std::sqrt(sqr_dists[0] / sqr_dists[1]);
            if (ratio < 0.8) {
              good_matches++;
            }
          }
        }

        float match_score = static_cast<float>(good_matches) /
                            static_cast<float>(submaps[i].features->size());

        if (match_score > 0.3) {
          submaps[i].loop_closure_candidates.push_back({j, match_score});

          if constexpr (VISUALIZE) {
            if (ReUseX::Visualizer::isInitialised()) {
              auto viewer =
                  ReUseX::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();

              auto color = pcl::GlasbeyLUT::at(i);

              const std::string name =
                  fmt::format("loop_candidate_{}_{}", i, j);

              // Visualize the loop closure candidate
              viewer->addPointCloud<PointT>(
                  submaps[j].map,
                  pcl::visualization::PointCloudColorHandlerCustom<PointT>(
                      submaps[j].map, color.r, color.g, color.b),
                  fmt::format("loop_candidate_{}_{}", i, j));

              const std::string name_keypoints =
                  fmt::format("loop_candidate_keypoints_{}_{}", i, j);

              viewer->addPointCloud<PointT>(
                  submaps[j].keypoints,
                  pcl::visualization::PointCloudColorHandlerCustom<PointT>(
                      submaps[j].keypoints, 255, 0, 0),
                  name_keypoints);

              viewer->setPointCloudRenderingProperties(
                  pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                  name_keypoints);

              viewer->addLine<pcl::PointXYZ>(
                  pcl::PointXYZ(poses[submaps[i].indices[0]].translation().x(),
                                poses[submaps[i].indices[0]].translation().y(),
                                poses[submaps[i].indices[0]].translation().z()),
                  pcl::PointXYZ(poses[submaps[j].indices[0]].translation().x(),
                                poses[submaps[j].indices[0]].translation().y(),
                                poses[submaps[j].indices[0]].translation().z()),
                  0.6875, 0.5429, 0.5546, fmt::format("edge_{}_{}", i, j));
              viewer->setShapeRenderingProperties(
                  pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2,
                  fmt::format("edge_{}_{}", i, j));

              viewer->spinOnce(1);
            }
          } // End of visualization
        } // End of if match_score
      }

      spdlog::debug("Submap {}: found {} loop closure candidates", i,
                    submaps[i].loop_closure_candidates.size());

      // Visualize the edge between submaps
      if constexpr (VISUALIZE) {
        if (ReUseX::Visualizer::isInitialised()) {
          auto viewer = ReUseX::Visualizer::getInstance()
                            ->getViewer<pcl::visualization::PCLVisualizer>();
          viewer->removeAllPointClouds();
          viewer->spinOnce(1);
        }
      }

      ++logger;
    }

    if constexpr (VISUALIZE) {
      if (ReUseX::Visualizer::isInitialised()) {
        auto viewer = ReUseX::Visualizer::getInstance()
                          ->getViewer<pcl::visualization::PCLVisualizer>();
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->spinOnce(100);
      }
    }
    report.addLine(logger.GetLogger()->name(), sw.elapsed().count());
  }

  if (false) {
    auto logger =
        spdmon::LoggerProgress("Add Submaps to Optimizer", submaps.size());
    spdlog::stopwatch sw;

    for (size_t i = 0; i < submaps.size(); ++i) {

      const auto &submap = submaps[i];
      if (submap.indices.empty()) {
        spdlog::warn("Submap {} is empty, skipping", i);
        ++logger;
        continue;
      }

      for (const auto &pair : submap.loop_closure_candidates) {
        spdlog::trace("Processing loop closure candidate {} -> {}", i,
                      pair.first);
        const size_t j = pair.first;
        const float match_score = pair.second;

        // INFO: Match correspondences firts maybe and ICP
        // INFO: estimate correspondences
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33,
                                                    pcl::FPFHSignature33>
            est;
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
        est.setInputSource(submaps[i].features);
        est.setInputTarget(submaps[j].features);
        est.determineCorrespondences(*correspondences);

        // INFO: Duplication rejection Duplicate
        pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(
            new pcl::Correspondences());
        pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
        corr_rej_one_to_one.setInputCorrespondences(correspondences);
        corr_rej_one_to_one.getCorrespondences(
            *correspondences_result_rej_one_to_one);

        // INFO: Correspondance rejection RANSAC
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>
            rejector_sac;
        pcl::CorrespondencesPtr correspondences_filtered(
            new pcl::Correspondences());
        rejector_sac.setInputSource(submaps[i].keypoints);
        rejector_sac.setInputTarget(submaps[j].keypoints);
        rejector_sac.setInlierThreshold(
            2.5); // distance in m, not the squared distance
        rejector_sac.setMaximumIterations(1000000);
        rejector_sac.setRefineModel(false);
        rejector_sac.setInputCorrespondences(
            correspondences_result_rej_one_to_one);

        rejector_sac.getCorrespondences(*correspondences_filtered);
        correspondences.swap(correspondences_filtered);

        spdlog::debug("Submap {} -> {}: found {} correspondences after "
                      "rejection vs. {} ",
                      i, j, correspondences_filtered->size(),
                      correspondences->size());

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // INFO: Transformation Estimation method 1
        // transform = rejector_sac.getBestTransformation(); //

        // INFO: Transformation Estimation method 2
        pcl::registration::TransformationEstimationSVD<PointT, PointT>
            transformation_estimation;
        transformation_estimation.estimateRigidTransformation(
            *submaps[i].keypoints, *submaps[j].keypoints, *correspondences,
            transform);

        // Fine alignment using ICP
        ICP reg;
        reg.setMaximumIterations(50);
        reg.setMaxCorrespondenceDistance(0.1);
        reg.setTransformationEpsilon(1e-8);
        reg.setEuclideanFitnessEpsilon(1e-8);
        reg.setRANSACIterations(1000);
        reg.setRANSACOutlierRejectionThreshold(0.02);

        // Set point representation
        const float alpha[4] = {1, 1, 1, 0.1};
        custom_mapping point_representation{};
        auto point_representation_ptr =
            pcl::make_shared<const custom_mapping>(point_representation);
        reg.setPointRepresentation(point_representation_ptr);

        spdlog::trace("ICP registration {} -> {}", i, j);

        CloudPtr dummy(new Cloud());
        CloudPtr src(new Cloud());
        CloudPtr tgt(new Cloud());

        auto gf = ReUseX::Filters::GridFilter<PointT>(0.05);
        gf->setInputCloud(submap.map);
        gf->filter(*src);
        gf->setInputCloud(submaps[j].map);
        gf->filter(*tgt);

        reg.setInputSource(src);
        reg.setInputTarget(tgt);

        reg.registerVisualizationCallback(reg_visualization_callback<PointT>);
        reg.align(*dummy /*, transform*/);

        if (!reg.hasConverged()) {
          spdlog::warn("ICP failed to converge between submaps {} and {}", i,
                       j);
          continue;
        }

        if (reg.getFinalTransformation().block<3, 1>(0, 3).norm() > 2.) {
          spdlog::warn(
              "ICP translation between submaps {} and {} is too large: "
              "{}",
              i, j, reg.getFinalTransformation().block<3, 1>(0, 3).norm());
          continue;
        }

        transform = reg.getFinalTransformation();

        auto score = reg.getFitnessScore();

        // TODO: Add edges for loop closure candidates
        g2o::EdgeSE3 *e = new g2o::EdgeSE3();
        e->setVertex(0, optimizer->vertex(j)); // j > tgt
        e->setVertex(1, optimizer->vertex(i)); // i > src
        e->setMeasurement(getMeasurment(poses[submaps[j].indices[0]],
                                        poses[submap.indices[0]], transform));
        e->setInformation(information_matrix.cast<double>() * score);

        e->setRobustKernel(
            g2o::RobustKernelFactory::instance()->construct(kernelName));

        optimizer->addEdge(e);
      }

      ++logger;
    }
    report.addLine(logger.GetLogger()->name(), sw.elapsed().count());
  }

  spdlog::stopwatch sw;
  optimizer->initializeOptimization();
  optimizer->optimize(maxIterations);
  spdlog::info("Graph optimization completed in {}s", sw);

  {
    auto logger = spdmon::LoggerProgress("Update Submaps", submaps.size());
    spdlog::stopwatch sw;
    for (size_t i = 0; i < submaps.size(); ++i) {
      const auto &submap = submaps[i];
      if (submap.indices.empty()) {
        spdlog::warn("Submap {} is empty, skipping", i);
        ++logger;
        continue;
      }

      // Update the pose of the submap
      g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(optimizer->vertex(i));

      auto old_pose = poses[submap.indices[0]];
      auto new_pose = Eigen::Affine3f(v->estimate().matrix().cast<float>());

      auto delta = old_pose.inverse() * new_pose;

      pcl::transformPointCloud(*submap.map, *submap.map, delta);
      pcl::transformPointCloud(*submap.keypoints, *submap.keypoints, delta);
      ++logger;
    }
    report.addLine(logger.GetLogger()->name(), sw.elapsed().count());
  }

  if constexpr (VISUALIZE) {
    spdlog::trace("Visualize submaps");

    if (ReUseX::Visualizer::isInitialised()) {

      // INFO: Creat new paht for visualization
      pcl::PolygonMesh::Ptr mesh_path_optimised(new pcl::PolygonMesh());
      pcl::PointCloud<pcl::PointXYZ>::Ptr path_cloud_optimised(
          new pcl::PointCloud<pcl::PointXYZ>());

      mesh_path_optimised->polygons.push_back(pcl::Vertices());

      for (size_t i = 0; i < optimizer->vertices().size(); ++i) {
        const auto v = static_cast<g2o::VertexSE3 *>(optimizer->vertex(i));
        const auto pose = Eigen::Affine3f(v->estimate().matrix().cast<float>());

        // INFO: Add intermeidiate poses to the path
        auto relative_xfrom = poses[submaps[i].indices[0]].inverse() * pose;

        for (size_t j = 0; j < submaps[i].indices.size(); ++j) {
          size_t idx = submaps[i].indices[j];
          auto pose_j = poses[idx];

          pose_j = relative_xfrom * pose_j;

          pcl::PointXYZ p_j(pose_j.translation().x(), pose_j.translation().y(),
                            pose_j.translation().z());
          path_cloud_optimised->points.push_back(p_j);

          // Add the edge to the mesh
          mesh_path_optimised->polygons.back().vertices.push_back(
              path_cloud_optimised->points.size() - 1);
        }
      }

      auto viewer = ReUseX::Visualizer::getInstance()
                        ->getViewer<pcl::visualization::PCLVisualizer>();

      viewer->removeAllPointClouds();

      // INFO: Visualize the paths
      pcl::toPCLPointCloud2(*path_cloud, mesh->cloud);
      viewer->addPolylineFromPolygonMesh(*mesh, "path");
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "path");
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "path");

      // INFO: Visualize the new path
      pcl::toPCLPointCloud2(*path_cloud_optimised, mesh_path_optimised->cloud);
      viewer->addPolylineFromPolygonMesh(*mesh_path_optimised,
                                         "path_optimised");
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "path_optimised");
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "path_optimised");

      for (size_t i = 0; i < submaps.size(); ++i) {
        CloudPtr map = submaps[i].map;
        CloudPtr keypoints = submaps[i].keypoints;

        const pcl::RGB c = pcl::GlasbeyLUT::at(i);

        viewer->addPointCloud<PointT>(map, RGBHandler(map),
                                      fmt::format("submap_cloud_{}", i));
        viewer->addPointCloud<PointT>(keypoints,
                                      CustomColorHander(keypoints, 184, 16, 33),
                                      fmt::format("submap_keypoints_{}", i));

        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
            fmt::format("submap_keypoints_{}", i));
      }
    }

    ReUseX::Visualizer::getInstance()->wait();
  }

  report.print();

  return path;
}
