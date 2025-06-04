#define PCL_NO_PRECOMPILE // This needs to be called before any pcl import

#include "icp.hh"
// #include "spdmon.hh"
#include "fmt_formatter.hh"
#include "types/Filters.hh"
#include "types/Geometry/PointCloud.hh"
#include "visualizer/visualizer.hh"

#include <spdlog/spdlog.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/memory.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <eigen3/Eigen/Core>

#include <fmt/color.h>
#include <fmt/printf.h>

#define VISUALIZE 0

using namespace std::chrono_literals;

namespace ReUseX {

// Define a new point representation for < x, y, z, curvature >
template <typename PointT>
class MyPointRepresentation : public pcl::PointRepresentation<PointT> {
  using pcl::PointRepresentation<PointT>::nr_dimensions_;

    public:
  MyPointRepresentation() {
    // Define the number of dimensions
    nr_dimensions_ = 0;

    if constexpr (pcl::traits::has_xyz_v<PointT>)
      nr_dimensions_ += 3;

    // if constexpr (pcl::traits::has_curvature_v<PointT>)
    //     nr_dimensions_ += 1;

    if constexpr (pcl::traits::has_color_v<PointT>)
      nr_dimensions_ += 3;
  };

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray(const PointT &p, float *out) const {
    // < x, y, z, curvature, r, g, b >

    size_t idx = 0;
    if constexpr (pcl::traits::has_xyz_v<PointT>) {
      out[idx] = p.x;
      out[idx + 1] = p.y;
      out[idx + 2] = p.z;
      idx += 3;
    }

    // if constexpr (pcl::traits::has_curvature_v<PointT>){
    //     out[idx] = p.curvature; // Seems not to be always set and in that
    //     case returns Inf and results in a failure in the KDTree idx++;
    // }

    if constexpr (pcl::traits::has_color_v<PointT>) {
      out[idx] = static_cast<float>(p.r / 255.0);
      out[idx + 1] = static_cast<float>(p.g / 255.0);
      out[idx + 2] = static_cast<float>(p.b / 255.0);
      idx += 3;
    }
  }
};

bool step_view = false;
std::function<void(const pcl::visualization::KeyboardEvent &)>
    keyboardEvnetCallback(
        [&step_view](const pcl::visualization::KeyboardEvent &event) {
          if (event.keyUp()) {
            switch (event.getKeyCode()) {
            case 'n':
              // case 110:
              step_view = true;
              break;
            }
          }
        });

////////////////////////////////////////////////////////////////////////////////
#define icp_source "icp_source"
#define icp_target "icp_target"
#define icp_source_filtered "icp_source_filtered"
#define icp_target_filtered "icp_target_filtered"
template <typename PointT>
std::tuple<Eigen::Matrix4f, double>
icp(const typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
    const typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
    const std::vector<typename pcl::Filter<PointT>::Ptr> &filters,
    const double maxCorrespondence) {

  using Ptr = typename pcl::PointCloud<PointT>::Ptr;
  using Color = pcl::visualization::PointCloudColorHandlerCustom<PointT>;
  using RegType = std::conditional_t<
      pcl::traits::has_normal_v<PointT>,
      pcl::IterativeClosestPointWithNormals<PointT, PointT>,
      // pcl::GeneralizedIterativeClosestPoint<PointT, PointT>>;
      // pcl::IterativeClosestPointNonLinear<PointT, PointT>>;
      pcl::IterativeClosestPoint<PointT, PointT>>; // NonLinear

  spdlog::info("Running ICP");
  spdlog::debug("Number of Filters = {}", filters.size());
  spdlog::debug("maxCorrespondence = {:.6f}", maxCorrespondence);

  Ptr input_src(new pcl::PointCloud<PointT>);
  Ptr input_tgt(new pcl::PointCloud<PointT>);

  input_src = cloud_src->makeShared(); // Makes a copy
  input_tgt = cloud_tgt->makeShared(); // Makes a copy

#if VISUALIZE
  // Orient the point clouds for ICP the same way as theose in the viewer
  Eigen::Affine3f pose_source, pose_target;
  pose_source = Eigen::Affine3f::Identity();
  pose_source.translation().template head<3>() =
      input_src->sensor_origin_.template head<3>();
  pose_source.linear() = input_src->sensor_orientation_.toRotationMatrix();

  pose_target = Eigen::Affine3f::Identity();
  pose_target.translation().template head<3>() =
      input_tgt->sensor_origin_.template head<3>();
  pose_target.linear() = input_tgt->sensor_orientation_.toRotationMatrix();

  pcl::transformPointCloud(*input_src, *input_src, pose_source);
  pcl::transformPointCloud(*input_tgt, *input_tgt, pose_target);
#endif

  input_src->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 1);
  input_src->sensor_orientation_ = Eigen::Quaternionf::Identity();
  input_tgt->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 1);
  input_tgt->sensor_orientation_ = Eigen::Quaternionf::Identity();

#if VISUALIZE
  // Add point clouds to viewer for visualization
  if (Visualizer::Visualizer::isInitialised()) {
    pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
    auto viewer = Visualizer::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();

    Color blue(cloud_src, 0, 0, 255);
    Color red(cloud_tgt, 128, 0, 0);
    viewer->addPointCloud<PointT>(cloud_src, blue, icp_source);
    viewer->addPointCloud<PointT>(cloud_tgt, red, icp_target);

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, icp_source);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, icp_target);

    viewer->spinOnce(100);
  }
#endif

  // Apply filters
  spdlog::trace("Apply filters");
  for (auto filter : filters) {
    filter->setInputCloud(input_src);
    filter->filter(*input_src);

    filter->setInputCloud(input_tgt);
    filter->filter(*input_tgt);
  }

  spdlog::debug("Number of point in cloud: {} & {}", input_src->points.size(),
                input_tgt->points.size());

#if VISUALIZE
  // Add point clouds to viewer for visualization
  spdlog::trace("Visualize Intial state");
  if (Visualizer::Visualizer::isInitialised()) {
    auto viewer = Visualizer::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();

    Color green(input_src, 0, 255, 0);
    Color red(input_tgt, 255, 0, 0);

    viewer->addPointCloud<PointT>(input_src, green, icp_source_filtered);
    viewer->addPointCloud<PointT>(input_tgt, red, icp_target_filtered);

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, icp_source_filtered);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, icp_target_filtered);

    viewer->resetCamera();
    viewer->spinOnce(100);
  }
#endif

  if (input_src->empty() || input_tgt->empty()) {
    fmt::print(fmt::fg(fmt::color::red),
               "Pair alignment failed: Empty point cloud!\n");
    return std::make_tuple<Eigen::Matrix4f, double>(Eigen::Matrix4f::Identity(),
                                                    -1.0);
  }

  // Compute point features
  // https://github.com/evil0sheep/pcl-slam/blob/master/src/pcl_slam.cpp

  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation<PointT> point_representation;

  // ... and weight the 'curvature' dimension so that it is balanced against x,
  // y, and z
  float alpha[7] = {1.0, 1.0, 1.0, 5, 5, 5, 0.0};

  point_representation.setRescaleValues(alpha);

  // Align
  RegType reg;
  // Set the point representation
  reg.setPointRepresentation(
      pcl::make_shared<const MyPointRepresentation<PointT>>(
          point_representation));

  reg.setTransformationEpsilon(1e-8);
  reg.setEuclideanFitnessEpsilon(1e-8);
  reg.setMaximumIterations(50);
  // typename pcl::registration::
  //     TransformationEstimationPointToPlaneLLS<PointT, PointT>::Ptr trans_lls(
  //         new pcl::registration::TransformationEstimationPointToPlaneLLS<
  //             PointT, PointT>);
  // typename pcl::registration::TransformationEstimationSVD<PointT,
  // PointT>::Ptr
  //     trans_svd(
  //         new pcl::registration::TransformationEstimationSVD<PointT,
  //         PointT>);
  // reg.setTransformationEstimation(trans_svd);
  // reg.setRANSACIterations(1000);
  // reg.setRANSACOutlierRejectionThreshold(0.02);
  //  Set the maximum distance between two correspondences (src<->tgt) to 10cm
  //  Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(maxCorrespondence);

  reg.setInputSource(input_src);
  reg.setInputTarget(input_tgt);

  std::shared_ptr<pcl::PointCloud<PointT>> dummy(new pcl::PointCloud<PointT>());

#if VISUALIZE
  // Add point clouds to viewer for visualization
  if (Visualizer::Visualizer::isInitialised()) {
    auto viewer = Visualizer::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();

    spdlog::debug("Add Keyboard Even Keybard Callback");
    viewer->registerKeyboardCallback(keyboardEvnetCallback);

    // Construct the callback function to visualize the registration process
    std::function<typename pcl::Registration<
        PointT, PointT>::UpdateVisualizerCallbackSignature>
    callback([viewer, &step_view](const pcl::PointCloud<PointT> &c1,
                                  const pcl::Indices &idxs1,
                                  const pcl::PointCloud<PointT> &c2,
                                  const pcl::Indices &idxs2) -> void {
      // turn idxs into one correspondence vector
      pcl::Correspondences correspondences;
      for (size_t i = 0; i < idxs1.size(); ++i) {
        correspondences.push_back(pcl::Correspondence(idxs1[i], idxs2[i], 1));
      }

      // viewer needs ptrs, not values (expensiveish)
      const typename pcl::PointCloud<PointT>::Ptr input_src = c1.makeShared();
      const typename pcl::PointCloud<PointT>::Ptr input_tgt = c2.makeShared();

      Color green(input_src, 0, 255, 0);
      viewer->removePointCloud(icp_source_filtered);
      viewer->addPointCloud<PointT>(input_src, green, icp_source_filtered);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
          icp_source_filtered);

      Color red(input_tgt, 255, 0, 0);
      viewer->removePointCloud(icp_target_filtered);
      viewer->addPointCloud<PointT>(input_tgt, red, icp_target_filtered);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
          icp_target_filtered);

      // many lines missing
      if (!viewer->updateCorrespondences<PointT>(input_src, input_tgt,
                                                 correspondences)) {
        viewer->addCorrespondences<PointT>(input_src, input_tgt,
                                           correspondences, "correspondences");
      }
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "correspondences");

      // Step through optimisation
      if (!viewer->wasStopped() && !step_view &&
          !Visualizer::getInstance()->skip())
        spdlog::debug("Waiting for next stepn. To step press 'n'");

      while (!viewer->wasStopped() && !step_view && !Visualizer::skip()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
      }
      step_view = false;
    });

    // Register the callback
    reg.registerVisualizationCallback(callback);

    // Registration loop
    reg.align(*dummy);

  } else {
#endif
    // Automate registration
    reg.align(*dummy); // This does the actual computation
#if VISUALIZE
  }
#endif

  Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity();

  if (reg.hasConverged()) {
    pairTransform = reg.getFinalTransformation().inverse();
    // spdlog::debug("{} -> {} Fiiness score: {} after {}
    //  iterations",
    //                input_src->header.frame_id, input_tgt->header.frame_id,
    //                reg.getFitnessScore(), reg.nr_iterations_);
  }

  else
    spdlog::warn("Unable has not converge");
  //     fmt::print(fmt::fg(fmt::color::red), "Pair alignment failed to
  //     converge for {} & {}!\n", input_src->header.frame_id,
  //     input_tgt->header.frame_id);

  // if (reg.hasConverged())
  //     fmt::print(fmt::fg(fmt::color::green), "Pair {} & {} converged with
  //     score: {} after {} iterations\n", input_src->header.frame_id,
  //     input_tgt->header.frame_id, reg.getFitnessScore(),
  //     reg.nr_iterations_);

#if VISUALIZE
  // Clean up viewer-
  if (Visualizer::Visualizer::isInitialised()) {
    // Get the viewer
    auto viewer = Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();

    // Set the addCoordinateSystem
    viewer->addCoordinateSystem(1.5, Eigen::Affine3f(pairTransform), "xform");

    // Wait for user to view the result
    spdlog::debug("Before waiting for user to view the result");
    Visualizer::getInstance()->wait();

    // Remove the point clouds
    viewer->removePointCloud(icp_source);
    viewer->removePointCloud(icp_target);
    viewer->removePointCloud(icp_source_filtered);
    viewer->removePointCloud(icp_target_filtered);
  }
#endif

  return std::make_tuple(pairTransform, reg.getFitnessScore());
  // reg.getConvergeCriteria()->getRelativeMSE();
}

template <typename PointT>
Eigen::Matrix4f
pair_align(const typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
           const typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
           std::optional<float> grid_size) {

  std::vector<typename pcl::Filter<PointT>::Ptr> filters;

  if constexpr (pcl::traits::has_color_v<PointT>)
    filters.push_back(Filters::HighConfidenceFilter<PointT>());

  if (grid_size.has_value())
    filters.push_back(Filters::GridFilter<PointT>(grid_size.value()));

  auto [xform, fitness_score] = icp<PointT>(cloud_src, cloud_tgt, filters);

  return xform;
}

template Eigen::Matrix4f
pair_align<PointT>(const typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
                   const typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
                   std::optional<float> grid_size);

template std::tuple<Eigen::Matrix4f, double> icp<pcl::PointXYZRGBA>(
    const typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_src,
    const typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_tgt,
    const std::vector<typename pcl::Filter<pcl::PointXYZRGBA>::Ptr> &filters,
    const double maxCorrespondence);

template std::tuple<Eigen::Matrix4f, double> icp<PointXYZRGBANormal>(
    const typename pcl::PointCloud<PointXYZRGBANormal>::ConstPtr cloud_src,
    const typename pcl::PointCloud<PointXYZRGBANormal>::ConstPtr cloud_tgt,
    const std::vector<typename pcl::Filter<PointXYZRGBANormal>::Ptr> &filters,
    const double maxCorrespondence);
} // namespace ReUseX
