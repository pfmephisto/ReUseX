#define PCL_NO_PRECOMPILE // This needs to be called before any pcl import

#include "icp.hh"
// #include "functions/spdmon.hh"
#include "types/Filters.hh"
#include "types/Geometry/PointCloud.hh"
#include "visualizer/visualizer.hh"

#include <spdlog/spdlog.h>

#include <pcl/common/impl/transforms.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/memory.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fmt/color.h>
#include <fmt/printf.h>

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

////////////////////////////////////////////////////////////////////////////////
#define icp_source "icp_source"
#define icp_target "icp_target"
#define icp_source_filtered "icp_source_filtered"
#define icp_target_filtered "icp_target_filtered"
template <typename PointT>
Eigen::Matrix4f icp(const typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
                    const typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
                    std::vector<typename pcl::Filter<PointT>::Ptr> filters,
                    const double maxCorrespondence) {

  using Ptr = typename pcl::PointCloud<PointT>::Ptr;
  using Color = pcl::visualization::PointCloudColorHandlerCustom<PointT>;

  spdlog::info("Running ICP");
  spdlog::debug("maxCorrespondence = {}", maxCorrespondence);

  Ptr input_src(new pcl::PointCloud<PointT>);
  Ptr input_tgt(new pcl::PointCloud<PointT>);

  input_src = cloud_src->makeShared(); // Makes a copy
  input_tgt = cloud_tgt->makeShared(); // Makes a copy

  // Add point clouds to viewer for visualization
  if (Visualizer::Visualizer::isInitialised()) {
    auto viewer = Visualizer::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();

    Color blue(cloud_src, 0, 0, 255);
    Color red(cloud_tgt, 255, 0, 0);
    viewer->addPointCloud<PointT>(cloud_src, blue, icp_source);
    viewer->addPointCloud<PointT>(cloud_tgt, red, icp_target);

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, icp_source);
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, icp_target);

    viewer->spinOnce(100);
  }

  // Temporary grid filters
  auto vg =
      std::shared_ptr<pcl::VoxelGrid<PointT>>(new pcl::VoxelGrid<PointT>());
  vg->setLeafSize(0.1, 0.1, 0.1);
  filters.clear();
  // filters.push_back(vg);
  // filters.push_back(ReUseX::Filters::HighConfidenceFilter<PointT>());
  filters.push_back(ReUseX::Filters::SIVFeatures<PointT>());

  // Apply filters
  for (auto filter : filters) {
    filter->setInputCloud(input_src);
    filter->filter(*input_src);

    filter->setInputCloud(input_tgt);
    filter->filter(*input_tgt);
  }

  //// Defining a rotation matrix and translation vector
  // Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

  //// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  // double theta = M_PI / 8; // The angle of rotation in radians
  // transformation_matrix(0, 0) = std::cos(theta);
  // transformation_matrix(0, 1) = -sin(theta);
  // transformation_matrix(1, 0) = sin(theta);
  // transformation_matrix(1, 1) = std::cos(theta);

  //// A translation on Z axis (0.4 meters)
  // transformation_matrix(2, 3) = 0.4;

  // input_src = input_tgt->makeShared(); // Makes a copy
  // pcl::transformPointCloud(*input_src, *input_src, transformation_matrix);

  // Add point clouds to viewer for visualization
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

  if (input_src->empty() || input_tgt->empty()) {
    fmt::print(fmt::fg(fmt::color::red),
               "Pair alignment failed: Empty point cloud!\n");
    return Eigen::Matrix4f::Identity();
  }

  // Compute point features
  // https://github.com/evil0sheep/pcl-slam/blob/master/src/pcl_slam.cpp

  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation<PointT> point_representation;

  // ... and weight the 'curvature' dimension so that it is balanced against x,
  // y, and z
  float alpha[7] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  point_representation.setRescaleValues(alpha);

  // Align
  using RegType =
      std::conditional_t<pcl::traits::has_normal_v<PointT>,
                         pcl::IterativeClosestPointWithNormals<PointT, PointT>,
                         pcl::GeneralizedIterativeClosestPoint<PointT, PointT>>;
  // pcl::IterativeClosestPointNonLinear<PointT, PointT>>;
  // pcl::IterativeClosestPoint<PointT, PointT>>; // NonLinear

  RegType reg;

  reg.setInputSource(input_src);
  reg.setInputTarget(input_tgt);

  reg.setTransformationEpsilon(1e-8);
  reg.setEuclideanFitnessEpsilon(1e-8);
  reg.setMaximumIterations(50);
  reg.setRANSACOutlierRejectionThreshold(0.05);
  reg.setRANSACIterations(1000);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(maxCorrespondence);

  // Set the point representation
  reg.setPointRepresentation(
      pcl::make_shared<const MyPointRepresentation<PointT>>(
          point_representation));

  Ptr dummy =
      std::shared_ptr<pcl::PointCloud<PointT>>(new pcl::PointCloud<PointT>());
  // Add point clouds to viewer for visualization
  if (Visualizer::Visualizer::isInitialised()) {
    auto viewer = Visualizer::Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();

    Eigen::Matrix4f prev;

    reg.setMaximumIterations(1);
    for (size_t i = 0; i < 80; ++i) {

      // Update the input to be able to take another step
      reg.setInputSource(input_src);
      reg.align(*dummy);

      pcl::transformPointCloud(*input_src, *input_src,
                               reg.getFinalTransformation().inverse());

      // Color green(input_src, 0, 255, 0);
      // viewer->removePointCloud(icp_source_filtered);
      // viewer->addPointCloud<PointT>(input_src, green, icp_source_filtered);
      // viewer->setPointCloudRenderingProperties(
      //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
      //     icp_source_filtered);

      viewer->spinOnce(100);

      // if the difference between this transformation and the previous one
      // is smaller than the threshold, refine the process by reducing
      // the maximal correspondence distance
      if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) <
          reg.getTransformationEpsilon()) {
        reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() -
                                         0.001);
        spdlog::debug("Updating max Correspondence Distance = {:.3f}",
                      reg.getMaxCorrespondenceDistance());
      }
      prev = reg.getLastIncrementalTransformation();
    }

  } else {
    // Automate registration
    reg.align(*dummy); // This does the actual computation
  }

  Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity();

  if (reg.hasConverged()) {
    pairTransform = reg.getFinalTransformation().inverse();
    // spdlog::debug("{} -> {} Fiiness score: {} after {} iterations",
    //               input_src->header.frame_id, input_tgt->header.frame_id,
    //               reg.getFitnessScore(), reg.nr_iterations_);
  }

  else
    spdlog::warn("Unable has not converge");
  //     fmt::print(fmt::fg(fmt::color::red), "Pair alignment failed to converge
  //     for {} & {}!\n", input_src->header.frame_id,
  //     input_tgt->header.frame_id);

  // if (reg.hasConverged())
  //     fmt::print(fmt::fg(fmt::color::green), "Pair {} & {} converged with
  //     score: {} after {} iterations\n", input_src->header.frame_id,
  //     input_tgt->header.frame_id, reg.getFitnessScore(), reg.nr_iterations_);

  // Clean up viewer-
  if (Visualizer::Visualizer::isInitialised()) {
    Visualizer::getInstance()->wait();
    auto viewer = Visualizer::getInstance()
                      ->getViewer<pcl::visualization::PCLVisualizer>();
    viewer->removePointCloud(icp_source);
    viewer->removePointCloud(icp_target);
    viewer->removePointCloud(icp_source_filtered);
    viewer->removePointCloud(icp_target_filtered);
  }

  return pairTransform;
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

  return icp<PointT>(cloud_src, cloud_tgt, filters);
}

template Eigen::Matrix4f
pair_align<PointT>(const typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
                   const typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
                   std::optional<float> grid_size);

template Eigen::Matrix4f icp<pcl::PointXYZRGBA>(
    const typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_src,
    const typename pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_tgt,
    std::vector<typename pcl::Filter<pcl::PointXYZRGBA>::Ptr> filters,
    const double maxCorrespondence);

} // namespace ReUseX
