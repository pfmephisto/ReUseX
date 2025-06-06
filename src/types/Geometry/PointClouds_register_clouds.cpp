#define PCL_NO_PRECOMPILE
#include "PointCloud.hh"
#include "PointClouds.hh"
#include "core/spdmon.hh"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <fmt/color.h>
#include <fmt/printf.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/io/file_io.h>
#include <pcl/memory.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation<PointT> {
  using pcl::PointRepresentation<PointT>::nr_dimensions_;

public:
  MyPointRepresentation() {
    // Define the number of dimensions
    nr_dimensions_ = 3;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray(const PointT &p, float *out) const {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    // out[3] = p.curvature;
  }
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */
void pairAlign(ReUseX::PointCloud::Cloud::Ptr cloud_src,
               const ReUseX::PointCloud::Cloud::Ptr cloud_tgt,
               Eigen::Matrix4f &pairTransform,
               std::optional<float> grid_size = {},
               std::optional<int32_t> confidence_filter = {}) {
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets

  ReUseX::PointCloud::Cloud::Ptr input_src(new ReUseX::PointCloud::Cloud);
  ReUseX::PointCloud::Cloud::Ptr input_tgt(new ReUseX::PointCloud::Cloud);

  input_src = cloud_src;
  input_tgt = cloud_tgt;

  ReUseX::PointCloud::Cloud::Ptr src(new ReUseX::PointCloud::Cloud);
  ReUseX::PointCloud::Cloud::Ptr tgt(new ReUseX::PointCloud::Cloud);

  if (confidence_filter.has_value()) {
    pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>());
    range_cond->addComparison(
        pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "label", pcl::ComparisonOps::EQ, confidence_filter.value())));

    // build the filter
    pcl::ConditionalRemoval<PointT> condrem;
    condrem.setCondition(range_cond);
    condrem.setKeepOrganized(false);

    condrem.setInputCloud(input_src);
    condrem.filter(*src);

    condrem.setInputCloud(input_src);
    condrem.filter(*tgt);

    input_src = src;
    input_tgt = tgt;
  }

  if (grid_size.has_value()) {
    auto size = grid_size.value();
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(size, size, size);
    grid.setInputCloud(input_src);
    grid.filter(*src);

    grid.setInputCloud(input_tgt);
    grid.filter(*tgt);
  }

  // Compute point features
  // https://github.com/evil0sheep/pcl-slam/blob/master/src/pcl_slam.cpp

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;

  // ... and weight the 'curvature' dimension so that it is balanced against x,
  // y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues(alpha);

  //
  // Align
  pcl::IterativeClosestPoint<PointT, PointT> reg; // NonLinear
  reg.setTransformationEpsilon(1e-8);

  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(0.05);

  // Set the point representation
  reg.setPointRepresentation(
      pcl::make_shared<const MyPointRepresentation>(point_representation));

  reg.setInputSource(src);
  reg.setInputTarget(tgt);

  ReUseX::PointCloud::Cloud::Ptr reg_result = src;
  reg.setMaximumIterations(50);
  reg.setRANSACIterations(1000);
  reg.setRANSACOutlierRejectionThreshold(0.05);
  reg.align(*reg_result);
  if (reg.hasConverged()) {
    pairTransform = reg.getFinalTransformation().inverse();
    pcl::transformPointCloudWithNormals(*cloud_src, *cloud_src, pairTransform);
  }
}

namespace ReUseX {

template <class T> PointClouds<T> PointClouds<T>::register_clouds() {

  // Calculate Registartion
  ///////////////////////////////////////////////////////////////////////////////
  auto transforms =
      std::vector<Eigen::Matrix4f>(data.size(), Eigen::Matrix4f::Identity());

  std::shared_ptr<spdmon::LoggerProgress> monitor;
  monitor =
      std::make_shared<spdmon::LoggerProgress>("Registration", data.size());

  ++(*monitor);

  spdlog::stopwatch sw;
#pragma omp parallel for shared(transforms)
  for (std::size_t i = 1; i < data.size(); ++i) {

    PointCloud::Cloud::Ptr target; // = load(files[i-1]);
    PointCloud::Cloud::Ptr source; // = load(files[i]);

    // If the underlying type is a string, load the file
    if constexpr (std::is_same<T, std::string>::value) {
      target = PointCloud::load(data[i - 1]);
      source = PointCloud::load(data[i]);
      // Oterwise it is assumed to be a PointCloud::Ptr
    } else {
      target = data[i - 1];
      source = data[i];
    }

    Eigen::Matrix4f pairTransform;

    // ICP
    pairAlign(source, target, pairTransform, 0.1f, 2U);
    transforms[i] = pairTransform;

    ++(*monitor);
  }

  // Compute compound transformations
  ///////////////////////////////////////////////////////////////////////////////
  monitor.reset();
  monitor = std::make_shared<spdmon::LoggerProgress>(
      "Compute compound transformations", data.size());

  auto compund_transforms =
      std::vector<Eigen::Matrix4f>(data.size(), Eigen::Matrix4f::Identity());
  for (std::size_t i = 1; i < data.size(); ++i) {
    compund_transforms[i] = compund_transforms[i - 1] * transforms[i];
  }

  // Update Position
  ///////////////////////////////////////////////////////////////////////////////
  monitor.reset();
  monitor =
      std::make_shared<spdmon::LoggerProgress>("Update position", data.size());
  ++(*monitor);

#pragma omp parallel for shared(data)
  for (std::size_t i = 1; i < data.size(); ++i) {

    // If the underlying type is a string, load the file
    if constexpr (std::is_same<T, std::string>::value) {
      PointCloud point_cloud = PointCloud::load(data[i]);
      pcl::transformPointCloudWithNormals(*point_cloud, *point_cloud,
                                          compund_transforms[i]);
      point_cloud.save(data[i], true);
      // Oterwise it is assumed to be a PointCloud::Ptr
    } else {
      pcl::transformPointCloudWithNormals(*data[i], *data[i],
                                          compund_transforms[i]);
    }

    ++(*monitor);
  }

  spdlog::info("Registration took {}", sw);

  return *this;
}

template PointCloudsInMemory PointCloudsInMemory::register_clouds();
template PointCloudsOnDisk PointCloudsOnDisk::register_clouds();

} // namespace ReUseX
