#pragma once

#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <spdlog/spdlog.h>

namespace ReUseX {
template <typename PointT> class SIFTFilter : public pcl::Filter<PointT> {
private:
  using PointTInt = std::conditional_t<pcl::traits::has_intensity_v<PointT>,
                                       PointT, pcl::PointXYZI>;

  using Cloud = pcl::PointCloud<PointTInt>;
  using CloudPtr = typename Cloud::Ptr;

  pcl::SIFTKeypoint<PointTInt, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> keypoints;

protected:
  using pcl::Filter<PointT>::filter_name_;
  using pcl::Filter<PointT>::getClassName;
  using pcl::Filter<PointT>::input_;
  using pcl::Filter<PointT>::indices_;

  using PointCloud = typename pcl::Filter<PointT>::PointCloud;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  using Ptr = std::shared_ptr<SIFTFilter<PointT>>;
  using CostPtr = std::shared_ptr<const SIFTFilter<PointT>>;

  SIFTFilter() {
    filter_name_ = "SIFT Filter";

    // Parameters
    // Min scale, number of octaves, scale per octave
    sift.setScales(0.01f, 3, 2);
    sift.setMinimumContrast(0.001f);
  };
  ~SIFTFilter() override = default;

protected:
  void applyFilter(PointCloud &output) override {

    if constexpr (!pcl::traits::has_intensity_v<PointT>) {
      CloudPtr temp(new Cloud);
      temp->resize(output.size());
      pcl::copyPointCloud(output, *temp);
      sift.setInputCloud(temp);
    } else {
      sift.setInputCloud(input_);
    }

    sift.compute(keypoints);
    pcl::KdTreeFLANN<PointT> kdtree{};
    kdtree.setInputCloud(input_);
    if (keypoints.size() == 0)
      spdlog::warn("no points");
    indices_->resize(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); ++i) {
      /*pcl::PointWithScale*/ auto kp = keypoints[i];
      PointT p = PointT();
      p.x = kp.x;
      p.y = kp.y;
      p.z = kp.z;
      std::vector<int> pointIdxKNNSearch(1);
      std::vector<float> pointKNNSquaredDistance(1);
      kdtree.nearestKSearchT(p, 1, pointIdxKNNSearch, pointKNNSquaredDistance);
      (*indices_)[i] = pointIdxKNNSearch[0];
    }

    pcl::ExtractIndices<PointT> ex(false);
    ex.setInputCloud(input_);
    ex.setIndices(indices_);
    ex.filter(output);
  };
};
} // namespace ReUseX
