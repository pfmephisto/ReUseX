#pragma once
#include <memory>
#define PCL_NO_PRECOMPILE

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

namespace ReUseX {

constexpr static uint32_t RGBA_ALPHA_UPPER_BOUND =
    static_cast<uint32_t>(255 << 24 | 255 << 16 | 255 << 8 | 255);
constexpr static uint32_t RGBA_ALPHA_LOWER_BOUND =
    static_cast<uint32_t>(255 << 24);

class Filters {

    public:
  template <typename PointT>
  static typename pcl::ConditionalRemoval<PointT>::Ptr HighConfidenceFilter() {

    // Filter point with high confidence
    typename pcl::ConditionAnd<PointT>::Ptr range_cond(
        new pcl::ConditionAnd<PointT>());

    // Add a condition to filter points where the alpha channel is greater than
    // or equal to the threshold
    range_cond->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(
        new pcl::FieldComparison<PointT>("rgba", pcl::ComparisonOps::LE,
                                         RGBA_ALPHA_UPPER_BOUND)));

    range_cond->addComparison(typename pcl::FieldComparison<PointT>::ConstPtr(
        new pcl::FieldComparison<PointT>("rgba", pcl::ComparisonOps::GE,
                                         RGBA_ALPHA_LOWER_BOUND)));

    // build the filter
    typename pcl::ConditionalRemoval<PointT>::Ptr condrem =
        typename pcl::ConditionalRemoval<PointT>::Ptr(
            new pcl::ConditionalRemoval<PointT>());
    condrem->setCondition(range_cond);
    condrem->setKeepOrganized(false);

    return condrem;
  };

  template <typename PointT>
  static typename pcl::VoxelGrid<PointT>::Ptr GridFilter(float size = 0.02) {

    typename pcl::VoxelGrid<PointT>::Ptr grid =
        typename pcl::VoxelGrid<PointT>::Ptr(new pcl::VoxelGrid<PointT>());
    grid->setLeafSize(size, size, size);

    return grid;
  };

  template <typename PointT>
  static typename pcl::CropBox<PointT>::Ptr
  CropBox(const float dist, const Eigen::Vector3f origin) {
    typename pcl::CropBox<PointT>::Ptr box =
        typename pcl::CropBox<PointT>::Ptr(new pcl::CropBox<PointT>());

    const float min_x = origin.x() - dist;
    const float min_y = origin.y() - dist;
    const float min_z = origin.z() - dist;
    const float max_x = origin.x() + dist;
    const float max_y = origin.y() + dist;
    const float max_z = origin.z() + dist;
    box->setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
    box->setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
    box->setNegative(false);

    return box;
  };
};
} // namespace ReUseX
