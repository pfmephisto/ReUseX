#pragma once
#include <memory>
#define PCL_NO_PRECOMPILE

#include "SIFTFilter.hh"

#include <pcl/filters/conditional_removal.h>
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
  static typename std::shared_ptr<SIFTFilter<PointT>> SIVFeatures() {
    return std::shared_ptr<SIFTFilter<PointT>>(new SIFTFilter<PointT>());
  };
};
} // namespace ReUseX
