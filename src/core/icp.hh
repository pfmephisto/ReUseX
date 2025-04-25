#pragma once
#include <optional>

#include <Eigen/Core>
#include <pcl/filters/filter.h>
#include <pcl/pcl_base.h>

namespace ReUseX {

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param grid_size the size of the grid
 * \return the resultant aligned source PointCloud
 */
template <typename PointT>
Eigen::Matrix4f
pair_align(const typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
           const typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
           std::optional<float> grid_size = {});

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param filters the filters to apply
 * \return the resultant aligned source PointCloud
 */
template <typename PointT>
std::tuple<Eigen::Matrix4f, double>
icp(const typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
    const typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
    std::vector<typename pcl::Filter<PointT>::Ptr> &filters =
        std::vector<typename pcl::Filter<PointT>::Ptr>(),
    const double maxCorrespondence = 0.2);
} // namespace ReUseX
