// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/core/logging.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_regularization/regularize_planes.h>
#include <CGAL/property_map.h>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <range/v3/to_container.hpp>
#include <range/v3/view/transform.hpp>

#include <vector>

template <typename Kernel, typename Scalar> struct plane_map {

  using key_type =
      Eigen::Matrix<Scalar, 4, 1>; // The iterator's value type is an index
  using value_type = typename Kernel::Plane_3; // The object manipulated by the
                                               // algorithm is a Plane
  using reference = value_type; // The object does not exist in memory, so
                                // there's no reference
  using category =
      boost::readable_property_map_tag; // The property map is used both

  using FT = typename Kernel::FT;

  value_type operator[](const key_type &p) const {
    return value_type(FT(p(0)), FT(p(1)), FT(p(2)), FT(p(3)));
  }
  friend value_type get(const plane_map &, const key_type &p) {
    return value_type(FT(p(0)), FT(p(1)), FT(p(2)), FT(p(3)));
  };

  friend void put(const plane_map &, key_type &p, const value_type &val) {
    p(0) = static_cast<Scalar>(CGAL::to_double(val.a()));
    p(1) = static_cast<Scalar>(CGAL::to_double(val.b()));
    p(2) = static_cast<Scalar>(CGAL::to_double(val.c()));
    p(3) = static_cast<Scalar>(CGAL::to_double(val.d()));
  };
};

template <typename Kernel, typename PointT> struct point_map {

  using key_type = PointT;
  using value_type = typename Kernel::Point_3;
  using reference = value_type;
  using category = boost::readable_property_map_tag;
  value_type operator[](const key_type &pt) const {
    return value_type(pt.x, pt.y, pt.z);
  }

  friend value_type get(const point_map &, const key_type &pt) {
    return value_type(pt.x, pt.y, pt.z);
  };

  friend void put(const point_map &, key_type &pt, const value_type &val) {
    pt.x = static_cast<typename PointT::ScalarType>(CGAL::to_double(val.x()));
    pt.y = static_cast<typename PointT::ScalarType>(CGAL::to_double(val.y()));
    pt.z = static_cast<typename PointT::ScalarType>(CGAL::to_double(val.z()));
  };
};

namespace ReUseX::geometry {
template <typename Scalar> using Plane = Eigen::Matrix<Scalar, 4, 1>;

template <typename Scalar>
using PlaneVector =
    std::vector<Plane<Scalar>, Eigen::aligned_allocator<Plane<Scalar>>>;

template <typename Scalar, typename PointT>
auto regularizePlanes(PlaneVector<Scalar> &planes,
                      typename pcl::PointCloud<PointT>::ConstPtr points,
                      std::vector<pcl::IndicesPtr> &inliers,
                      double angle_threshold = 25.0,
                      double distance_threshold = 0.01) {
  ReUseX::core::trace(
      "Regularizing planes with threshold: {} degrees and {} distance",
      angle_threshold, distance_threshold);

  using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
  using FT = typename Kernel::FT;

  std::vector<int> point_plane_index(points->size(), -1);
  for (size_t i = 0; i < inliers.size(); ++i) {
    for (const auto &idx : *(inliers[i])) {
      point_plane_index[idx] = static_cast<int>(i);
    }
  }

  CGAL::Shape_regularization::Planes::regularize_planes(
      *points, point_map<Kernel, PointT>(), planes, plane_map<Kernel, Scalar>(),
      CGAL::Random_access_property_map(
          point_plane_index), /* point index to plane index */
      /*regularize_parallelism=*/true,
      /*regularize_orthogonality=*/true,
      /*regularize_coplanarity=*/true,
      /*regularize_axis_symmetry=*/true,
      /*tolerance_angle=*/FT(angle_threshold),
      /*tolerance_coplanarity=*/FT(distance_threshold));

  // CGAL::Shape_regularization::Planes::regularize_planes(
  //     planes, *points,
  //     CGAL::parameters::plane_map(plane_map<Kernel, Scalar>())
  //         .point_map(point_map<Kernel, PointT>())
  //         .plane_index_map(plane_index_map(inliers, points->size()))
  //         // CGAL::Random_access_property_map(point_plane_index))
  //         //.regularize_coplanarity(true)
  //         //.maximum_offset(FT(0.01))
  //         //.regularize_parallelism(true)
  //         //.regularize_orthogonality(true)
  //         //.regularize_axis_symmetry(true)
  //         //.symmetry_direction(Kernel::Vector_3(0, 0, 1))
  //         .maximum_angle(FT(threshold)));

  return planes;
}
} // namespace ReUseX::geometry
