#pragma once

#include "types/Plane.hh"
#include "types/PointCloud.hh"


#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
// #include <pcl/segmentation/region_growing.h>
// #include <pcl/segmentation/planar_region.h>

#include <CGAL/linear_least_squares_fitting_3.h>

namespace linkml {

    static linkml::Plane fit_plane_thorugh_points(std::vector<Point> const& points){


        Plane best_fit_plane;
        CGAL::linear_least_squares_fitting_3(
            points.begin(),
            points.end(),
            best_fit_plane,
            CGAL::Dimension_tag<0>() // 0D: points
        );

        Point com = CGAL::centroid(points.begin(), points.end());
        com = best_fit_plane.projection(com);

        return linkml::Plane(best_fit_plane.a(), best_fit_plane.b(), best_fit_plane.c(), best_fit_plane.d(), com.x(), com.y(), com.z());
    }

    static linkml::Plane fit_plane_thorugh_points(PointCloud::Cloud::ConstPtr cloud,  pcl::Indices const & indecies){


        Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
        Eigen::Vector4f clust_centroid = Eigen::Vector4f::Zero ();
        Eigen::Matrix3f clust_cov;

        pcl::computeMeanAndCovarianceMatrix (*cloud, indecies, clust_cov, clust_centroid);
        Eigen::Vector4f plane_params;
        
        EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
        EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
        pcl::eigen33 (clust_cov, eigen_value, eigen_vector);
        plane_params[0] = eigen_vector[0];
        plane_params[1] = eigen_vector[1];
        plane_params[2] = eigen_vector[2];
        plane_params[3] = 0;
        plane_params[3] = -1 * plane_params.dot (clust_centroid);

        vp -= clust_centroid;
        float cos_theta = vp.dot (plane_params);
        if (cos_theta < 0)
        {
            plane_params *= -1;
            plane_params[3] = 0;
            plane_params[3] = -1 * plane_params.dot (clust_centroid);
        }

        return linkml::Plane(plane_params[0], plane_params[1], plane_params[2], plane_params[3], clust_centroid[0], clust_centroid[1], clust_centroid[2]);
    }
}