#pragma once
#include <vector>

#include <types/Plane.hh>
#include <types/PointCloud.hh>
#include <cmath>


namespace ReUseX {

    
    std::vector<pcl::PointIndices> refine(
        PointCloud::Cloud::Ptr const cloud, 
        std::vector<pcl::PointIndices> const & clusters,
        float angle_threashhold =  25 * (M_PI / 180), //25° in radiance => 0.436332312999
        float distance_threshhold = 0.5
        );
}