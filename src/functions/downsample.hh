#pragma once
#include "types/PointCloud.hh"


namespace ReUseX
{
    void downsample(PointCloud::Cloud::Ptr cloud, double leaf_size);
}