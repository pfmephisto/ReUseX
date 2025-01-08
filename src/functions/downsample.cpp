#include "functions/downsample.hh"
#include "functions/progress_bar.hh"
#include "types/Geometry/PointCloud.hh"
#include "types/Accumulators.hh"

#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/types.h>



namespace ReUseX
{
    void downsample(PointCloud::Cloud::Ptr cloud, double leaf_size){

        pcl::octree::OctreePointCloudPointVector<PointCloud::Cloud::PointType> octree(leaf_size);
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();


        std::vector<pcl::octree::OctreePointCloudPointVector<PointCloud::Cloud::PointType>::LeafNodeIterator> nodes;
        for (auto it = octree.leaf_depth_begin(), it_end = octree.leaf_depth_end(); it != it_end; ++it)
            nodes.push_back(it);

        size_t leaf_count = octree.getLeafCount();
        PointCloud filtered_cloud = PointCloud();
        filtered_cloud->resize(leaf_count);

        
        #pragma omp parallel for shared(filtered_cloud, nodes)
        for (size_t i = 0; i < leaf_count; i++){
            pcl::octree::OctreeContainerPointIndices& container = nodes[i].getLeafContainer();

            pcl::Indices indexVector;
            container.getPointIndices(indexVector);

            Accumulators<PointCloud::Cloud::PointType>::type acc;

            for (auto const& index : indexVector){
                PointCloud::Cloud::PointType point = cloud->at(index);
                boost::fusion::for_each (acc, pcl::detail::AddPoint<PointCloud::Cloud::PointType> (point));
            }

            boost::fusion::for_each (acc, pcl::detail::GetPoint< PointCloud::Cloud::PointType> (filtered_cloud->at(i), indexVector.size()));

        }

        std::swap(*cloud, *filtered_cloud);
    }
    

} // namespace ReUseX
