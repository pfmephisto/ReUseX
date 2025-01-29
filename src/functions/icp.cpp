#include "icp.hh"
#include "../types/Geometry/PointCloud.hh"
#include "../types/Filters.hh"

#include <pcl/registration/icp.h>

#include <fmt/printf.h>
#include <fmt/color.h>


constexpr static uint32_t RGBA_ALPHA_UPPER_BOUND = static_cast<uint32_t>(255 << 24 | 255 << 16 | 255 << 8 | 255);
constexpr static uint32_t RGBA_ALPHA_LOWER_BOUND = static_cast<uint32_t>(255 << 24);


namespace ReUseX
{

    // Define a new point representation for < x, y, z, curvature >
    template <typename PointT>
    class MyPointRepresentation : public pcl::PointRepresentation<PointT>
    {
        using pcl::PointRepresentation<PointT>::nr_dimensions_;
        public:
        MyPointRepresentation ()
        {
            // Define the number of dimensions
            nr_dimensions_ = 0;

            if constexpr (pcl::traits::has_xyz_v<PointT>)
                nr_dimensions_ += 3;

            // if constexpr (pcl::traits::has_curvature_v<PointT>)
            //     nr_dimensions_ += 1;
            
            if constexpr (pcl::traits::has_color_v<PointT>)
                nr_dimensions_ += 3;
            
        };



        // Override the copyToFloatArray method to define our feature vector
        virtual void copyToFloatArray (const PointT &p, float * out) const
        {
            // < x, y, z, curvature, r, g, b >

            size_t idx = 0;
            if constexpr (pcl::traits::has_xyz_v<PointT>){
                out[idx] = p.x;
                out[idx + 1] = p.y;
                out[idx + 2] = p.z;
                idx += 3;
            }

            // if constexpr (pcl::traits::has_curvature_v<PointT>){
            //     out[idx] = p.curvature; // Seems not to be always set and in that case returns Inf and results in a failure in the KDTree
            //     idx++;
            // }

            if constexpr (pcl::traits::has_color_v<PointT>){
                out[idx] = static_cast<float>(p.r / 255.0);
                out[idx + 1] = static_cast<float>(p.g / 255.0);
                out[idx + 2] = static_cast<float>(p.b / 255.0);
                idx += 3;
            }
        }
    };


    ////////////////////////////////////////////////////////////////////////////////

    template <typename PointT>
    Eigen::Matrix4f pair_align (
        const typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
        const typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
        std::vector<typename pcl::Filter<PointT>::Ptr> filters
    ){

        using Ptr = typename pcl::PointCloud<PointT>::Ptr; 

        Ptr input_src(new pcl::PointCloud<PointT>);
        Ptr input_tgt(new pcl::PointCloud<PointT>);

        input_src = cloud_src->makeShared();
        input_tgt = cloud_tgt->makeShared(); 
        
        // Apply filters
        for (auto filter : filters){
            filter->setInputCloud(input_src);
            filter->filter(*input_src);

            filter->setInputCloud(input_tgt);
            filter->filter(*input_tgt);
        }

        if (input_src->empty() || input_tgt->empty()){
            fmt::print(fmt::fg(fmt::color::red), "Pair alignment failed: Empty point cloud!\n");
            return Eigen::Matrix4f::Identity();
        }



        // Compute point features
        // https://github.com/evil0sheep/pcl-slam/blob/master/src/pcl_slam.cpp



        // Instantiate our custom point representation (defined above) ...
        MyPointRepresentation<PointT> point_representation;

        // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
        float alpha[7] = {1.0 , 1.0 , 1.0, 1.0, 1.0, 1.0, 1.0};

        point_representation.setRescaleValues (alpha);



        // Align
        // pcl::IterativeClosestPoint<PointT,PointT> reg; //NonLinear
        pcl::IterativeClosestPointWithNormals<PointT,PointT> reg;
        
        reg.setInputSource(input_src);
        reg.setInputTarget(input_tgt);
        
        reg.setTransformationEpsilon (1e-8);

        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance(0.20);  

        // Set the point representation
        reg.setPointRepresentation(pcl::make_shared<const MyPointRepresentation<PointT>>(point_representation));


        reg.setMaximumIterations(50);
        // reg.setRANSACIterations(1000);
        reg.setEuclideanFitnessEpsilon(1);
        
        reg.setRANSACOutlierRejectionThreshold(0.05);

        auto result = pcl::PointCloud<PointT>();
        reg.align(result); // This does the actual computation

        Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity();

        if (reg.hasConverged())
            pairTransform = reg.getFinalTransformation().inverse();


        // else
        //     fmt::print(fmt::fg(fmt::color::red), "Pair alignment failed to converge for {} & {}!\n", input_src->header.frame_id, input_tgt->header.frame_id);

        // if (reg.hasConverged())
        //     fmt::print(fmt::fg(fmt::color::green), "Pair {} & {} converged with score: {} after {} iterations\n", input_src->header.frame_id, input_tgt->header.frame_id, reg.getFitnessScore(), reg.nr_iterations_);
        

        return pairTransform;

    }

    template <typename PointT>
    Eigen::Matrix4f pair_align (
        const typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
        const typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
        std::optional<float> grid_size
    ){

        std::vector<typename pcl::Filter<PointT>::Ptr> filters;


        if constexpr (pcl::traits::has_color_v<PointT>)
            filters.push_back(Filters::HighConfidenceFilter<PointT>());

    
        if (grid_size.has_value())
            filters.push_back(Filters::GridFilter<PointT>(grid_size.value()));


        return pair_align<PointT>(cloud_src, cloud_tgt, filters);

    }


    // template Eigen::Matrix4f pair_align<pcl::PointXYZRGBA> (
    //     typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src,
    //     const typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tgt,
    //     std::optional<float> grid_size
    // );

    template Eigen::Matrix4f pair_align<PointT> (
        const typename pcl::PointCloud<PointT>::ConstPtr cloud_src,
        const typename pcl::PointCloud<PointT>::ConstPtr cloud_tgt,
        std::optional<float> grid_size
    );
} // namespace ReUseX