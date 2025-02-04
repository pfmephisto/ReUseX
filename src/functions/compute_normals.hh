#pragma once
#define PCL_NO_PRECOMPILE

#include "progress_bar.hh"
#include "io.hh"

#include <pcl/features/normal_3d_omp.h>

#include <filesystem>

namespace fs = std::filesystem;

namespace ReUseX
{
    
    /**
     * @brief Computes normals for a given set of point clouds.
     * 
     * This function takes a container of input point clouds, computes the normals for each point cloud,
     * and returns a container of point clouds with the computed normals concatenated to the original points.
     * 
     * @tparam PointTIn The type of the input point cloud points.
     * @tparam PointTOut The type of the output point cloud points.
     * @tparam InAllocator The allocator type for the input container. Defaults to std::allocator.
     * @tparam InContainer The container type for the input point clouds. Defaults to std::vector.
     * @tparam OutContainer The container type for the output point clouds. Defaults to std::vector with Eigen::aligned_allocator.
     * 
     * @param src The container of input point clouds.
     * @return OutContainer The container of output point clouds with computed normals.
     */
    template <
        typename PointTIn, 
        typename PointTOut,
        typename InAllocator = std::allocator<typename pcl::PointCloud<PointTIn>::Ptr>,
        typename InContainer = std::vector<typename pcl::PointCloud<PointTIn>::Ptr, InAllocator>,
        typename OutContainer = std::vector<typename pcl::PointCloud<PointTOut>::Ptr, Eigen::aligned_allocator<typename pcl::PointCloud<PointTOut>::Ptr>>
    >
    static OutContainer compute_normals(InContainer src ){

        using Cloud                 = typename pcl::PointCloud<PointTOut>;
        using CloudPtr              = typename pcl::PointCloud<PointTOut>::Ptr;
        using CloudNormal           = typename pcl::PointCloud<pcl::Normal>;
        using CloudNormalPtr        = typename pcl::PointCloud<pcl::Normal>::Ptr;
        using Tree                  = typename pcl::search::KdTree<PointTIn>;
        using TreePtr               = typename pcl::search::KdTree<PointTIn>::Ptr;
        using NormalEstimationOMP   = pcl::NormalEstimationOMP<PointTIn, pcl::Normal>;
        using NormalEstimation      = pcl::NormalEstimation<PointTIn, pcl::Normal>;


        // Load the clouds and compute normals
        auto clouds = OutContainer(src.size());
        auto progress_bar = util::progress_bar(src.size(), "Computing normals");

        #pragma omp parallel for shared(clouds)
        for (std::size_t i = 0; i < clouds.size(); ++i){

            NormalEstimationOMP ne;
            // NormalEstimation ne;
            ne.setInputCloud(src[i]);

            TreePtr tree(new Tree);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(0.15);
            ne.useSensorOriginAsViewPoint();

            CloudNormalPtr normals(new CloudNormal);
            ne.compute(*normals);

            clouds[i] = CloudPtr(new Cloud()); // Allocate memory
            pcl::concatenateFields(*src[i], *normals, *clouds[i]);

            // Copy the sensor information
            clouds[i]->sensor_origin_ = src[i]->sensor_origin_;
            clouds[i]->sensor_orientation_ = src[i]->sensor_orientation_;

            progress_bar.update();
        }
        progress_bar.stop();
        return clouds;

    }

    /**
     * @brief Computes the normals for a given point cloud.
     * 
     * This function takes an input point cloud and computes the normals for each point
     * in the cloud. It uses the PCL (Point Cloud Library) for normal estimation and 
     * concatenates the computed normals with the input cloud to produce an output cloud 
     * that contains both the original points and their corresponding normals.
     * 
     * @tparam PointTIn The type of the input point cloud.
     * @tparam PointTOut The type of the output point cloud.
     * @param cloud_in The input point cloud (of type PointTIn::Ptr).
     * @return PointTOut::Ptr The output point cloud containing both the original points 
     *         and their computed normals.
     */
    template < typename PointT_In, typename PointT_Out >
    static typename pcl::PointCloud<PointT_Out>::Ptr compute_normals(typename pcl::PointCloud<PointT_In>::Ptr cloud_in ){

        using Cloud_Out             = typename pcl::PointCloud<PointT_Out>;
        using Cloud_OutPtr          = typename pcl::PointCloud<PointT_Out>::Ptr;
        using CloudNormal           = typename pcl::PointCloud<pcl::Normal>;
        using CloudNormalPtr        = typename pcl::PointCloud<pcl::Normal>::Ptr;
        using Tree                  = typename pcl::search::KdTree<PointT_In>;
        using TreePtr               = typename pcl::search::KdTree<PointT_In>::Ptr;
        using NormalEstimationOMP   = pcl::NormalEstimationOMP<PointT_In, pcl::Normal>;


        NormalEstimationOMP ne;
        ne.setInputCloud(cloud_in);

        TreePtr tree(new Tree);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.15);
        ne.useSensorOriginAsViewPoint();

        CloudNormalPtr normals(new CloudNormal);
        ne.compute(*normals);

        auto cloud_out = Cloud_OutPtr(new Cloud_Out()); // Allocate memory
        pcl::concatenateFields(*cloud_in, *normals, *cloud_out);

        // Copy the sensor information
        cloud_out->sensor_origin_ = cloud_in->sensor_origin_;
        cloud_out->sensor_orientation_ = cloud_in->sensor_orientation_;

        return cloud_out;
    }

    /**
     * @brief Computes normals for a set of point clouds and saves the results.
     * 
     * This function takes a vector of file paths, loads the point clouds from these files,
     * computes the normals for each point cloud, and then saves the resulting point clouds
     * with normals back to the respective files.
     * 
     * @tparam PointTIn  The type of the input point cloud.
     * @tparam PointTOut The type of the output point cloud with normals.
     * @param paths      A vector of file paths to the input point cloud files.
     */
    template  <typename PointTIn, typename PointTOut>
    static void compute_normals(std::vector<fs::path> paths){
        
        using OutContainer  = std::vector<typename pcl::PointCloud<PointTOut>::Ptr, Eigen::aligned_allocator<typename pcl::PointCloud<PointTOut>::Ptr>>;
        using InContainer   = std::vector<typename pcl::PointCloud<PointTIn >::Ptr, Eigen::aligned_allocator<typename pcl::PointCloud<PointTIn >::Ptr>>;
        

        // Load the clouds and compute normals
        InContainer clouds_in;
        std::transform(paths.begin(), paths.end(), std::back_inserter(clouds_in), [](fs::path const& file){ return load<PointTIn>(file); });


        OutContainer clouds_out = compute_normals<PointTIn, PointTOut>(clouds_in);
        clouds_in.clear();


        util::progress_bar progress_bar(paths.size(), "Saving clouds");
        #pragma omp parallel for shared(clouds_out, paths)
        for (std::size_t i = 0; i < clouds_out.size(); ++i){
            save<PointTOut>(paths[i], clouds_out[i]);
            progress_bar.update();
        };
        progress_bar.stop();
    }

    /**
     * @brief Computes normals for a set of point cloud files in the specified directory.
     * 
     * This function iterates through the given directory, collects all files with a ".pcd" extension,
     * sorts them, and then computes normals for each file.
     * 
     * @tparam PointTIn The type of the input point.
     * @tparam PointTOut The type of the output point.
     * @param path The path to the directory containing the point cloud files.
     */
    template <typename PointTIn, typename PointTOut>
    void compute_normals(fs::path path){

        // Get the files & sort them
        std::vector<fs::path> files;
        for (const auto & entry : std::filesystem::directory_iterator(path))
            if (entry.is_regular_file() && entry.path().extension() == ".pcd")
                files.push_back(entry.path());
        std::sort(files.begin(), files.end());

        compute_normals<PointTIn, PointTOut>(files);
    }




} // namespace ReUseX
