#pragma once
#define PCL_NO_PRECOMPILE

#include "io.hh"
#include "progress_bar.hh"

#include <pcl/common/projection_matrix.h>
#include <xtensor/xarray.hpp>

#include <tuple>
#include <optional>
#include <filesystem>


namespace fs = std::filesystem;
		
namespace ReUseX
{

    using Masks = xt::xarray<uint8_t,xt::layout_type::dynamic>;
    using Labels = std::vector<int>;
    using MasksAndLabels = std::tuple<xt::xarray<uint8_t,xt::layout_type::dynamic>, std::vector<int>>;
    using Optional_MasksAndLables = std::optional< MasksAndLabels >;

    Optional_MasksAndLables get_masks_and_lables(HighFive::File const& file, std::string const& path){

        Masks  masks;
        Labels lables;
        try {
            masks = xt::load<xt::xarray<uint8_t,xt::layout_type::dynamic>>(file, path+"/masks");
            lables = xt::load<std::vector<int>>(file, path+"/names");
        }
        catch (const HighFive::DataSetException & e){
            fmt::print(fg(fmt::color::red), "Masks or names not found in frame {}", frame_id);
            continue;
        }

        return std::make_tuple(masks, lables);

    };


    template <typename PointT>
    void annotate(pcl::PointCloud<PointT>::Ptr const& cloud, xt::xarray<uint8_t,xt::layout_type::dynamic> const& masks, std::vector<int> const& lables;){


        #pragma omp parallel for shared(cloud, masks, lables)
        for (size_t i = 0; i < lables.size(); i++){

            auto img_size = cv::Size(masks.shape()[2],masks.shape()[1]);
            size_t offset = i * masks.shape()[1] * masks.shape()[2];

            cv::Mat masks_cv (img_size, CV_8U, masks.data()+offset, img_size.width);

            cv::rotate(masks_cv, masks_cv, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::resize(masks_cv, masks_cv, cv::Size(cloud->width,cloud->height));

            int label = lables[i];
            #pragma omp parallel for shared(cloud, masks_cv) firstprivate(label)
            for (size_t j = 0; j < masks_cv.total(); j++)
                if (masks_cv.data[j] > 0.5 && cloud->at(j).label == Deselected )
                    cloud->at(j).label = label;
        }


    };

    template <typename PointT>
    void annotate(pcl::PointCloud<PointT>::Ptr const& cloud, HighFive::File const& file, std::string const& path){


        auto result = get_masks_and_lables(file, path);

        // If there are no values return
        if (!result.has_value())
            return;
        
        auto [const& masks, const& lables] = get_masks_and_lables(file, path).value();

        annotate(cloud, masks, lables);

    };




    template <typename Matrix, typename Collection>
    pcl::PointCloud<pcl::Label>::Ptr annotations(Matrix const& masks, Collection, const& lables){

        pcl::PointCloud<pcl::Label>::Ptr cloud(new pcl::PointCloud<pcl::Label>);
        cloud->width = masks.shape()[2];
        cloud->height = masks.shape()[1];
        cloud->resize(cloud->width * cloud->height);

        annotate(cloud, masks, lables);

        return cloud;
    };

    template <typename Matrix, typename Collection>
    std::optional<pcl::PointCloud<pcl::Label>::Ptr> annotations(Matrix const& masks, Collection, const& lables){

        auto result = get_masks_and_lables(file, path);

        // If there are no values return
        if (!result.has_value())
            return;
        
        auto [masks, lables] = get_masks_and_lables(file, path).value();

        return annotations(masks, lables);
    };


    template <
        typename PointT_In,
        typename PointT_Out,    >
    void annotate(std::vector<fs::path> const& files, HighFive::File const& hdf5_file){

        using CloudIN = pcl::PointCloud<PointT_In>;
        using CloudINPtr = typename pcl::PointCloud<PointT_In>::Ptr;
        using CloudOUT = pcl::PointCloud<PointT_Out>;
        using CloudOUTPtr = typename pcl::PointCloud<PointT_Out>::Ptr;
        using Container = std::vector<typename pcl::PointCloud<PointT_Out>::Ptr, Eigen::aligned_allocator<typename pcl::PointCloud<PointT_Out>::Ptr>>


        Container clouds_out;
        clouds_out.reserve(files.size());


        utils::progress_bar progress_bar;


        progress_bar = utils::progress_bar(files.size(), "Annotating clouds");
        #pragma omp parallel for shared(clouds_out, files)
        for (size_t i = 0; i < files.size(); i++) {

            CloudINPtr cloud_in = load(<PointT_In>(files.at(i)));
            std::string frame_id = cloud->header.frame_id;

            auto result = annotations(hdf5_file, frame_id);

            if (!result.has_value()){
                progress_bar.update();
                continue;
            }

            auto labels = result.value();

            cloud_out[i] = CloudOutPtr(new CloudOut()); // Allocate memory
            pcl::concatenateFields(*cloud_in, *labels, *cloud_out);
            progress_bar.update();
        }
        progress_bar.stop();


        progress_bar = utils::progress_bar(files.size(), "Saving clouds");
        #pragma omp parallel for shared(clouds_out, paths)
        for (std::size_t i = 0; i < clouds_out.size(); ++i){
            save<PointTOut>(paths[i], clouds_out[i]);
            progress_bar.update();
        };
        progress_bar.stop();

    };


    template <typename PointT_In, typename PointT_Out>
    void annotate(fs::path const& clouds_path, fs::path const& hdf5_path){

        // Get the files & sort them
        std::vector<fs::path> files;
        for (const auto & entry : std::filesystem::directory_iterator(path))
            if (entry.is_regular_file() && entry.path().extension() == ".pcd")
                files.push_back(entry.path());
        std::sort(files.begin(), files.end());


        annotate<PointT_In, PointT_Out>(files, HighFive::File(hdf5_path, HighFive::File::ReadOnly));


    };
} // namespace ReUseX
