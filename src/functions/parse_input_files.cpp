
#define PCL_NO_PRECOMPILE
#include <pcl/memory.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <pcl/common/impl/accumulators.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/range_image/range_image.h> //TODO: TEST
#include <pcl/point_types_conversion.h> //TODO: TEST
#include <pcl/range_image/range_image_planar.h> //TODO: TEST
#include <opencv2/core/eigen.hpp> // TODO: TEST
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/conditional_removal.h>

#include <functions/depth_to_3d.hh>
#include <types/Yolo.hh>
#include <types/Accumulators.hh>

#include <functions/polyscope.hh>
#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>

#include <fmt/core.h>
#include <fmt/printf.h>
#include <fmt/format.h>
#include <fmt/color.h>

#include "parse_input_files.hh"
#include "types/Dataset.hh"
#include "functions/progress_bar.hh"
#include "functions/io.hh"

#include <Eigen/Dense>


#include <mutex>
#include <thread>
#include <future>


namespace fs = std::filesystem;

// #define DISPLAY

namespace ReUseX{

    template<typename PointCloud>
    static void setHeader(PointCloud & cloud, size_t const i,  Eigen::MatrixXd const & odometry){

        uint64_t time_stamp = odometry(0,0);
        std::string frame_id = fmt::format("{:06}", (int)odometry(0,1));

        auto pos = odometry.block<1,3>(0,2);  // x,y,z
        auto quat = odometry.block<1,4>(0,5); // x,y,z,w

        Eigen::Vector4f origin = Eigen::Vector4f(0, 0, 0, 0);
        origin.x() = pos(0);
        origin.y() = pos(1);
        origin.z() = pos(2);

        Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
        orientation.x() = quat(0);
        orientation.y() = quat(1);
        orientation.z() = quat(2);
        orientation.w() = quat(3);

        cloud.header.seq = i;
        cloud.header.frame_id = frame_id;
        cloud.header.stamp = time_stamp;

        cloud.sensor_origin_[0] = origin[0];
        cloud.sensor_origin_[1] = origin[1];
        cloud.sensor_origin_[2] = origin[2];
        cloud.sensor_origin_[3] = origin[3];

        cloud.sensor_orientation_ = orientation;

    }

    void parse_Dataset( 
        Dataset const & dataset, 
        std::string const & output_path,
        int start,
        std::optional<int> stop_in,
        int step){

            // TODO: Make output path optional.
            // And in case it is not set use a temp folder
            //auto path = fs::temp_directory_path() / "ReUseX";
            
            // TODO: Optionally clear the output folder before saving the files
            //if (fs::exists(path))
            //    fs::remove_all(path);
            

            auto fields = dataset.fields();

            bool check = true;
            check &= std::count(fields.begin(), fields.end(), Field::COLOR) == 1;
            check &= std::count(fields.begin(), fields.end(), Field::DEPTH) == 1;
            check &= std::count(fields.begin(), fields.end(), Field::CONFIDENCE) == 1;
            check &= std::count(fields.begin(), fields.end(), Field::ODOMETRY) == 1;
            check &= std::count(fields.begin(), fields.end(), Field::POSES) == 1;

            if( !check){
                fmt::print(fg(fmt::color::red), "Dataset does not contain all required fields [COLOR, DEPTH, CONFIDENCE, ODOMETRY, POSES]");
                return;
            }


            int stop;
            if (stop_in.has_value()) stop = stop_in.value();
            else stop = dataset.size();

            if (start < 0) start = dataset.size() + start;
            if (stop < 0) stop = dataset.size() + stop;
            if (start < 0 || start >= dataset.size() || stop < 0 || stop > dataset.size() || step == 0){
                fmt::print(fg(fmt::color::red), "Invalid start, stop or step\n");
                return;
            }

            
            size_t n_frames = (stop - start) / step;
            double ratio = (double)n_frames / (double)dataset.size() * 100;

            // Print info
            fmt::print("Number of frames: ");
            fmt::print(fg(fmt::color::red), "{}", n_frames);
            fmt::print(fmt::emphasis::italic,  " -> ( start: {}; step: {}; end: {} ) {:3.2f}% \n", start, step, stop, ratio);

            if (n_frames == 0) return;


#if DISPLAY
            pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
#endif


            auto progress_bar = util::progress_bar(n_frames,"Processing data");
            #pragma omp parallel for firstprivate(step, start, output_path) shared(dataset)
            for (size_t i = 0; i < n_frames; i++ ){

                size_t index = start + (i * step);
                auto data = dataset[index];


                
                pcl::RangeImagePlanar::Ptr points = pcl::RangeImagePlanar::Ptr(new pcl::RangeImagePlanar);
                auto intrinsic_matrix = dataset.intrinsic_matrix();
                auto fx = intrinsic_matrix(0,0);
                auto fy = intrinsic_matrix(1,1);
                auto cx = intrinsic_matrix(2,0);
                auto cy = intrinsic_matrix(2,1);

                auto depth = data.get<Field::DEPTH>();
                points->setDepthImage(depth.data(),
                    depth.rows(), depth.cols(), // ( Width, Height) The Eigne matrix seems reversed but it works when accessing the buffer
                    cx, cy, fx, fy);


                // depth.normalize();
                // depth *= 255;
                // cv::Mat depthCV;
                // cv::eigen2cv(depth, depthCV);
                // cv::imshow("Depth", depthCV);
                // cv::waitKey(0);


                pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>);
                pcl::copyPointCloud(*points, *cloud);

                auto width = points->width;
                auto height = points->height;
                // fmt::printf("Size: %dx%d\n", width, height);

                cv::Mat image = data.get<Field::COLOR>();

                cv::resize( image, image, cv::Size(width, height));

                auto confidence = data.get<Field::CONFIDENCE>();

                #pragma omp parallel for shared(cloud, image, confidence)
                for (size_t idx = 0; idx < cloud->size(); idx++){
                    cloud->at(idx).label = static_cast<uint32_t>(confidence(idx));
                    cv::Vec3b vec = image.at<cv::Vec3b>(idx);
                    cloud->at(idx).r = vec[0];
                    cloud->at(idx).g = vec[1];
                    cloud->at(idx).b = vec[2];
                }


                // cv::Mat confidenceCV;
                // confidence = (confidence.array() == 1).select(128, confidence); // Replace 1 with 128
                // confidence = (confidence.array() == 2).select(255, confidence); // Replace 2 with 255
                // cv::eigen2cv(confidence, confidenceCV);
                // cv::imshow("Confidence", confidenceCV);
                // cv::waitKey(0);


                pcl::transformPointCloud(*cloud, *cloud, data.get<Field::POSES>());

                setHeader(*cloud, i , data.get<Field::ODOMETRY>());

                // cv::destroyAllWindows();



#if DISPLAY
                // Display the cloud
                auto points_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*cloud, *points_xyz);
                viewer.showCloud (points_xyz , fmt::format("Cloud {}", i));
#endif



                // // Compute surface normals and curvature
                // pcl::NormalEstimationOMP<pcl::PointXYZRGBL, pcl::PointXYZRGBL> ne;
                // pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBL> ());
                // ne.setSearchMethod(tree);
                // // ne.setRadiusSearch (0.05);
                // ne.setKSearch(15);
                // ne.setInputCloud(cloud);
                // ne.compute(*cloud);


                // Save cloud
                std::filesystem::create_directory(output_path);
                std::filesystem::path file_path(fmt::format("{}/cloud_{}.pcd", output_path, cloud->header.frame_id));

                
                save<pcl::PointXYZRGBL>(file_path, cloud);

                progress_bar.update();
            }
            progress_bar.stop();


#if DISPLAY
            while (!viewer.wasStopped ())
            {
            }
#endif
        }
 

} // namespace ReUseX

