#pragma once
#include <optional>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>

#include "types/Geometry/PointCloud.hh"
#include "types/Data.hh"


namespace ReUseX {

    template <typename PointCloud>
    void depth_to_3d( 
        PointCloud  & point_cloud,
        Eigen::MatrixXd const& depths,
        Eigen::Matrix<double, 3, 3> const& intrinsic_matrix,
        Eigen::Matrix<double, 3, 3> const& pose,
        std::optional<cv::Mat> const& image = std::nullopt
    ){

        point_cloud.resize(depths.rows() * depths.cols());

        cv::Mat colors;
        if (image.has_value()) {
            cv::resize(image.value(), colors, cv::Size(depths.cols(), depths.rows())); // cv::INTER_LINEAR
            };

        #pragma omp parallel for collapse(2) shared(point_cloud, depths) firstprivate(intrinsic_matrix, pose)
        for (int row = 0; row < depths.rows(); row++){
            for (int col = 0; col < depths.cols(); col++){

                auto depth = depths(row,col);

                size_t index = row * depths.cols() + col;

                auto pos_h = Eigen::Vector3d(col,row, 1);
                auto pos_n = intrinsic_matrix.inverse() * pos_h;
                auto pos_3d = pos_n * depth;
                auto pos = pose * pos_3d;


                Eigen::Vector3d normal =  pos*-1;
                normal.normalize();
                normal = pose * normal;

                point_cloud[index].x  = pos[0];
                point_cloud[index].y  = pos[1];
                point_cloud[index].z  = pos[2];

                //FIXME: There is something wrong with the normal calculation
                // All normals are identical
                point_cloud[index].normal_x = normal[0];
                point_cloud[index].normal_y = normal[1];
                point_cloud[index].normal_z = normal[2];

                point_cloud[index].confidence = 0;
                point_cloud[index].semantic = 0;
                point_cloud[index].instance = 0;
                point_cloud[index].label = 0;
                

                if (image.has_value()) {
                    cv::Vec3b vec = colors.at<cv::Vec3b>(row, col);
                    point_cloud[index].r = vec[0];
                    point_cloud[index].g = vec[1];
                    point_cloud[index].b = vec[2];
                }
                else {
                    point_cloud[index].r = 0;
                    point_cloud[index].g = 0;
                    point_cloud[index].b = 0;
                }
            }
        }
        point_cloud.width = depths.cols();
        point_cloud.height = depths.rows();
        point_cloud.is_dense = true;
    }

} // namespace ReUseX