#include "PointClouds.hh"
#include <functions/polyscope.hh>

#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>
#include <polyscope/camera_view.h>


#include <fmt/format.h>


#define FOV_VERTICAL 1.08275438689828
#define ASPECT_RATIO 0.75

template <typename T>
ReUseX::PointClouds<T> ReUseX::PointClouds<T>::display(bool show_clouds){


    polyscope::myinit();

    if(show_clouds) for (size_t i = 0; i < data.size(); ++i){

        if constexpr (std::is_same<T, std::string>::value){
            auto cloud = PointCloud::load(data[i]);
            // polyscope::display<PointCloud const&>(cloud, fmt::format("Cloud {}", i).c_str());
            polyscope::display<pcl::PointCloud<PointT> const&>(*cloud, fmt::format("Cloud {}", i).c_str());
        } else if constexpr (std::is_same<T, PointCloud>::value){
            polyscope::display<pcl::PointCloud<PointT> const&>(*data[i], fmt::format("Cloud {}", i).c_str());
            // polyscope::display<PointCloud const&>(data[i], fmt::format("Cloud {}", i).c_str());
        }
    }

    std::vector<Eigen::Vector4f> path_node;
    std::vector<polyscope::CameraView*> views;
    std::vector<Eigen::Vector3f> x_axis;
    std::vector<Eigen::Vector3f> y_axis;
    std::vector<Eigen::Vector3f> z_axis;
    std::vector<int> seq;
    std::vector<std::array<size_t, 2>> path_edges;

    // path_node.resize(data.size());
    views.resize(data.size());
    // x_axis.resize(data.size());
    // y_axis.resize(data.size());
    // z_axis.resize(data.size());
    // seq.resize(data.size());
    // path_edges.resize(data.size()-1);





    // #pragma omp parallel for shared(data, path_node, path_edges, views, seq, x_axis, y_axis, z_axis)
    for (size_t i = 0; i < data.size(); ++i){

        PointCloud::Cloud::Ptr cloud;
        
        if constexpr (std::is_same<T, std::string>::value){
            cloud = PointCloud::load(data[i]);
        } else {
            cloud = data[i];
        }

        auto matrix = cloud->sensor_orientation_.toRotationMatrix().normalized();

        // path_node[i] = cloud->sensor_origin_;
        // x_axis[i] = matrix.col(0);
        // y_axis[i] = matrix.col(1);
        // z_axis[i] = matrix.col(2);

        // seq[i] = cloud->header.seq;

        // if(i > 0){
        //     path_edges[i-1] = {i-1, i};
        // }


        auto params = polyscope::CameraParameters(
            polyscope::CameraIntrinsics::fromFoVDegVerticalAndAspect(60, ASPECT_RATIO), 
            polyscope::CameraExtrinsics::fromVectors(
                glm::vec3{ cloud->sensor_origin_.coeff(0) , cloud->sensor_origin_.coeff(1),cloud->sensor_origin_.coeff(2)},
                glm::vec3{ matrix.coeff(0,2) , matrix.coeff(1,2), matrix.coeff(2,2)},
                glm::vec3{ -matrix.coeff(0,0) , -matrix.coeff(1,0), -matrix.coeff(2,0)})
        );
        views[i] = polyscope::registerCameraView(fmt::format("View {}", cloud->header.frame_id), params);
        views[i]->setWidgetColor({0.01, 0.01, 0.01});
        views[i]->setWidgetThickness(0.001);
    }

    // auto ps_path = polyscope::registerCurveNetwork("Path", path_node, path_edges);
    // ps_path->addNodeScalarQuantity("Sequence", seq);
    
    // auto ps_Xaxis = ps_path->addNodeVectorQuantity("XAxis", x_axis);
    // auto ps_Yaxis = ps_path->addNodeVectorQuantity("YAxis", y_axis);
    // auto ps_Zaxis = ps_path->addNodeVectorQuantity("ZAxis", z_axis);

    // ps_Xaxis->setVectorColor({1, 0, 0});
    // ps_Yaxis->setVectorColor({0, 1, 0});
    // ps_Zaxis->setVectorColor({0, 0, 1});

    // ps_Xaxis->setVectorRadius(0.001);
    // ps_Yaxis->setVectorRadius(0.001);
    // ps_Zaxis->setVectorRadius(0.001);

    // ps_Xaxis->setVectorLengthScale(0.02);
    // ps_Yaxis->setVectorLengthScale(0.02);
    // ps_Zaxis->setVectorLengthScale(0.02);

    // ps_Xaxis->setEnabled(true);
    // ps_Yaxis->setEnabled(true);
    // ps_Zaxis->setEnabled(true);


    polyscope::myshow();

    return *this;

}


template ReUseX::PointCloudsInMemory ReUseX::PointCloudsInMemory::display(bool show_clouds);
template ReUseX::PointCloudsOnDisk ReUseX::PointCloudsOnDisk::display(bool show_clouds);