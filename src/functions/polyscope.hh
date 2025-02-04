#pragma once
#define PCL_NO_PRECOMPILE

#include "types/Dataset.hh"

#include "types/Geometry/Mesh.hh"
#include "types/Geometry/PointCloud.hh"
#include "types/Geometry/Surface.hh"
#include "types/Geometry/AABB.hh"

#include "functions/crop_plane_with_aabb.hh"


#include <valarray>

#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>
#include <polyscope/surface_mesh.h>

#include <pcl/point_types.h>

#include <optional>

#include <opencv4/opencv2/core/types.hpp>



namespace polyscope  {

    static bool polyscope_enabled = false;

    static void myinit(){

        if (polyscope_enabled) return;

        polyscope::init();
        polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
        polyscope::view::setUpDir(polyscope::UpDir::YUp); //ZUp
        polyscope::options::transparencyMode = polyscope::TransparencyMode::Simple;
        polyscope_enabled = true;
    }

    static void myshow(){
        polyscope::show();
    }


    //// Define a function to display a error message in case the display function is not defined
    template <typename T>
    void display(T, std::optional<const std::string> = std::nullopt);


    enum class Field { 
        Points,
        RGB,
        Normal,
        Normal_color,
        Lables,
        Lables_color,
        Transparancy
    };

    
    template <Field F, typename Enable = void>
    struct FieldType;

    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Points ||
            F == Field::RGB ||
            F == Field::Normal ||
            F == Field::Normal_color
        >::type
    > {
        using type = std::array<float, 3>;
    };

    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Transparancy
        >::type
    > {
        using type = float;
    };

    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Lables
        >::type
    > {
        using type = uint32_t;
    };


    template <Field F>
    struct FieldType<
        F,
        typename std::enable_if<
            F == Field::Lables_color
        >::type> {
        using type = cv::Scalar;
    }; 

    template <Field F>
    class PolyscopeMap{
        private:
            pcl::PointCloud<PointT> const& cloud;
        public:
            PolyscopeMap(pcl::PointCloud<PointT> const& cloud) : cloud(cloud) {}

            size_t size() const { return cloud.points.size(); }
            typename FieldType<F>::type operator[](size_t idx) const {

                if constexpr (F == Field::Points && pcl::traits::has_xyz_v <PointT>) {
                    return std::array<float, 3>{cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z};
                } else if constexpr (F == Field::RGB && pcl::traits::has_color_v <PointT>) {
                    return std::array<float, 3>{
                        static_cast<float>(cloud.points[idx].r)/256,
                        static_cast<float>(cloud.points[idx].g)/256,
                        static_cast<float>(cloud.points[idx].b)/256};
                } else if constexpr (F == Field::Normal && pcl::traits::has_normal_v <PointT>) {
                    return std::array<float, 3>{
                        cloud.points[idx].normal_x,
                        cloud.points[idx].normal_y,
                        cloud.points[idx].normal_z};
                } else if constexpr (F == Field::Transparancy && pcl::traits::has_color_v <PointT>){
                    return static_cast<float>(cloud.points[idx].a)/256;
                } else if constexpr (F == Field::Lables && pcl::traits::has_label_v <PointT>){
                    return cloud.points[idx].label;
                } else if constexpr (F == Field::Lables_color && pcl::traits::has_label_v <PointT>){
                    return Color::from_index(cloud.points[idx].label);
                } else if constexpr (F == Field::Normal_color && pcl::traits::has_normal_v <PointT>) {
                    return std::array<float, 3>{
                        static_cast<float>(cloud.points[idx].normal_x + 1.0f)/2.0f,
                        static_cast<float>(cloud.points[idx].normal_y + 1.0f)/2.0f,
                        static_cast<float>(cloud.points[idx].normal_z + 1.0f)/2.0f
                    };
                };
            }
    };

}
