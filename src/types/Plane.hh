#pragma once

#include "types/Kernel.hh"
#include "types/Point.hh"
#include "types/Vector.hh"
#include "types/Direction.hh"

#include <Eigen/Dense>

namespace linkml {

    struct Plane: Kernel::Plane_3
    {

        Point origin = Point(0,0,0);


        Plane() :origin(){};

        Plane(float A, float B, float C, float D): Plane_3(A, B, C, D)
        {   
            origin = this->projection(Point(0,0,0));
        }
        Plane(float A, float B, float C, float D, float x, float y, float z): Plane_3(A, B, C, D)
        {
            origin = Point(x,y,z);
        }

        // tg::pos2 to2d(tg::pos3 p){

        //     auto mat = this->get_matrix_from_plane();
        //     mat = tg::inverse(mat);
        //     auto p_ = mat * (p-(normal*dis));
        //     return tg::pos2(p_.x, p_.y);
        // }
        // tg::pos3 to3d(tg::pos2 p){
        //     auto mat = this->get_matrix_from_plane();
        //     tg::pos3 p3d = tg::pos3(p.x, p.y, 0);
        //     return mat * p3d;
        // }

        // std::vector<tg::pos2> to2d(std::vector<tg::pos3> const& pts){

        //     auto mat = this->get_matrix_from_plane();
        //     mat = tg::inverse(mat);

        //     std::vector<tg::pos2> out;
        //     for (auto& p: pts){
        //         auto p_ = mat * (p-(normal*dis));
        //         out.emplace_back(tg::pos2(p_.x, p_.y));
        //     }

        //     // // Move all points closer to origin
        //     // auto center = (tg::vec2)tg::average(out);
        //     // std::transform(out.begin(), out.end(),out.begin(), [&](tg::pos2 & p){ return p - center;});

        //     return out;
        // }
        // std::vector<tg::pos3> to3d(std::vector<tg::pos2> const& pts){

        //     auto mat = this->get_matrix_from_plane();

        //     std::vector<tg::pos3> out;
        //     for (auto& p: pts){
        //         tg::pos3 p3d = tg::pos3(p.x, p.y, 0);
        //         auto p_ = mat * p3d;
        //         out.emplace_back(tg::pos3(p_.x, p_.y, p_.z));
        //     }

        //     // // Move all points closer to origin
        //     // auto center = (tg::vec3)tg::average(out);
        //     // std::transform(out.begin(), out.end(),out.begin(), [&](tg::pos3 & p){ return p - center;});

        //     return out;
        // }

        Direction orthogonal_direction(){
            return Direction(this->orthogonal_vector());
        }

        Direction normal(){
            return Direction(this->orthogonal_vector());
        }

        Direction X(){
            Vector vec = 1 - CGAL::abs(Vector(0,0,1) * this->orthogonal_vector()) > 0.5 ? Vector(0,0,1): Vector(1,0,0);
            return Direction(this->projection(this->origin+vec) - this->origin);
        }
        Direction Y(){
            auto X = this->X();
            return Direction(CGAL::cross_product(Vector(X.dx(), X.dy(), X.dz()), this->orthogonal_vector()));
        }

        Eigen::Matrix4d get_matrix_from_plane(){

            auto X = this->X();
            auto Y = this->Y();
            auto N = this->orthogonal_direction();
            
            Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();

            mat.coeffRef(0,0) = X.dx();
            mat.coeffRef(0,1) = X.dy();
            mat.coeffRef(0,2) = X.dz();

            mat.coeffRef(1,0) = Y.dx();
            mat.coeffRef(1,1) = Y.dy();
            mat.coeffRef(1,2) = Y.dz();

            mat.coeffRef(2,0) = N.dx();
            mat.coeffRef(2,1) = N.dy();
            mat.coeffRef(2,2) = N.dz();
            
            return mat;

        }
    
    };
}
