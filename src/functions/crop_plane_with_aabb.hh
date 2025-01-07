
#pragma once
#include "types/Plane.hh"
#include "types/Surface_Mesh.hh"
#include "types/Point.hh"
#include "types/Line.hh"

#include "functions/color.hh"
#include "functions/get_csystem.hh"


#include <CGAL/boost/graph/Euler_operations.h>
#include <cmath>

static auto comparator = [](std::pair<float, linkml::Point> p1, std::pair<float, linkml::Point> p2) {
    return p1.first < p2.first;
};
static std::array<linkml::Line, 12> get_segemets(linkml::AABB const & box) {

        auto segemets = std::array<linkml::Line, 12>();

        auto p0 = linkml::Point(box.min().x(),box.min().y(), box.min().z()); // 0
        auto p1 = linkml::Point(box.max().x(),box.min().y(), box.min().z()); // 1
        auto p2 = linkml::Point(box.min().x(),box.max().y(), box.min().z()); // 2
        auto p3 = linkml::Point(box.max().x(),box.max().y(), box.min().z()); // 3

        auto p4 = linkml::Point(box.min().x(),box.min().y(), box.max().z()); // 4
        auto p5 = linkml::Point(box.max().x(),box.min().y(), box.max().z()); // 5
        auto p6 = linkml::Point(box.min().x(),box.max().y(), box.max().z()); // 6
        auto p7 = linkml::Point(box.max().x(),box.max().y(), box.max().z()); // 7



        segemets[0] = linkml::Line(p0, p1); 
        segemets[1] = linkml::Line(p1, p3); 
        segemets[2] = linkml::Line(p3, p2); 
        segemets[3] = linkml::Line(p2, p0); 
        segemets[4] = linkml::Line(p4, p5); 
        segemets[5] = linkml::Line(p5, p7); 
        segemets[6] = linkml::Line(p7, p6);
        segemets[7] = linkml::Line(p6, p4);
        segemets[8] = linkml::Line(p0, p4);
        segemets[9] = linkml::Line(p1, p5);
        segemets[10]= linkml::Line(p3, p7);
        segemets[11]= linkml::Line(p2, p6);

        return segemets;
}
static std::vector<Point> get_points(std::array<linkml::Line, 12> const & segments, linkml::Plane const & plane){

    auto points = std::vector<Point>();

    for (auto & seg : segments){

        auto pt = CGAL::intersection(seg, plane);

        if (!pt.has_value()) continue;
        points.push_back(std::get<linkml::Point>(pt.value()));
    }


    return points;

}


#include <Eigen/Dense>
#include <Eigen/Geometry>



static std::vector<float> get_angles_in_plane(Eigen::Matrix3f mat, std::vector<linkml::Point> points, linkml::Point center ){

    auto angs = std::vector<float>();
    for (auto& p : points){
        auto v = Eigen::Vector3f(p.x()-center.x(), p.y()-center.y(), p.z()-center.z()).normalized();
        v = mat.inverse() *v;
        auto a =  std::atan2(v.x(), v.y());
        angs.push_back(a);
    }
    return angs;
}
static void make_unique(std::vector<linkml::Point> & collection){
    auto set = std::unordered_set<linkml::Point>();

    for (auto & item : collection)
        set.insert(item);

    collection.clear();

    for (auto & item : set)
        collection.push_back(item);
}


namespace linkml {

    static void crop_plane_with_aabb(Surface_mesh & mesh, const Box& box, const Plane plane ){
    
            auto const segemets = get_segemets(box);
            auto points = get_points(segemets, plane);

            
            make_unique(points);
    
            if (points.size() < 3) return;
    
            auto [mat, center] = get_csystem(points, plane);
            auto agles = get_angles_in_plane(mat, points, center);
    
            std::vector<std::pair<float, Point>> pairs;
            for (size_t i = 0; i < agles.size(); ++i) {
                pairs.emplace_back(agles[i], points[i]);
            }
    
            std::sort(pairs.begin(), pairs.end(), comparator);
    
            std::vector<Surface_mesh::vertex_index> vertecies;
            vertecies.reserve(pairs.size());
            for (auto & p : pairs)
                vertecies.push_back(mesh.add_vertex(Point(p.second.x(), p.second.y(), p.second.z())));


            int t = vertecies.size()-1;
            for (int i = 0; i < t-1; ++i)
                mesh.add_face(vertecies[i], vertecies[i+1], vertecies[t]);
            
            

    }

}