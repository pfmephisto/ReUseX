#pragma once
#include "types/Kernel.hh"
#include "types/Point.hh"
#include "types/FT.hh"
#include "types/Vector.hh"
#include "types/Direction.hh"
#include "types/AABB.hh"

#include <CGAL/Surface_mesh.h>
#include <CGAL/number_utils.h>

namespace ReUseX {
    
    using Surface_mesh = CGAL::Surface_mesh<Point>;

    class LinkMesh: public Surface_mesh
    {
    public:

        using Base = Surface_mesh;
        using PointT = Surface_mesh::Point;

        Base mesh = ((Base)*this);


        LinkMesh() : Surface_mesh() {}
        LinkMesh(const Surface_mesh& mesh) : Surface_mesh(mesh) {}
        LinkMesh(Surface_mesh&& mesh) : Surface_mesh(std::move(mesh)) {}
        LinkMesh(const LinkMesh& mesh) : Surface_mesh((Base)mesh) {}
        LinkMesh(LinkMesh&& mesh) : Surface_mesh(std::move((Base)mesh)) {}


        double volume() const;
        double area() const;
        Box get_bbox() const;
        std::vector<PointT> get_vertices() const;
        std::vector<int> get_faces() const;
        std::vector<int> get_colors() const;
        std::vector<float> get_textrueCoords() const;
    };
}