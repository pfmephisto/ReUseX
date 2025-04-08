#pragma once
#include "types/Kernel.hh"
#include "types/Point.hh"
#include "types/FT.hh"
#include "types/Vector.hh"
#include "types/Direction.hh"

#include "types/Geometry/AABB.hh"

#include <CGAL/Surface_mesh.h>
#include <CGAL/number_utils.h>

namespace ReUseX {

    struct Mesh: public CGAL::Surface_mesh<Point> {

        using Base = CGAL::Surface_mesh<Point>; // Alias for the base class
        using PointT = typename Base::Point;    // Alias for a member type of the base class

        double volume() const;
        double area() const;
        Box get_bbox() const;
        std::vector<PointT> get_vertices() const;
        std::vector<int> get_faces() const;
        std::vector<int> get_colors() const;
        std::vector<float> get_textrueCoords() const;
    };
}

// Reference for proper inheritance of CGAL::Surface_mesh
// https://doc.cgal.org/latest/Surface_mesh/Surface_mesh_2sm_derivation_8cpp-example.html

#define CGAL_GRAPH_TRAITS_INHERITANCE_CLASS_NAME ReUseX::Mesh
#define CGAL_GRAPH_TRAITS_INHERITANCE_BASE_CLASS_NAME CGAL::Surface_mesh<ReUseX::Point>
#include <CGAL/boost/graph/graph_traits_inheritance_macros.h>
 