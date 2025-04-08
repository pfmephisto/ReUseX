#include "Mesh.hh"

#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/bounding_box.h>


namespace PMP = CGAL::Polygon_mesh_processing;


namespace ReUseX
{
   double Mesh::volume() const { return CGAL::to_double(PMP::volume((Base)*this)); }
   double Mesh::area() const { return CGAL::to_double(PMP::area((Base)*this)); }
   Box Mesh::get_bbox() const
   {
      return CGAL::bounding_box(this->points().begin(), this->points().end());
   }
   std::vector<Mesh::PointT> Mesh::get_vertices() const { 

      auto mesh = (Base)*this;

      auto points = std::vector<PointT>();
      points.resize(mesh.number_of_vertices());

      for (auto v : mesh.vertices())
      {
         points[v.id()] = mesh.point(v);
      }

      return points;
   }
   std::vector<int> Mesh::get_faces() const { 

      auto mesh = (Base)*this;
      auto values = std::vector<int>();
      // values.resize(mesh.number_of_faces()*4);
      for (auto f : mesh.faces()){

         std::vector<typename Base::Vertex_index> vertices;
         for (auto v : mesh.vertices_around_face(CGAL::halfedge(f,mesh)))
            vertices.push_back(v);

         values.push_back(vertices.size());
         for (auto v : vertices)
            values.push_back(v.idx());


      }
      return values; 
   }
   
   // TODO: Implement this function
   std::vector<int> Mesh::get_colors() const { return std::vector<int>(); }
   std::vector<float> Mesh::get_textrueCoords() const { return std::vector<float>(); }

} // namespace ReUseX
