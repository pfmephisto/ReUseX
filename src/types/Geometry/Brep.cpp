#include "types/Geometry/Brep.hh"
#include "functions/polyscope.hh"

#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh/IO/OFF.h>
#include <CGAL/Optimal_bounding_box/oriented_bounding_box.h>
#include <CGAL/squared_distance_3.h>

#include <opennurbs_public.h>
#include <opennurbs_brep.h>


#include "types/Geometry/Plane.hh"
#include "types/Point2.hh"
#include "functions/fit_plane_thorugh_points.hh"

#include <vector>
#include <algorithm>
#include <filesystem>

// defining OPENNURBS_PUBLIC_INSTALL_DIR enables automatic linking using pragmas
// #define OPENNURBS_PUBLIC_INSTALL_DIR "../extern/opennurbs"
// uncomment the next line if you want to use opennurbs as a DLL
//#define OPENNURBS_IMPORTS
// #include "../extern/opennurbs/opennurbs_public.h"


namespace ReUseX
{


    Brep::Brep(Mesh const& mesh){

        ON_Mesh ONmesh = ON_Mesh();
        for (auto v: mesh.vertices())
            ONmesh.SetVertex(v.idx(), ON_3dPoint(mesh.point(v).x(), mesh.point(v).y(), mesh.point(v).z()));


        for (auto f: mesh.faces()){
            ON_SimpleArray<unsigned int> face;
            for (auto v : mesh.vertices_around_face(mesh.halfedge(f)))
                face.Append(v.idx());
            ONmesh.AddNgon(face);
        }
        
        ON_BrepFromMeshWithNgons(ONmesh.Topology(), false, true, 0.000001, this);

    }

    // void Brep::save(std::string const& filename) const { 
    //     std::ofstream file(filename);
    //     CGAL::IO::write_OFF(file, mesh);
    // }

    void Brep::save(std::string const& filename) const {
        ONX_Model model;

        model.m_sStartSectionComments = "ReUseX Export";
        model.m_properties.m_Application.m_application_name = "ReUseX";
        model.m_properties.m_Application.m_application_URL = L"https://github.com/pfmephisto/ReUseX";
        model.m_properties.m_Application.m_application_details = "ReUseX Export";

        model.m_properties.m_Notes.m_notes = "ReUseX Export";
        model.m_properties.m_Notes.m_bVisible = model.m_properties.m_Notes.m_notes.IsNotEmpty();

        model.m_properties.m_RevisionHistory.NewRevision();

        model.m_settings.m_ModelUnitsAndTolerances.m_unit_system = ON::LengthUnitSystem::Meters;
        model.m_settings.m_ModelUnitsAndTolerances.m_absolute_tolerance = 0.001;
        model.m_settings.m_ModelUnitsAndTolerances.m_angle_tolerance = ON_PI/180.0;
        model.m_settings.m_ModelUnitsAndTolerances.m_relative_tolerance = 0.01;

        auto layer_index = model.AddDefaultLayer(nullptr, ON_Color::UnsetColor);

        ON_3dmObjectAttributes* attributes = new ON_3dmObjectAttributes();
        attributes->m_layer_index = layer_index;
        attributes->m_name = "Brep";

        model.AddManagedModelGeometryComponent((ON_Object*)this, attributes);

        model.Write(filename.c_str(), 0);
    }

    Brep Brep::load(std::string const& filename) {

        if (!std::filesystem::exists(filename))
            throw std::runtime_error("File does not exist");
        if (!std::filesystem::is_regular_file(filename))
            throw std::runtime_error("File is not a regular file");


        if (std::filesystem::path(filename).extension() == ".off"){    
            std::ifstream file(filename);
            Mesh mesh;
            CGAL::IO::read_OFF(file, mesh);
            return Brep(mesh);
        }
        if (std::filesystem::path(filename).extension() == ".3dm"){
            ONX_Model model;
            if (!model.Read(filename.c_str()))
                throw std::runtime_error("Failed to read file");

            auto object = model.ComponentFromIndex(ON_ModelComponent::Type::ModelGeometry, 0);
            // if (object == nullptr)
            //     throw std::runtime_error("Failed to read object");

            // auto brep = dynamic_cast<ON_Brep*>(object);
            // if (brep == nullptr)
            //     throw std::runtime_error("Failed to read brep");

            // return Brep::B;

        }

        throw std::runtime_error("File format not supported: " + std::filesystem::path(filename).extension().string());
    }

    
    Mesh Brep::get_Mesh() const {

        // ON_SimpleArray<ON_Mesh*> mesh_list = ON_SimpleArray<ON_Mesh*>();
        // ON_Brep::CreateMesh(ON_MeshParameters::FastRenderMesh, mesh_list);

        // Mesh mesh;
        // for (int i = 0; i < mesh_list[0]->VertexCount(); i++)
        //     mesh.add_vertex(Point(
        //         mesh_list[0]->m_V[i].x,
        //         mesh_list[0]->m_V[i].y,
        //         mesh_list[0]->m_V[i].z));

        // for (int i = 0; i < mesh_list[0]->FaceCount(); i++){
        //     std::vector<Mesh::Vertex_index> face;

        //     // Check if the face is a triangle, quad or other
        //     int n = mesh_list[0]->m_F[i].IsTriangle() ? 3 :
        //             mesh_list[0]->m_F[i].IsQuad()    ? 4 : 0;

        //     for (int j = 0; j < n; j++)
        //         face.push_back(Mesh::Vertex_index(mesh_list[0]->m_F[i].vi[j]));
        //     mesh.add_face(face);
        // }

        return Mesh();

        // return mesh;
    }

    void Brep::display(std::string name, bool show ) const{
        polyscope::myinit();
        polyscope::display<ReUseX::Mesh const&>(this->get_Mesh(), name);
        if (show) polyscope::show();
    }

} // namespace ReUseX