// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/cgal_utils.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

namespace ReUseX::geometry::cgal {

Mesh pcl_to_cgal_mesh(const pcl::PolygonMesh& pcl_mesh) {
    Mesh mesh;

    // Decode vertices from PCL point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_mesh.cloud, cloud);

    // Add vertices to CGAL mesh and build index map
    std::vector<Mesh::Vertex_index> vertex_map;
    vertex_map.reserve(cloud.size());
    for (const auto& pt : cloud) {
        vertex_map.push_back(mesh.add_vertex(Point_3(pt.x, pt.y, pt.z)));
    }

    // Add faces
    for (const auto& poly : pcl_mesh.polygons) {
        std::vector<Mesh::Vertex_index> face_verts;
        face_verts.reserve(poly.vertices.size());
        for (auto idx : poly.vertices) {
            face_verts.push_back(vertex_map[idx]);
        }
        mesh.add_face(face_verts);
    }

    return mesh;
}

pcl::PolygonMesh cgal_to_pcl_mesh(const Mesh& cgal_mesh) {
    pcl::PolygonMesh pcl_mesh;

    // Create vertex cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(cgal_mesh.number_of_vertices());

    // Build vertex index map
    std::map<Mesh::Vertex_index, std::size_t> vertex_to_idx;
    std::size_t idx = 0;
    for (auto vi : cgal_mesh.vertices()) {
        const auto& p = cgal_mesh.point(vi);
        pcl::PointXYZ pt;
        pt.x = CGAL::to_double(p.x());
        pt.y = CGAL::to_double(p.y());
        pt.z = CGAL::to_double(p.z());
        cloud.push_back(pt);
        vertex_to_idx[vi] = idx++;
    }

    // Convert cloud to PCLPointCloud2
    pcl::toPCLPointCloud2(cloud, pcl_mesh.cloud);

    // Add faces
    pcl_mesh.polygons.reserve(cgal_mesh.number_of_faces());
    for (auto fi : cgal_mesh.faces()) {
        pcl::Vertices poly;
        for (auto vi : CGAL::vertices_around_face(cgal_mesh.halfedge(fi), cgal_mesh)) {
            poly.vertices.push_back(vertex_to_idx[vi]);
        }
        pcl_mesh.polygons.push_back(poly);
    }

    return pcl_mesh;
}

} // namespace ReUseX::geometry::cgal
