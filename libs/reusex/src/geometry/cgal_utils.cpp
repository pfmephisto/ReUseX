// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/cgal_utils.hpp"

#include <CGAL/Polygon_mesh_processing/connected_components.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <map>
#include <vector>

namespace reusex::geometry::cgal {

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

std::vector<pcl::PolygonMeshPtr>
decompose_mesh(const pcl::PolygonMesh &pcl_mesh) {
    if (pcl_mesh.polygons.empty())
        return {};

    // 1. Convert to CGAL mesh for connected component analysis
    Mesh cgal_mesh = pcl_to_cgal_mesh(pcl_mesh);

    // 2. Compute connected components
    auto fccmap = cgal_mesh.add_property_map<Mesh::Face_index, std::size_t>(
        "f:component", 0).first;
    std::size_t num_components =
        CGAL::Polygon_mesh_processing::connected_components(cgal_mesh, fccmap);

    if (num_components <= 1) {
        // Single component — return a copy of the original
        auto copy = pcl::PolygonMeshPtr(new pcl::PolygonMesh(pcl_mesh));
        return {copy};
    }

    // 3. Build CGAL face index → PCL face index mapping.
    //    pcl_to_cgal_mesh adds faces in order, so CGAL face indices
    //    correspond 1:1 to PCL polygon indices.
    //    Group PCL face indices by component.
    std::vector<std::vector<std::size_t>> comp_faces(num_components);
    {
        std::size_t fi = 0;
        for (auto f : cgal_mesh.faces()) {
            comp_faces[fccmap[f]].push_back(fi);
            ++fi;
        }
    }

    // 4. For each component, extract faces and vertices from the ORIGINAL
    //    PCL mesh to preserve full point cloud data (including RGB).
    std::vector<pcl::PolygonMeshPtr> result;
    result.reserve(num_components);

    // Determine point step and field layout from original cloud
    const auto &src_cloud = pcl_mesh.cloud;
    std::size_t point_step = src_cloud.point_step;
    std::size_t num_src_points = src_cloud.width * src_cloud.height;

    for (std::size_t c = 0; c < num_components; ++c) {
        const auto &faces = comp_faces[c];
        if (faces.empty())
            continue;

        // Collect unique vertex indices used by this component
        std::map<uint32_t, uint32_t> old_to_new;
        for (std::size_t fi : faces) {
            for (uint32_t vi : pcl_mesh.polygons[fi].vertices) {
                if (old_to_new.find(vi) == old_to_new.end()) {
                    auto new_idx = static_cast<uint32_t>(old_to_new.size());
                    old_to_new[vi] = new_idx;
                }
            }
        }

        // Build new cloud by copying raw point data (preserves all fields)
        auto out = pcl::PolygonMeshPtr(new pcl::PolygonMesh());
        out->cloud.fields = src_cloud.fields;
        out->cloud.point_step = point_step;
        out->cloud.height = 1;
        out->cloud.width = static_cast<uint32_t>(old_to_new.size());
        out->cloud.row_step = out->cloud.width * point_step;
        out->cloud.is_dense = src_cloud.is_dense;
        out->cloud.is_bigendian = src_cloud.is_bigendian;
        out->cloud.data.resize(out->cloud.width * point_step);

        for (const auto &[old_idx, new_idx] : old_to_new) {
            if (old_idx < num_src_points) {
                std::memcpy(out->cloud.data.data() + new_idx * point_step,
                            src_cloud.data.data() + old_idx * point_step,
                            point_step);
            }
        }

        // Build remapped faces
        out->polygons.reserve(faces.size());
        for (std::size_t fi : faces) {
            pcl::Vertices poly;
            poly.vertices.reserve(pcl_mesh.polygons[fi].vertices.size());
            for (uint32_t vi : pcl_mesh.polygons[fi].vertices)
                poly.vertices.push_back(old_to_new[vi]);
            out->polygons.push_back(std::move(poly));
        }

        result.push_back(std::move(out));
    }

    return result;
}

} // namespace reusex::geometry::cgal
