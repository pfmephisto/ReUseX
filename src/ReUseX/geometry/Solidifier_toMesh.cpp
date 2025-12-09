// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/geometry/Solidifier.hpp>

#include "spdlog/spdlog.h"

/*
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/circulator.h>
#include <CGAL/polygon_mesh_processing.h>
*/
#include <range/v3/view/enumerate.hpp>

// using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
// using Kernel = CGAL::Simple_cartesian<double>;
// using Point = Kernel::Point_3;
// using Mesh = CGAL::Surface_mesh<Point>;
//
// namespace PMP = CGAL::Polygon_mesh_processing;
namespace ReUseX::geometry {

auto Solidifier::toMesh(std::function<bool(const Cd)> filter)
    -> std::pair<Eigen::MatrixXd, Eigen::MatrixXi> {
  /*
 Mesh mesh;

 // INFO: Create vertices
 std::map<size_t, Mesh::Vertex_index> vertex_map;
 for (auto vit = _cc->vertices_begin(); vit != _cc->vertices_end(); ++vit) {
   auto &v = (*_cc)[*vit];
   vertex_map[v.id] = mesh.add_vertex(Point(v.pos[0], v.pos[1], v.pos[2]));
 }

 for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {

   if (!filter(*cit))
     continue; // Skip cells that do not pass the filter

   for (auto fit = _cc->faces_begin(*cit); fit != _cc->faces_end(*cit);
        ++fit) {

     std::vector<Mesh::Vertex_index> verts{};
     for (auto vit = _cc->vertices_begin(*fit); vit != _cc->vertices_end(*fit);
          ++vit)
       verts.push_back(vertex_map[(*_cc)[*vit].id]);

     mesh.add_face(verts);
   }
 }

 // INFO: Merge volumes using boolean union
 spdlog::trace("Corefining and computing union");

 // mesh.collect_garbage();
 PMP::triangulate_faces(mesh);

 // if (!PMP::corefine_and_compute_union(mesh, mesh, mesh))
 //   spdlog::error("Boolean union failed");

 // PMP::triangulate_faces(mesh);
 // PMP::remove_isolated_vertices(mesh);

 spdlog::debug("Mesh is triangulated: {}",
               CGAL::is_triangle_mesh(mesh) ? "yes" : "no");
 */

  // INFO: New section
  std::map<int, CellComplex::Vertex> vertex_indices{};
  std::vector<Eigen::Vector3i> faces{};

  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    if (!filter(*cit))
      continue; // Skip cells that do not pass the filter

    auto cell = *cit;
    for (auto fit = _cc->faces_begin(*cit); fit != _cc->faces_end(*cit);
         ++fit) {

      std::vector<CellComplex::Vertex> verts{};

      std::vector<CellComplex::Vertex> neighbor_cells{};
      neighbor_cells.reserve(2);

      auto [start, end] = boost::adjacent_vertices(*fit, *_cc);
      for (auto fit = start; fit != end; ++fit) {
        // Get neighboring cells
        if ((*_cc)[*fit].type == NodeType::Cell)
          neighbor_cells.push_back(*fit);
        // Get face vertices
        else if ((*_cc)[*fit].type == NodeType::Vertex)
          verts.push_back(*fit);
      }

      if (neighbor_cells.size() > 2)
        spdlog::warn("Face {} has more than two adjacent cells",
                     (*_cc)[*fit].id);

      if (neighbor_cells.size() == 2) {
        const auto a = neighbor_cells[0];
        const auto b = neighbor_cells[1];

        if (filter(a) && filter(b))
          continue; // Skip faces between two valid cells
      }

      // TODO: Check face oriehntation
      bool flip = false;
      if (neighbor_cells.size() >= 1) {

        Eigen::Vector3d face_normal = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < verts.size(); ++i) {
          const auto v0 = (*_cc)[verts[i]].pos;
          const auto v1 = (*_cc)[verts[(i + 1) % verts.size()]].pos;
          face_normal +=
              (v0 - (*_cc)[*fit].pos).cross(v1 - (*_cc)[*fit].pos).head<3>();
        }
        face_normal.normalize();

        Eigen::Vector3d cell_dir =
            ((*_cc)[cell].pos - (*_cc)[*fit].pos).head<3>();
        cell_dir.normalize();

        const double dot = face_normal.dot(cell_dir);
        if (dot > 0)
          flip = true;
      }

      // INFO: Create face
      if (verts.size() < 3) {
        spdlog::error("Face {} has less than 3 vertices", (*_cc)[*fit].id);
        continue;
      }

      for (auto v : verts)
        vertex_indices[(*_cc)[v].id] = v;

      if (flip)
        std::reverse(verts.begin(), verts.end());

      for (size_t i = 1; i + 1 < verts.size(); ++i) {
        Eigen::Vector3i face{};
        face[0] = (*_cc)[verts[0]].id;
        face[1] = (*_cc)[verts[i]].id;
        face[2] = (*_cc)[verts[i + 1]].id;
        faces.push_back(face);
      }

    } // end for faces
  } // end for cells

  // INFO: Renumber mesh
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  const size_t nV = vertex_indices.size();
  const size_t nF = faces.size();

  spdlog::trace("Converted mesh has {} vertices and {} faces", nV, nF);
  V.resize(nV, 3);
  F.resize(nF, 3);

  // Map vertex indices
  spdlog::trace("Mapping {} vertices", nV);
  std::unordered_map<int, int> vmap;
  vmap.reserve(nV);

  int id = 0;
  for (const auto &[old_id, v] : vertex_indices) {
    V.row(id) << (*_cc)[v].pos.head<3>().transpose();
    vmap[old_id] = id++;
  }

  // Convert faces
  spdlog::trace("Mapping {} faces", nF);
  for (size_t i = 0; i < faces.size(); ++i)
    F.row(i) = faces[i];
  F = F.unaryExpr([&](int x) { return vmap[x]; });

  spdlog::trace("Converted mesh to Eigen matrices");

  /*
  // INFO: Old section

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  // Number of vertices and faces
  const size_t nV = mesh.number_of_vertices();
  const size_t nF = mesh.number_of_faces();

  spdlog::trace("Converted mesh has {} vertices and {} faces", nV, nF);

  V.resize(nV, 3);
  F.resize(nF, 3);

  // TODO: Convert to modern C++ style
  // CGAL has containers

  // Map vertex indices
  std::unordered_map<Mesh::Vertex_index, int> vmap;
  vmap.reserve(nV);

  int id = 0;
  for (Mesh::Vertex_index v : mesh.vertices()) {
    const Point &p = mesh.point(v);
    V.row(id) << p.x(), p.y(), p.z();
    vmap[v] = id++;
  }

  // Convert faces
  int f_id = 0;
  for (Mesh::Face_index f : mesh.faces()) {

    CGAL::Vertex_around_face_circulator<Mesh> vc(mesh.halfedge(f), mesh), end;
    end = vc;

    size_t idx = 0;
    do {
      if (idx >= 3) {
        spdlog::error("Non-triangular Face ID: {}|{}", f_id, idx++);
      } else {
        Mesh::Vertex_index v = *vc;
        F(f_id, idx++) = vmap[v];
      }
    } while (++vc != end);
    ++f_id;
  }
  spdlog::trace("Converted mesh to Eigen matrices");
  */

  return {V, F};
}

} // namespace ReUseX::geometry
