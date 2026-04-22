// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/unweld.hpp"
#include "core/logging.hpp"
#include "geometry/utils.hpp"
#include "types.hpp"

#include <pcl/conversions.h>

#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

namespace ReUseX::geometry {

namespace {

/// Check whether two faces share an edge that passes through vertex @p v.
/// Both faces must contain v plus at least one other common vertex.
bool faces_share_edge_through(const pcl::Vertices &face_a,
                              const pcl::Vertices &face_b, uint32_t v) {
  for (uint32_t a : face_a.vertices) {
    if (a == v)
      continue;
    for (uint32_t b : face_b.vertices) {
      if (b == v)
        continue;
      if (a == b)
        return true;
    }
  }
  return false;
}

/// Group faces around vertex @p v into connected components where faces
/// in the same group share edges through v and have normals within threshold.
/// Returns a vector of group labels, one per entry in @p face_indices.
std::vector<uint32_t>
group_faces_around_vertex(uint32_t v,
                          const std::vector<uint32_t> &face_indices,
                          const std::vector<pcl::Vertices> &polygons,
                          const std::vector<Eigen::Vector3f> &face_normals,
                          float threshold_radians) {
  const auto n = static_cast<uint32_t>(face_indices.size());
  std::vector<uint32_t> group(n, UINT32_MAX);
  uint32_t next_group = 0;

  for (uint32_t start = 0; start < n; ++start) {
    if (group[start] != UINT32_MAX)
      continue;

    group[start] = next_group;
    std::queue<uint32_t> queue;
    queue.push(start);

    while (!queue.empty()) {
      uint32_t cur = queue.front();
      queue.pop();
      uint32_t fi_cur = face_indices[cur];

      for (uint32_t other = 0; other < n; ++other) {
        if (group[other] != UINT32_MAX)
          continue;

        uint32_t fi_other = face_indices[other];

        if (!faces_share_edge_through(polygons[fi_cur], polygons[fi_other], v))
          continue;

        float dot = face_normals[fi_cur].dot(face_normals[fi_other]);
        dot = std::clamp(dot, -1.0f, 1.0f);
        float angle = std::acos(dot);
        if (angle > threshold_radians)
          continue;

        group[other] = next_group;
        queue.push(other);
      }
    }
    ++next_group;
  }
  return group;
}

} // namespace

pcl::PolygonMeshPtr unweld_mesh(const pcl::PolygonMesh &mesh,
                                float threshold_radians) {
  auto result = pcl::PolygonMeshPtr(new pcl::PolygonMesh());

  if (mesh.polygons.empty()) {
    result->cloud = mesh.cloud;
    return result;
  }

  // 1. Extract point cloud
  CloudLocPtr cloud(new CloudLoc);
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

  // 2. Compute face normals
  std::vector<Eigen::Vector3f> face_normals(mesh.polygons.size());
  for (size_t i = 0; i < mesh.polygons.size(); ++i) {
    try {
      face_normals[i] = compute_polygon_normal(mesh.polygons[i], cloud);
    } catch (const std::runtime_error &) {
      face_normals[i] = Eigen::Vector3f::Zero();
      ReUseX::warn("Degenerate face {} assigned zero normal", i);
    }
  }

  // 3. Build vertex -> face adjacency
  const auto num_vertices = cloud->points.size();
  std::unordered_map<uint32_t, std::vector<uint32_t>> vertex_faces;
  vertex_faces.reserve(num_vertices);
  for (uint32_t fi = 0; fi < mesh.polygons.size(); ++fi) {
    for (uint32_t vi : mesh.polygons[fi].vertices) {
      vertex_faces[vi].push_back(fi);
    }
  }

  // 4. For each vertex, compute groups and assign new vertex IDs.
  //    Also record face -> group mapping for later polygon remapping.
  CloudLocPtr new_cloud(new CloudLoc);
  new_cloud->points.reserve(num_vertices);

  // vertex_group_to_new[old_v][group_id] = new_vertex_id
  std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>>
      vertex_group_to_new;

  // face_vertex_group[face_index][old_vertex] = group_id
  // We store this so we can remap polygon indices in step 5.
  std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>>
      face_vertex_group;

  for (auto &[v, flist] : vertex_faces) {
    if (flist.size() == 1) {
      uint32_t new_id = static_cast<uint32_t>(new_cloud->points.size());
      new_cloud->points.push_back(cloud->points[v]);
      vertex_group_to_new[v][0] = new_id;
      face_vertex_group[flist[0]][v] = 0;
      continue;
    }

    auto groups = group_faces_around_vertex(v, flist, mesh.polygons,
                                            face_normals, threshold_radians);

    // Find number of distinct groups
    uint32_t num_groups = 0;
    for (uint32_t g : groups)
      num_groups = std::max(num_groups, g + 1);

    // Create one new vertex per group
    for (uint32_t g = 0; g < num_groups; ++g) {
      uint32_t new_id = static_cast<uint32_t>(new_cloud->points.size());
      new_cloud->points.push_back(cloud->points[v]);
      vertex_group_to_new[v][g] = new_id;
    }

    // Record face -> group mapping
    for (size_t i = 0; i < flist.size(); ++i) {
      face_vertex_group[flist[i]][v] = groups[i];
    }
  }

  // 5. Remap polygon indices
  std::vector<pcl::Vertices> new_polygons(mesh.polygons.size());
  for (uint32_t fi = 0; fi < mesh.polygons.size(); ++fi) {
    const auto &old_verts = mesh.polygons[fi].vertices;
    new_polygons[fi].vertices.resize(old_verts.size());
    for (size_t vi = 0; vi < old_verts.size(); ++vi) {
      uint32_t old_v = old_verts[vi];
      uint32_t group_id = face_vertex_group[fi][old_v];
      new_polygons[fi].vertices[vi] = vertex_group_to_new[old_v][group_id];
    }
  }

  // 6. Build output mesh
  new_cloud->width = static_cast<uint32_t>(new_cloud->points.size());
  new_cloud->height = 1;

  pcl::toPCLPointCloud2(*new_cloud, result->cloud);
  result->polygons = std::move(new_polygons);

  ReUseX::debug("Unweld: {} -> {} vertices ({} faces)",
                       cloud->points.size(), new_cloud->points.size(),
                       result->polygons.size());

  return result;
}

} // namespace ReUseX::geometry
