// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/create_windows.hpp"

#include <pcl/conversions.h>
#include <pcl/surface/concave_hull.h>

#include <fmt/ranges.h>
#include <spdlog/spdlog.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <utility>

namespace ReUseX::geometry {

namespace {

/// Decode mesh cloud bytes into a vector of Eigen::Vector3d.
std::vector<Eigen::Vector3d>
decode_mesh_vertices(const pcl::PolygonMesh &mesh) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, pcl_cloud);

  std::vector<Eigen::Vector3d> verts(pcl_cloud.size());
  for (size_t i = 0; i < pcl_cloud.size(); ++i) {
    verts[i] = Eigen::Vector3d(pcl_cloud[i].x, pcl_cloud[i].y, pcl_cloud[i].z);
  }
  return verts;
}

/// Compute face normal via cross product for a triangle (v0, v1, v2).
Eigen::Vector3d face_normal(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1,
                            const Eigen::Vector3d &v2) {
  Eigen::Vector3d n = (v1 - v0).cross(v2 - v0);
  double len = n.norm();
  if (len < 1e-12)
    return Eigen::Vector3d::Zero();
  return n / len;
}

/// Triangle area (half magnitude of cross product).
double triangle_area(const Eigen::Vector3d &v0, const Eigen::Vector3d &v1,
                     const Eigen::Vector3d &v2) {
  return 0.5 * (v1 - v0).cross(v2 - v0).norm();
}

/// Sorted edge key for adjacency lookup.
using Edge = std::pair<int, int>;
Edge make_edge(int a, int b) {
  return a < b ? Edge{a, b} : Edge{b, a};
}

/// Compute the dominant plane normal for a set of 3D points using PCA.
/// Returns the normal vector corresponding to the smallest eigenvalue.
Eigen::Vector3d compute_instance_orientation(
    const std::vector<Eigen::Vector3d> &points) {

  if (points.size() < 3) {
    return Eigen::Vector3d::Zero();
  }

  // Compute centroid
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto &p : points) centroid += p;
  centroid /= static_cast<double>(points.size());

  // Build covariance matrix (3x3)
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto &p : points) {
    Eigen::Vector3d delta = p - centroid;
    cov += delta * delta.transpose();
  }
  cov /= static_cast<double>(points.size());

  // Eigendecomposition: eigenvectors are principal components
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if (solver.info() != Eigen::Success) {
    spdlog::warn("PCA eigendecomposition failed");
    return Eigen::Vector3d::Zero();
  }

  // Smallest eigenvalue → plane normal (sorted ascending)
  Eigen::Vector3d normal = solver.eigenvectors().col(0);
  normal.normalize();
  return normal;
}

} // namespace

std::vector<WallCandidate>
extract_wall_candidates(const pcl::PolygonMesh &mesh, float normal_z_threshold,
                        float coplanarity_angle_deg) {
  if (mesh.polygons.empty())
    return {};

  auto verts = decode_mesh_vertices(mesh);
  const size_t num_faces = mesh.polygons.size();

  // Compute per-face normals
  std::vector<Eigen::Vector3d> face_normals(num_faces);
  std::vector<double> face_areas(num_faces);
  for (size_t fi = 0; fi < num_faces; ++fi) {
    const auto &poly = mesh.polygons[fi];
    if (poly.vertices.size() < 3) {
      face_normals[fi] = Eigen::Vector3d::Zero();
      face_areas[fi] = 0.0;
      continue;
    }
    const auto &v0 = verts[poly.vertices[0]];
    const auto &v1 = verts[poly.vertices[1]];
    const auto &v2 = verts[poly.vertices[2]];
    face_normals[fi] = face_normal(v0, v1, v2);
    face_areas[fi] = triangle_area(v0, v1, v2);
  }

  // Build edge-to-face adjacency map
  std::map<Edge, std::vector<int>> edge_to_faces;
  for (size_t fi = 0; fi < num_faces; ++fi) {
    const auto &poly = mesh.polygons[fi];
    int nv = static_cast<int>(poly.vertices.size());
    for (int j = 0; j < nv; ++j) {
      int a = static_cast<int>(poly.vertices[j]);
      int b = static_cast<int>(poly.vertices[(j + 1) % nv]);
      edge_to_faces[make_edge(a, b)].push_back(static_cast<int>(fi));
    }
  }

  // Build face adjacency lists (shared edge = neighbor)
  std::vector<std::vector<int>> face_adj(num_faces);
  for (const auto &[edge, faces] : edge_to_faces) {
    if (faces.size() == 2) {
      face_adj[faces[0]].push_back(faces[1]);
      face_adj[faces[1]].push_back(faces[0]);
    }
  }

  // BFS to find connected components of approximately-vertical faces
  const double cos_threshold =
      std::cos(coplanarity_angle_deg * M_PI / 180.0);
  std::vector<bool> visited(num_faces, false);
  std::vector<WallCandidate> candidates;

  for (size_t seed = 0; seed < num_faces; ++seed) {
    if (visited[seed])
      continue;

    // NOTE: Vertical-only filter removed to support all orientations
    // (skylights, tilted windows, etc.)

    // Skip degenerate faces
    if (face_normals[seed].squaredNorm() < 1e-12) {
      visited[seed] = true;
      continue;
    }

    // BFS from seed: grow component while faces are coplanar and vertical
    std::vector<int> component;
    std::queue<int> queue;
    queue.push(static_cast<int>(seed));
    visited[seed] = true;

    // Seed normal for coplanarity check
    Eigen::Vector3d seed_normal = face_normals[seed];

    while (!queue.empty()) {
      int fi = queue.front();
      queue.pop();
      component.push_back(fi);

      for (int ni : face_adj[fi]) {
        if (visited[ni])
          continue;
        // Must be approximately coplanar with seed
        double dot = face_normals[ni].dot(seed_normal);
        if (std::abs(dot) < cos_threshold)
          continue;

        visited[ni] = true;
        queue.push(ni);
      }
    }

    if (component.empty())
      continue;

    // Compute area-weighted mean normal and centroid
    Eigen::Vector3d mean_normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_centroid = Eigen::Vector3d::Zero();
    double total_area = 0.0;

    for (int fi : component) {
      double area = face_areas[fi];
      mean_normal += face_normals[fi] * area;
      // Face centroid = mean of vertices
      const auto &poly = mesh.polygons[fi];
      Eigen::Vector3d fc = Eigen::Vector3d::Zero();
      for (auto vi : poly.vertices)
        fc += verts[vi];
      fc /= static_cast<double>(poly.vertices.size());
      mean_centroid += fc * area;
      total_area += area;
    }

    if (total_area < 1e-12)
      continue;

    mean_normal.normalize();
    mean_centroid /= total_area;

    // Ensure normal points in a consistent direction
    // (flip if it points "inward" — heuristic: prefer positive dot with centroid)
    // This is a simple heuristic; for indoor scans, normals typically
    // point toward the room center, so we keep them as-is.

    // Fit plane: n.dot(p) + d = 0 → d = -n.dot(centroid)
    Eigen::Vector4d plane;
    plane.head<3>() = mean_normal;
    plane[3] = -mean_normal.dot(mean_centroid);

    // Collect boundary vertices: vertices on edges shared by exactly 1 face
    // in the component
    std::set<int> component_set(component.begin(), component.end());
    std::set<int> boundary_vert_indices;

    for (int fi : component) {
      const auto &poly = mesh.polygons[fi];
      int nv = static_cast<int>(poly.vertices.size());
      for (int j = 0; j < nv; ++j) {
        int a = static_cast<int>(poly.vertices[j]);
        int b = static_cast<int>(poly.vertices[(j + 1) % nv]);
        Edge e = make_edge(a, b);
        const auto &faces = edge_to_faces.at(e);
        // Boundary edge: only 1 face in this component uses it
        int count_in_component = 0;
        for (int f : faces) {
          if (component_set.count(f))
            ++count_in_component;
        }
        if (count_in_component == 1) {
          boundary_vert_indices.insert(a);
          boundary_vert_indices.insert(b);
        }
      }
    }

    std::vector<Eigen::Vector3d> boundary_verts;
    boundary_verts.reserve(boundary_vert_indices.size());
    for (int vi : boundary_vert_indices)
      boundary_verts.push_back(verts[vi]);

    WallCandidate wc;
    wc.plane = plane;
    wc.centroid = mean_centroid;
    wc.normal = mean_normal;
    wc.boundary_vertices = std::move(boundary_verts);
    wc.face_indices = std::move(component);
    candidates.push_back(std::move(wc));
  }

  spdlog::debug("Extracted {} wall candidates from {} mesh faces",
                candidates.size(), num_faces);
  return candidates;
}

CreateWindowsResult
create_windows(CloudConstPtr cloud, CloudLConstPtr instance_labels,
               const std::map<uint32_t, uint32_t> &instance_to_semantic,
               const std::vector<WallCandidate> &walls,
               const std::vector<uint32_t> &window_semantic_labels,
               const CreateWindowsOptions &options) {

  CreateWindowsResult result;

  if (!cloud || !instance_labels || cloud->size() != instance_labels->size()) {
    spdlog::error("create_windows: invalid input (null or size mismatch)");
    return result;
  }

  if (walls.empty()) {
    spdlog::warn("create_windows: no wall candidates provided");
    return result;
  }

  // Build set of window semantic labels for fast lookup
  std::set<uint32_t> window_labels(window_semantic_labels.begin(),
                                   window_semantic_labels.end());

  // Identify window instances
  std::set<uint32_t> window_instance_ids;
  for (const auto &[inst_id, sem_id] : instance_to_semantic) {
    if (window_labels.count(sem_id))
      window_instance_ids.insert(inst_id);
  }

  if (window_instance_ids.empty()) {
    spdlog::info("create_windows: no window instances found for labels [{}]",
                 fmt::join(window_semantic_labels, ", "));
    return result;
  }

  spdlog::info("Processing {} window instances against {} wall candidates",
               window_instance_ids.size(), walls.size());

  // Precompute wall bounding boxes for quick rejection
  struct WallBBox {
    Eigen::Vector3d min_pt, max_pt;
  };
  std::vector<WallBBox> wall_bboxes(walls.size());
  for (size_t wi = 0; wi < walls.size(); ++wi) {
    if (walls[wi].boundary_vertices.empty())
      continue;
    Eigen::Vector3d lo = walls[wi].boundary_vertices[0];
    Eigen::Vector3d hi = lo;
    for (const auto &v : walls[wi].boundary_vertices) {
      lo = lo.cwiseMin(v);
      hi = hi.cwiseMax(v);
    }
    // Expand bbox slightly for tolerance
    Eigen::Vector3d margin(1.0, 1.0, 1.0);
    wall_bboxes[wi].min_pt = lo - margin;
    wall_bboxes[wi].max_pt = hi + margin;
  }

  int window_counter = 0;

  for (uint32_t inst_id : window_instance_ids) {
    // Collect points belonging to this instance
    std::vector<Eigen::Vector3d> inst_points;
    for (size_t i = 0; i < instance_labels->size(); ++i) {
      if ((*instance_labels)[i].label == inst_id) {
        const auto &pt = (*cloud)[i];
        inst_points.emplace_back(pt.x, pt.y, pt.z);
      }
    }

    if (inst_points.empty())
      continue;

    // Compute instance centroid
    Eigen::Vector3d inst_centroid = Eigen::Vector3d::Zero();
    for (const auto &p : inst_points)
      inst_centroid += p;
    inst_centroid /= static_cast<double>(inst_points.size());

    // Compute instance plane normal via PCA
    Eigen::Vector3d inst_normal = compute_instance_orientation(inst_points);

    if (inst_normal.norm() < 1e-6) {
      spdlog::warn("Instance {} has degenerate geometry, skipping", inst_id);
      result.unmatched_instances.push_back(static_cast<int>(inst_id));
      continue;
    }

    // Find best matching wall: minimize cost = w_angle * angle + w_dist * distance
    int best_wall = -1;
    double best_cost = std::numeric_limits<double>::max();
    const double w_angle = 1.0;      // Weight for angular difference (radians)
    const double w_dist = 0.5;       // Weight for spatial distance (meters)
    const double max_angle = 30.0 * M_PI / 180.0;  // 30-degree tolerance

    for (size_t wi = 0; wi < walls.size(); ++wi) {
      // Angular alignment: dot product → angle
      double dot = std::abs(inst_normal.dot(walls[wi].normal));
      dot = std::clamp(dot, -1.0, 1.0);
      double angle_diff = std::acos(dot);

      // Skip walls with poor angular alignment
      if (angle_diff > max_angle) continue;

      // Spatial distance: centroid to wall plane
      double dist = std::abs(walls[wi].plane.head<3>().dot(inst_centroid) +
                             walls[wi].plane[3]);

      // Bounding box check (penalize out-of-bbox)
      const auto &bb = wall_bboxes[wi];
      if ((inst_centroid.array() < bb.min_pt.array()).any() ||
          (inst_centroid.array() > bb.max_pt.array()).any()) {
        dist += 1.0;  // 1m penalty
      }

      // Combined cost
      double cost = w_angle * angle_diff + w_dist * dist;

      if (cost < best_cost) {
        best_cost = cost;
        best_wall = static_cast<int>(wi);
      }
    }

    if (best_wall < 0) {
      spdlog::warn("No suitable wall found for instance {} "
                   "(orientation mismatch or too far)", inst_id);
      result.unmatched_instances.push_back(static_cast<int>(inst_id));
      continue;
    }

    const auto &wall = walls[best_wall];

    // Construct intrinsic frame without Z-up assumption
    Eigen::Vector3d wall_normal = wall.normal;
    Eigen::Vector3d u_axis;
    Eigen::Vector3d z_up(0.0, 0.0, 1.0);

    // Check if wall is approximately vertical (|normal.z| < 0.3)
    if (std::abs(wall_normal.z()) < 0.3) {
      // Vertical wall: u-axis = normal × Z_up (horizontal)
      u_axis = wall_normal.cross(z_up);
      double u_len = u_axis.norm();
      if (u_len < 1e-6) {
        // Fallback: use arbitrary horizontal direction
        u_axis = Eigen::Vector3d(1.0, 0.0, 0.0);
      } else {
        u_axis /= u_len;
      }
    } else {
      // Horizontal or tilted wall: project normal to XY, rotate 90°
      Eigen::Vector2d normal_xy(wall_normal.x(), wall_normal.y());
      double xy_len = normal_xy.norm();

      if (xy_len > 1e-6) {
        normal_xy /= xy_len;
        u_axis = Eigen::Vector3d(-normal_xy.y(), normal_xy.x(), 0.0);
        u_axis.normalize();
      } else {
        // Horizontal ceiling/floor: arbitrary X-axis
        u_axis = Eigen::Vector3d(1.0, 0.0, 0.0);
      }
    }

    // v-axis = u × wall_normal (completes right-handed frame)
    Eigen::Vector3d v_axis = u_axis.cross(wall_normal);
    v_axis.normalize();

    // Project instance points onto wall plane → 2D (u, v)
    std::vector<Eigen::Vector2d> projected;
    projected.reserve(inst_points.size());
    for (const auto &p : inst_points) {
      // Project point onto wall plane
      double signed_dist = wall.normal.dot(p) + wall.plane[3];
      Eigen::Vector3d on_plane = p - signed_dist * wall.normal;
      Eigen::Vector3d delta = on_plane - wall.centroid;
      projected.emplace_back(delta.dot(u_axis), delta.dot(v_axis));
    }

    // Compute boundary
    CoplanarPolygon polygon;
    std::vector<Eigen::Vector3d> boundary_3d;

    if (options.mode == WindowBoundaryMode::rectangle) {
      // AABB in local 2D
      double u_min = projected[0].x(), u_max = projected[0].x();
      double v_min = projected[0].y(), v_max = projected[0].y();
      for (const auto &p : projected) {
        u_min = std::min(u_min, p.x());
        u_max = std::max(u_max, p.x());
        v_min = std::min(v_min, p.y());
        v_max = std::max(v_max, p.y());
      }

      // 4 corners in CCW order (viewed from wall exterior)
      boundary_3d.resize(4);
      boundary_3d[0] = wall.centroid + u_min * u_axis + v_min * v_axis;
      boundary_3d[1] = wall.centroid + u_max * u_axis + v_min * v_axis;
      boundary_3d[2] = wall.centroid + u_max * u_axis + v_max * v_axis;
      boundary_3d[3] = wall.centroid + u_min * u_axis + v_max * v_axis;
    } else {
      // Concave hull in 2D using PCL
      pcl::PointCloud<pcl::PointXYZ>::Ptr hull_input(
          new pcl::PointCloud<pcl::PointXYZ>);
      hull_input->resize(projected.size());
      for (size_t i = 0; i < projected.size(); ++i) {
        (*hull_input)[i].x = static_cast<float>(projected[i].x());
        (*hull_input)[i].y = static_cast<float>(projected[i].y());
        (*hull_input)[i].z = 0.0f;
      }

      pcl::PointCloud<pcl::PointXYZ> hull_output;
      pcl::ConcaveHull<pcl::PointXYZ> chull;
      chull.setInputCloud(hull_input);
      chull.setAlpha(options.alpha);
      chull.reconstruct(hull_output);

      boundary_3d.reserve(hull_output.size());
      for (const auto &hp : hull_output) {
        double u_coord = hp.x;
        double v_coord = hp.y;
        boundary_3d.push_back(wall.centroid + u_coord * u_axis +
                              v_coord * v_axis);
      }
    }

    if (boundary_3d.size() < 3) {
      spdlog::warn("Instance {} produced degenerate boundary ({} vertices)",
                   inst_id, boundary_3d.size());
      result.unmatched_instances.push_back(static_cast<int>(inst_id));
      continue;
    }

    // Offset along wall normal
    Eigen::Vector3d offset = wall.normal * static_cast<double>(options.wall_offset);
    for (auto &v : boundary_3d)
      v += offset;

    // Build polygon with offset plane
    polygon.vertices = std::move(boundary_3d);
    // Offset the plane constant: d' = d - offset_distance
    polygon.plane = wall.plane;
    polygon.plane[3] -= static_cast<double>(options.wall_offset);

    // Build BuildingComponent
    ++window_counter;
    BuildingComponent comp;
    comp.name = fmt::format("window_{}", window_counter);
    comp.type = ComponentType::window;
    comp.boundary = std::move(polygon);
    comp.confidence = -1.0; // auto-detected, no ML confidence
    comp.data = WindowData{};

    spdlog::debug("Created {} from instance {} ({} vertices, match cost {:.3f})",
                  comp.name, inst_id, comp.boundary.vertices.size(), best_cost);

    result.components.push_back(std::move(comp));
  }

  spdlog::info("Created {} window components ({} unmatched instances)",
               result.components.size(), result.unmatched_instances.size());
  return result;
}

} // namespace ReUseX::geometry
