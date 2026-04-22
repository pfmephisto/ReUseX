// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "geometry/create_windows.hpp"
#include "core/logging.hpp"
#include "geometry/cgal_utils.hpp"

#include <pcl/common/pca.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>

#include <fmt/ranges.h>

#include <Eigen/Eigenvalues>
#include <embree4/rtcore.h>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Shape_detection/Region_growing/Polygon_mesh.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace ReUseX::geometry {

namespace {

/// Context for counting ray hits with face orientation.
struct RayContext {
  RTCRayQueryContext ctx;
  int front_hits;
  int back_hits;
};

/// Result of a ray cast with face orientation information.
struct RayHitResult {
  double distance;   // Distance to hit (infinity if no hit)
  bool is_back_face; // True if hit back face, false if front face
};

enum class Placement {
  Outside,
  Inside,
  Mixed,
};

/// Occlusion filter that counts all hits and classifies them as front/back
/// face. Rejects every hit (valid[0] = 0) to continue traversal through entire
/// ray path.
void count_all_hits_filter(const RTCFilterFunctionNArguments *args) {
  if (args->valid[0] != -1)
    return;

  // Cast context back to RayContext (ctx is first member, so addresses match)
  auto *ctx = reinterpret_cast<RayContext *>(args->context);

  // For N=1, access ray and hit via C-style cast (Embree uses RTCRayN/RTCHitN)
  auto *ray = reinterpret_cast<RTCRay *>(args->ray);
  auto *hit = reinterpret_cast<RTCHit *>(args->hit);

  // Compute dot product of ray direction with geometric normal
  // Negative = front face (normal points back toward origin)
  // Positive = back face (normal points away from origin)
  float dot =
      ray->dir_x * hit->Ng_x + ray->dir_y * hit->Ng_y + ray->dir_z * hit->Ng_z;

  if (dot < 0.0f) {
    ctx->front_hits++;
  } else {
    ctx->back_hits++;
  }

  // Reject hit to continue traversal
  args->valid[0] = 0;
}

/// RAII wrapper for Embree ray tracing against a mesh.
/// Used to detect window-mesh intersections and internal window placement.
class MeshRayTracer {
  RTCDevice device_;
  RTCScene scene_;

    public:
  explicit MeshRayTracer(const pcl::PolygonMesh &mesh) {
    // Initialize Embree device and scene
    device_ = rtcNewDevice("verbose=0");
    if (!device_) {
      throw std::runtime_error("Failed to create Embree device");
    }

    // Set error handler
    rtcSetDeviceErrorFunction(
        device_,
        [](void *, RTCError err, const char *str) {
          if (err != RTC_ERROR_NONE) {
            ReUseX::error("Embree error: {}", str ? str : "unknown");
          }
        },
        nullptr);

    scene_ = rtcNewScene(device_);
    if (!scene_) {
      rtcReleaseDevice(device_);
      throw std::runtime_error("Failed to create Embree scene");
    }

    rtcSetSceneFlags(scene_, RTC_SCENE_FLAG_ROBUST);
    rtcSetSceneBuildQuality(scene_, RTC_BUILD_QUALITY_MEDIUM);

    // Convert mesh to Embree geometry
    RTCGeometry geom = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);

    // Extract vertices
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, pcl_cloud);

    // Set vertices
    float *vertices = (float *)rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float),
        pcl_cloud.size());

    for (size_t i = 0; i < pcl_cloud.size(); ++i) {
      vertices[3 * i + 0] = pcl_cloud[i].x;
      vertices[3 * i + 1] = pcl_cloud[i].y;
      vertices[3 * i + 2] = pcl_cloud[i].z;
    }

    // Set triangles
    uint32_t *indices = (uint32_t *)rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(uint32_t),
        mesh.polygons.size());

    for (size_t i = 0; i < mesh.polygons.size(); ++i) {
      const auto &poly = mesh.polygons[i];
      if (poly.vertices.size() >= 3) {
        indices[3 * i + 0] = poly.vertices[0];
        indices[3 * i + 1] = poly.vertices[1];
        indices[3 * i + 2] = poly.vertices[2];
      }
    }

    // Enable filter functions passed via RTCOccludedArguments
    rtcSetGeometryEnableFilterFunctionFromArguments(geom, true);

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene_, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene_);

    ReUseX::debug(
        "Embree scene initialized with {} vertices, {} triangles",
        pcl_cloud.size(), mesh.polygons.size());
  }

  ~MeshRayTracer() {
    if (scene_)
      rtcReleaseScene(scene_);
    if (device_)
      rtcReleaseDevice(device_);
  }

  // Non-copyable
  MeshRayTracer(const MeshRayTracer &) = delete;
  MeshRayTracer &operator=(const MeshRayTracer &) = delete;

  /// Cast a ray and return hit information including face orientation.
  /// Face orientation is determined by the dot product of the ray direction
  /// with the geometric normal: positive dot = back face, negative = front
  /// face.
  RayHitResult cast_ray(const Eigen::Vector3d &origin,
                        const Eigen::Vector3d &direction) const {
    RTCRayHit rayhit;
    rayhit.ray.org_x = static_cast<float>(origin.x());
    rayhit.ray.org_y = static_cast<float>(origin.y());
    rayhit.ray.org_z = static_cast<float>(origin.z());
    rayhit.ray.dir_x = static_cast<float>(direction.x());
    rayhit.ray.dir_y = static_cast<float>(direction.y());
    rayhit.ray.dir_z = static_cast<float>(direction.z());
    rayhit.ray.tnear = 0.0f;
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.mask = 0xFFFFFFFF;
    rayhit.ray.flags = 0;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(scene_, &rayhit);

    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
      // Extract and normalize geometric normal from Embree
      Eigen::Vector3f normal(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z);
      normal.normalize();

      // Check face orientation: back face if normal points in same direction as
      // ray
      Eigen::Vector3f dir_vec(direction.x(), direction.y(), direction.z());
      bool is_back = normal.dot(dir_vec) > 0;

      return RayHitResult{static_cast<double>(rayhit.ray.tfar), is_back};
    }

    // No hit
    return RayHitResult{
        std::numeric_limits<double>::infinity(),
        false // Face orientation irrelevant when no hit
    };
  }

  /// Test if a point is inside the mesh using all-hit ray tracing.
  /// Based on DreamWorks method: casts a single ray and counts all
  /// intersections. Point is inside if more back-face hits than front-face
  /// hits. Uses Embree occlusion filter to count all hits in a single
  /// traversal. Requires closed mesh (guaranteed in this codebase).
  bool is_point_inside_mesh(const Eigen::Vector3d &point) const {
    // Initialize ray context for counting
    RayContext ray_ctx;
    rtcInitRayQueryContext(&ray_ctx.ctx);
    ray_ctx.front_hits = 0;
    ray_ctx.back_hits = 0;

    // Set up occlusion filter arguments
    RTCOccludedArguments oargs;
    rtcInitOccludedArguments(&oargs);
    oargs.context = &ray_ctx.ctx;
    oargs.filter = count_all_hits_filter;
    oargs.flags = RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER;

    // Cast a single ray in arbitrary direction
    // DreamWorks paper: any direction works for closed meshes
    RTCRay ray;
    ray.org_x = static_cast<float>(point.x());
    ray.org_y = static_cast<float>(point.y());
    ray.org_z = static_cast<float>(point.z());
    ray.dir_x = 1.0f; // +X axis
    ray.dir_y = 0.0f;
    ray.dir_z = 0.0f;
    ray.tnear = 0.0f;
    ray.tfar = std::numeric_limits<float>::infinity();
    ray.mask = 0xFFFFFFFF;
    ray.flags = 0;

    // Perform occlusion query with filter counting all hits
    rtcOccluded1(scene_, &ray, &oargs);

    // Inside if more back-face hits (rays from inside hit back faces on exit)
    ReUseX::trace(
        "Point-in-mesh test: {} back hits vs. {} front hits → {}",
        ray_ctx.back_hits, ray_ctx.front_hits,
        (ray_ctx.back_hits > ray_ctx.front_hits ? "INSIDE" : "OUTSIDE"));
    return ray_ctx.back_hits > ray_ctx.front_hits;
  }
};

// Classify window placement relative to mesh volume.
Placement classify_window_placement(const CoplanarPolygon &window_boundary,
                                    const MeshRayTracer &tracer) {
  if (window_boundary.vertices.empty()) {
    return Placement::Mixed; // Degenerate case, reject
  }

  int inside_count = 0;
  int outside_count = 0;

  for (const auto &v : window_boundary.vertices) {
    if (tracer.is_point_inside_mesh(v)) {
      inside_count++;
    } else {
      outside_count++;
    }
  }

  int total = window_boundary.vertices.size();

  if (inside_count == total) {
    // All vertices inside → internal window
    ReUseX::trace(
        "Window classification: internal ({}/{} vertices inside)", inside_count,
        total);
    return Placement::Inside;
  } else if (outside_count == total) {
    // All vertices outside → valid exterior window
    ReUseX::trace(
        "Window classification: exterior ({}/{} vertices outside)",
        outside_count, total);
    return Placement::Outside;
  } else {
    // Mixed → window intersects mesh boundary
    ReUseX::debug(
        "Window classification: intersection ({} inside, {} outside)",
        inside_count, outside_count);
    return Placement::Mixed;
  }
}

/// Test if a point lies within a wall's boundary polygon.
/// Uses winding number algorithm for robust point-in-polygon test.
bool point_in_wall_boundary(const Eigen::Vector3d &point,
                            const WallCandidate &wall) {
  // Project point onto wall plane
  double signed_dist = wall.normal.dot(point) + wall.plane[3];
  Eigen::Vector3d on_plane = point - signed_dist * wall.normal;

  // Create 2D coordinate system in wall plane
  Eigen::Vector3d u_axis, v_axis;
  Eigen::Vector3d z_up(0.0, 0.0, 1.0);

  // u-axis: perpendicular to normal in horizontal plane
  if (std::abs(wall.normal.z()) < 0.9) {
    u_axis = wall.normal.cross(z_up);
    u_axis.normalize();
  } else {
    // Nearly horizontal wall, use X axis
    u_axis = Eigen::Vector3d(1.0, 0.0, 0.0);
  }

  v_axis = u_axis.cross(wall.normal);
  v_axis.normalize();

  // Convert point and boundary to 2D
  Eigen::Vector3d delta = on_plane - wall.centroid;
  Eigen::Vector2d point_2d(delta.dot(u_axis), delta.dot(v_axis));

  // Use first (outer) boundary loop
  // TODO: Handle holes properly by checking all loops
  if (wall.boundary_loops.empty()) {
    return false;
  }

  const auto &boundary = wall.boundary_loops[0]; // Outer loop

  std::vector<Eigen::Vector2d> boundary_2d;
  boundary_2d.reserve(boundary.size());
  for (const auto &v : boundary) {
    Eigen::Vector3d v_delta = v - wall.centroid;
    boundary_2d.emplace_back(v_delta.dot(u_axis), v_delta.dot(v_axis));
  }

  if (boundary_2d.size() < 3) {
    return false; // Degenerate polygon
  }

  // Winding number algorithm (counts how many times polygon winds around point)
  int winding_number = 0;
  for (size_t i = 0; i < boundary_2d.size(); ++i) {
    const auto &v1 = boundary_2d[i];
    const auto &v2 = boundary_2d[(i + 1) % boundary_2d.size()];

    if (v1.y() <= point_2d.y()) {
      if (v2.y() > point_2d.y()) {
        // Upward crossing
        double cross = (v2.x() - v1.x()) * (point_2d.y() - v1.y()) -
                       (point_2d.x() - v1.x()) * (v2.y() - v1.y());
        if (cross > 0) {
          winding_number++;
        }
      }
    } else {
      if (v2.y() <= point_2d.y()) {
        // Downward crossing
        double cross = (v2.x() - v1.x()) * (point_2d.y() - v1.y()) -
                       (point_2d.x() - v1.x()) * (v2.y() - v1.y());
        if (cross < 0) {
          winding_number--;
        }
      }
    }
  }

  return winding_number != 0;
}

Eigen::Vector3d projectPointOnPlane(const Eigen::Vector3d &p,
                                    const Eigen::Vector4d &plane) {
  Eigen::Vector3d n = plane.head<3>();
  double d = plane.w();
  double signedDist = (n.dot(p) + d) / n.squaredNorm();
  Eigen::Vector3d projected_point = p - signedDist * n;

  return projected_point;
}

bool intersects_wallboundary(const CoplanarPolygon &polygon,
                             const WallCandidate &wall) {
  std::vector<Eigen::Vector3d> projected_points{};
  projected_points.reserve(polygon.vertices.size());

  for (auto const p : polygon.vertices)
    projected_points.push_back(projectPointOnPlane(p, wall.plane));

  // for (size_t i = 0; i < projected_points.size(); ++i) {
  //   auto p = projected_points[i];
  //   if (!point_in_wall_boundary(p, wall))
  //     polygon.vertices[i] = p;
  // }

  for (auto const p : projected_points) {
    if (!point_in_wall_boundary(p, wall))
      return true;
  }
  return false;
}

/// Compute distance from a point to a wall candidate, considering boundary.
/// If the point projects inside the wall boundary, returns perpendicular
/// distance to plane. If the point projects outside the boundary, returns
/// distance to the closest boundary point.
double distance_to_wall(const Eigen::Vector3d &point,
                        const WallCandidate &wall) {
  // // 1. Project point onto wall plane
  double signed_dist_to_plane = wall.plane.head<3>().dot(point) + wall.plane[3];
  Eigen::Vector3d projected_point = point - signed_dist_to_plane * wall.normal;

  // Eigen::Vector3d projected_point = projectPointOnPlane(point, wall.plane);

  // 2. Check if projected point is inside wall boundary
  if (point_in_wall_boundary(projected_point, wall)) {
    // Inside boundary: use perpendicular distance to plane
    return std::abs(signed_dist_to_plane);
  }

  // 3. Outside boundary: find closest point on boundary
  Eigen::Vector3d closest_boundary_point = projected_point;
  double min_boundary_dist_sq = std::numeric_limits<double>::infinity();

  // Check all boundary loops for closest point
  for (const auto &boundary : wall.boundary_loops) {
    for (size_t j = 0; j < boundary.size(); ++j) {
      const auto &v1 = boundary[j];
      const auto &v2 = boundary[(j + 1) % boundary.size()];

      // Closest point on edge segment [v1, v2] to projected_point
      Eigen::Vector3d edge = v2 - v1;
      Eigen::Vector3d to_point = projected_point - v1;
      double edge_len_sq = edge.squaredNorm();

      if (edge_len_sq < 1e-12) {
        // Degenerate edge, just use v1
        double dist_sq = (projected_point - v1).squaredNorm();
        if (dist_sq < min_boundary_dist_sq) {
          min_boundary_dist_sq = dist_sq;
          closest_boundary_point = v1;
        }
        continue;
      }

      // Parameter t for closest point on line
      double t = to_point.dot(edge) / edge_len_sq;
      t = std::clamp(t, 0.0, 1.0); // Clamp to segment

      Eigen::Vector3d closest_on_edge = v1 + t * edge;
      double dist_sq = (projected_point - closest_on_edge).squaredNorm();

      if (dist_sq < min_boundary_dist_sq) {
        min_boundary_dist_sq = dist_sq;
        closest_boundary_point = closest_on_edge;
      }
    }
  }

  // 4. Distance is 3D distance from point to closest boundary point
  return (point - closest_boundary_point).norm();
}

} // namespace

std::vector<WallCandidate>
extract_wall_candidates(const pcl::PolygonMesh &mesh, float normal_z_threshold,
                        float coplanarity_angle_deg) {

  // Early return if there is no geometry
  if (mesh.polygons.empty())
    return {};

  // Convert PCL mesh to CGAL Surface_mesh for region growing
  auto cgal_mesh = cgal::pcl_to_cgal_mesh(mesh);
  const size_t num_faces = cgal_mesh.number_of_faces();

  // Compute face normals manually to match original cross-product orientation
  auto fnormals = cgal_mesh
                      .add_property_map<cgal::Mesh::Face_index, cgal::Vector_3>(
                          "f:normals", cgal::Vector_3(0, 0, 0))
                      .first;

  for (auto fi : cgal_mesh.faces()) {
    // Get vertices in order for cross product  // Match original: (v1-v0) ×
    // (v2-v0)
    auto h = cgal_mesh.halfedge(fi);
    auto v0 = cgal_mesh.point(cgal_mesh.source(h));
    h = cgal_mesh.next(h);
    auto v1 = cgal_mesh.point(cgal_mesh.source(h));
    h = cgal_mesh.next(h);
    auto v2 = cgal_mesh.point(cgal_mesh.source(h));

    // Compute cross product: (v1-v0) × (v2-v0)
    auto e1 = v1 - v0;
    auto e2 = v2 - v0;
    auto normal = CGAL::cross_product(e1, e2);

    // Normalize
    auto len = std::sqrt(CGAL::to_double(normal.squared_length()));
    if (len > 1e-12) {
      normal = normal / len;
    }

    fnormals[fi] = normal;
  }

  // Set up CGAL region growing for planar segmentation
  using Region_type =
      CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<
          cgal::K, cgal::Mesh>;
  using Neighbor_query =
      CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<cgal::Mesh>;
  using Region_growing =
      CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

  Neighbor_query neighbor_query(cgal_mesh);
  Region_type region_type(
      cgal_mesh, CGAL::parameters::maximum_angle(coplanarity_angle_deg));

  Region_growing rg(faces(cgal_mesh), neighbor_query, region_type);

  // Detect planar regions
  std::vector<typename Region_growing::Primitive_and_region> regions;
  rg.detect(std::back_inserter(regions));

  // Convert each region to a WallCandidate
  std::vector<WallCandidate> candidates;
  candidates.reserve(regions.size());

  for (const auto &[plane_primitive, face_indices] : regions) {
    if (face_indices.empty())
      continue;

    // Compute area-weighted normal and centroid from face properties
    // (Using face normals ensures consistent orientation from mesh winding)
    Eigen::Vector3d mean_normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    double total_area = 0.0;

    for (auto fi : face_indices) {
      double area = CGAL::Polygon_mesh_processing::face_area(fi, cgal_mesh);

      // Get face normal (respects mesh winding order)
      auto fn = fnormals[fi];
      Eigen::Vector3d face_normal = cgal::cgal_to_eigen(fn);
      mean_normal += face_normal * area;

      // Compute face centroid as mean of vertices
      Eigen::Vector3d fc = Eigen::Vector3d::Zero();
      int vert_count = 0;
      for (auto vi :
           CGAL::vertices_around_face(cgal_mesh.halfedge(fi), cgal_mesh)) {
        fc += cgal::cgal_to_eigen(cgal_mesh.point(vi));
        vert_count++;
      }
      if (vert_count > 0) {
        fc /= static_cast<double>(vert_count);
      }

      centroid += fc * area;
      total_area += area;
    }

    if (total_area < 1e-12)
      continue;

    mean_normal.normalize();
    centroid /= total_area;

    // Build Hessian plane equation: n.dot(p) + d = 0
    Eigen::Vector4d plane;
    plane.head<3>() = mean_normal;
    plane[3] = -mean_normal.dot(centroid);

    // Extract boundary vertices using half-edge traversal
    // Boundary loops are ordered by connectivity, not vertex index
    std::set<cgal::Mesh::Face_index> region_set(face_indices.begin(),
                                                face_indices.end());

    // 1. Collect boundary half-edges
    std::unordered_set<cgal::Mesh::Halfedge_index> boundary_hes;
    for (auto fi : face_indices) {
      for (auto he :
           CGAL::halfedges_around_face(cgal_mesh.halfedge(fi), cgal_mesh)) {
        auto opp_he = cgal_mesh.opposite(he);
        auto opp_face = cgal_mesh.face(opp_he);

        // Boundary condition: opposite face outside region or at mesh boundary
        if (opp_face == cgal::Mesh::null_face() ||
            region_set.find(opp_face) == region_set.end()) {
          boundary_hes.insert(he);
        }
      }
    }

    // 2. Build connectivity map: source(he) -> he
    // This allows walking from vertex to vertex along boundary
    std::unordered_map<cgal::Mesh::Vertex_index, cgal::Mesh::Halfedge_index>
        next_from;
    for (auto he : boundary_hes) {
      next_from[cgal_mesh.source(he)] = he;
    }

    // 3. Walk boundary loops
    std::vector<std::vector<Eigen::Vector3d>> boundary_loops;
    std::unordered_set<cgal::Mesh::Halfedge_index> visited;

    for (auto start_he : boundary_hes) {
      if (visited.count(start_he))
        continue; // Already processed in another loop

      std::vector<Eigen::Vector3d> loop;
      auto he = start_he;

      // Walk the loop until we return to start
      do {
        visited.insert(he);

        // Add source vertex (maintains traversal order)
        loop.push_back(
            cgal::cgal_to_eigen(cgal_mesh.point(cgal_mesh.source(he))));

        // Find next half-edge in loop
        auto next_v = cgal_mesh.target(he);
        auto it = next_from.find(next_v);

        if (it == next_from.end()) {
          // Non-closed loop (shouldn't happen on manifold mesh)
          ReUseX::warn(
              "Non-closed boundary loop detected in wall region");
          break;
        }

        he = it->second;
      } while (he != start_he);

      if (loop.size() >= 3) { // Valid polygon
        boundary_loops.push_back(std::move(loop));
      }
    }

    // Convert face indices to int vector (for PCL mesh indexing)
    std::vector<int> face_idx_vec;
    face_idx_vec.reserve(face_indices.size());
    for (auto fi : face_indices) {
      face_idx_vec.push_back(static_cast<int>(fi));
    }

    WallCandidate wc;
    wc.plane = plane;
    wc.centroid = centroid;
    wc.normal = mean_normal;
    wc.boundary_loops = std::move(boundary_loops);
    wc.face_indices = std::move(face_idx_vec);
    candidates.push_back(std::move(wc));
  }

  ReUseX::debug("Extracted {} wall candidates from {} mesh faces using "
                      "CGAL region growing",
                      candidates.size(), num_faces);
  return candidates;
}

CreateWindowsResult
create_windows(CloudConstPtr cloud, CloudLConstPtr instance_labels,
               const std::map<uint32_t, uint32_t> &instance_to_semantic,
               const pcl::PolygonMesh &mesh,
               const std::vector<uint32_t> &window_semantic_labels,
               const CreateWindowsOptions &options) {

  CreateWindowsResult result;

  if (!cloud || !instance_labels || cloud->size() != instance_labels->size()) {
    ReUseX::error(
        "create_windows: invalid input (null or size mismatch)");
    return result;
  }

  // Extract wall candidates from mesh
  ReUseX::info("Extracting wall candidates from mesh...");
  auto walls = ReUseX::geometry::extract_wall_candidates(mesh);
  ReUseX::info("Found {} wall candidates", walls.size());

  if (walls.empty()) {
    ReUseX::warn("create_windows: no wall candidates provided");
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
    ReUseX::info(
        "create_windows: no window instances found for labels [{}]",
        fmt::join(window_semantic_labels, ", "));
    return result;
  }

  ReUseX::info(
      "Processing {} window instances against {} wall candidates",
      window_instance_ids.size(), walls.size());

  // Initialize Embree ray tracer for mesh intersection tests
  ReUseX::debug("Initializing Embree ray tracer for mesh validation...");
  MeshRayTracer tracer(mesh);

  int window_counter = 0;
  pcl::PCA<PointT> pca;
  pca.setInputCloud(cloud);

  for (uint32_t inst_id : window_instance_ids) {

    pcl::PointIndices::Ptr indices(new pcl::PointIndices());
    for (size_t i = 0; i < instance_labels->size(); ++i)
      if ((*instance_labels)[i].label == inst_id)
        indices->indices.push_back(i);

    if (indices->indices.empty())
      continue;

    pca.setIndices(indices);

    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

    // TODO:: Step that need to be done
    // 3. Compare to plane normals

    Eigen::Vector3d inst_centroid = pca.getMean().head<3>().cast<double>();
    Eigen::Vector3d inst_normal = pca.getEigenVectors().col(2).cast<double>();

    ReUseX::debug(
        "Instance {}: {} points, centroid ({:.2f},{:.2f},{:.2f})", inst_id,
        indices->indices.size(), inst_centroid.x(), inst_centroid.y(),
        inst_centroid.z());

    // Find best matching wall: minimize cost = w_angle * angle + w_dist *
    // distance
    int best_wall = -1;
    double best_cost = std::numeric_limits<double>::max();
    const double w_angle = 1.0; // Weight for angular difference (radians)
    const double w_dist = 0.5;  // Weight for spatial distance (meters)

    // // Adaptive angular tolerance based on cluster size
    // // Base tolerance increased to 40° to handle noisy PCA from sensor data
    double base_tolerance = 40.0 * M_PI / 180.0; // 40 degrees
    double max_angle = base_tolerance;

    if (indices->indices.size() < 100) {
      // Medium/small clusters: relax tolerance proportionally
      double scale =
          1.0 + (100.0 - indices->indices.size()) / 100.0; // 1.0-2.0 range
      max_angle =
          std::min(base_tolerance * scale, 60.0 * M_PI / 180.0); // Max 60°

      ReUseX::debug("Instance {} ({} points): using tolerance {:.1f}°",
                          inst_id, indices->indices.size(),
                          max_angle * 180.0 / M_PI);
    }

    int candidates_checked = 0;
    int candidates_angle_rejected = 0;
    double min_angle_seen = std::numeric_limits<double>::max();

    for (size_t wi = 0; wi < walls.size(); ++wi) {
      // Angular alignment: dot product → angle
      double dot = std::abs(inst_normal.dot(walls[wi].normal));
      dot = std::clamp(dot, -1.0, 1.0);
      double angle_diff = std::acos(dot);

      candidates_checked++;
      min_angle_seen = std::min(min_angle_seen, angle_diff);

      // Skip walls with poor angular alignment
      if (angle_diff > max_angle) {
        candidates_angle_rejected++;
        continue;
      }

      // Spatial distance: centroid to wall (considering boundary)
      double dist = distance_to_wall(inst_centroid, walls[wi]);

      //// Bounding box check (penalize out-of-bbox)
      // const auto &bb = wall_bboxes[wi];
      // if ((inst_centroid.array() < bb.min_pt.array()).any() ||
      //     (inst_centroid.array() > bb.max_pt.array()).any()) {
      //   dist += 1.0; // 1m penalty
      // }

      // Combined cost
      double cost = w_angle * angle_diff + w_dist * dist;

      ReUseX::trace("  Wall {}: angle={:.1f}°, dist={:.2f}m, cost={:.3f}",
                          wi, angle_diff * 180.0 / M_PI, dist, cost);

      if (cost < best_cost) {
        best_cost = cost;
        best_wall = static_cast<int>(wi);
      }
    }

    // if (best_wall < 0) {
    //   ReUseX::warn(
    //       "No suitable wall found for instance {} (checked {} walls, "
    //       "{} rejected by angle, min angle {:.1f}°, tolerance {:.1f}°)",
    //       inst_id, candidates_checked, candidates_angle_rejected,
    //       min_angle_seen * 180.0 / M_PI, max_angle * 180.0 / M_PI);
    //   result.unmatched_instances.push_back(static_cast<int>(inst_id));
    //   continue;
    // }

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
    projected.reserve(indices->indices.size());
    for (const auto &i : indices->indices) {
      auto p = cloud->points[i].getVector3fMap().cast<double>();
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
      ReUseX::warn(
          "Instance {} produced degenerate boundary ({} vertices)", inst_id,
          boundary_3d.size());
      // result.unmatched_instances.push_back(static_cast<int>(inst_id));
      continue;
    }

    // Offset along wall normal
    Eigen::Vector3d offset =
        wall.normal * static_cast<double>(options.wall_offset);
    for (auto &v : boundary_3d)
      v += offset;

    // Build polygon with offset plane
    polygon.vertices = std::move(boundary_3d);
    // Offset the plane constant: d' = d - offset_distance
    polygon.plane = wall.plane;
    polygon.plane[3] -= static_cast<double>(options.wall_offset);

    // INFO: Filters
    // Filter 1: Classify window placement (inside/outside/intersection)
    auto placement = classify_window_placement(polygon, tracer);
    switch (placement) {
    case Placement::Mixed:
      if (options.include_internal) {
        ReUseX::debug("Window instance {} is internal but "
                            "include_internal is true, keeping",
                            inst_id);
        break;
      }
      [[fallthrough]];
    case Placement::Inside:
      ReUseX::debug(
          "Window instance   {} is internal or mixed, dropping ", inst_id);
      // result.unmatched_instances.push_back(static_cast<int>(inst_id));
      continue;

    case Placement::Outside:
      // Valid we continue
      break;
    }

    // TODO:: Add more check if window  candidate boundary is fully withing the
    // wall boundary
    if (intersects_wallboundary(polygon, wall)) {
      ReUseX::trace(
          "Window instance {} rejected, intersects wall boundary", inst_id);
      continue;
    }

    // Build BuildingComponent
    BuildingComponent comp;
    comp.name = fmt::format("window_{}", ++window_counter);
    // comp.type = ComponentType::window;
    comp.type = placement == Placement::Outside ? ComponentType::window
                                                : ComponentType::door;
    comp.boundary = std::move(polygon);
    comp.confidence = -1.0; // auto-detected, no ML confidence
    comp.data = WindowData{};

    ReUseX::debug("Created {} from instance {}, match cost {:.3f})", comp,
                        inst_id, best_cost);

    result.components.push_back(std::move(comp));
  }

  ReUseX::info("Created {} window components ({} unmatched instances)",
                     result.components.size(),
                     result.unmatched_instances.size());
  return result;
}

} // namespace ReUseX::geometry
