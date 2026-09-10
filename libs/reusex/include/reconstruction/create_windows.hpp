// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/geometry/BuildingComponent.hpp"
#include "reusex/types.hpp"

#include <Eigen/Core>
#include <pcl/PolygonMesh.h>

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace reusex::geometry {

/// A connected component of coplanar mesh faces that may host a window.
///
/// Orientation is *not* implied: by default every planar region qualifies,
/// floors and ceilings included, so skylights find a host surface. Restrict to
/// near-vertical walls with `CreateWindowsOptions::wall_normal_z_threshold`.
struct WallCandidate {
  Eigen::Vector4d plane;    ///< Hessian normal form [a,b,c,d]: ax+by+cz+d=0
  Eigen::Vector3d centroid; ///< Area-weighted centroid of component faces
  Eigen::Vector3d normal;   ///< Unit outward normal of the component

  /// Boundary loops ordered by half-edge traversal (outer loop first, then
  /// holes) Each loop is a sequence of vertices forming a closed polygon. First
  /// loop (boundary_loops[0]) is the outer boundary. Additional loops represent
  /// holes (e.g., embedded windows/doors).
  std::vector<std::vector<Eigen::Vector3d>> boundary_loops;

  std::vector<int> face_indices; ///< Mesh face indices in this component
};

/// How to compute the window boundary polygon.
enum class WindowBoundaryMode { rectangle, polyline };

/// Verticality-gate threshold at which the gate is off (#326).
///
/// The gate keeps a region when `|mean_normal.z| < normal_z_threshold`. Since
/// every unit normal has `|n.z| <= 1`, this value admits every orientation —
/// and it is treated as an exact opt-out so perfectly horizontal faces
/// (`|n.z| == 1`) are kept too.
inline constexpr float kWallVerticalityGateOff = 1.0f;

/// Configuration for the create_windows pipeline.
struct CreateWindowsOptions {
  WindowBoundaryMode mode = WindowBoundaryMode::rectangle;
  float wall_offset = 0.5f; ///< Offset along outward wall normal (meters)
  float alpha = 0.5f;       ///< ConcaveHull alpha for polyline mode
  /// Verticality gate on wall candidates: a coplanar mesh region is kept only
  /// when `|normal.z| < wall_normal_z_threshold`. The default
  /// `kWallVerticalityGateOff` accepts every orientation, which is what window
  /// detection wants — skylights and tilted roof glazing need their
  /// horizontal/oblique host surfaces (#326). Lower it (e.g. `0.3`) to
  /// restrict windows to near-vertical walls.
  float wall_normal_z_threshold = kWallVerticalityGateOff;
  float coplanarity_angle_deg =
      10.0f; ///< Max angle deviation within wall component
  bool include_internal =
      false; ///< Include windows inside mesh volume (default: false)
};

/// Output of create_windows().
struct CreateWindowsResult {
  std::vector<BuildingComponent> components;
  std::vector<int> unmatched_instances; ///< Instance IDs with no wall found
};

/// Extract planar wall candidates from a triangle mesh.
///
/// Decomposes the mesh into connected components of coplanar faces. Each
/// component becomes a WallCandidate with a fitted plane, centroid, outward
/// normal, and boundary vertices.
///
/// @param normal_z_threshold Verticality gate: a region is kept only when
///   `|mean_normal.z| < normal_z_threshold`. The default
///   `kWallVerticalityGateOff` keeps every orientation (floors, ceilings and
///   tilted roofs included) so skylight windows still find a host surface;
///   pass e.g. `0.3` to keep near-vertical walls only (#326).
/// @param coplanarity_angle_deg Maximum angular deviation inside one region.
std::vector<WallCandidate>
extract_wall_candidates(const pcl::PolygonMesh &mesh,
                        float normal_z_threshold = kWallVerticalityGateOff,
                        float coplanarity_angle_deg = 10.0f);

/// Resolves the stable GUID of an instance from its integer label id.
/// Returns an empty string when no GUID is available (unknown/legacy). The
/// caller (CLI) wires this to ProjectDB::instance_guid once per-instance
/// identity exists; the library keeps no dependency on the database. See #211.
using ResolveInstanceGuidFn = std::function<std::string(uint32_t)>;

/// Create window BuildingComponents from instance-labeled points and wall
/// geometry.
///
/// For each window instance, projects its points onto the nearest wall plane,
/// computes a boundary polygon (AABB or concave hull), and offsets it along
/// the outward wall normal. Performs validation to filter out windows that
/// intersect the mesh, are out of bounds, or are internal (optional).
///
/// \param resolve_instance_guid Optional callback mapping an instance label id
///        to its stable GUID; the result is stored on each component's
///        source_instance_guid for provenance (issue #211). Defaults to a
///        function returning empty (no provenance).
CreateWindowsResult
create_windows(CloudConstPtr cloud, CloudLConstPtr instance_labels,
               const std::map<uint32_t, uint32_t> &instance_to_semantic,
               const pcl::PolygonMesh &mesh,
               const std::vector<uint32_t> &window_semantic_labels,
               const CreateWindowsOptions &options = {},
               const ResolveInstanceGuidFn &resolve_instance_guid = {});

} // namespace reusex::geometry
