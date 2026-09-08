// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/core/SensorIntrinsics.hpp"
#include "reusex/types.hpp"

#include <opencv2/core/mat.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <rtabmap/core/DBDriver.h>

#include <filesystem>
#include <map>

namespace reusex::geometry {

/// Prepare the directory the generated texture images are staged in.
///
/// The library never writes to — or deletes from — the process working
/// directory (issue #245):
///
/// - `requested` empty (the default): a fresh, uniquely named directory is
///   created under `std::filesystem::temp_directory_path()`. Because it is
///   created here it is guaranteed not to have existed before, so no caller
///   data can be clobbered. Concurrent `rux create texture` runs therefore
///   never collide.
/// - `requested` non-empty: the path is made absolute and created if missing.
///   Existing content is left **untouched** — a caller-supplied directory is
///   never cleared, so the caller owns its lifetime.
///
/// @returns an absolute path to an existing, writable directory.
/// @throws std::runtime_error if the directory cannot be created.
std::filesystem::path
prepare_texture_dir(const std::filesystem::path &requested = {});

/// Camera data for texture mapping
struct CameraData {
  cv::Mat image;                     ///< Color image
  core::SensorIntrinsics intrinsics; ///< Camera intrinsics
  Eigen::Matrix4d pose;              ///< World pose (SE(3))
};

/// Quality parameters for texture projection
struct TextureQualityParams {
  float texels_per_meter = 400.0f; ///< Target texture detail (pixels per meter)
                                   ///< - adaptive resolution
  int min_resolution = 256;        ///< Minimum texture size (small surfaces)
  int max_resolution = 4096;       ///< Maximum texture size (large surfaces)
  int atlas_tile_size =
      2048; ///< Atlas tile size for PCL visualization (lower = less memory)
  float distance_threshold = 0.02f; ///< Max distance from point to surface
                                    ///< (meters) - smaller = sharper
  float search_radius = 0.04f;      ///< K-d tree search radius (meters)
  int max_neighbors = 100;          ///< Max points to check per pixel
  bool use_quadratic_falloff =
      true; ///< Use 1/d^2 instead of 1/d for sharper detail
  /// Directory the generated texture images are written to. Empty (the
  /// default) means "stage into a unique directory under the system temp
  /// directory" — see prepare_texture_dir(). A caller-supplied directory is
  /// created if missing but never cleared, and the caller owns its lifetime.
  std::filesystem::path texture_dir{};
};

pcl::TextureMesh::Ptr texture_mesh_with_cloud(
    pcl::PolygonMesh::Ptr mesh, CloudConstPtr cloud,
    CloudNConstPtr normals = nullptr, bool debug_distinct_colors = false,
    const TextureQualityParams &quality = TextureQualityParams());

/// Texture mesh using RTABMap signatures (legacy API)
/// @param texture_dir staging directory for the texture images; see
///        prepare_texture_dir() for the empty-default semantics.
pcl::TextureMesh::Ptr
texture_mesh(pcl::PolygonMesh::Ptr mesh,
             std::map<int, rtabmap::Transform> const &poses,
             std::map<int, rtabmap::Signature> const &nodes,
             const std::filesystem::path &texture_dir = {});

/// Texture mesh using simple camera data (ProjectDB API)
/// @param texture_dir staging directory for the texture images; see
///        prepare_texture_dir() for the empty-default semantics.
pcl::TextureMesh::Ptr
texture_mesh(pcl::PolygonMesh::Ptr mesh,
             std::map<int, CameraData> const &cameras,
             const std::filesystem::path &texture_dir = {});
} // namespace reusex::geometry
