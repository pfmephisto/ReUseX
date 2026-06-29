// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/geometry/Surfel.hpp"

#include <optional>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// Parameters controlling per-frame surfel extraction (back-projection +
/// normal estimation) used by the registration pipeline.
struct SurfelExtractionParams {
  float min_distance = 0.0f;     ///< minimum depth in meters
  float max_distance = 4.0f;     ///< maximum depth in meters
  int sampling_factor = 8;       ///< per-pixel subsampling (1=all, 2=half, ...)
  int confidence_threshold = 2;  ///< minimum confidence value
  float normal_radius = 0.1f;    ///< radius for normal estimation (m)
  float voxel_size = 0.03f;      ///< per-frame voxel downsample (m); <=0 disables
  bool apply_depth_filters = true;
};

/// Back-project one sensor frame into an optical-frame surfel set and seed its
/// world pose from the stored transform and local (optical->sensor) transform.
///
/// Points and normals are returned in the camera optical frame; normals are
/// estimated with the viewpoint at the optical-frame origin so they face the
/// camera. The returned `world_pose` equals worldTf * localTf, so that
/// world_pt = world_pose * optical_pt reproduces the reconstruction geometry.
///
/// @returns std::nullopt when the frame lacks usable color/depth, has invalid
///          intrinsics, or yields too few points for normal estimation.
std::optional<FrameSurfels>
extract_frame_surfels(ProjectDB &db, int node_id,
                      const SurfelExtractionParams &params);

} // namespace reusex::geometry
