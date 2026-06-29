// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/types.hpp"

#include <Eigen/Geometry>

namespace reusex::geometry {

/// A sensor frame represented as a surfel set for joint registration.
///
/// Points and normals are stored in the camera OPTICAL frame; the current
/// world pose maps them to world coordinates:
///   world_pt = world_pose * optical_pt
/// The registration optimizer refines `world_pose` while the points/normals
/// stay fixed in the optical frame. `world_pose` is the composition
/// worldTf * localTf, i.e. the seed equals to_affine(pose) *
/// to_affine(intrinsics.local_transform).
struct FrameSurfels {
  int node_id = -1;
  CloudPtr points;   ///< pcl::PointXYZRGB in optical frame
  CloudNPtr normals; ///< pcl::Normal in optical frame (unit, facing camera)
  Eigen::Affine3f world_pose = Eigen::Affine3f::Identity();
};

} // namespace reusex::geometry
