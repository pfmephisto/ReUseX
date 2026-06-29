// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Database glue for Joint Pairwise Registration: extract surfels from a
// ProjectDB, run the pure in-memory solver, and write refined poses back.
// Kept separate from JointPairwiseRegistration.cpp so the solver core stays
// free of any database dependency.

#include "geometry/registration/JointPairwiseRegistration.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "geometry/transform_utils.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

namespace reusex::geometry {

JprResult refine_sensor_poses(ProjectDB &db, const JprParams &params,
                              bool dry_run) {
  auto frameIds = db.sensor_frame_ids();
  std::sort(frameIds.begin(), frameIds.end()); // ensure temporal ordering

  std::vector<FrameSurfels> frames;
  std::vector<core::SensorIntrinsics> intrinsics; // parallel to `frames`
  frames.reserve(frameIds.size());
  for (int id : frameIds) {
    auto fs = extract_frame_surfels(db, id, params.surfel);
    if (!fs)
      continue;
    intrinsics.push_back(db.sensor_frame_intrinsics(id));
    frames.push_back(std::move(*fs));
  }

  if (frames.size() < 2)
    throw std::runtime_error(
        "JPR: fewer than 2 usable sensor frames with depth/pose");

  JointPairwiseRegistration jpr(params);
  JprResult result = jpr.refine(frames);

  if (dry_run) {
    core::info("JPR dry-run: poses NOT written back");
    return result;
  }

  // Write refined poses back. The stored `transform` column is worldTf, so we
  // remove the constant local (optical->sensor) transform that was folded into
  // world_pose during extraction:  worldTf = world_pose * localTf^-1.
  int written = 0;
  for (size_t k = 0; k < frames.size(); ++k) {
    const Eigen::Affine3f localTf = to_affine(intrinsics[k].local_transform);
    const Eigen::Affine3f worldTf = frames[k].world_pose * localTf.inverse();
    db.update_sensor_frame_pose(frames[k].node_id, to_array16(worldTf));
    ++written;
  }
  core::info("JPR: wrote {} refined poses back to database", written);
  return result;
}

} // namespace reusex::geometry
