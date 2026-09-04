// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Database glue for the plane-landmark pose-graph back-end: extract surfels
// from a ProjectDB, run the pure in-memory optimizer, and write optimized
// poses back. Kept separate from PlaneGraphOptimizer.cpp so the solver core
// stays free of any database dependency (mirrors refine_sensor_poses.cpp).

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "geometry/transform_utils.hpp"
#include "slam/PlaneGraphOptimizer.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

namespace reusex::geometry {

PlaneGraphResult optimize_sensor_poses(ProjectDB &db,
                                       const PlaneGraphOptions &options,
                                       bool dry_run) {
  auto frameIds = db.sensor_frame_ids();
  std::sort(frameIds.begin(), frameIds.end()); // ensure temporal ordering

  std::vector<FrameSurfels> frames;
  std::vector<core::SensorIntrinsics> intrinsics; // parallel to `frames`
  frames.reserve(frameIds.size());
  for (int id : frameIds) {
    auto fs = extract_frame_surfels(db, id, options.surfel);
    if (!fs)
      continue;
    intrinsics.push_back(db.sensor_frame_intrinsics(id));
    frames.push_back(std::move(*fs));
  }

  if (frames.size() < 2)
    throw std::runtime_error(
        "PlaneGraph: fewer than 2 usable sensor frames with depth/pose");

  // P2: assemble wide-baseline loop edges (before the optimizer mutates the
  // poses) and feed them into the same GNC graph. Indices refer to positions in
  // `frames`, so the parallel node-id and seed-pose vectors are built in the
  // same order. Two independent sources, unioned:
  //   (a) internally-detected ORB edges (--loop-closure), and
  //   (b) externally-computed edges from a JSON file (--loop-edges) produced by
  //       a learned matcher / offline oracle — the license-clean bridge.
  std::vector<LoopEdge> loop_edges;
  const bool want_loops =
      options.loop_closure.enable || !options.loop_edges_file.empty();
  if (want_loops) {
    std::vector<int> node_ids;
    std::vector<Eigen::Matrix4d> seed_poses;
    node_ids.reserve(frames.size());
    seed_poses.reserve(frames.size());
    for (const auto &f : frames) {
      node_ids.push_back(f.node_id);
      seed_poses.push_back(f.world_pose.matrix().cast<double>());
    }

    if (options.loop_closure.enable) {
      LoopClosureResult lc;
      loop_edges = detect_loop_edges(db, node_ids, seed_poses,
                                     options.loop_closure, &lc);
      core::info("PlaneGraph: loop closure proposed {} edges from {} matched "
                 "candidates",
                 lc.edges, lc.candidates);
    }

    if (!options.loop_edges_file.empty()) {
      LoopClosureResult ext;
      auto file_edges =
          load_loop_edges(options.loop_edges_file, node_ids, &ext);
      core::info("PlaneGraph: {} external loop edges loaded from '{}'",
                 ext.edges, options.loop_edges_file);

      // Seed-disagreement gate for EXTERNAL edges (the internal
      // detect_loop_edges path already applies this via
      // LoopClosureOptions::min_seed_disagreement; the file path must too or it
      // is a footgun). An edge whose relative translation already AGREES with
      // the seed carries no drift-correction information — imposing its noisier
      // matcher+depth estimate on already-correct poses only adds noise
      // (measured: honka laser-GT F 0.7958 -> 0.62 when 1336 redundant edges
      // were applied ungated). Dropping them makes the bridge a no-op on a
      // well-posed scan while keeping every large-disagreement
      // (drift-correcting) edge. <= 0 disables the gate.
      const double gate = options.loop_edges_min_seed_disagreement;
      int dropped = 0;
      for (auto &e : file_edges) {
        if (gate > 0.0 && e.i >= 0 && e.j >= 0) {
          const Eigen::Matrix4d rel_seed =
              seed_poses[e.i].inverse() * seed_poses[e.j];
          const double disagreement =
              (rel_seed.block<3, 1>(0, 3) - e.T_ij.block<3, 1>(0, 3)).norm();
          if (disagreement < gate) {
            ++dropped;
            continue;
          }
        }
        loop_edges.push_back(std::move(e));
      }
      if (dropped > 0)
        core::info("PlaneGraph: dropped {} external loop edges that agree with "
                   "the seed within {:.3f} m (non-informative); {} kept",
                   dropped, gate,
                   static_cast<int>(file_edges.size()) - dropped);
    }
  }

  PlaneGraphOptimizer optimizer(options);
  PlaneGraphResult result = optimizer.optimize(frames, loop_edges);

  if (dry_run) {
    core::info("PlaneGraph dry-run: poses NOT written back");
    return result;
  }

  if (result.landmarks == 0 && result.loop_edges == 0) {
    core::warn(
        "PlaneGraph: no landmarks or loop edges — poses NOT written back");
    return result;
  }

  if (!result.converged) {
    core::error("PlaneGraph: optimization did not converge — poses NOT "
                "written back (docs/STANDARDS.md §5: no silent failure)");
    return result;
  }

  // Write optimized poses back. The stored `transform` column is worldTf, so we
  // remove the constant local (optical->sensor) transform that was folded into
  // world_pose during extraction:  worldTf = world_pose * localTf^-1.
  int written = 0;
  for (size_t k = 0; k < frames.size(); ++k) {
    const Eigen::Affine3f localTf = to_affine(intrinsics[k].local_transform);
    const Eigen::Affine3f worldTf = frames[k].world_pose * localTf.inverse();
    db.update_sensor_frame_pose(frames[k].node_id, to_array16(worldTf));
    ++written;
  }
  core::info("PlaneGraph: wrote {} optimized poses back to database", written);
  return result;
}

} // namespace reusex::geometry
