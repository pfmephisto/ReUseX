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
#include <set>
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
  // The union is then de-duplicated per (i,j) pair and, when PCM is enabled,
  // consistency-filtered AS A WHOLE, so the file path gets the same
  // false-positive rejection the internal path has.
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

      // Cross-source dedup: an (i,j) pair already constrained by an internally
      // detected ORB edge must not receive a SECOND BetweenFactor from the
      // file. Two factors on one pair multiply the information, i.e. divide the
      // effective sigma by sqrt(2) — an unintended confidence boost on exactly
      // the pairs both front-ends happened to agree on. The internal edge wins:
      // it carries a verified RANSAC inlier count from this scan's own depth.
      std::set<std::pair<int, int>> internal_pairs;
      for (const auto &e : loop_edges)
        internal_pairs.emplace(std::min(e.i, e.j), std::max(e.i, e.j));

      // (i,j) keys of the external edges that survive the gates, so PCM's
      // verdict on the file's contribution can be reported separately below.
      std::set<std::pair<int, int>> external_pairs;

      int dropped = 0, dropped_dup = 0;
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
        const auto key = std::make_pair(std::min(e.i, e.j), std::max(e.i, e.j));
        if (internal_pairs.count(key) > 0) {
          ++dropped_dup;
          continue;
        }
        external_pairs.insert(key);
        loop_edges.push_back(std::move(e));
      }
      if (dropped > 0)
        core::info("PlaneGraph: dropped {} external loop edges that agree with "
                   "the seed within {:.3f} m (non-informative)",
                   dropped, gate);
      if (dropped_dup > 0)
        core::info("PlaneGraph: dropped {} external loop edges duplicating an "
                   "internally-detected pair (avoids double factors)",
                   dropped_dup);
      core::info("PlaneGraph: {} external loop edges kept after gating",
                 external_pairs.size());

      // Pairwise Consistency Maximization over the UNION. detect_loop_edges
      // PCM-filters its own output, but consistency is a property of the whole
      // edge set: without this, an aliasing false positive from the file would
      // reach the graph unfiltered — worst of all under --loop-trust, which
      // hands loop edges a far more generous GNC-TLS threshold. Gated on the
      // same options.pcm flag (`--loop-no-pcm`) so one switch governs both
      // sources. Skipped when the file contributed nothing, since the internal
      // set is already filtered and re-running would only repeat the work.
      if (options.loop_closure.pcm && !external_pairs.empty() &&
          loop_edges.size() > 2) {
        const size_t before = loop_edges.size();
        loop_edges = filter_consistent_loop_edges(
            std::move(loop_edges), seed_poses, options.loop_closure);
        size_t external_kept = 0;
        for (const auto &e : loop_edges)
          if (external_pairs.count({std::min(e.i, e.j), std::max(e.i, e.j)}) >
              0)
            ++external_kept;
        core::info("PlaneGraph: PCM kept {} of {} unioned loop edges; "
                   "{} of {} external edges rejected as inconsistent",
                   loop_edges.size(), before,
                   external_pairs.size() - external_kept,
                   external_pairs.size());
      } else if (!options.loop_closure.pcm && !external_pairs.empty()) {
        core::warn(
            "PlaneGraph: PCM disabled (--loop-no-pcm) — {} external loop "
            "edges enter the graph WITHOUT a consistency filter",
            external_pairs.size());
      }
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
