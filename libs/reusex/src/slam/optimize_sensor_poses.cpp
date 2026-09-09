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
  // same order. Three independent sources, unioned:
  //   (a) internally-detected ORB frame-pair edges (--loop-closure),
  //   (b) panorama-derived edges (--use-panoramas, issue #236), where one 360
  //       panorama is resected independently against each frame it matches, and
  //   (c) externally-computed edges from a JSON file (--loop-edges) produced by
  //       a learned matcher / offline oracle — the license-clean bridge.
  // The union is then de-duplicated per (i,j) pair and, when PCM is enabled,
  // consistency-filtered AS A WHOLE, so every source gets the same
  // false-positive rejection the internal path has.
  std::vector<LoopEdge> loop_edges;
  // Panorama edges (#236) are a THIRD source, unioned with the other two. They
  // are counted separately so a run's panorama contribution stays auditable
  // after PCM has filtered the union.
  std::set<std::pair<int, int>> panorama_pairs;
  bool pcm_done = false; // did a branch already PCM the union?
  const bool want_loops = options.loop_closure.enable ||
                          !options.loop_edges_file.empty() ||
                          options.panorama_loops.enable;
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

    // --- (c) panorama-derived edges (#236) ---------------------------------
    // A 360 panorama matches frames in every direction at once, so ONE
    // panorama ties a whole group of temporally distant frames together. The
    // measurement is built from per-frame resections in each frame's own
    // optical coordinates, so — unlike anything derived from the aligned
    // panorama pose — it does not merely restate the drifted seed trajectory.
    // The seed-disagreement and min_frame_gap gates are applied inside
    // detect_panorama_loop_edges from these same LoopClosureOptions.
    if (options.panorama_loops.enable) {
      PanoramaLoopResult pl;
      // #339: resolve the scale-relative gate here and hand
      // detect_panorama_loop_edges the already-absolute value, so all three
      // edge sources are judged against one gate computed from one extent.
      LoopClosureOptions pano_gates = options.loop_closure;
      pano_gates.min_seed_disagreement = static_cast<float>(
          seed_disagreement_gate(trajectory_extent(seed_poses),
                                 pano_gates.min_seed_disagreement_fraction,
                                 pano_gates.min_seed_disagreement));
      pano_gates.min_seed_disagreement_fraction = 0.0f;
      auto pano_edges = detect_panorama_loop_edges(
          db, node_ids, seed_poses, options.panorama_loops, pano_gates, &pl);

      if (pl.panoramas == 0) {
        // The user explicitly asked for panorama edges on a project that has
        // none. Not fatal — the graph is still solvable from planes/odometry —
        // but it must be loud, with the numbers (docs/STANDARDS.md §5).
        core::warn("PlaneGraph: --use-panoramas requested but this project "
                   "contains 0 panoramic images ({} sensor frames); no "
                   "panorama loop edges will be added. Import 360 imagery "
                   "with 'rux import 360' first.",
                   frames.size());
      }

      // Cross-source dedup: a pair already constrained by an internally
      // detected ORB edge must not receive a SECOND BetweenFactor (two factors
      // on one pair multiply the information). The ORB edge wins: it is a
      // direct frame-to-frame RANSAC on this scan's own depth, whereas a
      // panorama edge chains two independent resections.
      std::set<std::pair<int, int>> existing;
      for (const auto &e : loop_edges)
        existing.emplace(std::min(e.i, e.j), std::max(e.i, e.j));

      int dropped_dup = 0;
      for (auto &e : pano_edges) {
        const auto key = std::make_pair(std::min(e.i, e.j), std::max(e.i, e.j));
        if (existing.count(key) > 0) {
          ++dropped_dup;
          continue;
        }
        panorama_pairs.insert(key);
        loop_edges.push_back(std::move(e));
      }
      if (dropped_dup > 0)
        core::info("PlaneGraph: dropped {} panorama loop edges duplicating an "
                   "internally-detected pair (avoids double factors)",
                   dropped_dup);
      core::info("PlaneGraph: {} panorama loop edges kept from {}/{} panoramas "
                 "({} frame resections, {} pairs proposed; dropped {} gap / "
                 "{} seed-gate / {} per-panorama cap)",
                 panorama_pairs.size(), pl.panoramas_matched, pl.panoramas,
                 pl.frames_resected, pl.proposed, pl.dropped_gap,
                 pl.dropped_seed_gate, pl.dropped_cap);
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
      // (drift-correcting) edge. 0 disables the gate.
      //
      // #339: the gate is resolved against THIS capture's extent rather than
      // being a constant in metres, because the disagreement it is trying to
      // separate from noise (drift) grows with the capture and the noise does
      // not.
      const double extent = trajectory_extent(seed_poses);
      const double gate = seed_disagreement_gate(
          extent, options.loop_edges_min_seed_disagreement_fraction,
          options.loop_edges_min_seed_disagreement);
      core::info("PlaneGraph: trajectory extent {:.2f} m -> external "
                 "seed-disagreement gate {:.3f} m (fraction {:.5f}, floor "
                 "{:.3f} m)",
                 extent, gate,
                 options.loop_edges_min_seed_disagreement_fraction,
                 options.loop_edges_min_seed_disagreement);

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
        pcm_done = true;
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

    // Pairwise Consistency Maximization over the UNION for the panorama source.
    // detect_loop_edges PCM-filters its own output and the file branch above
    // PCMs the union when a file contributed, but consistency is a property of
    // the WHOLE edge set: without this, panorama edges would reach the graph
    // with no consistency filter whenever no --loop-edges file was given.
    // Governed by the same options.pcm flag (`--loop-no-pcm`) as every other
    // source, so one switch means one thing.
    if (!panorama_pairs.empty() && !pcm_done && loop_edges.size() > 2) {
      if (options.loop_closure.pcm) {
        const size_t before = loop_edges.size();
        loop_edges = filter_consistent_loop_edges(
            std::move(loop_edges), seed_poses, options.loop_closure);
        pcm_done = true;
        size_t pano_kept = 0;
        for (const auto &e : loop_edges)
          if (panorama_pairs.count({std::min(e.i, e.j), std::max(e.i, e.j)}) >
              0)
            ++pano_kept;
        core::info("PlaneGraph: PCM kept {} of {} unioned loop edges; "
                   "{} of {} panorama edges rejected as inconsistent",
                   loop_edges.size(), before, panorama_pairs.size() - pano_kept,
                   panorama_pairs.size());
      } else {
        core::warn("PlaneGraph: PCM disabled (--loop-no-pcm) — {} panorama "
                   "loop edges enter the graph WITHOUT a consistency filter",
                   panorama_pairs.size());
      }
    }
  }

  PlaneGraphOptimizer optimizer(options);
  PlaneGraphResult result = optimizer.optimize(frames, loop_edges);

  // Attribute the surviving edges back to the panorama source so a run's
  // panorama contribution is auditable AFTER PCM (STANDARDS §5: report the
  // numbers, do not leave "it did something" implicit).
  for (const auto &e : loop_edges)
    if (panorama_pairs.count({std::min(e.i, e.j), std::max(e.i, e.j)}) > 0)
      ++result.panorama_loop_edges;

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
