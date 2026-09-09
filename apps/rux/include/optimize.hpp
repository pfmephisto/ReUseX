// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

// Included for the sole purpose of MIRRORING the library defaults below
// (docs/STANDARDS.md §4): a CLI default that references the library's own
// value cannot silently drift away from it.
#include <reusex/slam/PlaneGraphOptimizer.hpp>

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

/// Options for the `rux optimize` subcommand (global plane-landmark pose-graph
/// optimization). Defaults mirror reusex::geometry::PlaneGraphOptions so the
/// library remains the single source of truth (docs/STANDARDS.md §4).
struct SubcommandOptimizeOptions {
  // Per-frame plane detection. Defaults are the "mid-density" sweet spot
  // measured on the office + MuSHRoom-honka scans: with per-observation inlier
  // weighting on, this beats the conservative (max=4/200/obs=5) defaults on
  // BOTH GT-free flatness and laser GT F-score, and is the first pose stage to
  // improve flatness without degrading GT (see docs/research/…). More/denser
  // planes than this only help when combined with inlier weighting.
  int max_planes_per_frame = 6;
  int min_plane_inliers = 120;
  float ransac_distance = 0.02f;
  float ransac_normal_angle = 20.0f;
  int ransac_iterations = 200;

  // Cross-frame association.
  float assoc_normal_angle = 10.0f;
  float assoc_distance = 0.10f;
  int min_observations = 4;
  float assoc_overlap_margin = 0.30f;
  float min_landmark_spread_ratio = 0.05f;

  // Alternating rounds.
  int assoc_rounds = 2;
  float assoc_round_tol = 0.02f;

  // Factor-graph noise.
  float odometry_sigma_rot = 0.005f;
  float odometry_sigma_trans = 0.01f;
  float underconstrained_odom_scale = 0.25f;
  float plane_sigma_normal = 0.24f;
  float plane_sigma_distance = 0.19f;
  std::string plane_noise = "inliers"; // uniform | inliers | fit
  bool no_plane_inlier_weight = false;
  float plane_weight_min = 0.5f;
  float plane_weight_max = 3.0f;
  float prior_sigma_rot = 0.001f;
  float prior_sigma_trans = 0.001f;

  // Solver.
  bool no_gnc = false; ///< disable Graduated Non-Convexity (plain LM)
  float gnc_inlier_cost = 5.67f;
  int iterations = 100;
  unsigned seed = 42;

  // Wide-baseline loop closure (P2; off by default). Defaults mirror the
  // LoopClosureOptions in the library header.
  bool loop_closure = false;
  std::string loop_proposal =
      "auto"; // auto | spatial | appearance | exhaustive
  int loop_min_frame_gap = 50;
  float loop_max_distance = 2.5f;
  float loop_max_view_angle = 45.0f;
  int loop_max_candidates = 5;
  int loop_min_inliers = 60;
  int loop_max_features = 3000;
  float loop_ratio_test = 0.85f;
  float loop_ransac_inlier_dist = 0.10f;
  float loop_max_seed_disagreement = 0.0f; // 0 = disabled (correct drift)
  float loop_min_seed_disagreement =
      0.10f; // drop redundant near-agreement edges
  // Trust loop edges: give them their own generous-but-finite GNC-TLS inlier
  // threshold (Huber under --no-gnc) instead of the shared --gnc-inlier-cost.
  bool loop_trust = false;
  float loop_trust_inlier_cost =
      reusex::geometry::PlaneGraphOptions{}.loop_trust_inlier_cost;
  bool loop_no_pcm = false; // disable pairwise-consistency filtering

  // Panorama-derived wide-baseline loop edges (#236; off by default). Defaults
  // mirror reusex::geometry::PlaneGraphOptions::panorama_loops so the library
  // stays the single source of truth (docs/STANDARDS.md §4).
  bool use_panoramas = false;
  int pano_max_frames =
      reusex::geometry::PlaneGraphOptions{}.panorama_loops.max_frames;
  int pano_min_inliers =
      reusex::geometry::PlaneGraphOptions{}.panorama_loops.min_frame_inliers;
  int pano_max_edges = reusex::geometry::PlaneGraphOptions{}
                           .panorama_loops.max_edges_per_panorama;
  int pano_n_yaw = reusex::geometry::PlaneGraphOptions{}.panorama_loops.n_yaw;
  int pano_max_features =
      reusex::geometry::PlaneGraphOptions{}.panorama_loops.max_features;
  double pano_max_distance =
      reusex::geometry::PlaneGraphOptions{}.panorama_loops.max_pano_distance;

  // External loop edges (license-clean learned-matcher bridge). Path to a JSON
  // file (schema "reusex.loop_edges.v1") produced by an out-of-process matcher
  // (XFeat / EfficientLoFTR / MapAnything, or an offline MASt3R ceiling
  // oracle). Unioned with any internally-detected --loop-closure edges. Empty =
  // none.
  std::string loop_edges_file;
  // Seed-disagreement gate for external edges (drop edges agreeing with the
  // seed within this many m; keeps the bridge a no-op on a well-posed scan). 0
  // = off. 0.5 m sits above the depth-noise floor, below any real drift.
  double loop_edges_min_disagreement = 0.50;

  // Surfel extraction (shared with `rux register`).
  float surfel_voxel = 0.03f;
  float min_distance = 0.0f;
  float max_distance = 4.0f;
  // Mirrors the plane-graph override of the shared surfel default (8 -> 6).
  int sampling_factor =
      reusex::geometry::PlaneGraphOptions{}.surfel.sampling_factor;
  int confidence_threshold = 2;

  bool dry_run = false;
};

void setup_subcommand_optimize(CLI::App &app,
                               std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_optimize(SubcommandOptimizeOptions const &opt,
                            const RuxOptions &global_opt);
