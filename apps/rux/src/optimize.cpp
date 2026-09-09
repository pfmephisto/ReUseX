// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "optimize.hpp"
#include "exit_status.hpp"
#include "stage_prerequisites.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

void setup_subcommand_optimize(CLI::App &app,
                               std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandOptimizeOptions>();
  auto *sub = app.add_subcommand(
      "optimize",
      "Globally optimize per-frame poses via a plane-landmark pose graph");

  sub->footer(R"(
DESCRIPTION:
  Globally optimizes the stored per-frame sensor poses of a single scan using a
  plane-landmark pose graph (issue #225, P1). Unlike 'rux register' — which
  polishes poses locally with frame-to-frame point-to-plane residuals and
  provably saturates because pairwise energy cannot express global consistency —
  this back-end changes the objective to *shared global plane landmarks*:

    1. detect the dominant planes each frame sees (RANSAC on its surfels),
    2. associate those observations across frames into persistent
       OrientedPlane3 landmarks (normal-angle + offset gating, min observations),
    3. build a GTSAM factor graph (per-frame Pose3 + odometry BetweenFactors from
       the seed poses + one OrientedPlane3Factor per observation + a soft prior
       on the first pose for gauge), and
    4. solve it with Graduated Non-Convexity (GNC) so a handful of bad plane
       associations are down-weighted rather than corrupting the solution.

  Optimized poses overwrite the stored transforms in place — re-import the
  RTABMap database to recover the originals, or use --dry-run to preview.

  Plane factors are weighted by their inlier support (a wall fit from thousands
  of surfels pulls harder than a small patch); disable with
  --no-plane-inlier-weight. Optionally, wide-baseline loop-closure edges (P2)
  can be detected from the RGB-D frames (ORB matches + stored depth -> RANSAC
  relative pose) and added as robust factors via --loop-closure.

EXAMPLES:
  rux optimize                        # Defaults (GNC on, inlier-weighted planes)
  rux optimize --dry-run              # Report statistics without writing
  rux optimize --loop-closure         # Add wide-baseline loop edges (P2, ORB)
  rux optimize --loop-closure --use-panoramas   # ...plus 360-panorama loop edges (#236)
  rux optimize --use-panoramas        # Panorama loop edges alone (no ORB pair search)
  rux optimize --loop-edges edges.json --loop-trust  # External learned-matcher edges
  rux optimize --min-observations 3 --assoc-distance 0.15
  rux optimize --no-gnc               # Plain Levenberg-Marquardt (no robustness)

WORKFLOW:
  1. rux import rtabmap scan.db       # Import sensor frames
  2. rux optimize                     # Global pose optimization (this command)
  3. rux create clouds                # Regenerate clouds with optimized poses

NOTES:
  - Requires at least 2 sensor frames with depth (run 'rux import' first)
  - Run 'rux create clouds' afterwards to rebuild the merged point cloud
  - When too few plane landmarks are found the poses are left unchanged
)");

  // --- Per-frame plane detection ---
  sub->add_option("--max-planes-per-frame", opt->max_planes_per_frame,
                  "Keep at most this many planes per frame")
      ->default_val(opt->max_planes_per_frame);
  sub->add_option("--min-plane-inliers", opt->min_plane_inliers,
                  "Reject planes with fewer supporting surfels")
      ->default_val(opt->min_plane_inliers);
  sub->add_option("--ransac-distance", opt->ransac_distance,
                  "Inlier point-to-plane distance (m)")
      ->default_val(opt->ransac_distance);
  sub->add_option("--ransac-normal-angle", opt->ransac_normal_angle,
                  "Inlier normal agreement threshold (deg)")
      ->default_val(opt->ransac_normal_angle);
  sub->add_option("--ransac-iterations", opt->ransac_iterations,
                  "RANSAC hypotheses per plane")
      ->default_val(opt->ransac_iterations);

  // --- Cross-frame association ---
  sub->add_option(
         "--assoc-normal-angle", opt->assoc_normal_angle,
         "Max normal angle to merge observations into a landmark (deg)")
      ->default_val(opt->assoc_normal_angle);
  sub->add_option("--assoc-distance", opt->assoc_distance,
                  "Max plane-offset difference to merge (m)")
      ->default_val(opt->assoc_distance);
  sub->add_option("--min-observations", opt->min_observations,
                  "Landmark must be seen by at least this many frames")
      ->default_val(opt->min_observations);
  sub->add_option("--assoc-overlap-margin", opt->assoc_overlap_margin,
                  "In-plane overlap slack for merging observations (m, <=0 "
                  "disables the overlap gate)")
      ->default_val(opt->assoc_overlap_margin);
  sub->add_option("--min-landmark-spread-ratio", opt->min_landmark_spread_ratio,
                  "Reject landmarks whose observation centroids are more "
                  "collinear than this SV ratio (<=0 disables)")
      ->default_val(opt->min_landmark_spread_ratio);

  // --- Alternating rounds ---
  sub->add_option("--assoc-rounds", opt->assoc_rounds,
                  "Associate/optimize/refit rounds (1 = one-shot)")
      ->default_val(opt->assoc_rounds);
  sub->add_option("--assoc-round-tol", opt->assoc_round_tol,
                  "Stop rounds when the round's max pose shift drops below (m)")
      ->default_val(opt->assoc_round_tol);

  // --- Factor-graph noise ---
  sub->add_option("--odometry-sigma-rot", opt->odometry_sigma_rot,
                  "Odometry rotation std (rad)")
      ->default_val(opt->odometry_sigma_rot);
  sub->add_option("--odometry-sigma-trans", opt->odometry_sigma_trans,
                  "Odometry translation std (m)")
      ->default_val(opt->odometry_sigma_trans);
  sub->add_option("--underconstrained-odom-scale",
                  opt->underconstrained_odom_scale,
                  "Odometry sigma multiplier for frames that span < 2 plane "
                  "normal directions (<1 tightens; 1 disables the guard)")
      ->default_val(opt->underconstrained_odom_scale);
  sub->add_option(
         "--odometry-noise", opt->odometry_noise,
         "Per-edge odometry noise model: fixed (default; every edge gets "
         "--odometry-sigma-*) or motion (sigma proportional to that edge's own "
         "seed motion, normalised by the run's median so the odometry chain's "
         "overall authority against the plane factors is unchanged). Note the "
         "measured result of issue #225: loosening odometry makes absolute GT "
         "accuracy monotonically WORSE on drifting captures, so 'motion' is a "
         "redistribution probe, not a drift fix")
      ->default_val(opt->odometry_noise)
      ->check(CLI::IsMember({"fixed", "motion"}));
  sub->add_option("--odometry-weight-min", opt->odometry_weight_min,
                  "Min per-edge odometry sigma scale (--odometry-noise motion)")
      ->default_val(opt->odometry_weight_min);
  sub->add_option("--odometry-weight-max", opt->odometry_weight_max,
                  "Max per-edge odometry sigma scale (--odometry-noise motion)")
      ->default_val(opt->odometry_weight_max);
  sub->add_flag("--odometry-robust", opt->odometry_robust,
                "Let GNC down-weight individual odometry factors instead of "
                "registering every one as a known inlier. Measured on the #225 "
                "ARKitScenes GT scans: it does not recover the regression, "
                "because smooth drift has no single wrong edge to demote");
  sub->add_option("--odometry-gnc-inlier-cost", opt->odometry_gnc_inlier_cost,
                  "GNC-TLS inlier threshold for odometry factors under "
                  "--odometry-robust (chi2(6, 0.99)/2 = 8.41)")
      ->default_val(opt->odometry_gnc_inlier_cost);
  sub->add_option("--plane-sigma-normal", opt->plane_sigma_normal,
                  "Plane-normal measurement std (rad)")
      ->default_val(opt->plane_sigma_normal);
  sub->add_option("--plane-sigma-distance", opt->plane_sigma_distance,
                  "Plane-distance measurement std (m)")
      ->default_val(opt->plane_sigma_distance);
  sub->add_option(
         "--plane-noise", opt->plane_noise,
         "Per-observation plane-factor noise model: inliers (default; one "
         "sqrt(median_N/N) scalar on both channels), fit (sigmas from each "
         "detection's residual RMS, support and in-plane extent, weighting the "
         "normal and distance channels separately), or uniform (no weighting). "
         "Use 'fit' on captures whose poses DRIFT — measured better than "
         "'inliers' on every metric of all three ARKitScenes GT scans, but "
         "worse on a non-drifting laser-GT scan, so it is not the default. "
         "Pair it with --plane-weight-min 0.10 --plane-weight-max 15. "
         "--plane-sigma-normal/-distance always describe a MEDIAN-quality "
         "observation, so switching models redistributes authority between "
         "observations without changing the plane term's overall weight")
      ->default_val(opt->plane_noise)
      ->check(CLI::IsMember({"uniform", "inliers", "fit"}));
  sub->add_flag("--no-plane-inlier-weight", opt->no_plane_inlier_weight,
                "Deprecated spelling of --plane-noise uniform (kept so the "
                "configurations recorded in issue #225 stay reproducible); "
                "overrides --plane-noise when given");
  sub->add_option("--plane-weight-min", opt->plane_weight_min,
                  "Min plane-factor sigma scale (most authority a single "
                  "observation may earn). Defaults are tuned for --plane-noise "
                  "fit; the legacy 'inliers' model was tuned at 0.5")
      ->default_val(opt->plane_weight_min);
  sub->add_option(
         "--plane-weight-max", opt->plane_weight_max,
         "Max plane-factor sigma scale (least authority). Defaults are "
         "tuned for --plane-noise fit; the legacy 'inliers' model was "
         "tuned at 3.0. If most observations hit the clamp, the clamp "
         "— not the fit quality — is setting the weights (warned about "
         "at run time); widen the range")
      ->default_val(opt->plane_weight_max);
  sub->add_option(
         "--plane-sigma-scale", opt->plane_sigma_scale,
         "Global multiplier on BOTH plane sigmas — the plane term's authority "
         "against the odometry chain. The plane term's weight in the objective "
         "goes as 1/scale^2, so >1 weakens it and <1 strengthens it; 1.0 is "
         "the shipped calibration (bit-identical to omitting the flag). Use it "
         "to sweep plane-term weight in one dimension (#225 §9)")
      ->default_val(opt->plane_sigma_scale)
      ->check(CLI::PositiveNumber);
  sub->add_flag("--no-plane-factors", opt->no_plane_factors,
                "Build the pose graph WITHOUT plane factors (the "
                "--plane-sigma-scale -> infinity limit, taken exactly): "
                "odometry + the frame-0 gauge prior only. Planes are still "
                "detected and associated for reporting. The solve then cannot "
                "move the seed trajectory, which makes this the 'plane term "
                "off' endpoint of a weight sweep");
  sub->add_option("--prior-sigma-rot", opt->prior_sigma_rot,
                  "First-pose gauge prior rotation std (rad)")
      ->default_val(opt->prior_sigma_rot);
  sub->add_option("--prior-sigma-trans", opt->prior_sigma_trans,
                  "First-pose gauge prior translation std (m)")
      ->default_val(opt->prior_sigma_trans);

  // --- Solver ---
  sub->add_flag("--no-gnc", opt->no_gnc,
                "Disable Graduated Non-Convexity (use plain LM)");
  sub->add_option("--gnc-inlier-cost", opt->gnc_inlier_cost,
                  "GNC TLS inlier threshold on factor error (0.5*chi^2)")
      ->default_val(opt->gnc_inlier_cost);
  sub->add_option("--iterations", opt->iterations,
                  "Levenberg-Marquardt (inner) iteration cap")
      ->default_val(opt->iterations);
  sub->add_option("--seed", opt->seed, "RANSAC seed (determinism)")
      ->default_val(opt->seed);

  // --- Wide-baseline loop closure (P2) ---
  sub->add_flag("--loop-closure", opt->loop_closure,
                "Detect wide-baseline loop edges (ORB + depth -> RANSAC "
                "relative pose) and add them as robust factors to the graph");
  sub->add_option(
         "--loop-proposal", opt->loop_proposal,
         "Loop-candidate proposal: auto (exhaustive on small scans, "
         "else appearance), appearance (pose-independent bag-of-words), "
         "spatial (seed-pose proximity; only for good poses), or "
         "exhaustive (all pairs; small scans)")
      ->default_val(opt->loop_proposal)
      ->check(CLI::IsMember({"auto", "appearance", "spatial", "exhaustive"}));
  sub->add_option("--loop-min-frame-gap", opt->loop_min_frame_gap,
                  "Only pair frames at least this far apart in index")
      ->default_val(opt->loop_min_frame_gap);
  sub->add_option("--loop-max-distance", opt->loop_max_distance,
                  "Max seed camera-centre distance to propose a loop pair (m)")
      ->default_val(opt->loop_max_distance);
  sub->add_option("--loop-max-view-angle", opt->loop_max_view_angle,
                  "Max viewing-direction angle to propose a loop pair (deg)")
      ->default_val(opt->loop_max_view_angle);
  sub->add_option("--loop-max-candidates", opt->loop_max_candidates,
                  "Max loop candidates per frame (nearest first)")
      ->default_val(opt->loop_max_candidates);
  sub->add_option("--loop-min-inliers", opt->loop_min_inliers,
                  "Reject a loop edge below this many RANSAC inliers")
      ->default_val(opt->loop_min_inliers);
  sub->add_option("--loop-max-features", opt->loop_max_features,
                  "ORB features per frame for loop matching")
      ->default_val(opt->loop_max_features);
  sub->add_option("--loop-ratio-test", opt->loop_ratio_test,
                  "Lowe ratio threshold for loop descriptor matches")
      ->default_val(opt->loop_ratio_test);
  sub->add_option("--loop-ransac-inlier-dist", opt->loop_ransac_inlier_dist,
                  "3D-3D RANSAC inlier threshold for loop relative pose (m)")
      ->default_val(opt->loop_ransac_inlier_dist);
  sub->add_option(
         "--loop-max-seed-disagreement", opt->loop_max_seed_disagreement,
         "Reject a loop edge whose translation disagrees with the seed "
         "poses by more than this (m; <=0 disables)")
      ->default_val(opt->loop_max_seed_disagreement);
  sub->add_option(
         "--loop-min-seed-disagreement", opt->loop_min_seed_disagreement,
         "Keep only loop edges that disagree with the seed poses by at "
         "least this (m) — drops redundant edges that would just add "
         "noise to already-good poses (0 keeps all)")
      ->default_val(opt->loop_min_seed_disagreement);
  sub->add_flag(
      "--loop-trust", opt->loop_trust,
      "Trust loop edges: give them their own relaxed GNC-TLS inlier "
      "threshold (--loop-trust-inlier-cost) instead of --gnc-inlier-cost, so "
      "large-drift corrections actually apply while a grossly-wrong edge is "
      "still truncated (a Huber kernel under --no-gnc). Needs PCM / a "
      "discriminative matcher to be safe, and looser --odometry-sigma-trans");
  sub->add_option("--loop-trust-inlier-cost", opt->loop_trust_inlier_cost,
                  "GNC-TLS inlier threshold for --loop-trust loop edges "
                  "(0.5*chi^2; generous but finite — an edge above it is "
                  "still down-weighted to zero)")
      ->default_val(opt->loop_trust_inlier_cost);
  sub->add_flag("--loop-no-pcm", opt->loop_no_pcm,
                "Disable pairwise-consistency (PCM) filtering of loop edges "
                "(governs ALL edge sources: --loop-closure, --use-panoramas "
                "and --loop-edges)");

  // --- Panorama-derived loop edges (#236) ---
  sub->add_flag(
      "--use-panoramas", opt->use_panoramas,
      "Derive wide-baseline loop edges from the project's 360 panoramas "
      "(issue #236). Each panorama is resected INDEPENDENTLY against every "
      "frame it matches, in that frame's own optical coordinates, so the "
      "relative pose it implies between two frames is a real measurement and "
      "not a restatement of the drifted seed poses. One panorama sees all "
      "directions at once, so it ties temporally distant frames that do not "
      "overlap each other. Unioned with --loop-closure / --loop-edges and "
      "gated by the same --loop-min-frame-gap / --loop-*-seed-disagreement / "
      "PCM machinery. Does NOT require 'rux align 360' — panorama poses are "
      "neither read nor written");
  sub->add_option("--pano-max-frames", opt->pano_max_frames,
                  "Frames swept across the whole trajectory and matched "
                  "against each panorama (the dominant cost)")
      ->default_val(opt->pano_max_frames);
  sub->add_option("--pano-min-inliers", opt->pano_min_inliers,
                  "Accept a panorama's per-frame resection above this many "
                  "gated bearing inliers")
      ->default_val(opt->pano_min_inliers);
  sub->add_option("--pano-max-edges", opt->pano_max_edges,
                  "Max loop edges contributed per panorama (highest joint "
                  "support first); caps the O(F^2) edges one panorama would "
                  "otherwise add from a single shared resection error")
      ->default_val(opt->pano_max_edges);
  sub->add_option("--pano-slices", opt->pano_n_yaw,
                  "Perspective slices rendered around the panorama equator")
      ->default_val(opt->pano_n_yaw);
  sub->add_option("--pano-max-features", opt->pano_max_features,
                  "ORB features per panorama slice / sensor frame")
      ->default_val(opt->pano_max_features);
  sub->add_option("--pano-max-distance", opt->pano_max_distance,
                  "Reject a resection placing the panorama further than this "
                  "from the frame (m; ill-conditioned bearing solve). <=0 "
                  "disables")
      ->default_val(opt->pano_max_distance);
  sub->add_option(
         "--loop-edges", opt->loop_edges_file,
         "Load externally-computed loop edges from a JSON file (schema "
         "reusex.loop_edges.v1) produced by an out-of-process matcher "
         "(XFeat / EfficientLoFTR / MapAnything, or an offline MASt3R oracle) "
         "and add them to the graph. Unioned with --loop-closure edges, "
         "de-duplicated per frame pair, then PCM-filtered as one set (unless "
         "--loop-no-pcm); the license-clean way to feed a learned matcher "
         "without linking it into the binary. Pair with --loop-trust to apply "
         "large-drift corrections.")
      ->check(CLI::ExistingFile);
  sub->add_option(
         "--loop-edges-min-disagreement", opt->loop_edges_min_disagreement,
         "Drop external (--loop-edges) edges whose relative translation agrees "
         "with the seed poses within this (m) — non-informative redundant "
         "edges "
         "that would only add matcher noise to already-correct poses (keeps "
         "the "
         "bridge a no-op on a well-posed scan). 0 disables the gate.")
      ->default_val(opt->loop_edges_min_disagreement);

  // --- Surfel extraction ---
  sub->add_option("--surfel-voxel", opt->surfel_voxel,
                  "Per-frame voxel downsample for surfels (m, <=0 disables)")
      ->default_val(opt->surfel_voxel);
  sub->add_option("--min-distance", opt->min_distance, "Minimum depth (m)")
      ->default_val(opt->min_distance);
  sub->add_option("--max-distance", opt->max_distance, "Maximum depth (m)")
      ->default_val(opt->max_distance);
  sub->add_option("--sampling-factor", opt->sampling_factor,
                  "Per-frame pixel subsampling factor")
      ->default_val(opt->sampling_factor);
  sub->add_option("--confidence", opt->confidence_threshold,
                  "Minimum confidence threshold")
      ->default_val(opt->confidence_threshold);

  sub->add_flag("--dry-run", opt->dry_run,
                "Compute and report statistics without writing poses back");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_optimize");
    rux::finish(run_subcommand_optimize(*opt, *global_opt));
  });
}

int run_subcommand_optimize(SubcommandOptimizeOptions const &opt,
                            const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Optimizing sensor poses in: {}", project_path.string());

  try {
    // std::clamp is UB when lo > hi, and the plane-factor weighting clamps
    // into [plane_weight_min, plane_weight_max]. Reject the inverted range
    // here instead of letting it reach the library.
    if (opt.plane_weight_min > opt.plane_weight_max) {
      spdlog::error("--plane-weight-min ({}) must be <= --plane-weight-max "
                    "({})",
                    opt.plane_weight_min, opt.plane_weight_max);
      spdlog::info("Resolution: pass a non-inverted range, e.g. "
                   "--plane-weight-min 0.5 --plane-weight-max 3.0");
      return RuxError::INVALID_ARGUMENT;
    }

    reusex::ProjectDB db(project_path);

    // Reuse the register prerequisites (>= 2 sensor frames carrying depth).
    if (int rc = rux::check_stage_prerequisites(
            db, reusex::core::PipelineStage::optimize);
        rc != RuxError::SUCCESS)
      return rc;

    reusex::geometry::PlaneGraphOptions options;
    options.max_planes_per_frame = opt.max_planes_per_frame;
    options.min_plane_inliers = opt.min_plane_inliers;
    options.ransac_distance = opt.ransac_distance;
    options.ransac_normal_angle = opt.ransac_normal_angle;
    options.ransac_iterations = opt.ransac_iterations;
    options.assoc_normal_angle = opt.assoc_normal_angle;
    options.assoc_distance = opt.assoc_distance;
    options.min_landmark_observations = opt.min_observations;
    options.assoc_overlap_margin = opt.assoc_overlap_margin;
    options.min_landmark_spread_ratio = opt.min_landmark_spread_ratio;
    options.assoc_rounds = opt.assoc_rounds;
    options.assoc_round_tol = opt.assoc_round_tol;
    options.odometry_sigma_rot = opt.odometry_sigma_rot;
    options.odometry_sigma_trans = opt.odometry_sigma_trans;
    options.underconstrained_odom_scale = opt.underconstrained_odom_scale;
    options.plane_sigma_normal = opt.plane_sigma_normal;
    options.plane_sigma_distance = opt.plane_sigma_distance;
    options.plane_noise_model =
        opt.no_plane_inlier_weight
            ? reusex::geometry::PlaneNoiseModel::uniform
            : (opt.plane_noise == "uniform"
                   ? reusex::geometry::PlaneNoiseModel::uniform
                   : (opt.plane_noise == "inliers"
                          ? reusex::geometry::PlaneNoiseModel::inlier_count
                          : reusex::geometry::PlaneNoiseModel::fit_geometry));
    options.odometry_noise_model =
        opt.odometry_noise == "motion"
            ? reusex::geometry::OdometryNoiseModel::motion
            : reusex::geometry::OdometryNoiseModel::fixed;
    options.odometry_weight_min = opt.odometry_weight_min;
    options.odometry_weight_max = opt.odometry_weight_max;
    options.odometry_robust = opt.odometry_robust;
    options.odometry_gnc_inlier_cost = opt.odometry_gnc_inlier_cost;
    options.plane_weight_min = opt.plane_weight_min;
    options.plane_weight_max = opt.plane_weight_max;
    options.plane_sigma_scale = opt.plane_sigma_scale;
    options.use_plane_factors = !opt.no_plane_factors;
    options.prior_sigma_rot = opt.prior_sigma_rot;
    options.prior_sigma_trans = opt.prior_sigma_trans;
    options.use_gnc = !opt.no_gnc;
    options.gnc_inlier_cost = opt.gnc_inlier_cost;
    options.max_iterations = opt.iterations;
    options.seed = opt.seed;
    options.surfel.min_distance = opt.min_distance;
    options.surfel.max_distance = opt.max_distance;
    options.surfel.sampling_factor = opt.sampling_factor;
    options.surfel.confidence_threshold = opt.confidence_threshold;
    options.surfel.voxel_size = opt.surfel_voxel;
    options.loop_closure.enable = opt.loop_closure;
    options.loop_closure.proposal =
        opt.loop_proposal == "spatial"
            ? reusex::geometry::LoopProposal::spatial
            : (opt.loop_proposal == "exhaustive"
                   ? reusex::geometry::LoopProposal::exhaustive
                   : (opt.loop_proposal == "appearance"
                          ? reusex::geometry::LoopProposal::appearance
                          : reusex::geometry::LoopProposal::automatic));
    options.loop_closure.min_frame_gap = opt.loop_min_frame_gap;
    options.loop_closure.max_candidate_distance = opt.loop_max_distance;
    options.loop_closure.max_view_angle = opt.loop_max_view_angle;
    options.loop_closure.max_candidates_per_frame = opt.loop_max_candidates;
    options.loop_closure.min_match_inliers = opt.loop_min_inliers;
    options.loop_closure.max_features = opt.loop_max_features;
    options.loop_closure.ratio_test = opt.loop_ratio_test;
    options.loop_closure.ransac_inlier_dist = opt.loop_ransac_inlier_dist;
    options.loop_closure.max_seed_disagreement = opt.loop_max_seed_disagreement;
    options.loop_closure.min_seed_disagreement = opt.loop_min_seed_disagreement;
    options.loop_closure.pcm = !opt.loop_no_pcm;
    options.loop_edges_trusted = opt.loop_trust;
    options.loop_trust_inlier_cost = opt.loop_trust_inlier_cost;
    options.loop_closure.seed = opt.seed;
    options.loop_edges_file = opt.loop_edges_file;
    options.loop_edges_min_seed_disagreement = opt.loop_edges_min_disagreement;
    options.panorama_loops.enable = opt.use_panoramas;
    options.panorama_loops.max_frames = opt.pano_max_frames;
    options.panorama_loops.min_frame_inliers = opt.pano_min_inliers;
    options.panorama_loops.max_edges_per_panorama = opt.pano_max_edges;
    options.panorama_loops.n_yaw = opt.pano_n_yaw;
    options.panorama_loops.max_features = opt.pano_max_features;
    options.panorama_loops.max_pano_distance = opt.pano_max_distance;
    options.panorama_loops.seed = opt.seed;

    int logId = db.log_pipeline_start(
        "pose_optimization_plane_graph",
        fmt::format(
            R"({{"min_observations":{},"assoc_normal_angle":{},"assoc_distance":{},"max_planes_per_frame":{},"min_plane_inliers":{},"plane_noise":"{}","use_gnc":{},"iterations":{},"seed":{},"dry_run":{},"loop_closure":{},"loop_trust":{},"pcm":{},"loop_edges_file":"{}","loop_edges_min_disagreement":{},"use_panoramas":{},"pano_max_frames":{},"pano_min_inliers":{}}})",
            opt.min_observations, opt.assoc_normal_angle, opt.assoc_distance,
            opt.max_planes_per_frame, opt.min_plane_inliers,
            opt.no_plane_inlier_weight ? "uniform" : opt.plane_noise,
            !opt.no_gnc, opt.iterations, opt.seed, opt.dry_run,
            opt.loop_closure, opt.loop_trust, !opt.loop_no_pcm,
            opt.loop_edges_file, opt.loop_edges_min_disagreement,
            opt.use_panoramas, opt.pano_max_frames, opt.pano_min_inliers));

    auto result =
        reusex::geometry::optimize_sensor_poses(db, options, opt.dry_run);

    db.log_pipeline_end(logId, true);

    spdlog::info("Plane-graph optimization: {} frames, {} plane detections, "
                 "{} landmarks, {} plane factors",
                 result.frames, result.planes_detected, result.landmarks,
                 result.plane_factors);
    spdlog::info("Factor-graph error {:.4f} -> {:.4f}, max pose shift {:.4f} m",
                 result.initial_error, result.final_error,
                 result.max_pose_shift);
    if (result.median_fit_sigma_normal > 0.0)
      spdlog::info(
          "Plane measurement noise: median fit sigma {:.5f} rad / {:.5f} m; "
          "{} strong and {} weak observations hit the sigma-scale clamp",
          result.median_fit_sigma_normal, result.median_fit_sigma_distance,
          result.plane_noise_clamped_low, result.plane_noise_clamped_high);
    if (opt.loop_closure || !opt.loop_edges_file.empty() || opt.use_panoramas)
      spdlog::info("Loop closure: {} wide-baseline edges added to the graph "
                   "({} of them panorama-derived)",
                   result.loop_edges, result.panorama_loop_edges);

    // Mirrors the library guard (PlaneGraphOptimizer::optimize): the poses are
    // only left untouched when there is NEITHER a landmark NOR a loop edge to
    // constrain them. With loop edges but no landmarks the graph was solved and
    // the poses WERE written, so this must not short-circuit.
    if (result.landmarks == 0 && result.loop_edges == 0) {
      spdlog::warn("No plane landmarks reached the minimum observation count "
                   "and no loop edges were added; poses were left unchanged. "
                   "Try lowering --min-observations or relaxing "
                   "--assoc-distance / --assoc-normal-angle.");
      return RuxError::SUCCESS;
    }

    if (!result.converged) {
      spdlog::error("Factor-graph optimizer failed to produce a solution; "
                    "poses were left unchanged (see log for details)");
      return RuxError::GENERIC;
    }

    if (opt.dry_run)
      spdlog::info("Dry run: poses were not modified");
    else
      spdlog::info("Optimized poses written. Run 'rux create clouds' to "
                   "regenerate the merged point cloud.");

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Pose optimization failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
