// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "align/panorama.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/PanoramaAlignment.hpp>

#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#include <cmath>
#include <filesystem>

void setup_subcommand_align_panorama(CLI::App &app,
                                     std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandAlignPanoramaOptions>();
  auto *sub = app.add_subcommand(
      "360", "Align 360 panoramas to the scan (content-based pose refinement)");

  sub->footer(R"(
DESCRIPTION:
  Refines each panorama's 6-DoF pose from image content instead of the EXIF
  timestamp placement. Virtual pinhole slices of the equirect are ORB-matched
  against nearby sensor frames, the matches are lifted to metric 3D via the
  frames' depth, and the panorama pose is resected with PnP + RANSAC.

EXAMPLES:
  rux align 360                    # align all panoramas, write refined poses
  rux align 360 --dry-run          # report inliers / RMS / correction only
  rux align 360 --overwrite        # re-align panoramas already aligned

WORKFLOW:
  1. rux import rtabmap scan.db     # sensor frames (with timestamps + depth)
  2. rux import 360 ./360/          # panoramas, seeded by timestamp
  3. rux align 360                  # refine panorama poses from content

NOTES:
  - Requires panoramas linked to a sensor frame (node_id) with depth.
  - A panorama that fails to align keeps its timestamp placement.
)");

  sub->add_flag("--dry-run", opt->dry_run,
                "Compute and report only; do not write poses");
  sub->add_flag("--overwrite", opt->overwrite,
                "Re-align panoramas that are already aligned");
  sub->add_option("--min-inliers", opt->min_inliers,
                  "Accept an alignment above this many pooled inliers")
      ->capture_default_str();
  sub->add_option("--window", opt->window,
                  "Candidate sensor frames on each side of the seed")
      ->capture_default_str();
  sub->add_option("--max-candidates", opt->max_candidates,
                  "Cap candidate frames matched per panorama")
      ->capture_default_str();
  sub->add_option("--n-yaw", opt->n_yaw,
                  "Perspective slices around the equator")
      ->capture_default_str();
  sub->add_option("--fov", opt->fov, "Per-slice horizontal FOV (deg)")
      ->capture_default_str();
  sub->add_option("--max-correction", opt->max_correction,
                  "Reject corrections beyond this distance in m (0 disables)")
      ->capture_default_str();
  sub->add_option("--figures", opt->figures_dir,
                  "Dump alignment comparison figures to this directory");
  sub->add_option("--figures-limit", opt->figures_limit,
                  "Cap number of panoramas figured")
      ->capture_default_str();

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_align_panorama");
    return run_subcommand_align_panorama(*opt, *global_opt);
  });
}

int run_subcommand_align_panorama(SubcommandAlignPanoramaOptions const &opt,
                                  const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Aligning 360 panoramas in project: {}", project_path.string());

  reusex::ProjectDB db(project_path);

  reusex::geometry::PanoramaAlignmentOptions ao;
  ao.candidate_window = opt.window;
  ao.max_candidates = opt.max_candidates;
  ao.n_yaw = opt.n_yaw;
  ao.fov_deg = opt.fov;
  ao.min_inliers = opt.min_inliers;
  ao.max_correction_m = opt.max_correction;

  auto panos = db.list_panoramic_images();
  if (panos.empty()) {
    spdlog::warn("No panoramas in project. Nothing to align.");
    return RuxError::SUCCESS;
  }

  int log_id = db.log_pipeline_start(
      "align_360", fmt::format(R"({{"panoramas":{}}})", panos.size()));

  int aligned = 0, skipped = 0, failed = 0, figured = 0;
  double sum_delta = 0.0;

  try {
    for (const auto &pano : panos) {
      if (pano.node_id < 0) {
        spdlog::info("{}: no linked sensor frame, skipping", pano.filename);
        ++skipped;
        continue;
      }
      if (pano.has_pose && !opt.overwrite) {
        spdlog::info("{}: already aligned (use --overwrite to redo)",
                     pano.filename);
        ++skipped;
        continue;
      }

      // Enable the correspondence figure for the first N panoramas figured.
      if (!opt.figures_dir.empty() && figured < opt.figures_limit) {
        ao.debug_dir = opt.figures_dir;
        ao.debug_name = fs::path(pano.filename).replace_extension().string();
      } else {
        ao.debug_dir.clear();
      }

      auto r = reusex::geometry::align_panorama(db, pano.id, pano.node_id, ao);
      if (!r.aligned) {
        spdlog::warn("{}: alignment failed (insufficient matches)",
                     pano.filename);
        ++failed;
        continue;
      }

      spdlog::info("{}: aligned — {} inliers, {:.3f} deg RMS, {:.3f} m "
                   "correction{}",
                   pano.filename, r.inliers, r.rms_deg, r.delta_from_seed_m,
                   opt.dry_run ? " (dry-run)" : "");
      sum_delta += r.delta_from_seed_m;
      ++aligned;
      if (!ao.debug_dir.empty())
        ++figured;

      if (!opt.dry_run)
        db.save_panorama_pose(pano.id, r.pose, r.inliers, r.rms_deg);
    }
    db.log_pipeline_end(log_id, true);
  } catch (...) {
    db.log_pipeline_end(log_id, false, "align_360 failed");
    throw;
  }

  spdlog::info("Alignment complete: {} aligned, {} skipped, {} failed"
               "{}. Mean correction {:.3f} m",
               aligned, skipped, failed,
               opt.dry_run ? " (dry-run, no poses written)" : "",
               aligned > 0 ? sum_delta / aligned : 0.0);

  return RuxError::SUCCESS;
}
