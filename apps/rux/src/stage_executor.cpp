// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Moved here from apps/rux/src/gui.cpp when `rux gui` moved into ruxd: the
// Qt client still runs stages in-process through rux_lib.

#include "stage_executor.hpp"
#include "optimize.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>

#include <fmt/format.h>

#include <exception>

namespace {

// LAYERING: reusex_pipeline does not link reusex_slam (GTSAM, #464). This
// executor lives in rux_lib, which links the full `reusex` umbrella, and is
// handed to the Qt client's JobRunner through GuiLaunch.

reusex::pipeline::StageResult
run_optimize_stage(reusex::ProjectDB &db,
                   const reusex::pipeline::StageContext &ctx) {
  // `rux optimize`'s own options with its flag defaults, then the job's
  // parameters through the reader the CLI uses too (optimize.hpp): a GUI run
  // and the copied `rux optimize …` line solve the same problem.
  reusex::geometry::PlaneGraphOptions options =
      plane_graph_options(SubcommandOptimizeOptions{});
  bool dry_run = false;
  try {
    apply_optimize_parameters(options, dry_run, ctx.parameters);
  } catch (const std::exception &e) {
    return reusex::pipeline::StageResult::invalid(e.what());
  }

  try {
    auto result = reusex::geometry::optimize_sensor_poses(db, options, dry_run);

    if (result.landmarks == 0 && result.loop_edges == 0)
      return reusex::pipeline::StageResult::success(
          fmt::format("no plane landmarks reached min_observations={} and no "
                      "loop edges present; poses left unchanged",
                      options.min_landmark_observations));

    if (!result.converged)
      return reusex::pipeline::StageResult::failure(
          "factor-graph optimizer failed to produce a solution; poses left "
          "unchanged");

    if (dry_run)
      return reusex::pipeline::StageResult::success(
          fmt::format("dry run: {} frames, {} landmarks, {:.4f} -> {:.4f} "
                      "error, max shift {:.4f} m (poses not written)",
                      result.frames, result.landmarks, result.initial_error,
                      result.final_error, result.max_pose_shift));

    return reusex::pipeline::StageResult::success(
        fmt::format("{} frames, {} landmarks, {:.4f} -> {:.4f} error, max "
                    "shift {:.4f} m",
                    result.frames, result.landmarks, result.initial_error,
                    result.final_error, result.max_pose_shift),
        // `sensor_frames` is the artifact optimized poses are written into.
        {{"table", "sensor_frames", static_cast<int64_t>(result.frames)}});
  } catch (const std::exception &e) {
    return reusex::pipeline::StageResult::failure(
        fmt::format("pose optimization failed: {}", e.what()));
  }
}

} // namespace

/// StageExecutor that handles `optimize` directly and delegates everything else
/// to the default executor (clouds / planes / rooms / instances / mesh).
reusex::pipeline::StageExecutor make_stage_executor() {
  auto default_exec = reusex::pipeline::default_stage_executor();
  return [default_exec = std::move(default_exec)](
             const reusex::pipeline::StageContext &ctx)
             -> reusex::pipeline::StageResult {
    if (ctx.stage == reusex::pipeline::JobStage::optimize) {
      try {
        reusex::ProjectDB db(ctx.project, /*readOnly=*/false);
        return run_optimize_stage(db, ctx);
      } catch (const std::exception &e) {
        return reusex::pipeline::StageResult::failure(fmt::format(
            "could not open project '{}': {}", ctx.project.string(), e.what()));
      }
    }
    return default_exec(ctx);
  };
}
