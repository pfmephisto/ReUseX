// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Database-level pipeline stage execution (#265, Phase 1).
//
// The algorithms in segmentation/ and reconstruction/ are pure functions over
// PCL clouds; the "load the named clouds out of a ProjectDB, run the algorithm,
// write the results back, record it in pipeline_log" wrapper used to live only
// inside apps/rux/src/create/*.cpp. This header lifts that wrapper DOWN into
// the library so both `rux create <stage>` and the GUI/daemon job runners drive
// the exact same code path (issue #265: "shared logic goes DOWN into the
// library, not sideways between apps").
//
// Layering: this is the `pipeline` module, which sits ABOVE the Layer-3 peers
// (segmentation / reconstruction / slam / io) — it is the one place allowed to
// link several of them at once. See docs/STANDARDS.md §1.

#include <atomic>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::pipeline {

/// A pipeline stage that can be executed as a job.
///
/// Deliberately narrower than core::PipelineStage (which also covers stages
/// with no runner yet) and than core::Stage (which is a progress-reporting
/// phase, not a unit of work). Extending this enum is how a stage becomes
/// GUI-runnable; see docs/gui/openapi.yaml, which lists the same names.
enum class JobStage {
  clouds,    ///< back-project sensor frames into "cloud" + "normals"
  planes,    ///< detect planar surfaces -> "planes"/"plane_*"
  rooms,     ///< partition planes into rooms -> "rooms"
  instances, ///< split semantic labels into spatial instances
};

/// Canonical lower-case stage name (the token used on the wire and the CLI).
std::string_view to_string(JobStage stage);

/// Parse a canonical stage name. Returns nullopt for an unknown name.
std::optional<JobStage> parse_job_stage(std::string_view name);

/// Every name accepted by parse_job_stage(), in pipeline order.
std::vector<std::string> job_stage_names();

/// True when the stage threads the cancel token into its inner loop, i.e.
/// cancellation takes effect mid-run rather than only between jobs.
bool stage_supports_cancellation(JobStage stage);

/// Outcome of one stage execution.
struct StageResult {
  bool ok = false;        ///< The stage completed and wrote its outputs.
  bool cancelled = false; ///< The stage stopped early on a cancel request.
  std::string message;    ///< Human-readable summary or failure reason.

  static StageResult success(std::string message = {});
  static StageResult failure(std::string message);
  static StageResult cancel(std::string message = "cancelled");
};

/// Everything a stage execution needs, independent of any HTTP/CLI front end.
struct StageContext {
  /// Path to the `.rux` project the stage reads and writes.
  std::filesystem::path project;
  /// Which stage to run.
  JobStage stage = JobStage::clouds;
  /// Stage parameters as a JSON object (empty string = all defaults). Keys
  /// mirror the option-struct fields of the underlying algorithm; unknown keys
  /// are ignored. The same string is persisted into `pipeline_log.parameters`.
  std::string parameters;
  /// Cooperative cancellation flag, owned by the caller. May be null.
  const std::atomic_bool *cancel_token = nullptr;

  /// True when a cancel token is present and set.
  bool is_cancelled() const noexcept;
};

/// Run a stage against an already-open, writable ProjectDB.
///
/// Never throws: algorithm/IO failures are converted into a failed StageResult
/// and, when a pipeline_log row was opened, closed out with the error message
/// (STANDARDS §5 — no silent failure).
StageResult run_stage(ProjectDB &db, const StageContext &ctx);

/// Open `ctx.project` read-write and run the stage against it.
StageResult run_stage(const StageContext &ctx);

/// Stage execution strategy. Exists so JobRunner's state machine can be tested
/// against a fake executor without touching real scan data.
using StageExecutor = std::function<StageResult(const StageContext &)>;

/// The executor that actually runs the pipeline (a thin wrap of run_stage).
StageExecutor default_stage_executor();

} // namespace reusex::pipeline
