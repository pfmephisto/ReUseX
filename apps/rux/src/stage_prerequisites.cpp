// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "stage_prerequisites.hpp"
#include "global-params.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <spdlog/spdlog.h>

#include <set>

namespace rux {

int check_stage_prerequisites(
    const reusex::ProjectDB &db, reusex::core::PipelineStage stage,
    const reusex::core::ArtifactOverrides &overrides) {
  std::vector<reusex::core::ValidationIssue> issues;
  try {
    reusex::core::check_stage_inputs(db, stage, issues, overrides);
  } catch (const std::exception &e) {
    // Reading the project state failed, which is not the same as the contract
    // being unsatisfied — say so rather than blaming the user's inputs.
    spdlog::error("Could not check '{}' stage prerequisites: {}",
                  reusex::core::to_string(stage), e.what());
    return RuxError::IO;
  }

  bool refused = false;
  // A stage missing six inputs derives the same "run these in order" hint six
  // times. Printing it once keeps the refusal readable; the per-issue hints are
  // still all there for `rux validate --json` and the HTTP front end.
  std::set<std::string> printed_hints;
  for (const auto &issue : issues) {
    if (issue.severity == reusex::core::ValidationSeverity::warning) {
      spdlog::warn("{}", issue.message);
      continue;
    }
    refused = true;
    spdlog::error("{}", issue.message);
    // At `warn`, not `info`: rux's default level is warn, so the hint the old
    // per-command checks logged at info was invisible unless the user already
    // knew to pass -v. A resolution nobody sees is not a resolution.
    if (!issue.hint.empty() && printed_hints.insert(issue.hint).second)
      spdlog::warn("Resolution: {}", issue.hint);
  }

  return refused ? RuxError::INVALID_ARGUMENT : RuxError::SUCCESS;
}

} // namespace rux
