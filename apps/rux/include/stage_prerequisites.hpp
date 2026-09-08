// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The CLI's one gate onto the pipeline stage contract (#246).
//
// This replaces apps/rux/src/validation.cpp, which was a second, hand-written
// copy of the prerequisite rules in core/stage_contract.hpp. Because it was
// separate, `rux validate --stage mesh` and `rux create mesh` could disagree,
// and `rux create texture` was gated by a check nobody called. There is now
// exactly one implementation, and this is a thin reporting wrapper over it.

#include <reusex/core/validate.hpp>

#include <string>

namespace reusex {
class ProjectDB;
}

namespace rux {

/// Check @p stage's documented inputs against @p db and report any problem on
/// the CLI log, including the derived "run these commands in order" hint.
///
/// @param overrides Substitutions for the artifact names a flag can retarget
///        (e.g. `{{"labels", opt.semantic_cloud_name}}`).
/// @returns RuxError::SUCCESS when the contract is satisfied, otherwise
///          RuxError::INVALID_ARGUMENT — the inputs are the caller's mistake,
///          not an internal failure.
int check_stage_prerequisites(
    const reusex::ProjectDB &db, reusex::core::PipelineStage stage,
    const reusex::core::ArtifactOverrides &overrides = {});

} // namespace rux
