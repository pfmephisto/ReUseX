// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The optimize stage's JSON parameters -> PlaneGraphOptions, in ONE place.
//
// `rux optimize`, the Qt client's in-process stage and ruxd's web GUI stage
// all read the stage parameters (min_observations, assoc_rounds, no_gnc,
// dry_run; pipeline::stage_parameters(JobStage::optimize)) through this
// function, on top of the library defaults the CLI flags mirror
// (STANDARDS §4). A CLI line and a GUI run with the same parameters therefore
// solve the same problem.

#include <reusex/slam/PlaneGraphOptimizer.hpp>

#include <string_view>

namespace reusex::geometry {

/// Apply the optimize stage's parameter JSON to @p options and @p dry_run.
/// Keys that are absent or null keep the value already there; unknown keys are
/// ignored. `"no_gnc": true` turns GNC off and `false` leaves it as it was.
/// An empty string means "no parameters".
/// @throws std::invalid_argument when @p parameters is not a JSON object, or a
///         known key has the wrong type.
void apply_optimize_parameters(PlaneGraphOptions &options, bool &dry_run,
                               std::string_view parameters);

} // namespace reusex::geometry
