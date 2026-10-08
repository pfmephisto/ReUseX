// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The mesh stage's parameters -> MeshOptions, in ONE place.
//
// `rux create mesh` drives the solver itself (it also feeds the interactive
// viewer), while the GUI and the Qt client run the mesh stage through
// run_stage(). Both read their options here, from the same JSON keys
// (stage_parameters(JobStage::mesh)), so a CLI line and a GUI run with the
// same parameters solve the same problem.

#include <reusex/reconstruction/mesh.hpp>

#include <string>
#include <string_view>

namespace reusex::pipeline {

struct MeshStageOptions {
  geometry::MeshOptions options;
  std::string output_name = "mesh";
};

/// Read the mesh stage's JSON parameters ("" = defaults). Unknown keys are
/// ignored.
/// @throws std::invalid_argument for a non-object JSON or an unknown solver.
MeshStageOptions mesh_options_from_parameters(std::string_view parameters);

} // namespace reusex::pipeline
