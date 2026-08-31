// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// Severity of a single validation finding.
enum class ValidationSeverity { warning, error };

/// One problem discovered by a validation check.
struct ValidationIssue {
  std::string check; ///< Machine-readable check id (e.g. "orphaned_passport").
  std::string message; ///< Human-readable description.
  ValidationSeverity severity = ValidationSeverity::error;
};

/// Aggregate result of running all validation checks over a project.
struct ValidationReport {
  std::vector<ValidationIssue> issues;

  /// True when no error-severity issue was found.
  bool ok() const;
  size_t error_count() const;
  size_t warning_count() const;
};

// ── Individual checks (each appends to `out`) ──────────────────────────────
// These are read-only. Split out so they can be unit-tested in isolation.

/// Material passports not referenced by any instance_materials link.
/// Reported as warnings (an unused passport is not necessarily a bug).
void check_orphaned_passports(const ProjectDB &db,
                              std::vector<ValidationIssue> &out);

/// instance_materials rows whose instance row or passport is missing.
void check_dangling_instance_materials(const ProjectDB &db,
                                       std::vector<ValidationIssue> &out);

/// instances rows with no corresponding label_definitions entry.
void check_instances_without_label_defs(const ProjectDB &db,
                                        std::vector<ValidationIssue> &out);

/// Parallel/sibling clouds (cloud, labels, normals, planes, rooms, instances)
/// whose point counts disagree — they are supposed to be index-aligned.
void check_sibling_cloud_sizes(const ProjectDB &db,
                               std::vector<ValidationIssue> &out);

/// Run every check and aggregate the findings.
ValidationReport validate_project(const ProjectDB &db);

// ── Stage input contracts (#222) ───────────────────────────────────────────
// Each pipeline stage consumes a known set of named clouds/tables and produces
// another (documented in docs/CONTRACTS.md). `rux validate --stage <name>`
// asserts a stage's *inputs* exist and are mutually consistent before the stage
// runs, so a missing/misaligned prerequisite fails loudly instead of crashing
// deep inside the algorithm.

/// Pipeline stages that have an input contract. Ordered by pipeline position.
enum class PipelineStage {
  import,    ///< extract raw sensor frames from a scan
  optimize,  ///< refine per-frame sensor poses (a.k.a. register)
  clouds,    ///< back-project sensor frames into "cloud" + "normals"
  planes,    ///< detect planar surfaces
  rooms,     ///< partition into rooms
  instances, ///< separate semantic labels into spatial instances
  mesh,      ///< generate the reconstructed mesh
};

/// Parse a stage name (e.g. "mesh", "register") to a PipelineStage.
/// Returns std::nullopt for an unknown name. "register" is an alias for
/// "optimize".
std::optional<PipelineStage> parse_pipeline_stage(std::string_view name);

/// Canonical lower-case name of a stage (the primary CLI token).
std::string_view to_string(PipelineStage stage);

/// All stage names accepted by parse_pipeline_stage(), for help text.
std::vector<std::string> pipeline_stage_names();

/// Assert that @p stage's input clouds/tables exist in @p db and are
/// index-aligned. Appends an issue per missing input or size mismatch.
void check_stage_inputs(const ProjectDB &db, PipelineStage stage,
                        std::vector<ValidationIssue> &out);

/// Run check_stage_inputs for a single stage and aggregate the findings.
ValidationReport validate_stage(const ProjectDB &db, PipelineStage stage);

} // namespace reusex::core
