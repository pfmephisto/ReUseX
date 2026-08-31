// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>
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

} // namespace reusex::core
