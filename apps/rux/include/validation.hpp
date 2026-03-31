// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <reusex/core/ProjectDB.hpp>
#include <string>
#include <vector>

namespace rux::validation {

/// Result of validation with error message and resolution hint
struct ValidationResult {
  bool success;
  std::string error_message;           // What's missing/wrong
  std::string resolution_hint;         // Command to run to fix it
  std::vector<std::string> missing_data; // Specific missing items

  explicit operator bool() const { return success; }

  static ValidationResult ok() {
    return ValidationResult{true, "", "", {}};
  }

  static ValidationResult error(std::string msg, std::string hint,
                                std::vector<std::string> missing = {}) {
    return ValidationResult{false, std::move(msg), std::move(hint),
                            std::move(missing)};
  }
};

// Validation functions for each create command
ValidationResult validate_clouds_prerequisites(const ReUseX::ProjectDB &db);
ValidationResult validate_planes_prerequisites(const ReUseX::ProjectDB &db);
ValidationResult validate_rooms_prerequisites(const ReUseX::ProjectDB &db);
ValidationResult validate_mesh_prerequisites(const ReUseX::ProjectDB &db);
ValidationResult validate_texture_prerequisites(const ReUseX::ProjectDB &db);
ValidationResult validate_project_prerequisites(const ReUseX::ProjectDB &db);
ValidationResult validate_annotate_prerequisites(const ReUseX::ProjectDB &db);
ValidationResult validate_instances_prerequisites(const ReUseX::ProjectDB &db,
                                                    const std::string &semantic_cloud_name);

} // namespace rux::validation
