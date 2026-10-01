// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Survey operations that need a project. Thin over ProjectDB storage and the
/// pure rules in survey.hpp — the gate and redistribution live here, not in
/// ProjectDB, which stays storage-only.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/survey.hpp"

#include <map>
#include <vector>

namespace reusex::core {

std::map<int64_t, EnvironmentStatus>
environment_statuses(const ProjectDB &db); // every type
EnvironmentStatus environment_status_of(const ProjectDB &db, int64_t type_id);
/// Approving a type whose miljøstatus is `afventer` throws SamplePendingError.
ProjectDB::SurveyTypeRecord set_review_status(ProjectDB &db, int64_t type_id,
                                              ReviewStatus status);
/// Scales the type's parts to sum to `total` (redistribute_quantity).
/// Throws std::invalid_argument when the type has no parts or total < 0.
std::vector<ProjectDB::SurveyPartRecord>
set_type_quantity(ProjectDB &db, int64_t type_id, double total);
std::vector<TypeTotals>
type_totals(const ProjectDB &db); // one per survey type, same order
/// Sample edit with the stage/result rule enforced (validate_sample_state on
/// the merged state).
ProjectDB::SampleRecord
update_sample_checked(ProjectDB &db, int64_t id,
                      const ProjectDB::SamplePatch &patch);

} // namespace reusex::core
