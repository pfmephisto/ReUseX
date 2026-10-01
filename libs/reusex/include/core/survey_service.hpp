// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Survey operations that need a project. Thin over ProjectDB storage and the
/// pure rules in survey.hpp — the gate and redistribution live here, not in
/// ProjectDB, which stays storage-only.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/survey.hpp"

#include <cstddef>
#include <map>
#include <string>
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

/// Options for `sync_survey` — the cloud names it reads from.
struct SurveySyncOptions {
  std::string instances_cloud = "instances"; // mirrors `rux create materials`
  std::string semantic_cloud = "labels";
  std::string rooms_cloud = "rooms";
};
/// Counts of what `sync_survey` changed.
struct SurveySyncReport {
  std::size_t types_created = 0;
  std::size_t parts_created = 0;
  std::size_t parts_existing = 0;
  bool rooms_assigned = false;
};
/// Fill survey_types / survey_parts from the instances table: one type per
/// semantic class, one bygningsdel (survey part) per instance, placed in the
/// room most of its points fall in. Idempotent — only adds instances that
/// have no part yet (has_survey_part_for, keyed on instance guid) and never
/// touches existing types/parts, so edits made in the GUI survive a rerun.
/// Throws std::runtime_error naming `rux create instances` if the instances
/// cloud does not exist.
SurveySyncReport sync_survey(ProjectDB &db, const SurveySyncOptions &opts = {});

} // namespace reusex::core
