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
#include <optional>
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

/// One survey type as the PDF report's Kortlægning section lists it.
struct ReportSurveyRow {
  std::string name, bim7aa_code, eak_code, unit;
  double quantity = 0.0; // sum of the type's parts
  std::optional<double> mass_t;
  Treatment treatment = Treatment::genanvendelse;
  EnvironmentStatus environment = EnvironmentStatus::ren_screening;
};
/// One row per **reportable** survey type, in id order, for the PDF report's
/// survey section ("kun godkendte mængder indgår"). A type is listed when it is
/// approved and not awaiting a sample (core::reportable) and it does not block
/// the waste report — so the rows agree with fractions_by_eak: an approved,
/// non-bevaring type without tonnes blocks there (reason `mass`) and is left
/// out here too. A bevaring type without tonnes never blocks (it is not
/// waste), so it is listed with mass_t nullopt. `quantity` is the sum of the
/// type's parts.
std::vector<ReportSurveyRow> report_survey_rows(const ProjectDB &db);

/// Options for `sync_survey` — the cloud names it reads from.
struct SurveySyncOptions {
  std::string instances_cloud =
      std::string(kDefaultInstanceCloud); // mirrors `rux create materials`
  std::string semantic_cloud = "labels";
  std::string rooms_cloud = "rooms";
};
/// Counts of what `sync_survey` changed.
struct SurveySyncReport {
  std::size_t types_created = 0;
  std::size_t parts_created = 0;
  std::size_t parts_existing = 0;
  bool rooms_assigned = false;
  /// Parts whose instance_guid no longer resolves to an instance row (the
  /// instance was dropped, or re-created without carrying the guid over).
  /// Kept and still counted toward their type's quantity — review or re-file
  /// them; sync_survey does not delete or hide them.
  std::size_t parts_orphaned = 0;
  std::vector<std::string> orphaned_codes;
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
