// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/survey_service.hpp"

#include <stdexcept>

namespace reusex::core {

std::map<int64_t, EnvironmentStatus> environment_statuses(const ProjectDB &db) {
  std::map<int64_t, std::vector<SampleState>> linked;
  for (const auto &s : db.samples())
    for (auto t : s.type_ids)
      linked[t].push_back({s.stage, s.result});
  std::map<int64_t, EnvironmentStatus> out;
  for (const auto &t : db.survey_types())
    out[t.id] = environment_status(linked[t.id]);
  return out;
}

EnvironmentStatus environment_status_of(const ProjectDB &db, int64_t type_id) {
  std::vector<SampleState> linked;
  for (const auto &s : db.samples_for_type(type_id))
    linked.push_back({s.stage, s.result});
  return environment_status(linked);
}

ProjectDB::SurveyTypeRecord set_review_status(ProjectDB &db, int64_t type_id,
                                              ReviewStatus status) {
  if (!db.survey_type(type_id))
    throw std::out_of_range("no survey type " + std::to_string(type_id));
  if (status == ReviewStatus::approved &&
      environment_status_of(db, type_id) == EnvironmentStatus::afventer) {
    std::string codes;
    for (const auto &s : db.samples_for_type(type_id))
      if (s.stage != SampleStage::svar)
        codes += (codes.empty() ? "" : ", ") + s.code;
    throw SamplePendingError("survey type " + std::to_string(type_id) +
                             " cannot be approved while sample(s) " + codes +
                             " await a lab answer");
  }
  ProjectDB::SurveyTypePatch p;
  p.review_status = status;
  return db.update_survey_type(type_id, p);
}

std::vector<ProjectDB::SurveyPartRecord>
set_type_quantity(ProjectDB &db, int64_t type_id, double total) {
  if (total < 0.0)
    throw std::invalid_argument("quantity must not be negative");
  std::vector<ProjectDB::SurveyPartRecord> parts;
  for (auto &p : db.survey_parts())
    if (p.type_id == type_id)
      parts.push_back(std::move(p));
  if (parts.empty())
    throw std::invalid_argument("survey type " + std::to_string(type_id) +
                                " has no parts to distribute a quantity over");
  std::vector<double> current;
  for (const auto &p : parts)
    current.push_back(p.quantity);
  const auto next = redistribute_quantity(current, total);
  std::vector<ProjectDB::SurveyPartRecord> out;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    ProjectDB::SurveyPartPatch patch;
    patch.quantity = next[i];
    out.push_back(db.update_survey_part(parts[i].code, patch));
  }
  return out;
}

std::vector<TypeTotals> type_totals(const ProjectDB &db) {
  const auto env = environment_statuses(db);
  std::vector<TypeTotals> out;
  for (const auto &t : db.survey_types())
    out.push_back(
        {t.treatment, t.review_status, t.mass_t, t.eak_code, env.at(t.id)});
  return out;
}

ProjectDB::SampleRecord
update_sample_checked(ProjectDB &db, int64_t id,
                      const ProjectDB::SamplePatch &patch) {
  const auto current = db.sample(id);
  if (!current)
    throw std::out_of_range("no sample " + std::to_string(id));
  validate_sample_state({patch.stage.value_or(current->stage),
                         patch.result.value_or(current->result)});
  return db.update_sample(id, patch);
}

} // namespace reusex::core
