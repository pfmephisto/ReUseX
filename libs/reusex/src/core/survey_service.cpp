// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/survey_service.hpp"

#include "reusex/core/logging.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

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
  std::vector<std::pair<std::string, double>> code_quantities;
  for (std::size_t i = 0; i < parts.size(); ++i)
    code_quantities.emplace_back(parts[i].code, next[i]);
  db.set_survey_part_quantities(code_quantities);
  std::vector<ProjectDB::SurveyPartRecord> out;
  for (const auto &p : db.survey_parts())
    if (p.type_id == type_id)
      out.push_back(p);
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

namespace {
std::vector<std::uint32_t> labels_of(const ProjectDB &db,
                                     const std::string &name) {
  std::vector<std::uint32_t> out;
  if (const auto cloud = db.point_cloud_label(name))
    for (const auto &p : *cloud)
      out.push_back(p.label);
  return out;
}
} // namespace

SurveySyncReport sync_survey(ProjectDB &db, const SurveySyncOptions &opts) {
  if (!db.has_point_cloud(opts.instances_cloud))
    throw std::runtime_error("sync_survey: no instance cloud '" +
                             opts.instances_cloud +
                             "' — run `rux create instances` first");
  SurveySyncReport report;

  // Room per instance, when a rooms cloud aligned with the instance cloud
  // exists.
  std::map<std::uint32_t, std::uint32_t> room_of;
  std::map<int, std::string> room_names;
  if (db.has_point_cloud(opts.rooms_cloud)) {
    const auto inst = labels_of(db, opts.instances_cloud);
    const auto rooms = labels_of(db, opts.rooms_cloud);
    if (inst.size() == rooms.size()) {
      room_of = majority_room(inst, rooms);
      room_names = db.label_definitions(opts.rooms_cloud);
      report.rooms_assigned = true;
    } else {
      reusex::warn("sync_survey: '{}' has {} labels but '{}' has {} — clouds "
                   "are out of sync; "
                   "parts get no room (re-run `rux create rooms`)",
                   opts.rooms_cloud, rooms.size(), opts.instances_cloud,
                   inst.size());
    }
  } else {
    reusex::warn("sync_survey: no '{}' cloud; parts get no room (run `rux "
                 "create rooms`)",
                 opts.rooms_cloud);
  }

  const auto class_names = db.has_point_cloud(opts.semantic_cloud)
                               ? db.label_definitions(opts.semantic_cloud)
                               : std::map<int, std::string>{};
  std::map<int, int64_t> type_for_class;
  for (const auto &t : db.survey_types())
    if (t.semantic_class != kManualSemanticClass)
      type_for_class.try_emplace(t.semantic_class, t.id);

  auto instances = db.instances(opts.instances_cloud);
  std::sort(instances.begin(), instances.end(),
            [](const auto &a, const auto &b) {
              return a.instance_id < b.instance_id;
            });
  int next = db.max_survey_part_number();
  for (const auto &inst : instances) {
    if (db.has_survey_part_for(opts.instances_cloud, inst.instance_id)) {
      ++report.parts_existing;
      continue;
    }
    auto it = type_for_class.find(inst.semantic_class);
    if (it == type_for_class.end()) {
      ProjectDB::SurveyTypeRecord t;
      const auto name = class_names.find(inst.semantic_class);
      t.name = inst.semantic_class < 0 ? "Uklassificeret"
               : name != class_names.end()
                   ? name->second
                   : "Klasse " + std::to_string(inst.semantic_class);
      t.semantic_class = inst.semantic_class;
      it = type_for_class.emplace(inst.semantic_class, db.add_survey_type(t).id)
               .first;
      ++report.types_created;
    }
    ProjectDB::SurveyPartRecord part;
    part.code = part_code(++next);
    part.type_id = it->second;
    part.cloud_name = opts.instances_cloud;
    part.instance_id = inst.instance_id;
    if (const auto r = room_of.find(inst.instance_id); r != room_of.end()) {
      part.room_id = r->second;
      const auto n = room_names.find(static_cast<int>(r->second));
      part.room_name = n != room_names.end()
                           ? n->second
                           : "Rum " + std::to_string(r->second);
    }
    db.add_survey_part(part);
    ++report.parts_created;
  }
  if (instances.empty())
    reusex::warn(
        "sync_survey: instance cloud '{}' has no instances; nothing to survey",
        opts.instances_cloud);

  // Parts whose instance_guid is set but no longer resolves to an instance
  // row: the instance was deleted, or recreated without carrying the guid
  // over (e.g. `rux create instances --clear`). They keep their code and
  // quantity and still count toward their type's total — report, don't hide
  // or delete (that is a Phase 3 product decision).
  for (const auto &p : db.survey_parts())
    if (p.instance_guid && !p.instance_id) {
      ++report.parts_orphaned;
      report.orphaned_codes.push_back(p.code);
    }
  if (report.parts_orphaned > 0) {
    std::string codes;
    const std::size_t shown =
        std::min<std::size_t>(20, report.orphaned_codes.size());
    for (std::size_t i = 0; i < shown; ++i)
      codes += (i ? ", " : "") + report.orphaned_codes[i];
    if (report.orphaned_codes.size() > shown)
      codes += "…";
    reusex::warn("sync_survey: {} part(s) link to instances that no longer "
                 "exist ({}); their quantities still count toward their "
                 "types — review or re-file them",
                 report.parts_orphaned, codes);
  }
  return report;
}

} // namespace reusex::core
