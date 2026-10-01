// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/survey.hpp"

#include <reusex/core/survey.hpp>
#include <reusex/core/survey_service.hpp>

#include <nlohmann/json.hpp>

#include <map>
#include <set>

namespace rux::gui {
namespace {
using json = nlohmann::json;
namespace core = reusex::core;

template <typename T> json opt(const std::optional<T> &v) {
  return v ? json(*v) : json(nullptr);
}

json counts_json(
    const std::vector<reusex::ProjectDB::SurveyTypeRecord> &types) {
  int queue = 0, approved = 0, rejected = 0;
  for (const auto &t : types) {
    if (t.review_status == core::ReviewStatus::queue)
      ++queue;
    else if (t.review_status == core::ReviewStatus::approved)
      ++approved;
    else
      ++rejected;
  }
  return {{"queue", queue},
          {"approved", approved},
          {"rejected", rejected},
          {"all", queue + approved}};
}
} // namespace

json survey_part_json(const reusex::ProjectDB::SurveyPartRecord &p) {
  return {{"code", p.code},
          {"type_id", p.type_id},
          {"cloud", opt(p.cloud_name)},
          {"instance_id", opt(p.instance_id)},
          {"room_id", opt(p.room_id)},
          {"room_name", p.room_name},
          {"quantity", p.quantity},
          {"starred", p.starred},
          {"note", p.note},
          {"material_guid", opt(p.material_guid)},
          {"instance_guid", opt(p.instance_guid)}};
}

json survey_type_json(
    const reusex::ProjectDB::SurveyTypeRecord &t,
    const std::vector<reusex::ProjectDB::SurveyPartRecord> &parts,
    core::EnvironmentStatus env, const std::vector<int64_t> &sample_ids) {
  json part_list = json::array();
  double quantity = 0.0;
  for (const auto &p : parts) {
    part_list.push_back(survey_part_json(p));
    quantity += p.quantity;
  }
  return {{"id", t.id},
          {"name", t.name},
          {"eak_code", t.eak_code},
          {"eak_name", std::string(core::eak_fraction_name(t.eak_code))},
          {"bim7aa_code", t.bim7aa_code},
          {"unit", t.unit},
          {"treatment", std::string(core::to_string(t.treatment))},
          {"review_status", std::string(core::to_string(t.review_status))},
          {"confidence", opt(t.confidence)},
          {"mass_t", opt(t.mass_t)},
          {"note", t.note},
          {"starred", t.starred},
          {"semantic_class", t.semantic_class},
          {"environment_status", std::string(core::to_string(env))},
          {"sample_ids", sample_ids},
          {"quantity", quantity},
          {"parts", std::move(part_list)},
          {"created_at", t.created_at},
          {"updated_at", t.updated_at}};
}

json sample_json(const reusex::ProjectDB::SampleRecord &s) {
  return {{"id", s.id},
          {"code", s.code},
          {"title", s.title},
          {"what", s.what},
          {"stage", std::string(core::to_string(s.stage))},
          {"result", s.result == core::SampleResult::none
                         ? json(nullptr)
                         : json(std::string(core::to_string(s.result)))},
          {"type_ids", s.type_ids},
          {"created_at", s.created_at},
          {"updated_at", s.updated_at}};
}

json survey_json(const reusex::ProjectDB &db) {
  const auto types = db.survey_types();
  const auto env = core::environment_statuses(db);
  std::map<int64_t, std::vector<reusex::ProjectDB::SurveyPartRecord>> parts_of;
  for (auto &p : db.survey_parts())
    parts_of[p.type_id].push_back(std::move(p));
  std::map<int64_t, std::vector<int64_t>> samples_of;
  for (const auto &s : db.samples())
    for (auto t : s.type_ids)
      samples_of[t].push_back(s.id);
  json list = json::array();
  for (const auto &t : types)
    list.push_back(
        survey_type_json(t, parts_of[t.id], env.at(t.id), samples_of[t.id]));
  return {{"types", std::move(list)}, {"counts", counts_json(types)}};
}

json survey_summary_json(const reusex::ProjectDB &db) {
  const auto types = db.survey_types();
  const auto totals = core::type_totals(db);
  const auto breakdown = core::circularity_breakdown(totals);
  json circ = json::object();
  double total = 0.0;
  for (std::size_t i = 0; i < core::kTreatmentCount; ++i) {
    circ[std::string(core::to_string(static_cast<core::Treatment>(i)))] =
        breakdown[i];
    total += breakdown[i];
  }
  const double reuse =
      breakdown[static_cast<std::size_t>(core::Treatment::bevaring)] +
      breakdown[static_cast<std::size_t>(core::Treatment::genbrug)];
  int pending = 0;
  for (const auto &s : db.samples())
    if (s.stage != core::SampleStage::svar)
      ++pending;

  json unlabeled = nullptr;
  if (db.has_point_cloud("instances")) {
    if (const auto cloud = db.point_cloud_label("instances")) {
      std::size_t n = 0;
      for (const auto &p : *cloud)
        if (p.label == 0)
          ++n;
      unlabeled = n;
    }
  }

  json empty_rooms = json::array();
  if (db.has_point_cloud("rooms")) {
    if (const auto cloud = db.point_cloud_label("rooms")) {
      std::set<std::uint32_t> present;
      for (const auto &p : *cloud)
        if (p.label != 0)
          present.insert(p.label);
      std::set<std::uint32_t> covered;
      for (const auto &p : db.survey_parts())
        if (p.room_id)
          covered.insert(*p.room_id);
      const auto names = db.label_definitions("rooms");
      for (auto r : present)
        if (!covered.contains(r)) {
          const auto n = names.find(static_cast<int>(r));
          empty_rooms.push_back(n != names.end() ? n->second
                                                 : "Rum " + std::to_string(r));
        }
    }
  }

  return {{"counts", counts_json(types)},
          {"circularity", std::move(circ)},
          {"total_mass_t", total},
          {"reuse_share", total > 0.0 ? json(reuse / total) : json(nullptr)},
          {"pending_samples", pending},
          {"unlabeled_points", std::move(unlabeled)},
          {"rooms_without_parts", std::move(empty_rooms)}};
}

json survey_fractions_json(const reusex::ProjectDB &db) {
  const auto report = core::fractions_by_eak(core::type_totals(db));
  json list = json::array();
  for (const auto &f : report.fractions)
    list.push_back({{"eak_code", f.eak_code},
                    {"name", f.name},
                    {"treatment", std::string(core::to_string(f.treatment))},
                    {"mass_t", f.mass_t}});
  return {{"fractions", std::move(list)},
          {"total_t", report.total_t},
          {"blocking_types", report.blocking_types},
          {"ready", report.blocking_types == 0}};
}

json samples_json(const reusex::ProjectDB &db) {
  json list = json::array();
  for (const auto &s : db.samples())
    list.push_back(sample_json(s));
  return {{"samples", std::move(list)}};
}

} // namespace rux::gui
