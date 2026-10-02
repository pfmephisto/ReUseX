// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/survey.hpp"

#include <reusex/core/survey.hpp>
#include <reusex/core/survey_service.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

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
  return {
      {"code", p.code},
      {"type_id", p.type_id},
      {"cloud", opt(p.cloud_name)},
      {"instance_id", opt(p.instance_id)},
      {"room_id", opt(p.room_id)},
      {"room_name", p.room_name},
      {"quantity", p.quantity},
      {"starred", p.starred},
      {"note", p.note},
      {"material_guid", opt(p.material_guid)},
      {"instance_guid", opt(p.instance_guid)},
      {"orphaned", p.instance_guid.has_value() && !p.instance_id.has_value()}};
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

  const std::string instances_cloud = core::SurveySyncOptions{}.instances_cloud;
  const std::string rooms_cloud = core::SurveySyncOptions{}.rooms_cloud;

  json unlabeled = nullptr;
  json classified = nullptr;
  if (db.has_point_cloud(instances_cloud)) {
    if (const auto cloud = db.point_cloud_label(instances_cloud)) {
      std::size_t n = 0;
      for (const auto &p : *cloud)
        if (p.label == 0)
          ++n;
      unlabeled = n;
      if (!cloud->empty())
        classified =
            1.0 - static_cast<double>(n) / static_cast<double>(cloud->size());
    }
  }

  int contaminated = 0;
  for (const auto &t : totals)
    if (t.status != core::ReviewStatus::rejected &&
        t.environment == core::EnvironmentStatus::forurenet)
      ++contaminated;

  json empty_rooms = json::array();
  if (db.has_point_cloud(rooms_cloud)) {
    if (const auto cloud = db.point_cloud_label(rooms_cloud)) {
      std::set<std::uint32_t> present;
      for (const auto &p : *cloud)
        if (p.label != 0)
          present.insert(p.label);
      std::set<std::uint32_t> covered;
      for (const auto &p : db.survey_parts())
        if (p.room_id)
          covered.insert(*p.room_id);
      const auto names = db.label_definitions(rooms_cloud);
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
          {"classified_share", std::move(classified)},
          {"contaminated_types", contaminated},
          {"unlabeled_points", std::move(unlabeled)},
          {"rooms_without_parts", std::move(empty_rooms)}};
}

namespace {
/// Tonnes on the wire: 6 decimals, so sums like 190 + 6.8 + 2.4 print as 199.2
/// rather than 199.20000000000002.
double wire_tonnes(double t) { return std::round(t * 1e6) / 1e6; }
} // namespace

json survey_fractions_json(const reusex::ProjectDB &db) {
  const auto report = core::fractions_by_eak(core::type_totals(db));
  json list = json::array();
  for (const auto &f : report.fractions)
    list.push_back({{"eak_code", f.eak_code},
                    {"name", f.name},
                    {"treatment", std::string(core::to_string(f.treatment))},
                    {"mass_t", wire_tonnes(f.mass_t)},
                    {"contaminated", f.contaminated}});
  json blocking = json::array();
  for (const auto &b : report.blocking)
    blocking.push_back(
        {{"type_id", b.type_id},
         {"name", b.name},
         {"eak_code", b.eak_code},
         {"treatment", std::string(core::to_string(b.treatment))},
         {"mass_t", opt(b.mass_t)},
         {"reason", std::string(core::to_string(b.reason))}});
  // Ready means there is something to report and nothing holds it back: an
  // empty survey (or one that is all bevaring) has no fraction to send.
  const bool ready = !list.empty() && report.blocking_types == 0;
  return {{"fractions", std::move(list)},
          {"total_t", wire_tonnes(report.total_t)},
          {"blocking_types", report.blocking_types},
          {"blocking", std::move(blocking)},
          {"ready", ready}};
}

json samples_json(const reusex::ProjectDB &db) {
  json list = json::array();
  for (const auto &s : db.samples())
    list.push_back(sample_json(s));
  return {{"samples", std::move(list)}};
}

// ===========================================================================
// writes (#265 Phase 2)
// ===========================================================================

namespace {

json parse_object(const std::string &body) {
  auto j = json::parse(body.empty() ? "{}" : body, nullptr,
                       /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object())
    throw HttpError(400, "request body must be a JSON object");
  return j;
}
std::optional<std::string> opt_string(const json &j, const char *key) {
  auto it = j.find(key);
  if (it == j.end())
    return std::nullopt;
  if (!it->is_string())
    throw HttpError(400, std::string("'") + key + "' must be a string");
  return it->get<std::string>();
}
std::optional<bool> opt_bool(const json &j, const char *key) {
  auto it = j.find(key);
  if (it == j.end())
    return std::nullopt;
  if (!it->is_boolean())
    throw HttpError(400, std::string("'") + key + "' must be a boolean");
  return it->get<bool>();
}
/// Present-and-null clears; present-and-number sets; absent leaves alone.
std::optional<std::optional<double>> opt_nullable_number(const json &j,
                                                         const char *key) {
  auto it = j.find(key);
  if (it == j.end())
    return std::nullopt;
  if (it->is_null())
    return std::optional<double>{};
  if (!it->is_number())
    throw HttpError(400, std::string("'") + key + "' must be a number or null");
  const double v = it->get<double>();
  if (!std::isfinite(v))
    throw HttpError(400, std::string("'") + key + "' must be a finite number");
  return std::optional<double>{v};
}
std::optional<double> opt_quantity(const json &j, const char *key) {
  auto it = j.find(key);
  if (it == j.end())
    return std::nullopt;
  if (!it->is_number())
    throw HttpError(400, std::string("'") + key + "' must be a number >= 0");
  const double v = it->get<double>();
  if (!std::isfinite(v))
    throw HttpError(400, std::string("'") + key + "' must be a finite number");
  if (v < 0.0)
    throw HttpError(400, std::string("'") + key + "' must be a number >= 0");
  return v;
}
template <typename E>
std::optional<E> opt_enum(const json &j, const char *key,
                          std::optional<E> (*parse)(std::string_view)) {
  const auto s = opt_string(j, key);
  if (!s)
    return std::nullopt;
  const auto v = parse(*s);
  if (!v)
    throw HttpError(400,
                    std::string("'") + key + "' has no value '" + *s + "'");
  return v;
}
std::vector<int64_t> id_list(const json &j, const char *key) {
  auto it = j.find(key);
  if (it == j.end())
    return {};
  if (!it->is_array())
    throw HttpError(400,
                    std::string("'") + key + "' must be an array of integers");
  std::vector<int64_t> out;
  for (const auto &v : *it) {
    if (!v.is_number_integer())
      throw HttpError(400, std::string("'") + key +
                               "' must be an array of integers");
    out.push_back(v.get<int64_t>());
  }
  return out;
}
/// Runs f, translating library exceptions into the documented statuses.
template <typename F> auto mapped(F &&f) -> decltype(f()) {
  try {
    return f();
  } catch (const HttpError &) {
    throw;
  } catch (const core::SamplePendingError &e) {
    throw HttpError(422, e.what());
  } catch (const std::out_of_range &e) {
    throw HttpError(404, e.what());
  } catch (const std::invalid_argument &e) {
    throw HttpError(422, e.what());
  }
}
json type_by_id(const reusex::ProjectDB &db, int64_t id) {
  // survey_json(db) is bound to a local first, not iterated inline: binding
  // a range-based for's hidden __range directly to `survey_json(db).at(...)`
  // binds to the *reference* .at() returns, not to the temporary itself, so
  // the temporary (and the array backing it) is destroyed before the loop
  // body runs — a dangling-reference trap only C++23's P2644 fixes, and this
  // project builds as C++20.
  const auto survey = survey_json(db);
  for (const auto &t : survey.at("types"))
    if (t.at("id") == id)
      return t;
  throw HttpError(404, "no survey type " + std::to_string(id));
}

} // namespace

json sync_survey_json(reusex::ProjectDB &db, const std::string &body) {
  const auto j = parse_object(body);
  core::SurveySyncOptions opts;
  if (auto v = opt_string(j, "instances_cloud"))
    opts.instances_cloud = *v;
  if (auto v = opt_string(j, "semantic_cloud"))
    opts.semantic_cloud = *v;
  if (auto v = opt_string(j, "rooms_cloud"))
    opts.rooms_cloud = *v;
  try {
    const auto r = core::sync_survey(db, opts);
    return {{"types_created", r.types_created},
            {"parts_created", r.parts_created},
            {"parts_existing", r.parts_existing},
            {"instances_seen", r.instances_seen},
            {"instances_backfilled", r.instances_backfilled},
            {"rooms_assigned", r.rooms_assigned},
            {"parts_orphaned", r.parts_orphaned},
            {"orphaned_codes", r.orphaned_codes},
            {"links_restored", r.links_restored}};
  } catch (const std::runtime_error &e) {
    if (std::string(e.what()).find("rux create instances") != std::string::npos)
      throw HttpError(422, e.what());
    throw;
  }
}

json create_survey_type_json(reusex::ProjectDB &db, const std::string &body) {
  const auto j = parse_object(body);
  reusex::ProjectDB::SurveyTypeRecord t;
  const auto name = opt_string(j, "name");
  if (!name || name->empty())
    throw HttpError(400, "'name' is required and must be non-empty");
  t.name = *name;
  if (auto v = opt_string(j, "eak_code"))
    t.eak_code = *v;
  if (auto v = opt_string(j, "bim7aa_code"))
    t.bim7aa_code = *v;
  if (auto v = opt_string(j, "unit"))
    t.unit = *v;
  if (auto v = opt_enum<core::Treatment>(j, "treatment",
                                         core::treatment_from_string))
    t.treatment = *v;
  // Manually created types are never matched by sync_survey's
  // semantic_class seeding (kManualSemanticClass, distinct from the -1
  // "Uklassificeret" sentinel sync_survey itself uses).
  t.semantic_class = core::kManualSemanticClass;
  return type_by_id(db, db.add_survey_type(t).id);
}

json patch_survey_type_json(reusex::ProjectDB &db, int64_t id,
                            const std::string &body) {
  const auto j = parse_object(body);
  reusex::ProjectDB::SurveyTypePatch p;
  p.name = opt_string(j, "name");
  if (p.name && p.name->empty())
    throw HttpError(400, "'name' must be non-empty");
  p.eak_code = opt_string(j, "eak_code");
  p.bim7aa_code = opt_string(j, "bim7aa_code");
  p.unit = opt_string(j, "unit");
  p.note = opt_string(j, "note");
  p.treatment =
      opt_enum<core::Treatment>(j, "treatment", core::treatment_from_string);
  p.confidence = opt_nullable_number(j, "confidence");
  if (p.confidence && *p.confidence &&
      (**p.confidence < 0.0 || **p.confidence > 1.0))
    throw HttpError(400, "'confidence' must be within [0, 1]");
  p.mass_t = opt_nullable_number(j, "mass_t");
  p.starred = opt_bool(j, "starred");
  const auto quantity = opt_quantity(j, "quantity");
  const auto status = opt_enum<core::ReviewStatus>(
      j, "review_status", core::review_status_from_string);
  return mapped([&] {
    if (!db.survey_type(id))
      throw std::out_of_range("no survey type " + std::to_string(id));
    // Check every refusal before the first write, so a refused request
    // changes nothing.
    if (status == core::ReviewStatus::approved &&
        core::environment_status_of(db, id) ==
            core::EnvironmentStatus::afventer)
      core::set_review_status(db, id, *status); // throws SamplePendingError
    if (quantity) {
      bool has_parts = false;
      for (const auto &part : db.survey_parts())
        has_parts = has_parts || part.type_id == id;
      if (!has_parts)
        throw std::invalid_argument(
            "survey type has no parts to distribute a quantity over");
    }
    db.update_survey_type(id, p);
    if (quantity)
      core::set_type_quantity(db, id, *quantity);
    if (status)
      core::set_review_status(db, id, *status);
    return type_by_id(db, id);
  });
}

json patch_survey_part_json(reusex::ProjectDB &db, const std::string &code,
                            const std::string &body) {
  const auto j = parse_object(body);
  reusex::ProjectDB::SurveyPartPatch p;
  if (auto it = j.find("type_id"); it != j.end()) {
    if (!it->is_number_integer())
      throw HttpError(400, "'type_id' must be an integer");
    p.type_id = it->get<int64_t>();
  }
  p.quantity = opt_quantity(j, "quantity");
  p.starred = opt_bool(j, "starred");
  p.note = opt_string(j, "note");
  p.room_name = opt_string(j, "room_name");
  return mapped(
      [&] { return survey_part_json(db.update_survey_part(code, p)); });
}

json create_sample_json(reusex::ProjectDB &db, const std::string &body) {
  const auto j = parse_object(body);
  const auto title = opt_string(j, "title");
  if (!title || title->empty())
    throw HttpError(400, "'title' is required and must be non-empty");
  const auto what = opt_string(j, "what").value_or("");
  const auto types = id_list(j, "type_ids");
  // add_sample checks every refusal (unknown type -> out_of_range -> 404)
  // before its first write, and inserts the row and its links in one
  // transaction. A new sample always starts at `planlagt`.
  return mapped(
      [&] { return sample_json(db.add_sample(*title, what, types)); });
}

json patch_sample_json(reusex::ProjectDB &db, int64_t id,
                       const std::string &body) {
  const auto j = parse_object(body);
  reusex::ProjectDB::SamplePatch p;
  p.title = opt_string(j, "title");
  p.what = opt_string(j, "what");
  p.stage =
      opt_enum<core::SampleStage>(j, "stage", core::sample_stage_from_string);
  if (auto it = j.find("result"); it != j.end()) {
    if (it->is_null())
      p.result = core::SampleResult::none;
    else
      p.result = opt_enum<core::SampleResult>(j, "result",
                                              core::sample_result_from_string);
    if (p.result == core::SampleResult::none && !it->is_null())
      throw HttpError(400, "'result' must be 'ren', 'forurenet' or null");
  }
  return mapped(
      [&] { return sample_json(core::update_sample_checked(db, id, p)); });
}

void delete_sample(reusex::ProjectDB &db, int64_t id) {
  if (!db.delete_sample(id))
    throw HttpError(404, "no sample " + std::to_string(id));
}

json set_sample_links_json(reusex::ProjectDB &db, int64_t id,
                           const std::string &body) {
  const auto j = parse_object(body);
  if (!j.contains("type_ids"))
    throw HttpError(400, "'type_ids' is required");
  const auto types = id_list(j, "type_ids");
  return mapped([&] {
    db.set_sample_links(id, types);
    return sample_json(*db.sample(id));
  });
}

} // namespace rux::gui
