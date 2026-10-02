// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/resources.hpp"

#include <reusex/core/resource_export.hpp>
#include <reusex/core/resource_keys.hpp>
#include <reusex/core/resource_templates.hpp>
#include <reusex/core/resources.hpp>

#include <optional>
#include <vector>

namespace rux::gui {
namespace {
using json = nlohmann::json;
namespace core = reusex::core;

json parse_body(const std::string &body) {
  auto j = json::parse(body.empty() ? "{}" : body, nullptr,
                       /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object())
    throw HttpError(400, "request body must be a JSON object");
  return j;
}

json key_json(const core::ResourceKey &k) {
  return {{"id", k.id},
          {"label", k.label},
          {"category", k.category},
          {"scope", std::string(core::to_string(k.scope))},
          {"data_type", k.data_type},
          {"unit", k.unit.empty() ? json(nullptr) : json(k.unit)},
          {"options", k.options},
          {"editable", k.editable}};
}

json resource_json(const core::Resource &r) {
  json values = json::object();
  for (const auto &v : r.values)
    values[v.key] = v.value ? json(*v.value) : json(nullptr);
  return {{"code", r.code},
          {"type_id", r.type_id},
          {"manual", r.manual},
          {"values", std::move(values)}};
}

/// The ?template=<id> value; nullopt without the parameter.
/// @throws HttpError(400) for an empty or non-integer value — an empty one
///         is not "no template" and must not 404 as template 0.
std::optional<int64_t> template_id_param(const Params &params) {
  const auto raw = params.find("template");
  if (!raw)
    return std::nullopt;
  if (raw->empty())
    throw HttpError(400, "query parameter 'template' must be an integer, "
                         "got an empty value");
  return params.integer("template", 0);
}

/// The template ?template=<id> names; nullopt without the parameter.
/// @throws HttpError(400) (see template_id_param), std::out_of_range (→ 404).
std::optional<core::TemplateView> template_param(const reusex::ProjectDB &db,
                                                 const Params &params) {
  const auto id = template_id_param(params);
  if (!id)
    return std::nullopt;
  return core::template_view(db, *id);
}

std::optional<std::vector<std::string>>
keys_of(const std::optional<core::TemplateView> &view) {
  if (!view)
    return std::nullopt;
  return view->resolved.keys;
}

json template_json(const core::TemplateView &v) {
  return {{"id", v.record.id},
          {"name", v.record.name},
          {"members", core::members_json(v.members)},
          {"csv", core::csv_options_json(v.csv)},
          {"seed", v.record.seed ? json(*v.record.seed) : json(nullptr)},
          {"resolved_keys", v.resolved.keys},
          {"missing", core::members_json(v.resolved.missing)},
          {"created_at", v.record.created_at},
          {"updated_at", v.record.updated_at}};
}

core::TemplateInput template_input(const json &j) {
  core::TemplateInput in;
  if (const auto it = j.find("name"); it != j.end()) {
    if (!it->is_string())
      throw HttpError(400, "'name' must be a string");
    in.name = it->get<std::string>();
  }
  if (const auto it = j.find("members"); it != j.end())
    in.members = *it; // shape checked by core::parse_members (400)
  if (const auto it = j.find("csv"); it != j.end())
    in.csv = *it; // checked by core::parse_csv_options (400)
  return in;
}

json template_list(const reusex::ProjectDB &db) {
  json list = json::array();
  for (const auto &v : core::template_views(db))
    list.push_back(template_json(v));
  return list;
}
} // namespace

json resource_keys_json(const reusex::ProjectDB &db) {
  json list = json::array();
  for (const auto &k : core::key_catalogue(db))
    list.push_back(key_json(k));
  return list;
}

json resources_json(const reusex::ProjectDB &db, const Params &params) {
  return map_library_errors([&] {
    const auto view = template_param(db, params);
    json list = json::array();
    for (const auto &r : core::list_resources(db, keys_of(view)))
      list.push_back(resource_json(r));
    json out{{"resources", std::move(list)}};
    if (view)
      out["template"] = {
          {"id", view->record.id},
          {"resolved_keys", view->resolved.keys},
          {"missing", core::members_json(view->resolved.missing)}};
    return out;
  });
}

json patch_resource_json(reusex::ProjectDB &db, const std::string &code,
                         const Params &params, const std::string &body) {
  const auto j = parse_body(body);
  const auto it = j.find("values");
  if (it == j.end() || !it->is_object())
    throw HttpError(400, "'values' is required: an object of key id -> "
                         "string or null");
  std::vector<core::ResourceWrite> writes;
  for (const auto &[key, value] : it->items()) {
    if (value.is_null())
      writes.push_back({key, std::nullopt});
    else if (value.is_string())
      writes.push_back({key, value.get<std::string>()});
    else
      throw HttpError(400, "value for '" + key + "' must be a string or null");
  }
  return map_library_errors([&] {
    const auto view = template_param(db, params);
    const auto result = core::patch_resource(db, code, writes, keys_of(view));
    json siblings = json::array();
    for (const auto &s : result.siblings)
      siblings.push_back(resource_json(s));
    return json{{"resource", resource_json(result.resource)},
                {"siblings", std::move(siblings)}};
  });
}

json create_resource_json(reusex::ProjectDB &db, const std::string &body) {
  const auto j = parse_body(body);
  const auto t = j.find("type_id");
  if (t == j.end() || !t->is_number_integer())
    throw HttpError(400, "'type_id' is required and must be an integer");
  std::optional<std::string> name;
  if (const auto n = j.find("name"); n != j.end() && !n->is_null()) {
    if (!n->is_string())
      throw HttpError(400, "'name' must be a string");
    name = n->get<std::string>();
  }
  return map_library_errors([&] {
    return resource_json(core::create_resource(db, t->get<int64_t>(), name));
  });
}

void delete_resource(reusex::ProjectDB &db, const std::string &code) {
  map_library_errors([&] { core::delete_resource(db, code); });
}

Blob resources_csv_blob(const reusex::ProjectDB &db, const Params &params) {
  const auto id = template_id_param(params);
  if (!id)
    throw HttpError(400, "'template' is required: the CSV's columns come from "
                         "a template");
  return map_library_errors([&] {
    const auto csv = core::export_resources_csv(db, *id);
    Blob b;
    b.content_type = "text/csv; charset=utf-8";
    b.data.assign(csv.begin(), csv.end());
    return b;
  });
}

json templates_json(const reusex::ProjectDB &db) {
  return {{"templates", template_list(db)}};
}

json create_template_json(reusex::ProjectDB &db, const std::string &body) {
  const auto in = template_input(parse_body(body));
  return map_library_errors(
      [&] { return template_json(core::create_template(db, in)); });
}

json patch_template_json(reusex::ProjectDB &db, int64_t id,
                         const std::string &body) {
  const auto in = template_input(parse_body(body));
  return map_library_errors(
      [&] { return template_json(core::update_template(db, id, in)); });
}

void delete_template(reusex::ProjectDB &db, int64_t id) {
  map_library_errors([&] {
    core::delete_template(db, id);
    return 0;
  });
}

json duplicate_template_json(reusex::ProjectDB &db, int64_t id) {
  return map_library_errors(
      [&] { return template_json(core::duplicate_template(db, id)); });
}

json restore_seed_templates_json(reusex::ProjectDB &db) {
  return map_library_errors([&] {
    json names = json::array();
    for (const auto &v : core::restore_seed_templates(db))
      names.push_back(v.record.name);
    return json{{"restored", std::move(names)},
                {"templates", template_list(db)}};
  });
}

} // namespace rux::gui
