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

/// The template ?template=<id> names; nullopt without the parameter.
/// @throws HttpError(400) for a non-integer, std::out_of_range (→ 404).
std::optional<core::TemplateView> template_param(const reusex::ProjectDB &db,
                                                 const Params &params) {
  if (!params.find("template"))
    return std::nullopt;
  return core::template_view(db, params.integer("template", 0));
}

std::optional<std::vector<std::string>>
keys_of(const std::optional<core::TemplateView> &view) {
  if (!view)
    return std::nullopt;
  return view->resolved.keys;
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
  if (!params.find("template"))
    throw HttpError(400, "'template' is required: the CSV's columns come from "
                         "a template");
  const auto id = params.integer("template", 0);
  return map_library_errors([&] {
    const auto csv = core::export_resources_csv(db, id);
    Blob b;
    b.content_type = "text/csv; charset=utf-8";
    b.data.assign(csv.begin(), csv.end());
    return b;
  });
}

} // namespace rux::gui
