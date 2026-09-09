// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/edits.hpp"

#include "gui/api.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <charconv>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace rux::gui {
namespace {

using json = nlohmann::json;

/// Cloud whose label names are a record, not a caption.
///
/// `create instances` writes names of the form `SM<class>-<id> (<n>p)`, and
/// both the v10 schema migration and io/export_scene.cpp parse the semantic
/// class back out of that string. Letting a user type over it would destroy
/// data while looking like a cosmetic edit, so the whole cloud is off limits
/// rather than the format being defended field by field.
constexpr std::string_view kUnrenamableCloud = "instances";

/// Parse a request body and pull out the required sub-object.
///
/// @throws HttpError(400) if the body is not an object or @p key is missing or
///         is not an object itself.
json required_object(const std::string &body, std::string_view key) {
  auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    throw HttpError(400, "request body must be a JSON object");

  auto it = parsed.find(key);
  if (it == parsed.end() || !it->is_object())
    throw HttpError(400, std::string("'").append(key) +
                             "' is required and must be a JSON object");
  return *it;
}

/// Parse a JSON object key as a positive label id.
int label_id_of(const std::string &key) {
  int id = 0;
  const char *first = key.data();
  const char *last = first + key.size();
  auto [ptr, ec] = std::from_chars(first, last, id);
  if (ec != std::errc{} || ptr != last)
    throw HttpError(400, "label id '" + key + "' is not an integer");
  if (id <= 0)
    throw HttpError(400, "label id " + key +
                             " is not a class: 0 is unlabeled and ids are "
                             "positive");
  return id;
}

} // namespace

json patch_cloud_labels(reusex::ProjectDB &db, const std::string &name,
                        const std::string &body) {
  if (!db.has_point_cloud(name))
    throw HttpError(404, "no such cloud '" + name + "'");

  if (name == kUnrenamableCloud)
    throw HttpError(
        409, "label names of the '" + name +
                 "' cloud encode the semantic class and instance id and are "
                 "parsed back by the pipeline; they cannot be renamed here");

  const json patch = required_object(body, "labels");

  auto legend = db.label_definitions(name);
  if (legend.empty())
    throw HttpError(404, "cloud '" + name + "' has no label definitions");

  // Validate the whole patch before applying any of it. Storage replaces the
  // cloud's map wholesale, so a half-applied patch is not a partial success —
  // it is a rewrite of the map with some edits in and some out.
  std::map<int, std::string> updated = legend;
  for (const auto &[key, value] : patch.items()) {
    const int id = label_id_of(key);

    if (!value.is_string())
      throw HttpError(400, "label " + key + " must be renamed to a string");
    const auto label = value.get<std::string>();
    if (label.empty())
      throw HttpError(400, "label " + key +
                               " cannot be renamed to an empty "
                               "name");

    if (legend.find(id) == legend.end())
      throw HttpError(400, "label " + key + " is not defined for cloud '" +
                               name +
                               "'; this renames existing classes rather than "
                               "creating new ones");

    updated[id] = label;
  }

  if (updated != legend)
    db.save_label_definitions(name, updated);

  return cloud_labels_json(db, name);
}

json patch_material(reusex::ProjectDB &db, const std::string &guid,
                    const std::string &body) {
  const auto guids = db.list_passport_guids();
  if (std::find(guids.begin(), guids.end(), guid) == guids.end())
    throw HttpError(404, "no such material passport '" + guid + "'");

  const json patch = required_object(body, "properties");

  // Validate first, write second — same reasoning as the label patch, except
  // here the failure mode is a passport left half-edited with no indication of
  // which half took.
  std::vector<std::pair<std::string, std::optional<std::string>>> edits;
  for (const auto &[key, value] : patch.items()) {
    if (key.empty())
      throw HttpError(400, "a property name cannot be empty");

    if (value.is_null()) {
      edits.emplace_back(key, std::nullopt);
      continue;
    }
    if (!value.is_string())
      throw HttpError(400, "property '" + key +
                               "' must be a string, or null to clear it");
    edits.emplace_back(key, value.get<std::string>());
  }

  const auto stored = db.passport_stored_properties(guid);
  for (const auto &[field, value] : edits) {
    if (value) {
      db.set_passport_property(guid, field, *value);
      continue;
    }
    // Clearing a property the passport never had is the state the caller asked
    // for, so it succeeds silently. ProjectDB would throw, which would turn an
    // idempotent request into a 500 on its second run.
    if (stored.find(field) != stored.end())
      db.delete_passport_property(guid, field);
  }

  return material_json(db, guid);
}

} // namespace rux::gui
