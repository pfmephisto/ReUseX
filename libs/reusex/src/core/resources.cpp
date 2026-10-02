// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/resources.hpp"

#include "reusex/core/survey.hpp"
#include "reusex/core/survey_service.hpp"

#include <algorithm>
#include <map>

namespace reusex::core {
namespace {

struct Context {
  std::vector<ResourceKey> catalogue;
  std::map<int64_t, ProjectDB::SurveyTypeRecord> types;
  std::map<int64_t, EnvironmentStatus> env;
};

Context load(const ProjectDB &db) {
  Context c;
  c.catalogue = key_catalogue(db);
  for (auto &t : db.survey_types())
    c.types.emplace(t.id, std::move(t));
  c.env = environment_statuses(db);
  return c;
}

std::optional<std::string> builtin_value(const ResourceKey &k,
                                         const ProjectDB::SurveyPartRecord &p,
                                         const ProjectDB::SurveyTypeRecord &t,
                                         EnvironmentStatus e) {
  const auto &f = k.field;
  if (f == "name")
    return t.name;
  if (f == "quantity")
    return format_number(p.quantity);
  if (f == "unit")
    return t.unit;
  if (f == "eak")
    return t.eak_code;
  if (f == "bim7aa")
    return t.bim7aa_code;
  if (f == "treatment")
    return std::string(to_string(t.treatment));
  if (f == "environment")
    return std::string(to_string(e));
  if (f == "room")
    return p.room_name;
  if (f == "mass_t")
    return t.mass_t ? std::optional<std::string>(format_number(*t.mass_t))
                    : std::nullopt;
  if (f == "note")
    return p.note;
  if (f == "starred")
    return std::string(p.starred ? "true" : "false");
  return std::nullopt;
}

Resource build(const ProjectDB &db, const Context &c,
               const ProjectDB::SurveyPartRecord &p,
               const std::optional<std::vector<std::string>> &keys) {
  Resource r{p.code, p.type_id, !p.instance_guid.has_value(), {}};
  const auto &t = c.types.at(p.type_id);
  const auto e = c.env.at(p.type_id);
  const auto stored = p.material_guid
                          ? db.passport_stored_properties(*p.material_guid)
                          : std::map<std::string, std::string>{};
  auto value = [&](const ResourceKey &k) -> std::optional<std::string> {
    if (k.source == KeySource::builtin)
      return builtin_value(k, p, t, e);
    const auto it = stored.find(k.field);
    return it == stored.end() ? std::nullopt
                              : std::optional<std::string>(it->second);
  };
  if (keys) {
    for (const auto &id : *keys) {
      const auto *k = find_key(c.catalogue, id);
      r.values.push_back({id, k ? value(*k) : std::nullopt});
    }
    return r;
  }
  for (const auto &k : c.catalogue) {
    auto v = value(k);
    if (k.source == KeySource::builtin || v)
      r.values.push_back({k.id, std::move(v)});
  }
  return r;
}

bool leksikon_field_named(std::string_view name) {
  for (const auto &f : leksikon_fields())
    if (f.field_name == name)
      return true;
  return false;
}

void check_column_name(const ProjectDB &db, const std::string &name,
                       const std::string &except_id) {
  if (name.empty())
    throw std::invalid_argument("a column name cannot be empty");
  for (const auto &d : db.list_property_definitions())
    if (d.name == name && d.id != except_id)
      throw NameConflictError("a column named '" + name + "' already exists");
  if (leksikon_field_named(name))
    throw NameConflictError("'" + name +
                            "' is a leksikon field name; choose another "
                            "column name");
}
} // namespace

std::optional<std::string> value_of(const Resource &r, std::string_view key) {
  for (const auto &v : r.values)
    if (v.key == key)
      return v.value;
  return std::nullopt;
}

std::vector<Resource>
list_resources(const ProjectDB &db,
               const std::optional<std::vector<std::string>> &keys) {
  const auto c = load(db);
  std::vector<Resource> out;
  for (const auto &p : db.survey_parts())
    out.push_back(build(db, c, p, keys));
  return out;
}

Resource resource(const ProjectDB &db, std::string_view code,
                  const std::optional<std::vector<std::string>> &keys) {
  const auto p = db.survey_part(code);
  if (!p)
    throw std::out_of_range("no resource '" + std::string(code) + "'");
  return build(db, load(db), *p, keys);
}

ResourcePatchResult
patch_resource(ProjectDB &db, std::string_view code,
               const std::vector<ResourceWrite> &writes,
               const std::optional<std::vector<std::string>> &keys) {
  const auto part = db.survey_part(code);
  if (!part)
    throw std::out_of_range("no resource '" + std::string(code) + "'");
  const auto catalogue = key_catalogue(db);

  // Validate everything first, write second (same rule as patch_material):
  // a refused request changes nothing.
  struct Planned {
    const ResourceKey *key;
    std::optional<std::string> value;
  };
  std::vector<Planned> plan;
  for (const auto &w : writes) {
    const auto *k = find_key(catalogue, w.key);
    if (!k)
      throw KeyValueError(w.key, "unknown key '" + w.key + "'");
    plan.push_back({k, normalise_value(*k, w.value)});
  }

  ProjectDB::SurveyTypePatch tp;
  ProjectDB::SurveyPartPatch pp;
  bool type_written = false;
  std::vector<Planned> passport;
  for (const auto &pl : plan) {
    if (pl.key->source != KeySource::builtin) {
      passport.push_back(pl);
      continue;
    }
    const auto &f = pl.key->field;
    const auto &v = pl.value; // normalise_value refused nulls where invalid
    if (f == "name")
      tp.name = *v;
    else if (f == "unit")
      tp.unit = *v;
    else if (f == "eak")
      tp.eak_code = v.value_or("");
    else if (f == "bim7aa")
      tp.bim7aa_code = v.value_or("");
    else if (f == "treatment")
      tp.treatment = *treatment_from_string(*v);
    else if (f == "mass_t")
      tp.mass_t = v ? std::optional<double>(*parse_number(*v))
                    : std::optional<double>{};
    else if (f == "quantity")
      pp.quantity = *parse_number(*v);
    else if (f == "room")
      pp.room_name = v.value_or("");
    else if (f == "note")
      pp.note = v.value_or("");
    else if (f == "starred")
      pp.starred = *v == "true";
    type_written = type_written || pl.key->scope == KeyScope::type;
  }

  {
    ProjectDB::Transaction tx(db);
    if (type_written)
      db.update_survey_type(part->type_id, tp);
    db.update_survey_part(code, pp); // no-op when the patch is empty
    if (!passport.empty()) {
      std::optional<std::string> guid = part->material_guid;
      const bool any_set =
          std::any_of(passport.begin(), passport.end(),
                      [](const Planned &p) { return p.value.has_value(); });
      if (any_set)
        guid = db.ensure_resource_passport(code);
      if (guid) {
        const auto stored = db.passport_stored_properties(*guid);
        for (const auto &pl : passport) {
          if (pl.value)
            db.set_passport_property(*guid, pl.key->field, *pl.value);
          else if (stored.count(pl.key->field) != 0)
            db.delete_passport_property(*guid, pl.key->field);
        }
      }
    }
    tx.commit();
  }

  ResourcePatchResult out{resource(db, code, keys), {}};
  if (type_written)
    for (const auto &r : list_resources(db, keys))
      if (r.type_id == part->type_id && r.code != code)
        out.siblings.push_back(r);
  return out;
}

Resource create_resource(ProjectDB &db, int64_t type_id,
                         const std::optional<std::string> &name) {
  if (!db.survey_type(type_id))
    throw std::out_of_range("no survey type " + std::to_string(type_id));
  if (name && name->empty())
    throw std::invalid_argument("'name' must be non-empty when given");
  ProjectDB::SurveyPartRecord rec;
  rec.code = part_code(db.max_survey_part_number() + 1);
  rec.type_id = type_id;
  {
    ProjectDB::Transaction tx(db);
    db.add_survey_part(rec);
    if (name) {
      const auto guid = db.ensure_resource_passport(rec.code);
      db.set_passport_property(guid, kDesignationField, *name);
    }
    tx.commit();
  }
  return resource(db, rec.code);
}

void delete_resource(ProjectDB &db, std::string_view code) {
  const auto part = db.survey_part(code);
  if (!part)
    throw std::out_of_range("no resource '" + std::string(code) + "'");
  if (part->instance_guid)
    throw ResourceConflictError(
        "resource '" + std::string(code) + "' comes from the scan (instance " +
        *part->instance_guid + ") and cannot be deleted");
  ProjectDB::Transaction tx(db);
  db.delete_survey_part(code);
  if (part->material_guid && !db.is_passport_linked(*part->material_guid))
    db.delete_material_passport(*part->material_guid);
  tx.commit();
}

ProjectDB::PropertyDefinition create_column(ProjectDB &db,
                                            ProjectDB::PropertyDefinition def) {
  check_column_name(db, def.name, "");
  def.id = db.add_property_definition(def.name, def.type, def.options,
                                      def.sort_order, def.width);
  return def;
}

ProjectDB::PropertyDefinition
update_column(ProjectDB &db, const std::string &id, const ColumnPatch &p) {
  const auto defs = db.list_property_definitions();
  const auto it = std::find_if(defs.begin(), defs.end(),
                               [&](const auto &d) { return d.id == id; });
  if (it == defs.end())
    throw std::out_of_range("no column '" + id + "'");
  auto def = *it;
  const auto old_name = def.name;
  if (p.name)
    def.name = *p.name;
  if (p.type)
    def.type = *p.type;
  if (p.options)
    def.options = *p.options;
  if (p.sort_order)
    def.sort_order = *p.sort_order;
  if (p.width)
    def.width = *p.width;
  if (def.name != old_name)
    check_column_name(db, def.name, id);
  ProjectDB::Transaction tx(db);
  if (def.name != old_name)
    db.rename_passport_field(old_name, def.name);
  db.update_property_definition(id, def.name, def.type, def.options,
                                def.sort_order, def.width);
  tx.commit();
  return def;
}

} // namespace reusex::core
