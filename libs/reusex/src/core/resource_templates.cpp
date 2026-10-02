// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/resource_templates.hpp"

#include "reusex/core/logging.hpp"

#include <algorithm>
#include <initializer_list>
#include <set>

namespace reusex::core {

std::vector<TemplateMember> parse_members(const nlohmann::json &members) {
  if (!members.is_array())
    throw std::invalid_argument("'members' must be an array");
  std::vector<TemplateMember> out;
  for (std::size_t i = 0; i < members.size(); ++i) {
    const auto &m = members[i];
    const std::string bad = "members[" + std::to_string(i) +
                            "] must be {\"category\": name} or {\"key\": id}";
    if (!m.is_object() || m.size() != 1)
      throw std::invalid_argument(bad);
    const auto it = m.begin();
    if ((it.key() != "category" && it.key() != "key") || !it->is_string() ||
        it->get<std::string>().empty())
      throw std::invalid_argument(bad);
    out.push_back(
        {it.key() == "category" ? MemberKind::category : MemberKind::key,
         it->get<std::string>()});
  }
  return out;
}

std::vector<TemplateMember> read_members(std::string_view stored,
                                         std::string_view template_name) {
  const auto j = nlohmann::json::parse(stored, nullptr, false);
  try {
    if (!j.is_discarded())
      return parse_members(j);
  } catch (const std::invalid_argument &e) {
    reusex::warn("template '{}': stored members unreadable ({}); treated as "
                 "empty",
                 template_name, e.what());
    return {};
  }
  reusex::warn("template '{}': stored members are not JSON; treated as empty",
               template_name);
  return {};
}

nlohmann::json members_json(const std::vector<TemplateMember> &members) {
  auto out = nlohmann::json::array();
  for (const auto &m : members) {
    nlohmann::json obj = nlohmann::json::object();
    obj[m.kind == MemberKind::category ? "category" : "key"] = m.ref;
    out.push_back(std::move(obj));
  }
  return out;
}

ResolvedTemplate resolve_template(const std::vector<TemplateMember> &members,
                                  const std::vector<ResourceKey> &catalogue) {
  ResolvedTemplate out;
  std::set<std::string> seen;
  auto add = [&](const std::string &id) {
    if (seen.insert(id).second)
      out.keys.push_back(id);
  };
  for (const auto &m : members) {
    if (m.kind == MemberKind::key) {
      if (find_key(catalogue, m.ref))
        add(m.ref);
      else
        out.missing.push_back(m);
      continue;
    }
    bool any = false;
    for (const auto &k : catalogue)
      if (k.category == m.ref) {
        add(k.id);
        any = true;
      }
    if (!any)
      out.missing.push_back(m);
  }
  return out;
}

std::vector<SeedTemplate> seed_templates() {
  SeedTemplate full{std::string(kSeedMaterialepas), "Materialepas (fuld)", {}};
  for (auto &c : leksikon_categories())
    full.members.push_back({MemberKind::category, std::move(c)});
  SeedTemplate screening{
      std::string(kSeedScreening), "Hurtig genbrugsscreening", {}};
  for (const auto &k : builtin_keys())
    screening.members.push_back({MemberKind::key, k.id});
  return {full, screening};
}

namespace {
void pick(const nlohmann::json &csv, const char *field, std::string &target,
          std::initializer_list<const char *> allowed) {
  const auto it = csv.find(field);
  if (it == csv.end())
    return;
  if (it->is_string())
    for (const char *a : allowed)
      if (it->get<std::string>() == a) {
        target = a;
        return;
      }
  std::string list;
  for (const char *a : allowed)
    list += (list.empty() ? "" : ", ") + nlohmann::json(a).dump();
  throw std::invalid_argument(std::string("'csv.") + field +
                              "' must be one of " + list);
}
} // namespace

CsvOptions parse_csv_options(const nlohmann::json &csv) {
  if (!csv.is_object())
    throw std::invalid_argument("'csv' must be an object");
  CsvOptions o;
  pick(csv, "delimiter", o.delimiter, {";", ",", "\t"});
  pick(csv, "encoding", o.encoding, {"utf-8", "utf-8-bom"});
  pick(csv, "header", o.header, {"label", "key"});
  for (auto it = csv.begin(); it != csv.end(); ++it)
    if (it.key() != "delimiter" && it.key() != "encoding" &&
        it.key() != "header")
      o.extra[it.key()] = *it;
  return o;
}

CsvOptions read_csv_options(std::string_view stored,
                            std::string_view template_name) {
  auto j = nlohmann::json::parse(stored, nullptr, false);
  if (j.is_discarded() || !j.is_object()) {
    reusex::warn("template '{}': stored CSV options unreadable; using "
                 "defaults",
                 template_name);
    return {};
  }
  for (const char *field : {"delimiter", "encoding", "header"}) {
    if (!j.contains(field))
      continue;
    nlohmann::json probe = nlohmann::json::object();
    probe[field] = j[field];
    try {
      (void)parse_csv_options(probe);
    } catch (const std::invalid_argument &e) {
      reusex::warn("template '{}': stored CSV option ignored ({}); using the "
                   "default",
                   template_name, e.what());
      j.erase(field);
    }
  }
  return parse_csv_options(j);
}

nlohmann::json csv_options_json(const CsvOptions &o) {
  nlohmann::json out = o.extra.is_object() ? o.extra : nlohmann::json::object();
  out["delimiter"] = o.delimiter;
  out["encoding"] = o.encoding;
  out["header"] = o.header;
  return out;
}

TemplateMember legacy_column_member(std::string_view column,
                                    const std::vector<ResourceKey> &catalogue) {
  for (const auto &k : catalogue)
    if (k.source == KeySource::column && k.label == column)
      return {MemberKind::key, k.id};
  for (const auto &k : catalogue)
    if (k.label == column)
      return {MemberKind::key, k.id};
  for (const auto &k : catalogue)
    if (k.source == KeySource::leksikon && k.field == column)
      return {MemberKind::key, k.id};
  return {MemberKind::key, std::string(kLegacyKeyPrefix) + std::string(column)};
}

std::vector<std::string>
legacy_columns(const std::vector<TemplateMember> &members,
               const std::vector<ResourceKey> &catalogue) {
  std::vector<std::string> out;
  for (const auto &m : members) {
    if (m.kind != MemberKind::key)
      continue;
    if (m.ref.rfind(kLegacyKeyPrefix, 0) == 0) {
      out.push_back(m.ref.substr(kLegacyKeyPrefix.size()));
      continue;
    }
    if (const auto *k = find_key(catalogue, m.ref);
        k && k->source == KeySource::column)
      out.push_back(k->label);
  }
  return out;
}

std::string unique_name(std::string_view base, std::string_view tag,
                        const std::vector<std::string> &taken) {
  auto is_taken = [&](const std::string &n) {
    return std::find(taken.begin(), taken.end(), n) != taken.end();
  };
  const std::string stem = std::string(base) + " (" + std::string(tag);
  std::string candidate = stem + ")";
  for (int n = 2; is_taken(candidate); ++n)
    candidate = stem + " " + std::to_string(n) + ")";
  return candidate;
}

} // namespace reusex::core
