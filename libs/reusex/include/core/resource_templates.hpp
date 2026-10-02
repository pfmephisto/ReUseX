// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Templates
/// (docs/superpowers/specs/2026-10-02-resources-templates-ia-design.md §5): an
/// ordered list of category and key members that resolves, against the live key
/// catalogue, to an ordered list of key ids. The pure half is here first; the
/// ProjectDB-backed service follows below.

#include "reusex/core/resource_keys.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

enum class MemberKind { category, key };
struct TemplateMember {
  MemberKind kind = MemberKind::key;
  std::string ref; ///< category name, or key id
  bool operator==(const TemplateMember &) const = default;
};

/// Parse `[{"category": name} | {"key": id}, …]`. Shape only: a category or
/// key that does not exist is accepted and later reported as missing.
/// @throws std::invalid_argument naming the bad element.
std::vector<TemplateMember> parse_members(const nlohmann::json &members);
/// Stored members, tolerantly: unreadable JSON logs warn and reads as none.
std::vector<TemplateMember> read_members(std::string_view stored,
                                         std::string_view template_name);
nlohmann::json members_json(const std::vector<TemplateMember> &members);

struct ResolvedTemplate {
  std::vector<std::string> keys;       ///< ordered, first position wins
  std::vector<TemplateMember> missing; ///< members resolving to nothing
};
/// Spec §5.2: a category member expands to every catalogue key in that
/// category (catalogue order), a key member adds that key, duplicates keep
/// their first position, and members that resolve to nothing go to missing.
ResolvedTemplate resolve_template(const std::vector<TemplateMember> &members,
                                  const std::vector<ResourceKey> &catalogue);

inline constexpr std::string_view kSeedMaterialepas = "materialepas";
inline constexpr std::string_view kSeedScreening = "screening";
struct SeedTemplate {
  std::string tag, name;
  std::vector<TemplateMember> members;
};
/// "Materialepas (fuld)" (one category member per leksikon category) and
/// "Hurtig genbrugsscreening" (the 11 sys: keys), in that order.
std::vector<SeedTemplate> seed_templates();

/// A template's CSV export options (templates.csv).
struct CsvOptions {
  std::string delimiter = ";";                     ///< ";" | "," | "\t"
  std::string encoding = "utf-8-bom";              ///< "utf-8" | "utf-8-bom"
  std::string header = "label";                    ///< "label" | "key"
  nlohmann::json extra = nlohmann::json::object(); ///< unknown fields, kept
};
/// @throws std::invalid_argument when @p csv is not an object or a known
///         field has a value outside its set.
CsvOptions parse_csv_options(const nlohmann::json &csv);
/// Stored options, tolerantly: a bad field logs warn and uses its default.
CsvOptions read_csv_options(std::string_view stored,
                            std::string_view template_name);
nlohmann::json csv_options_json(const CsvOptions &options);

/// Export-template columns (schema v21 `config.columns`) as members. Matches
/// a user column label, then any key label, then a leksikon field name;
/// otherwise a `legacy:<name>` key member that never resolves (so it shows
/// up in `missing`).
inline constexpr std::string_view kLegacyKeyPrefix = "legacy:";
TemplateMember legacy_column_member(std::string_view column,
                                    const std::vector<ResourceKey> &catalogue);
/// The reverse, for the /export-templates view: legacy members give back
/// their name, user-column members their current label; other members are
/// left out.
std::vector<std::string>
legacy_columns(const std::vector<TemplateMember> &members,
               const std::vector<ResourceKey> &catalogue);

/// First of "<base> (<tag>)", "<base> (<tag> 2)", "<base> (<tag> 3)", …
/// that is not in @p taken.
std::string unique_name(std::string_view base, std::string_view tag,
                        const std::vector<std::string> &taken);

} // namespace reusex::core
