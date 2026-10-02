<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Resources & Templates Phase 1 — Backend Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give `rux gui` a resource/template backend: schema v25 (one passport per survey part, a `templates` table with two seeds, `export_templates` and On-site's `samples.part_code` gone), a key catalogue, template resolution, value routing, CSV and PDF "Ressourcetabel" output, and the HTTP API of spec §4.4 and §5.5. The current frontend must keep working, unchanged.

**Architecture:** Library-first. Pure rules live in `libs/reusex/include/core/resource_keys.hpp` (catalogue, value validation) and `core/resource_templates.hpp` (members, resolver, seeds, CSV options, legacy-column mapping). Storage lives in `ProjectDB` (v25 migration, template CRUD, a public `ProjectDB::Transaction`, lazy per-part passports). DB-aware operations live in `core/resources.hpp` (read and route values, add/delete resources, user columns), `core/resource_templates.hpp` (template service) and `core/resource_export.hpp` (CSV, PDF tables). `rux gui` handlers in `apps/rux/src/gui/resources.cpp` only parse, call one library function and map exceptions to status codes. The old `/export-templates` routes (rux gui **and** ruxd) keep working because `ProjectDB`'s `*_export_template` methods become a view over `templates`. The old `/material-columns` routes and the new `/resources/columns` routes share their handlers.

**Tech Stack:** C++20, sqlite3, nlohmann_json, fmt, Crow (via `rux_gui_lib`), Typst (runtime, PDF), Catch2 v3, OpenAPI YAML.

**Spec:** `docs/superpowers/specs/2026-10-02-resources-templates-ia-design.md` (§2, §4, §5, §6.3 backend parts, §7, §8 backend bullets, §9 C++/API tests). Phase split: this plan is Phase 1 of 4; Phases 2–4 are frontend and rely on the HTTP contract built here.

## Global Constraints

- Naming per CLAUDE.md: snake_case functions, PascalCase types, enum values snake_case, no `get_` prefix, members `name_`. Library code is in `reusex::core`, and server code is in `rux::gui`.
- Public headers include siblings with the `reusex/` prefix (`#include "reusex/core/resource_keys.hpp"`). Tests include `<core/...>` like the existing ones.
- Every new file carries the SPDX header: `// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` followed by `// SPDX-License-Identifier: GPL-3.0-or-later`.
- Schema: `LATEST_SCHEMA_VERSION = 25`. The table names stay `material_passports`, `passport_property_values`, `material_property_definitions` and so on (spec §2). New names say "resource" or "template".
- Key ids are exactly `sys:<name>`, `lex:<leksikon_guid>` and `col:<material_property_definitions.id>`. Categories: built-in = `Kortlægning`, user columns = `Egne felter`, leksikon = the `property_definitions.category` string (`Owner`, `Description`, `Product`, `Certifications`, `Dimensions`, `History`, `Condition`, `Pollution`, `Environmental`, `Fire`), in leksikon (MaterialEPAS section) order.
- Built-in keys, in this order, with label/scope/editable: `sys:name` Betegnelse type; `sys:quantity` Mængde part; `sys:unit` Enhed type; `sys:eak` EAK type; `sys:bim7aa` BIM7AA type; `sys:treatment` Behandling type (enum); `sys:environment` Miljøstatus type **read-only**; `sys:room` Rum part; `sys:mass_t` Tons type; `sys:note` Note part; `sys:starred` Vigtig part.
- Passport values for `lex:`/`col:` keys go through `ProjectDB::set_passport_property` under the leksikon `name_en` (= `field_name`) or the column's display name. That mapping lives only in `core/resource_keys.*` (`ResourceKey::field`). It is never sent on the wire.
- `templates` DDL is spec §5.1 verbatim. Seed names and tags: `Materialepas (fuld)` / `materialepas`, `Hurtig genbrugsscreening` / `screening`.
- Duplicate name: `"<name> (kopi)"`, then `"<name> (kopi 2)"`, `"(kopi 3)"`, … An export template whose name clashes gets `" (eksport)"` (then `" (eksport 2)"`, …).
- HTTP status codes (spec §7): unknown resource code or template id → 404; unknown key id, read-only key, bad enum, bad number → 400 naming the key; duplicate template or column name → 409; deleting an instance-backed resource → 409; writer lock busy → 503 (existing `with_write`).
- Every write validates first and writes second, inside one `ProjectDB::Transaction`.
- CSV formula guard: a cell starting with `=`, `+`, `-`, `@`, TAB or CR gets a leading `'`.
- PDF Ressourcetabel: tables of at most 8 columns, each led by a Betegnelse column.
- Every new route needs four things: an `endpoint_table()` row (`apps/rux/src/gui/api.cpp`), a `Server.cpp` registration, an entry in the literal set in `tests/unit/rux_gui/test_gui_api.cpp`, and a `docs/gui/openapi.yaml` path whose `summary` equals the table's summary. `scripts/check-openapi.py` must pass (`ctest -R gui_api_contract_parses`).
- `rux_gui_lib` must not link VTK or `reusex_vision`.
- No silent failure (STANDARDS §5). The migration logs its counts at `info`. Anything split, skipped or unmatched is logged at `warn` with the numbers. A template that has missing members logs `warn` once per template per request.
- **No frontend file changes in this phase.** These must keep working against the new backend: ExportPage (through the `/export-templates` view), Materialedata (`/materials`, `/material-columns`), Kortlægning (`/survey`), Miljø (`/samples`) and Rapport (`/reports`).
- Builds (in the worktree; the devshell is already active via direnv): `cmake -B build -DCMAKE_BUILD_TYPE=Release`, then `cmake --build build --target <targets>`. Run in the foreground with timeout 600000 ms and re-run on a timeout (ccache makes the retry fast). The unit test binary is `reusex_unit_tests`, which includes `tests/unit/core` and `tests/unit/rux_gui`. Tests: `cd build && ctest --output-on-failure --parallel $(nproc) -R '<pattern>'`. Catch2 v3; temp paths come from `tests/support/temp_path.hpp`.
- Commits: never `--no-verify`. Each message ends with
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and
  `Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf`.

## Rulings on spec ambiguities (binding for this plan)

1. **The old column path is `/api/v1/material-columns`**, not `/api/v1/materials/columns`. Phase 1 adds `/api/v1/resources/columns` (same handlers), and Phase 3 deletes `/material-columns`.
2. **`export_templates` routes stay** in Phase 1 as a view over `templates`. The `ProjectDB::*_export_template` methods keep their signatures (ruxd uses them too). `config.columns` maps to and from key members: a user column becomes `col:<id>`, and anything unmatched becomes a `legacy:<name>` key member. That member never resolves, so it shows in `missing` (spec §5.4). Phase 4 deletes the `rux gui` export-template routes. ruxd keeps the view.
3. **Migration order:** the seeds are inserted first (the new table is always empty there), and then the export templates are moved with the clash suffix. If step 6 ran literally after step 5, a project with export templates would get no seeds.
4. **Matching an export column "by display name":** first the label of a user column, then any key label, then a leksikon `field_name`. Otherwise it becomes `legacy:<name>`.
5. **The leksikon catalogue comes from the compiled MaterialEPAS traits** (`json_export::section_descriptors()`), not from the `property_definitions` rows. Those rows only exist once a passport has been added. Nested object-array fields (dangerous substances, emissions) are skipped. Labels are the humanised `field_name` (`width_mm` → `Width mm`). The unit is derived from the `_mm`/`_m2`/`_m3`/`_kg` suffix. `ensure_resource_passport` populates `property_definitions` so that leksikon writes never land under an auto-created `custom:` definition.
6. **`passport_guid` uniqueness** comes from `CREATE UNIQUE INDEX idx_survey_parts_passport`, because SQLite's `ADD COLUMN` cannot carry `UNIQUE`.
7. **A part's passport (`SurveyPartRecord::material_guid`) is `COALESCE(survey_parts.passport_guid, instance_materials.material_guid)`.** `ProjectDB::set_instance_material` (used by `PUT /instances/.../material` and `rux create materials`) also moves the backing part's `passport_guid` to the new passport, unless another part owns it. In that case it logs `warn` and the part keeps its own passport.
8. **Wire values are `string | null`.** Numbers accept `.` or a single `,` decimal separator and are stored with `.`. Booleans are `"true"`/`"false"`. Dates are `YYYY-MM-DD`. `""` on a number/enum/boolean/date key means clear. Clearing a non-nullable built-in key (`sys:name`, `sys:quantity`, `sys:unit`, `sys:treatment`, `sys:starred`) is a 400. Clearing `sys:eak`/`bim7aa`/`room`/`note` stores `""`, and clearing `sys:mass_t` stores NULL.
9. **Scopes:** `lex:` and `col:` keys have scope `part`, because the passport belongs to the part. The wire `scope` is `"type"` or `"part"`.
10. **`POST /resources {type_id, name?}`** stores `name` as the resource's leksikon `designation` (Betegnelse of the construction item). It does not use the type-scoped `sys:name`.
11. **Response envelopes** (the spec names the item shapes only). `GET /resources/keys` → a JSON array. `GET /resources` → `{resources:[…], template?:{id, resolved_keys, missing}}`. `PATCH /resources/<code>` (optional `?template=`) → `{resource, siblings}`. `POST /templates/restore-seeds` → `{restored:[names], templates:[…]}`. `GET /templates` → `{templates:[…]}`. Each resource also carries `manual: bool`.
12. **The CSV route requires `?template=`** (400 without it). Its options are `{delimiter: ";"|","|"\t", encoding: "utf-8"|"utf-8-bom", header: "label"|"key"}`, with defaults `;`, `utf-8-bom`, `label`. Unknown fields are kept verbatim. In `label` mode the cells use display values (Danish treatment/miljøstatus labels, `Ja`/`Nej`). In `key` mode the cells are raw. Lines end in CRLF.
13. **PDF Ressourcetabel** lists every resource whose type is not `rejected`, ordered by code. Its lead cell is `"<Betegnelse> · <code>"`, so the parts of one type stay distinguishable. The report request field is `resource_template_id` (an integer or null).
14. **Template members are validated for shape only.** A key or category that does not exist is stored and then reported in `missing`.
15. **Column names must be unique** among user columns and must not equal a leksikon `field_name`. Otherwise the request gets a 409, on both column paths. A rename moves the stored values in the same transaction. It is refused with a 409 if values are already stored under the new name.
16. **The sample-create route ignores `part_code` and `stage`** (unknown keys are ignored). The library keeps `add_sample(..., stage)`, but `part_code` is gone from `SampleRecord` and `add_sample`.

## Review Focus

- **Blank and cleared values on typed keys.** A user empties a number cell and the client sends `""`. That must clear the value, not 400. Clearing a non-nullable built-in must 400 and change nothing. (Task 1 `ResourceKeys_Normalise_BlankClears_RequiredRefuses`.)
- **Relinking an instance after v25** (Instanser page, `rux create materials`). The part's passport must follow the new link when nobody else owns that passport. When another part owns it, the part keeps its own passport and a `warn` is logged. It must never be silently split-brained. (Task 5 `ResourceStore_SetInstanceMaterial_MovesPartPassportUnlessOwned`.)
- **Export-template names that clash** with a seed **or with each other** (`export_templates.name` was never unique), and a row with unreadable config. Every row must be moved with a unique name and none may be lost. (Task 4 `MigrationV25_MovesExportTemplates_ClashUnmatchedAndBroken`.)
- **Renaming a user column onto a name that already holds stored values** (from a column deleted earlier). The request must 409, and the values must not be merged. Renaming back to the old name and renaming to the same name must work. (Task 6 `Resources_RenameColumn_RefusesNameWithStoredValues`.)
- **"Gendan standardskabeloner" when the user owns a template named like a seed.** The seed must be inserted with a suffix, not fail with 409/500. Running it twice must insert nothing the second time. (Task 7 `TemplateService_RestoreSeeds_SuffixOnNameClash_Idempotent`.)

---

## File Structure

| File | Responsibility |
|---|---|
| `libs/reusex/include/core/resource_keys.hpp`, `src/core/resource_keys.cpp` (new) | key catalogue, leksikon fields, value validation/normalisation, number parse/format |
| `libs/reusex/include/core/resource_templates.hpp`, `src/core/resource_templates.cpp` (new) | members, resolver, seeds, CSV options, legacy column mapping, unique names (pure), plus the template service over ProjectDB |
| `libs/reusex/include/core/resources.hpp`, `src/core/resources.cpp` (new) | read resources, route value writes, add/delete resources, user columns |
| `libs/reusex/include/core/resource_export.hpp`, `src/core/resource_export.cpp` (new) | display values, CSV builder, PDF table chunking, report section |
| `libs/reusex/include/core/ProjectDB.hpp`, `src/core/ProjectDB.cpp` | v25 migration, `Transaction`, template CRUD, export-template view, lazy passport, part delete, field rename, link sync; `part_code` removal |
| `libs/reusex/include/core/survey.hpp` | `NameConflictError` |
| `libs/reusex/include/core/report_generator.hpp`, `src/core/report_generator.cpp`, `apps/rux/resources/report.typ` | Ressourcetabel section |
| `apps/rux/include/gui/resources.hpp`, `apps/rux/src/gui/resources.cpp` (new) | resources + templates HTTP handlers |
| `apps/rux/src/gui/api.cpp`, `Server.cpp`, `edits.cpp`, `survey.cpp`, `include/gui/{api,edits,survey}.hpp`, `apps/rux/src/gui.cpp` | routes, column handlers, report body, sample route, help text |
| `apps/rux/frontend/dev/seed-survey-demo.sh` | drop the On-site demo sample |
| `docs/gui/openapi.yaml` | contract |
| `tests/support/survey_fixture.hpp` (new) | shared instance/type/part/passport fixture |
| tests (new): `tests/unit/core/test_resource_keys.cpp`, `test_resource_templates.cpp`, `test_project_db_migration_v25.cpp`, `test_project_db_resources.cpp`, `test_resources.cpp`, `test_resource_template_service.cpp`, `test_resource_export.cpp`, `tests/unit/rux_gui/test_gui_resources.cpp` | |
| tests (changed): `tests/unit/core/test_project_db_exports.cpp`, `test_project_db_samples.cpp`, `test_report_survey.cpp`, `tests/unit/rux_gui/test_gui_api.cpp`, `test_gui_survey.cpp`, `test_gui_reports.cpp`, `test_gui_server_socket.cpp` | |

Task order: 1–2 are pure, 3 removes On-site, 4–5 are storage, 6–8 are services, 9–11 are HTTP/PDF and 12 is verification. Every task leaves the whole suite green.

---

### Task 1: Resource key catalogue (pure)

**Files:**
- Create: `libs/reusex/include/core/resource_keys.hpp`, `libs/reusex/src/core/resource_keys.cpp`
- Test: `tests/unit/core/test_resource_keys.cpp`

**Interfaces:**
- Consumes: `ProjectDB::PropertyDefinition`, `ProjectDB::list_property_definitions()`, `core::traits::PropertyType`/`PropertyTraits`, `core::json_export::section_descriptors()`, `core::to_string(Treatment|EnvironmentStatus)`.
- Produces (exact):

```cpp
namespace reusex::core {
enum class KeySource { builtin, leksikon, column };
enum class KeyScope { type, part };
std::string_view to_string(KeyScope); // "type" | "part"
inline constexpr std::string_view kBuiltinCategory = "Kortlægning";
inline constexpr std::string_view kColumnCategory = "Egne felter";
struct ResourceKey {
  std::string id, label, category;
  KeyScope scope = KeyScope::part;
  std::string data_type = "text"; // text | number | enum | boolean | date
  std::string unit;               // "" when none
  std::vector<std::string> options;
  bool editable = true;
  KeySource source = KeySource::builtin;
  std::string field;   // storage name; never on the wire
  bool integer = false;
};
struct LeksikonField { std::string guid, field_name, category; traits::PropertyType type; };
std::vector<LeksikonField> leksikon_fields();
std::vector<std::string> leksikon_categories();
std::vector<ResourceKey> builtin_keys();
std::vector<ResourceKey> leksikon_keys();
ResourceKey column_key(const ProjectDB::PropertyDefinition &def);
std::vector<ResourceKey> key_catalogue(const std::vector<ProjectDB::PropertyDefinition> &columns);
std::vector<ResourceKey> key_catalogue(const ProjectDB &db);
const ResourceKey *find_key(const std::vector<ResourceKey> &catalogue, std::string_view id);
class KeyValueError : public std::invalid_argument { public: KeyValueError(std::string key, const std::string &message); const std::string &key() const noexcept; };
std::optional<double> parse_number(std::string_view text);
std::string format_number(double v);
std::optional<std::string> normalise_value(const ResourceKey &key, const std::optional<std::string> &value);
}
```

- [ ] **Step 1: Write the failing test.** Create `tests/unit/core/test_resource_keys.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// The resource key catalogue (resources/templates spec §4.3): built-in keys,
// leksikon keys from the compiled MaterialEPAS traits, user columns, and the
// one place values are validated and normalised.

#include <catch2/catch_test_macros.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/materialepas_json_export.hpp>
#include <core/resource_keys.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

using namespace reusex::core;
using reusex::ProjectDB;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_resource_keys") {}
};
const ResourceKey &key_named(const std::vector<ResourceKey> &keys,
                             const std::string &field) {
  const auto it = std::find_if(keys.begin(), keys.end(),
                               [&](const auto &k) { return k.field == field; });
  REQUIRE(it != keys.end());
  return *it;
}
} // namespace

TEST_CASE("ResourceKeys_Builtins_MatchTheSpecTable", "[resources][keys]") {
  const auto keys = builtin_keys();
  std::vector<std::string> ids;
  for (const auto &k : keys)
    ids.push_back(k.id);
  CHECK(ids == std::vector<std::string>{
                   "sys:name", "sys:quantity", "sys:unit", "sys:eak",
                   "sys:bim7aa", "sys:treatment", "sys:environment",
                   "sys:room", "sys:mass_t", "sys:note", "sys:starred"});
  for (const auto &k : keys) {
    CHECK(k.category == "Kortlægning");
    CHECK(k.source == KeySource::builtin);
    CHECK(k.editable == (k.id != "sys:environment"));
  }
  CHECK(keys[0].label == "Betegnelse");
  CHECK(keys[0].scope == KeyScope::type);
  CHECK(keys[1].scope == KeyScope::part);
  CHECK(keys[1].data_type == "number");
  CHECK(keys[5].data_type == "enum");
  CHECK(keys[5].options ==
        std::vector<std::string>{"bevaring", "genbrug", "genanvendelse",
                                 "nyttiggoerelse", "bortskaffelse"});
  CHECK(keys[6].options ==
        std::vector<std::string>{"ren_screening", "afventer", "forurenet",
                                 "ren_proevesvar"});
  CHECK(keys[8].unit == "t");
  CHECK(keys[10].data_type == "boolean");
  CHECK(to_string(KeyScope::type) == "type");
}

TEST_CASE("ResourceKeys_Leksikon_EveryTopLevelPropertyInLeksikonOrder",
          "[resources][keys]") {
  std::size_t expected = 0;
  for (const auto &sd : json_export::section_descriptors())
    for (std::size_t i = 0; i < sd.property_count; ++i)
      if (sd.properties[i].type != traits::PropertyType::ObjectArray)
        ++expected;
  const auto keys = leksikon_keys();
  REQUIRE(keys.size() == expected);
  CHECK(keys.front().id == "lex:0Bwj05D$55V931bq9VaBE5");
  CHECK(keys.front().field == "contact_email");
  CHECK(keys.front().label == "Contact email");
  CHECK(keys.front().category == "Owner");
  CHECK(keys.front().scope == KeyScope::part);
  const auto &width = key_named(keys, "width_mm");
  CHECK(width.data_type == "number");
  CHECK(width.unit == "mm");
  CHECK_FALSE(width.integer);
  CHECK(key_named(keys, "year_of_installation").integer);
  CHECK(key_named(keys, "volume_m3").unit == "m³");
  const auto &reach = key_named(keys, "contains_reach_substances");
  CHECK(reach.data_type == "enum");
  CHECK(reach.options == std::vector<std::string>{"yes", "no", "unknown"});
  CHECK(key_named(keys, "has_epd").data_type == "boolean");
  CHECK(leksikon_categories() ==
        std::vector<std::string>{"Owner", "Description", "Product",
                                 "Certifications", "Dimensions", "History",
                                 "Condition", "Pollution", "Environmental",
                                 "Fire"});
}

TEST_CASE("ResourceKeys_LeksikonCategories_MatchPropertyDefinitions",
          "[resources][keys]") {
  // ProjectDB files each leksikon field under a category string; the
  // catalogue must use the same one, or a category template member would
  // miss fields.
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    MaterialPassport p;
    p.metadata.document_guid = "guid-p";
    db.add_material_passport(p, "");
  }
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  REQUIRE(sqlite3_prepare_v2(raw,
                             "SELECT leksikon_guid, name_en, category FROM "
                             "property_definitions;",
                             -1, &s, nullptr) == SQLITE_OK);
  std::map<std::string, std::pair<std::string, std::string>> stored;
  while (sqlite3_step(s) == SQLITE_ROW)
    stored[reinterpret_cast<const char *>(sqlite3_column_text(s, 0))] = {
        reinterpret_cast<const char *>(sqlite3_column_text(s, 1)),
        reinterpret_cast<const char *>(sqlite3_column_text(s, 2))};
  sqlite3_finalize(s);
  sqlite3_close(raw);
  for (const auto &f : leksikon_fields()) {
    INFO(f.field_name);
    REQUIRE(stored.count(f.guid) == 1);
    CHECK(stored[f.guid].first == f.field_name);
    CHECK(stored[f.guid].second == f.category);
  }
}

TEST_CASE("ResourceKeys_Catalogue_BuiltinLeksikonThenColumns",
          "[resources][keys]") {
  ProjectDB::PropertyDefinition sel;
  sel.id = "c1";
  sel.name = "Stand";
  sel.type = "select";
  sel.options = {"God", "Dårlig"};
  ProjectDB::PropertyDefinition multi;
  multi.id = "c2";
  multi.name = "Mærker";
  multi.type = "multiselect";
  const auto cat = key_catalogue({sel, multi});
  REQUIRE(cat.size() == builtin_keys().size() + leksikon_keys().size() + 2);
  CHECK(cat.front().id == "sys:name");
  const auto &a = cat[cat.size() - 2];
  CHECK(a.id == "col:c1");
  CHECK(a.label == "Stand");
  CHECK(a.category == "Egne felter");
  CHECK(a.data_type == "enum");
  CHECK(a.options == std::vector<std::string>{"God", "Dårlig"});
  CHECK(a.field == "Stand");
  CHECK(a.source == KeySource::column);
  CHECK(cat.back().data_type == "text");
  CHECK(find_key(cat, "col:c2") == &cat.back());
  CHECK(find_key(cat, "col:nope") == nullptr);
}

TEST_CASE("ResourceKeys_Normalise_NumbersEnumsBooleansDates",
          "[resources][keys]") {
  const auto cat = key_catalogue(std::vector<ProjectDB::PropertyDefinition>{});
  const auto &qty = *find_key(cat, "sys:quantity");
  CHECK(normalise_value(qty, std::string("12,5")) == "12.5");
  CHECK(normalise_value(qty, std::string(" 3 ")) == "3");
  CHECK_THROWS_AS(normalise_value(qty, std::string("-1")), KeyValueError);
  CHECK_THROWS_AS(normalise_value(qty, std::string("1,2,3")), KeyValueError);
  CHECK_THROWS_AS(normalise_value(qty, std::string("nan")), KeyValueError);
  const auto &year = key_named(cat, "year_of_installation");
  CHECK(normalise_value(year, std::string("1968")) == "1968");
  CHECK_THROWS_AS(normalise_value(year, std::string("1968.5")), KeyValueError);
  const auto &tr = *find_key(cat, "sys:treatment");
  CHECK(normalise_value(tr, std::string("genbrug")) == "genbrug");
  try {
    normalise_value(tr, std::string("genbrugt"));
    FAIL("expected KeyValueError");
  } catch (const KeyValueError &e) {
    CHECK(e.key() == "sys:treatment");
    CHECK(std::string(e.what()).find("sys:treatment") != std::string::npos);
  }
  const auto &star = *find_key(cat, "sys:starred");
  CHECK(normalise_value(star, std::string("true")) == "true");
  CHECK_THROWS_AS(normalise_value(star, std::string("ja")), KeyValueError);
  ProjectDB::PropertyDefinition d;
  d.id = "d";
  d.name = "Dato";
  d.type = "date";
  const auto date = column_key(d);
  CHECK(normalise_value(date, std::string("2026-10-02")) == "2026-10-02");
  CHECK_THROWS_AS(normalise_value(date, std::string("2026-13-02")),
                  KeyValueError);
  CHECK_THROWS_AS(normalise_value(date, std::string("02-10-2026")),
                  KeyValueError);
  CHECK(normalise_value(*find_key(cat, "sys:note"), std::string("")) == "");
}

TEST_CASE("ResourceKeys_Normalise_BlankClears_RequiredRefuses",
          "[resources][keys]") {
  const auto cat = key_catalogue(std::vector<ProjectDB::PropertyDefinition>{});
  // "" on a typed key is a clear, not a parse error.
  CHECK_FALSE(normalise_value(*find_key(cat, "sys:mass_t"), std::string(""))
                  .has_value());
  CHECK_FALSE(
      normalise_value(*find_key(cat, "sys:mass_t"), std::nullopt).has_value());
  for (const char *id : {"sys:name", "sys:quantity", "sys:unit",
                         "sys:treatment", "sys:starred"}) {
    INFO(id);
    CHECK_THROWS_AS(normalise_value(*find_key(cat, id), std::nullopt),
                    KeyValueError);
  }
  CHECK_THROWS_AS(normalise_value(*find_key(cat, "sys:quantity"),
                                  std::string("")),
                  KeyValueError);
  CHECK_THROWS_AS(normalise_value(*find_key(cat, "sys:name"), std::string("")),
                  KeyValueError);
  CHECK_THROWS_AS(normalise_value(*find_key(cat, "sys:environment"),
                                  std::string("forurenet")),
                  KeyValueError);
  CHECK(format_number(290.0) == "290");
  CHECK(format_number(0.1) == "0.1");
  CHECK(parse_number("+4") == 4.0);
  CHECK_FALSE(parse_number("").has_value());
}
```

- [ ] **Step 2: Run the test to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests` (timeout 600000)
Expected: compile error `core/resource_keys.hpp: No such file or directory`.

- [ ] **Step 3: Write the header.** Create `libs/reusex/include/core/resource_keys.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// The resource key catalogue (docs/superpowers/specs/
/// 2026-10-02-resources-templates-ia-design.md §4.3): every key a resource
/// (survey part) can carry, and the ONE mapping from a key id to where its
/// value is stored. The API and the frontend only see key ids; the storage
/// field names (leksikon name_en, column display name) stay in here.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/materialepas_traits.hpp"

#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace reusex::core {

/// Where a key's value lives.
enum class KeySource { builtin, leksikon, column };
/// What a write changes: the survey type (all its parts) or the one part.
enum class KeyScope { type, part };
std::string_view to_string(KeyScope); // "type" | "part"

inline constexpr std::string_view kBuiltinCategory = "Kortlægning";
inline constexpr std::string_view kColumnCategory = "Egne felter";

struct ResourceKey {
  std::string id; ///< "sys:<name>" | "lex:<leksikon_guid>" | "col:<column id>"
  std::string label;
  std::string category;
  KeyScope scope = KeyScope::part;
  std::string data_type = "text"; ///< text | number | enum | boolean | date
  std::string unit;               ///< "" when none
  std::vector<std::string> options; ///< enum choices
  bool editable = true;
  KeySource source = KeySource::builtin;
  /// Storage name: the sys name ("quantity"), the leksikon field name_en
  /// ("width_mm") or the user column's display name. Never on the wire.
  std::string field;
  /// Number keys that only take whole numbers (leksikon Integer fields).
  bool integer = false;
};

/// One top-level leksikon property, in leksikon (MaterialEPAS section) order.
struct LeksikonField {
  std::string guid;       ///< leksikon GUID == property_definitions.leksikon_guid
  std::string field_name; ///< == property_definitions.name_en
  std::string category;   ///< == property_definitions.category
  traits::PropertyType type = traits::PropertyType::String;
};
/// Nested object-array fields (dangerous substances, emissions) are skipped:
/// they are rows, not one value.
std::vector<LeksikonField> leksikon_fields();
/// Distinct categories of leksikon_fields(), in order.
std::vector<std::string> leksikon_categories();

std::vector<ResourceKey> builtin_keys(); ///< the 11 sys: keys, spec order
std::vector<ResourceKey> leksikon_keys();
ResourceKey column_key(const ProjectDB::PropertyDefinition &def);
/// Built-in, then leksikon, then user columns (in the given order) — the
/// "catalogue order" template categories expand in.
std::vector<ResourceKey>
key_catalogue(const std::vector<ProjectDB::PropertyDefinition> &columns);
std::vector<ResourceKey> key_catalogue(const ProjectDB &db);
/// nullptr when @p id is not in @p catalogue.
const ResourceKey *find_key(const std::vector<ResourceKey> &catalogue,
                            std::string_view id);

/// A write the catalogue refuses: unknown key, read-only key, or a value the
/// key's type does not accept. key() names the key for the 400 body.
class KeyValueError : public std::invalid_argument {
    public:
  KeyValueError(std::string key, const std::string &message)
      : std::invalid_argument(message), key_(std::move(key)) {}
  const std::string &key() const noexcept { return key_; }

    private:
  std::string key_;
};

/// Finite number with '.' or one ',' as decimal separator; surrounding
/// blanks and a leading '+' are allowed. nullopt otherwise.
std::optional<double> parse_number(std::string_view text);
/// Shortest round-trip text: 290.0 -> "290", 0.1 -> "0.1".
std::string format_number(double v);

/// Validate @p value for @p key and return it normalised for storage
/// (numbers with '.', booleans "true"/"false"); nullopt means clear. "" on a
/// number/enum/boolean/date key also means clear.
/// @throws KeyValueError when the key is read-only, the value does not fit
///         its data_type/options, sys:quantity is negative, sys:name is
///         empty, or a clear hits sys:name/quantity/unit/treatment/starred.
std::optional<std::string>
normalise_value(const ResourceKey &key,
                const std::optional<std::string> &value);

} // namespace reusex::core
```

- [ ] **Step 4: Write the implementation.** Create `libs/reusex/src/core/resource_keys.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/resource_keys.hpp"

#include "reusex/core/materialepas_json_export.hpp"
#include "reusex/core/survey.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <set>

namespace reusex::core {
namespace {
using traits::PropertyType;

/// The property_definitions.category ProjectDB files each section under
/// (ProjectDB.cpp, ensureAllPropertyDefinitions). The test
/// ResourceKeys_LeksikonCategories_MatchPropertyDefinitions pins the two.
const char *category_of(const traits::PropertyDescriptor *props) {
  using namespace traits;
  if (props == PropertyTraits<Owner>::properties())
    return "Owner";
  if (props == PropertyTraits<ConstructionItemDescription>::properties())
    return "Description";
  if (props == PropertyTraits<ProductInformation>::properties())
    return "Product";
  if (props == PropertyTraits<Certifications>::properties())
    return "Certifications";
  if (props == PropertyTraits<Dimensions>::properties())
    return "Dimensions";
  if (props == PropertyTraits<Condition>::properties())
    return "Condition";
  if (props == PropertyTraits<Pollution>::properties())
    return "Pollution";
  if (props == PropertyTraits<EnvironmentalPotential>::properties())
    return "Environmental";
  if (props == PropertyTraits<FireProperties>::properties())
    return "Fire";
  if (props == PropertyTraits<History>::properties())
    return "History";
  return nullptr;
}

std::string humanize(std::string_view field) {
  std::string out(field);
  std::replace(out.begin(), out.end(), '_', ' ');
  if (!out.empty())
    out[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(out[0])));
  return out;
}

std::string unit_of(std::string_view field) {
  auto ends = [&](std::string_view s) {
    return field.size() >= s.size() &&
           field.substr(field.size() - s.size()) == s;
  };
  if (ends("_mm"))
    return "mm";
  if (ends("_m3"))
    return "m³";
  if (ends("_m2"))
    return "m²";
  if (ends("_kg"))
    return "kg";
  return "";
}

bool is_iso_date(std::string_view v) {
  if (v.size() != 10 || v[4] != '-' || v[7] != '-')
    return false;
  for (std::size_t i : {0, 1, 2, 3, 5, 6, 8, 9})
    if (!std::isdigit(static_cast<unsigned char>(v[i])))
      return false;
  const int month = (v[5] - '0') * 10 + (v[6] - '0');
  const int day = (v[8] - '0') * 10 + (v[9] - '0');
  return month >= 1 && month <= 12 && day >= 1 && day <= 31;
}

/// Built-in keys whose column is NOT NULL with no meaningful empty value.
bool clearable(const ResourceKey &k) {
  static const std::set<std::string, std::less<>> required{
      "sys:name", "sys:quantity", "sys:unit", "sys:treatment", "sys:starred"};
  return required.find(k.id) == required.end();
}
} // namespace

std::string_view to_string(KeyScope s) {
  return s == KeyScope::type ? "type" : "part";
}

std::vector<LeksikonField> leksikon_fields() {
  std::vector<LeksikonField> out;
  for (const auto &sd : json_export::section_descriptors()) {
    const char *category = category_of(sd.properties);
    if (!category)
      throw std::logic_error(std::string("leksikon section '") + sd.name_en +
                             "' has no property_definitions category");
    for (std::size_t i = 0; i < sd.property_count; ++i) {
      const auto &p = sd.properties[i];
      if (p.type == PropertyType::ObjectArray)
        continue;
      out.push_back({p.leksikon_guid, p.field_name, category, p.type});
    }
  }
  return out;
}

std::vector<std::string> leksikon_categories() {
  std::vector<std::string> out;
  for (const auto &f : leksikon_fields())
    if (std::find(out.begin(), out.end(), f.category) == out.end())
      out.push_back(f.category);
  return out;
}

std::vector<ResourceKey> builtin_keys() {
  auto k = [](const char *name, const char *label, KeyScope scope,
              const char *type, bool editable = true) {
    ResourceKey key;
    key.id = std::string("sys:") + name;
    key.label = label;
    key.category = std::string(kBuiltinCategory);
    key.scope = scope;
    key.data_type = type;
    key.editable = editable;
    key.source = KeySource::builtin;
    key.field = name;
    return key;
  };
  std::vector<ResourceKey> out{
      k("name", "Betegnelse", KeyScope::type, "text"),
      k("quantity", "Mængde", KeyScope::part, "number"),
      k("unit", "Enhed", KeyScope::type, "text"),
      k("eak", "EAK", KeyScope::type, "text"),
      k("bim7aa", "BIM7AA", KeyScope::type, "text"),
      k("treatment", "Behandling", KeyScope::type, "enum"),
      k("environment", "Miljøstatus", KeyScope::type, "enum", false),
      k("room", "Rum", KeyScope::part, "text"),
      k("mass_t", "Tons", KeyScope::type, "number"),
      k("note", "Note", KeyScope::part, "text"),
      k("starred", "Vigtig", KeyScope::part, "boolean"),
  };
  for (auto t : {Treatment::bevaring, Treatment::genbrug,
                 Treatment::genanvendelse, Treatment::nyttiggoerelse,
                 Treatment::bortskaffelse})
    out[5].options.emplace_back(to_string(t));
  for (auto e : {EnvironmentStatus::ren_screening, EnvironmentStatus::afventer,
                 EnvironmentStatus::forurenet,
                 EnvironmentStatus::ren_proevesvar})
    out[6].options.emplace_back(to_string(e));
  out[8].unit = "t";
  return out;
}

std::vector<ResourceKey> leksikon_keys() {
  std::vector<ResourceKey> out;
  for (const auto &f : leksikon_fields()) {
    ResourceKey k;
    k.id = "lex:" + f.guid;
    k.label = humanize(f.field_name);
    k.category = f.category;
    k.scope = KeyScope::part;
    k.source = KeySource::leksikon;
    k.field = f.field_name;
    k.unit = unit_of(f.field_name);
    switch (f.type) {
    case PropertyType::Integer:
      k.data_type = "number";
      k.integer = true;
      break;
    case PropertyType::Double:
      k.data_type = "number";
      break;
    case PropertyType::Boolean:
      k.data_type = "boolean";
      break;
    case PropertyType::TriState:
      k.data_type = "enum";
      k.options = {"yes", "no", "unknown"};
      break;
    default: // String, StringArray, EnumValue, EnumArray: stored as text
      k.data_type = "text";
      break;
    }
    out.push_back(std::move(k));
  }
  return out;
}

ResourceKey column_key(const ProjectDB::PropertyDefinition &def) {
  ResourceKey k;
  k.id = "col:" + def.id;
  k.label = def.name;
  k.category = std::string(kColumnCategory);
  k.scope = KeyScope::part;
  k.source = KeySource::column;
  k.field = def.name;
  if (def.type == "number" || def.type == "date" || def.type == "boolean")
    k.data_type = def.type;
  else if (def.type == "select") {
    k.data_type = "enum";
    k.options = def.options;
  } else
    k.data_type = "text"; // text, multiselect, url
  return k;
}

std::vector<ResourceKey>
key_catalogue(const std::vector<ProjectDB::PropertyDefinition> &columns) {
  auto out = builtin_keys();
  for (auto &k : leksikon_keys())
    out.push_back(std::move(k));
  for (const auto &d : columns)
    out.push_back(column_key(d));
  return out;
}

std::vector<ResourceKey> key_catalogue(const ProjectDB &db) {
  return key_catalogue(db.list_property_definitions());
}

const ResourceKey *find_key(const std::vector<ResourceKey> &catalogue,
                            std::string_view id) {
  for (const auto &k : catalogue)
    if (k.id == id)
      return &k;
  return nullptr;
}

std::optional<double> parse_number(std::string_view text) {
  const auto b = text.find_first_not_of(" \t");
  if (b == std::string_view::npos)
    return std::nullopt;
  std::string s(text.substr(b, text.find_last_not_of(" \t") - b + 1));
  if (s.find('.') == std::string::npos) {
    const auto c = s.find(',');
    if (c != std::string::npos && s.find(',', c + 1) == std::string::npos)
      s[c] = '.';
  }
  if (!s.empty() && s[0] == '+')
    s.erase(0, 1); // from_chars rejects a leading '+'
  double v = 0.0;
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (ec != std::errc{} || ptr != s.data() + s.size() || !std::isfinite(v))
    return std::nullopt;
  return v;
}

std::string format_number(double v) { return fmt::format("{}", v); }

std::optional<std::string>
normalise_value(const ResourceKey &key,
                const std::optional<std::string> &value) {
  if (!key.editable)
    throw KeyValueError(key.id, "'" + key.id + "' is read-only");
  const bool blank =
      !value || (value->empty() && key.data_type != "text");
  if (blank) {
    if (!clearable(key))
      throw KeyValueError(key.id, "'" + key.id + "' cannot be cleared");
    return std::nullopt;
  }
  const std::string &v = *value;
  if (key.data_type == "number") {
    const auto n = parse_number(v);
    if (!n)
      throw KeyValueError(key.id,
                          "'" + key.id + "' must be a number, got '" + v + "'");
    if (key.integer && std::floor(*n) != *n)
      throw KeyValueError(key.id, "'" + key.id +
                                      "' must be a whole number, got '" + v +
                                      "'");
    if (key.id == "sys:quantity" && *n < 0.0)
      throw KeyValueError(key.id, "'sys:quantity' must be a number >= 0");
    return format_number(*n);
  }
  if (key.data_type == "boolean") {
    if (v == "true" || v == "false")
      return v;
    throw KeyValueError(key.id,
                        "'" + key.id + "' must be \"true\" or \"false\"");
  }
  if (key.data_type == "enum") {
    if (std::find(key.options.begin(), key.options.end(), v) !=
        key.options.end())
      return v;
    throw KeyValueError(key.id, "'" + key.id + "' has no value '" + v + "'");
  }
  if (key.data_type == "date") {
    if (is_iso_date(v))
      return v;
    throw KeyValueError(key.id,
                        "'" + key.id + "' must be a date YYYY-MM-DD, got '" +
                            v + "'");
  }
  if (key.id == "sys:name" && v.empty())
    throw KeyValueError(key.id, "'sys:name' must be non-empty");
  return v;
}

} // namespace reusex::core
```

`sys:name` with `""`: `data_type` is `text`, so the `blank` branch does not fire, and the final check throws. That is the behaviour the test expects.

- [ ] **Step 5: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests && cd build && ctest --output-on-failure --parallel $(nproc) -R 'ResourceKeys_'`
Expected: 6 tests pass.

- [ ] **Step 6: Commit.**

```bash
git add libs/reusex/include/core/resource_keys.hpp libs/reusex/src/core/resource_keys.cpp tests/unit/core/test_resource_keys.cpp
git commit -m "feat(core): resource key catalogue and value normalisation

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 2: Template members, resolver, seeds and CSV options (pure)

**Files:**
- Create: `libs/reusex/include/core/resource_templates.hpp`, `libs/reusex/src/core/resource_templates.cpp`
- Test: `tests/unit/core/test_resource_templates.cpp`

**Interfaces:**
- Consumes: Task 1 (`ResourceKey`, `KeySource`, `find_key`, `builtin_keys`, `leksikon_categories`).
- Produces (exact). Task 7 appends the DB service to this same header.

```cpp
namespace reusex::core {
enum class MemberKind { category, key };
struct TemplateMember { MemberKind kind = MemberKind::key; std::string ref; bool operator==(const TemplateMember &) const = default; };
std::vector<TemplateMember> parse_members(const nlohmann::json &members);       // invalid_argument
std::vector<TemplateMember> read_members(std::string_view stored, std::string_view template_name); // tolerant
nlohmann::json members_json(const std::vector<TemplateMember> &members);
struct ResolvedTemplate { std::vector<std::string> keys; std::vector<TemplateMember> missing; };
ResolvedTemplate resolve_template(const std::vector<TemplateMember> &members, const std::vector<ResourceKey> &catalogue);
inline constexpr std::string_view kSeedMaterialepas = "materialepas";
inline constexpr std::string_view kSeedScreening = "screening";
struct SeedTemplate { std::string tag, name; std::vector<TemplateMember> members; };
std::vector<SeedTemplate> seed_templates();
struct CsvOptions { std::string delimiter = ";"; std::string encoding = "utf-8-bom"; std::string header = "label"; nlohmann::json extra = nlohmann::json::object(); };
CsvOptions parse_csv_options(const nlohmann::json &csv);                       // strict, invalid_argument
CsvOptions read_csv_options(std::string_view stored, std::string_view template_name); // tolerant
nlohmann::json csv_options_json(const CsvOptions &options);
inline constexpr std::string_view kLegacyKeyPrefix = "legacy:";
TemplateMember legacy_column_member(std::string_view column, const std::vector<ResourceKey> &catalogue);
std::vector<std::string> legacy_columns(const std::vector<TemplateMember> &members, const std::vector<ResourceKey> &catalogue);
std::string unique_name(std::string_view base, std::string_view tag, const std::vector<std::string> &taken);
}
```

- [ ] **Step 1: Write the failing test.** Create `tests/unit/core/test_resource_templates.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Template members and resolution (spec §5.2), seeds (§5.3), CSV options,
// and the export_templates column mapping (§5.4) — pure, no project.

#include <catch2/catch_test_macros.hpp>

#include <core/resource_keys.hpp>
#include <core/resource_templates.hpp>

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using namespace reusex::core;
using reusex::ProjectDB;
using json = nlohmann::json;

namespace {
ProjectDB::PropertyDefinition column(const char *id, const char *name) {
  ProjectDB::PropertyDefinition d;
  d.id = id;
  d.name = name;
  d.type = "text";
  return d;
}
TemplateMember cat(const char *c) { return {MemberKind::category, c}; }
TemplateMember key(const std::string &k) { return {MemberKind::key, k}; }
} // namespace

TEST_CASE("ResourceTemplates_Resolve_ExpandsDedupsAndReportsMissing",
          "[resources][templates]") {
  const auto catalogue = key_catalogue({column("a", "Bredde")});
  const auto r = resolve_template(
      {key("sys:note"), cat("Kortlægning"), key("col:a"), key("col:gone"),
       cat("Ingen"), key("sys:name")},
      catalogue);
  REQUIRE(r.keys.size() == 12);
  CHECK(r.keys[0] == "sys:note"); // first position wins
  CHECK(r.keys[1] == "sys:name");
  CHECK(r.keys[10] == "sys:starred");
  CHECK(r.keys[11] == "col:a");
  CHECK(r.missing == std::vector<TemplateMember>{key("col:gone"), cat("Ingen")});
}

TEST_CASE("ResourceTemplates_Resolve_NewKeyAppearsUnderCategoryMember",
          "[resources][templates]") {
  const std::vector<TemplateMember> members{cat("Egne felter")};
  CHECK(resolve_template(members, key_catalogue({column("a", "A")})).keys ==
        std::vector<std::string>{"col:a"});
  CHECK(resolve_template(members,
                         key_catalogue({column("a", "A"), column("b", "B")}))
            .keys == std::vector<std::string>{"col:a", "col:b"});
  CHECK(resolve_template(members, key_catalogue({})).missing.size() == 1);
}

TEST_CASE("ResourceTemplates_Members_ParseValidatesShape",
          "[resources][templates]") {
  const auto m = parse_members(
      json::parse(R"([{"category":"Owner"},{"key":"sys:name"}])"));
  CHECK(m == std::vector<TemplateMember>{cat("Owner"), key("sys:name")});
  CHECK(members_json(m) ==
        json::parse(R"([{"category":"Owner"},{"key":"sys:name"}])"));
  for (const char *bad :
       {R"({"key":"x"})", R"([{"key":""}])", R"([{"key":1}])",
        R"([{"category":"a","key":"b"}])", R"([{"other":"x"}])", R"(["x"])"}) {
    INFO(bad);
    CHECK_THROWS_AS(parse_members(json::parse(bad)), std::invalid_argument);
  }
  // Stored JSON that is broken reads as no members instead of failing.
  CHECK(read_members("not json", "T").empty());
  CHECK(read_members(R"([{"key":"sys:name"}])", "T") ==
        std::vector<TemplateMember>{key("sys:name")});
}

TEST_CASE("ResourceTemplates_Seeds_MaterialepasAndScreening",
          "[resources][templates]") {
  const auto seeds = seed_templates();
  REQUIRE(seeds.size() == 2);
  CHECK(seeds[0].tag == "materialepas");
  CHECK(seeds[0].name == "Materialepas (fuld)");
  REQUIRE(seeds[0].members.size() == leksikon_categories().size());
  CHECK(seeds[0].members.front() == cat("Owner"));
  CHECK(seeds[1].tag == "screening");
  CHECK(seeds[1].name == "Hurtig genbrugsscreening");
  std::vector<TemplateMember> screening;
  for (const auto &k : builtin_keys())
    screening.push_back(key(k.id));
  CHECK(seeds[1].members == screening);
  // Every leksikon key resolves through the full seed.
  const auto catalogue = key_catalogue({});
  CHECK(resolve_template(seeds[0].members, catalogue).keys.size() ==
        leksikon_keys().size());
}

TEST_CASE("ResourceTemplates_CsvOptions_DefaultsExtrasAndValidation",
          "[resources][templates]") {
  const auto d = parse_csv_options(json::object());
  CHECK(d.delimiter == ";");
  CHECK(d.encoding == "utf-8-bom");
  CHECK(d.header == "label");
  const auto o = parse_csv_options(
      json::parse(R"({"delimiter":",","header":"key","sort":"code"})"));
  CHECK(o.delimiter == ",");
  CHECK(o.header == "key");
  CHECK(o.extra == json::parse(R"({"sort":"code"})"));
  CHECK(csv_options_json(o) ==
        json::parse(R"({"delimiter":",","encoding":"utf-8-bom",)"
                    R"("header":"key","sort":"code"})"));
  CHECK(parse_csv_options(json::parse(R"({"delimiter":"\t"})")).delimiter ==
        "\t");
  CHECK_THROWS_AS(parse_csv_options(json::parse(R"({"delimiter":"|"})")),
                  std::invalid_argument);
  CHECK_THROWS_AS(parse_csv_options(json::parse(R"({"encoding":"latin1"})")),
                  std::invalid_argument);
  CHECK_THROWS_AS(parse_csv_options(json::parse("[]")), std::invalid_argument);
  // Stored options are read tolerantly: a bad field falls back to default.
  const auto r = read_csv_options(R"({"delimiter":"|","header":"key"})", "T");
  CHECK(r.delimiter == ";");
  CHECK(r.header == "key");
  CHECK(read_csv_options("garbage", "T").delimiter == ";");
}

TEST_CASE("ResourceTemplates_LegacyColumns_RoundTrip",
          "[resources][templates]") {
  const auto catalogue = key_catalogue({column("c1", "Bredde")});
  CHECK(legacy_column_member("Bredde", catalogue) == key("col:c1"));
  CHECK(legacy_column_member("Note", catalogue) == key("sys:note"));
  CHECK(legacy_column_member("width_mm", catalogue).ref.rfind("lex:", 0) == 0);
  CHECK(legacy_column_member("kind", catalogue) == key("legacy:kind"));
  CHECK(legacy_columns({key("legacy:kind"), key("col:c1"), key("sys:note"),
                        cat("Owner"), key("col:gone")},
                       catalogue) ==
        std::vector<std::string>{"kind", "Bredde"});
}

TEST_CASE("ResourceTemplates_UniqueName_NumbersUntilFree",
          "[resources][templates]") {
  CHECK(unique_name("A", "kopi", {"A"}) == "A (kopi)");
  CHECK(unique_name("A", "kopi", {"A", "A (kopi)"}) == "A (kopi 2)");
  CHECK(unique_name("A", "kopi", {"A", "A (kopi)", "A (kopi 2)"}) ==
        "A (kopi 3)");
  CHECK(unique_name("Mine", "eksport", {}) == "Mine (eksport)");
}
```

In `..._Resolve_ExpandsDedupsAndReportsMissing`, `cat("Kortlægning")` expands to the 11 built-ins. `sys:note` is already at position 0, so the list is `sys:note`, then the 10 other built-ins in catalogue order, then `col:a`. That makes 12. Index 1 is `sys:name`, index 10 is `sys:starred` and index 11 is `col:a`. The final `key("sys:name")` is a duplicate and is dropped.

- [ ] **Step 2: Run the test to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error `core/resource_templates.hpp: No such file or directory`.

- [ ] **Step 3: Write the header.** Create `libs/reusex/include/core/resource_templates.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Templates (docs/superpowers/specs/2026-10-02-resources-templates-ia-design.md
/// §5): an ordered list of category and key members that resolves, against
/// the live key catalogue, to an ordered list of key ids. The pure half is
/// here first; the ProjectDB-backed service follows below.

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
  std::string delimiter = ";";        ///< ";" | "," | "\t"
  std::string encoding = "utf-8-bom"; ///< "utf-8" | "utf-8-bom"
  std::string header = "label";       ///< "label" | "key"
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
```

- [ ] **Step 4: Write the implementation.** Create `libs/reusex/src/core/resource_templates.cpp`:

```cpp
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
    out.push_back({it.key() == "category" ? MemberKind::category
                                          : MemberKind::key,
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
  SeedTemplate screening{std::string(kSeedScreening),
                         "Hurtig genbrugsscreening",
                         {}};
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
  nlohmann::json out =
      o.extra.is_object() ? o.extra : nlohmann::json::object();
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
  return {MemberKind::key,
          std::string(kLegacyKeyPrefix) + std::string(column)};
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
```

- [ ] **Step 5: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests && cd build && ctest --output-on-failure --parallel $(nproc) -R 'ResourceTemplates_'`
Expected: 7 tests pass.

- [ ] **Step 6: Commit.**

```bash
git add libs/reusex/include/core/resource_templates.hpp libs/reusex/src/core/resource_templates.cpp tests/unit/core/test_resource_templates.cpp
git commit -m "feat(core): template members, resolver, seeds and CSV options

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 3: On-site backend removal (code, not yet the column)

This removes On-site from the library, the API, the help text and the demo fixture. The `samples.part_code` column itself is dropped by the v25 migration in Task 5. Until then nothing reads or writes it.

**Files:**
- Modify: `libs/reusex/include/core/ProjectDB.hpp` (`SampleRecord`, `add_sample` declaration and doc, lines ~1034–1067)
- Modify: `libs/reusex/src/core/ProjectDB.cpp` (`read_sample_row`, `sample_columns`, `add_sample`, `samples()`, `sample()`, lines ~8044–8210)
- Modify: `apps/rux/src/gui/survey.cpp` (`sample_json`, `create_sample_json`), `apps/rux/include/gui/survey.hpp:108-115`
- Modify: `apps/rux/src/gui.cpp:384-391` (help text)
- Modify: `apps/rux/frontend/dev/seed-survey-demo.sh`
- Modify: `docs/gui/openapi.yaml` (`/samples` POST, `Sample` schema)
- Test: `tests/unit/core/test_project_db_samples.cpp`, `tests/unit/rux_gui/test_gui_survey.cpp`

**Interfaces:**
- Produces: `ProjectDB::add_sample(std::string_view title, std::string_view what, const std::vector<int64_t> &type_ids = {}, core::SampleStage stage = core::SampleStage::planlagt)`. `SampleRecord` loses `part_code`. The sample JSON loses `part_code`. `POST /samples` reads only `title`, `what` and `type_ids`.

- [ ] **Step 1: Change the tests first.** In `tests/unit/core/test_project_db_samples.cpp`:
  - Delete the `// GUI Phase 6 (On-site)…` comments and these four TEST_CASEs: `Samples_Add_WithPartCode_RoundTrips`, `Samples_Add_UnknownPart_ThrowsAndWritesNothing`, `Samples_MigratesFromV23_PartCodeNull` and `Samples_Add_WithPart_LinksThePartsTypeOnce`.
  - In `Samples_Add_BadTypeOrStage_ThrowsAndWritesNothing` and `Samples_Add_FailureMidTransaction_RollsBack`, drop the `std::nullopt` argument from every `add_sample` call. `db.add_sample("x", "", std::nullopt, {9999})` becomes `db.add_sample("x", "", {9999})`, and `db.add_sample("x", "", std::nullopt, {}, core::SampleStage::svar)` becomes `db.add_sample("x", "", {}, core::SampleStage::svar)`.
  - If `add_part` in the anonymous namespace has no callers left (`grep -n "add_part(" tests/unit/core/test_project_db_samples.cpp`), delete it. If `#include <sqlite3.h>` is unused once the migration test is gone, delete it too. Keep it if `exec_raw` still uses it.

  Then add this test at the end of the file:

```cpp
TEST_CASE("Samples_Add_TypesAndStage_NoPartCode", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = add_type(db, "Vinduespartier, aluminium");
  const auto u = add_type(db, "Fuger");
  const auto s =
      db.add_sample("Asbest", "", {u, t, u}, core::SampleStage::udtaget);
  CHECK(s.type_ids == std::vector<int64_t>{t, u});
  CHECK(s.stage == core::SampleStage::udtaget);
  CHECK(db.samples_for_type(t).size() == 1);
}
```

In `tests/unit/rux_gui/test_gui_survey.cpp`, delete `CreateSample_AtAPart_LinksItsTypeAndRecordsTheCode`. Then replace `CreateSample_PartAndStage_RefusalsWriteNothing` with:

```cpp
TEST_CASE("CreateSample_Refusals_WriteNothing_OnsiteFieldsIgnored",
          "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK(status_of([&] { create_sample_json(db, R"({"title":""})"); }) == 400);
  CHECK(status_of([&] { create_sample_json(db, R"({"what":"x"})"); }) == 400);
  CHECK(status_of([&] {
          create_sample_json(db, R"({"title":"x","type_ids":[9999]})");
        }) == 404);
  CHECK(db.samples().empty());
  // On-site's fields are gone: the route ignores them like any unknown key.
  const auto s = create_sample_json(
      db, R"({"title":"PCB","part_code":"RX-404","stage":"svar"})");
  CHECK(s.at("stage") == "planlagt");
  CHECK_FALSE(s.contains("part_code"));
  CHECK_FALSE(samples_json(db).at("samples").at(0).contains("part_code"));
}
```

- [ ] **Step 2: Run the build to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error in `test_project_db_samples.cpp`. `add_sample("Asbest", "", {u, t, u}, …)` does not convert to `const std::optional<std::string> &`.

- [ ] **Step 3: Change the library.** In `ProjectDB.hpp`, delete the `part_code` member and its comment from `SampleRecord`. Replace the `add_sample` doc and declaration with:

```cpp
  /// Register a sample, its type links and its initial stage in one
  /// transaction (duplicate type ids collapse).
  /// @throws std::out_of_range when a `type_ids` entry names no survey type.
  /// @throws std::invalid_argument when `stage` is not planlagt/udtaget.
  /// Nothing is written (and no P-## code consumed) when it throws.
  SampleRecord
  add_sample(std::string_view title, std::string_view what,
             const std::vector<int64_t> &type_ids = {},
             core::SampleStage stage = core::SampleStage::planlagt);
```

In `ProjectDB.cpp`:
  - In `read_sample_row`, delete the two lines that read column 8 into `r.part_code`.
  - Replace the `sample_columns(bool)` helper and its comment with a constant:

```cpp
/// The sample columns read_sample_row expects.
constexpr const char *kSampleColumns =
    "id, code, title, what, stage, result, created_at, updated_at";
```

  - In `samples()` and `sample()`, replace `sample_columns(impl_->columnExists("samples", "part_code"))` with `kSampleColumns`.
  - In `add_sample`, change the signature to match the header. Delete the `if (part_code) { … }` block. Change the INSERT to the following, and delete the `if (part_code) bind_text(stmt, 5, …) else sqlite3_bind_null(stmt, 5);` lines:

```cpp
      sqlite3_stmt *stmt = prepare_or_throw(
          impl_->db,
          "INSERT INTO samples (code, title, what, stage) "
          "VALUES (?,?,?,?) RETURNING id;",
          "add_sample");
```

  Also change the comment `// The row, its links and its stage land together: a sample never exists with its part's type unlinked.` to `// The row, its links and its stage land together.`

- [ ] **Step 4: Change the GUI.** In `apps/rux/src/gui/survey.cpp`, delete `{"part_code", opt(s.part_code)},` from `sample_json`. In `create_sample_json`, delete the `part_code` and `stage` parsing, the stage check and its comment, and the comment about the part's type. The body becomes:

```cpp
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
```

In `apps/rux/include/gui/survey.hpp`, replace the `POST /samples` doc with:

```cpp
/// `POST /samples`: register a new environmental sample at stage `planlagt`.
/// Body: `{ "title" (required), "what"?, "type_ids"? }`; other keys are
/// ignored.
/// @throws HttpError(400) when `title` is missing or empty.
/// @throws HttpError(404) when a `type_ids` entry is unknown.
```

In `apps/rux/src/gui.cpp`, replace the second NOTES bullet (`To use On-site from a phone…`) with:

```
  - To reach the GUI from another machine on the same network, bind the LAN
    address and allow the page's own origin, e.g.
    --bind 192.168.1.20 --allow-origin http://192.168.1.20:8420
    Anyone who can reach that address can then read and change the project
    and run pipeline stages: there is still NO authentication, so do this only
    on a network you trust.
```

- [ ] **Step 5: Change the demo fixture.** In `apps/rux/frontend/dev/seed-survey-demo.sh`:
  - In the header comment, replace `, and — what On-site writes — a ★ part with a note (RX-014) and P-06 taken at RX-013.` with `, and a ★ part with a note (RX-014).`
  - Delete line 28 (the `--varied needs a rux with samples.part_code` guard).
  - In the `--varied` heredoc, delete the `INSERT INTO samples (id,code,title,what,stage,result,part_code) VALUES (6,…'RX-013');` statement and its `INSERT INTO sample_links (sample_id,type_id) VALUES (6,8);`.
  - In type 9's note, replace `★ fra on-site:` with `★ fra besigtigelsen:`.

- [ ] **Step 6: Change the contract.** In `docs/gui/openapi.yaml`, under `/samples` → `post`:
  - Set the description to `Creates a sample at stage \`planlagt\` with the next P-## code. Every refusal is checked before the first write.` followed by the existing writer-lock line.
  - Delete the `part_code` and `stage` request properties.
  - Change the `"400"` description to `title is missing or empty`.

  In `components.schemas.Sample`, remove `part_code` from `required` and from `properties`.

- [ ] **Step 7: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests rux && cd build && ctest --output-on-failure --parallel $(nproc) -R 'Samples_|CreateSample|SampleJson|gui_api_contract_parses|Survey'`
Expected: all pass. Then confirm nothing outside the migration code and the fixture's past still names the column: `grep -rn "part_code" libs apps/rux/src apps/rux/include docs/gui tests/unit` should match only `ProjectDB.cpp` (the v24 migration) and `survey.cpp`/`survey_service.cpp`/`test_survey*.cpp` (`core::part_code()`, the RX-code generator, which stays).

- [ ] **Step 8: Commit.**

```bash
git add -A libs/reusex apps/rux/src apps/rux/include apps/rux/frontend/dev/seed-survey-demo.sh docs/gui/openapi.yaml tests/unit
git commit -m "refactor: remove On-site from the backend (sample part_code, help text, demo)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 4: Schema v25 part 1 — `templates` table, seeds, export-template move, storage, legacy view

**Files:**
- Modify: `libs/reusex/include/core/survey.hpp` (add `NameConflictError` after `SamplePendingError`)
- Modify: `libs/reusex/include/core/ProjectDB.hpp` (template records, export-template doc)
- Modify: `libs/reusex/src/core/ProjectDB.cpp` (`LATEST_SCHEMA_VERSION`, `runMigrations`, new `migrateToV25` + Impl helpers, `Impl::listPropertyDefinitions`, the whole `// --- Export Templates (schema v21) ---` block at ~7513–7680 replaced)
- Test: `tests/unit/core/test_project_db_migration_v25.cpp` (new), `tests/unit/core/test_project_db_exports.cpp` (rewritten)

**Interfaces:**
- Consumes: Task 1 `key_catalogue(const std::vector<PropertyDefinition>&)`. Task 2 `seed_templates`, `members_json`, `read_members`, `legacy_column_member`, `legacy_columns`, `unique_name`, `kLegacyKeyPrefix`.
- Produces:

```cpp
namespace reusex::core {
class NameConflictError : public std::runtime_error { public: using std::runtime_error::runtime_error; };
}
// ProjectDB
struct ResourceTemplateRecord {
  int64_t id = 0;
  std::string name;
  std::string members_json = "[]";
  std::string csv_json = "{}";
  std::optional<std::string> seed; // "materialepas" | "screening" | nullopt
  std::string created_at, updated_at;
};
struct ResourceTemplatePatch { std::optional<std::string> name, members_json, csv_json; };
[[nodiscard]] std::vector<ResourceTemplateRecord> resource_templates() const; // by id; {} before v25
[[nodiscard]] std::optional<ResourceTemplateRecord> resource_template(int64_t id) const;
ResourceTemplateRecord add_resource_template(const ResourceTemplateRecord &rec); // NameConflictError
ResourceTemplateRecord update_resource_template(int64_t id, const ResourceTemplatePatch &patch); // out_of_range, NameConflictError
bool delete_resource_template(int64_t id);
// unchanged signatures, now a view over `templates`:
ExportTemplateRecord add_export_template(...); list_export_templates(); export_template(id); update_export_template(...); delete_export_template(id);
```

- [ ] **Step 1: Write the failing migration test.** Create `tests/unit/core/test_project_db_migration_v25.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v24 -> v25 (resources/templates spec §4.2, §5.3, §5.4): a fresh
// project is created at the latest version and rolled back to v24 with raw
// SQL, then reopened read-write so migrateToV25 runs on real v24 data.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resource_templates.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <filesystem>
#include <string>
#include <vector>

using reusex::ProjectDB;
namespace core = reusex::core;
namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_migration_v25") {}
};

void exec_raw(const fs::path &path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, &err);
  INFO((err ? err : ""));
  sqlite3_free(err);
  sqlite3_close(raw);
  REQUIRE(rc == SQLITE_OK);
}

/// Undo everything v25 adds, so the next read-write open migrates again.
void roll_back_to_v24(const fs::path &path) {
  exec_raw(path, R"sql(
    DROP TABLE templates;
    CREATE TABLE export_templates (
      id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL,
      config TEXT NOT NULL DEFAULT '{}',
      created_at TEXT NOT NULL DEFAULT (datetime('now')),
      updated_at TEXT NOT NULL DEFAULT (datetime('now')));
    DELETE FROM schema_version WHERE version >= 25;
  )sql");
}

bool table_exists(const fs::path &path, const char *name) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  sqlite3_prepare_v2(raw,
                     "SELECT 1 FROM sqlite_master WHERE type='table' AND "
                     "name=?;",
                     -1, &s, nullptr);
  sqlite3_bind_text(s, 1, name, -1, SQLITE_STATIC);
  const bool found = sqlite3_step(s) == SQLITE_ROW;
  sqlite3_finalize(s);
  sqlite3_close(raw);
  return found;
}

std::vector<std::string> names(const ProjectDB &db) {
  std::vector<std::string> out;
  for (const auto &t : db.resource_templates())
    out.push_back(t.name);
  return out;
}
} // namespace

TEST_CASE("MigrationV25_FreshProject_HasBothSeeds", "[ProjectDB][migration]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == 25);
  const auto list = db.resource_templates();
  REQUIRE(list.size() == 2);
  CHECK(list[0].name == "Materialepas (fuld)");
  CHECK(list[0].seed == std::optional<std::string>("materialepas"));
  CHECK(core::read_members(list[0].members_json, "") ==
        core::seed_templates()[0].members);
  CHECK(list[1].name == "Hurtig genbrugsscreening");
  CHECK(list[1].seed == std::optional<std::string>("screening"));
  CHECK(list[1].csv_json == "{}");
  CHECK_FALSE(table_exists(tmp.path, "export_templates"));
}

TEST_CASE("MigrationV25_MovesExportTemplates_ClashUnmatchedAndBroken",
          "[ProjectDB][migration]") {
  TempDB tmp;
  std::string bredde;
  {
    ProjectDB db(tmp.path);
    bredde = db.add_property_definition("Bredde", "number", {}, 0);
  }
  roll_back_to_v24(tmp.path);
  exec_raw(tmp.path, R"sql(
    INSERT INTO export_templates (name, config) VALUES
      ('Materialepas (fuld)', '{"columns":["Bredde","kind"],"delimiter":","}'),
      ('Mine', '{"columns":["Note"]}'),
      ('Mine', '{}'),
      ('Ødelagt', 'not json');
  )sql");
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == 25);
  CHECK_FALSE(table_exists(tmp.path, "export_templates"));
  CHECK(names(db) == std::vector<std::string>{
                         "Materialepas (fuld)", "Hurtig genbrugsscreening",
                         "Materialepas (fuld) (eksport)", "Mine",
                         "Mine (eksport)", "Ødelagt"});
  const auto list = db.resource_templates();
  CHECK(core::read_members(list[2].members_json, "") ==
        std::vector<core::TemplateMember>{
            {core::MemberKind::key, "col:" + bredde},
            {core::MemberKind::key, "legacy:kind"}});
  CHECK(json::parse(list[2].csv_json) == json::parse(R"({"delimiter":","})"));
  CHECK_FALSE(list[2].seed.has_value());
  CHECK(core::read_members(list[3].members_json, "") ==
        std::vector<core::TemplateMember>{{core::MemberKind::key, "sys:note"}});
  CHECK(list[5].members_json == "[]");
  CHECK(list[5].csv_json == "{}");
}

TEST_CASE("MigrationV25_SecondOpenIsANoOp", "[ProjectDB][migration]") {
  TempDB tmp;
  { ProjectDB db(tmp.path); }
  roll_back_to_v24(tmp.path);
  exec_raw(tmp.path,
           "INSERT INTO export_templates (name, config) VALUES ('A', '{}');");
  { ProjectDB db(tmp.path); }
  ProjectDB db(tmp.path);
  CHECK(names(db) == std::vector<std::string>{"Materialepas (fuld)",
                                              "Hurtig genbrugsscreening", "A"});
}

TEST_CASE("TemplatesStore_Crud_NameIsUnique", "[ProjectDB][templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  ProjectDB::ResourceTemplateRecord rec;
  rec.name = "Ny";
  rec.members_json = R"([{"key":"sys:name"}])";
  const auto added = db.add_resource_template(rec);
  CHECK(added.id > 0);
  CHECK(added.members_json == rec.members_json);
  CHECK_FALSE(added.created_at.empty());
  CHECK_THROWS_AS(db.add_resource_template(rec), core::NameConflictError);
  ProjectDB::ResourceTemplatePatch p;
  p.name = "Hurtig genbrugsscreening";
  CHECK_THROWS_AS(db.update_resource_template(added.id, p),
                  core::NameConflictError);
  p.name = "Omdøbt";
  p.csv_json = R"({"delimiter":","})";
  const auto updated = db.update_resource_template(added.id, p);
  CHECK(updated.name == "Omdøbt");
  CHECK(updated.csv_json == R"({"delimiter":","})");
  CHECK(updated.members_json == rec.members_json);
  CHECK_THROWS_AS(db.update_resource_template(9999, p), std::out_of_range);
  CHECK(db.delete_resource_template(added.id));
  CHECK_FALSE(db.delete_resource_template(added.id));
  CHECK_FALSE(db.resource_template(added.id).has_value());
}
```

Then rewrite `tests/unit/core/test_project_db_exports.cpp`. Keep its SPDX header, `TempDB`, `build_v20_fixture` and includes, and replace the file comment with `// The /export-templates view over the v25 templates table (resources/templates spec §5.4): CRUD round trips keep working for ExportPage and ruxd.`. Add `#include <core/resource_templates.hpp>` and `#include <nlohmann/json.hpp>`. Replace every TEST_CASE with:

```cpp
TEST_CASE("ExportTemplates_FreshDB_ListsTheSeeds", "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
  const auto list = db.list_export_templates();
  REQUIRE(list.size() == 2);
  CHECK(list[0].name == "Materialepas (fuld)");
  // A category-only template has no legacy columns: ExportPage reads [] as
  // "all columns".
  CHECK(nlohmann::json::parse(list[0].config_json).at("columns") ==
        nlohmann::json::array());
}

TEST_CASE("ExportTemplates_MigratesFromV20", "[ProjectDB][exports]") {
  TempDB tmp;
  build_v20_fixture(tmp.path);
  ProjectDB db(tmp.path, /*readOnly=*/false);
  REQUIRE(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK(db.list_export_templates().size() == 2);
}

TEST_CASE("ExportTemplates_AddFetch_RoundTripsColumnsThroughMembers",
          "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto col = db.add_property_definition("Bredde", "number", {}, 0);
  const auto rec = db.add_export_template(
      "Valg", R"({"columns":["kind","id","Bredde"],"delimiter":","})");
  CHECK(rec.name == "Valg");
  const auto cfg = nlohmann::json::parse(rec.config_json);
  CHECK(cfg.at("columns") ==
        nlohmann::json::parse(R"(["kind","id","Bredde"])"));
  CHECK(cfg.at("delimiter") == ",");
  const auto stored = db.resource_template(rec.id);
  REQUIRE(stored.has_value());
  CHECK(core::read_members(stored->members_json, "") ==
        std::vector<core::TemplateMember>{
            {core::MemberKind::key, "legacy:kind"},
            {core::MemberKind::key, "legacy:id"},
            {core::MemberKind::key, "col:" + col}});
  const auto fetched = db.export_template(rec.id);
  REQUIRE(fetched.has_value());
  CHECK(fetched->config_json == rec.config_json);
  CHECK_FALSE(db.export_template(9999).has_value());
  CHECK_THROWS_AS(db.add_export_template("Valg", "{}"),
                  core::NameConflictError);
}

TEST_CASE("ExportTemplates_Update_ColumnsAndName_KeepsOtherOptions",
          "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto orig =
      db.add_export_template("Old", R"({"columns":["kind"],"delimiter":","})");
  const auto updated =
      db.update_export_template(orig.id, "New", R"({"columns":["kind","id"]})");
  CHECK(updated.name == "New");
  const auto cfg = nlohmann::json::parse(updated.config_json);
  CHECK(cfg.at("columns") == nlohmann::json::parse(R"(["kind","id"])"));
  // A body carrying only columns (all ExportPage sends) keeps csv options.
  CHECK(cfg.at("delimiter") == ",");
  CHECK_THROWS_AS(db.update_export_template(9999, "x", "{}"),
                  std::runtime_error);
}

TEST_CASE("ExportTemplates_Delete_AndOrder", "[ProjectDB][exports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.add_export_template("First", "{}");
  const auto second = db.add_export_template("Second", "{}");
  CHECK(db.delete_export_template(second.id));
  CHECK_FALSE(db.delete_export_template(second.id));
  const auto list = db.list_export_templates();
  REQUIRE(list.size() == 3);
  CHECK(list[2].name == "First");
}
```

- [ ] **Step 2: Run the build to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests`
Expected: compile errors. `ResourceTemplateRecord`, `resource_templates` and `core::NameConflictError` are undeclared.

- [ ] **Step 3: Add `NameConflictError`.** In `libs/reusex/include/core/survey.hpp`, add this after `SamplePendingError`:

```cpp
/// A name that must be unique (template, user column, stored field) is
/// already taken. The GUI answers 409.
class NameConflictError : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};
```

- [ ] **Step 4: Add the records to `ProjectDB.hpp`.** Replace the `// --- Export Templates (schema v21) ---` block (the struct and its five declarations) with:

```cpp
  // --- Resource templates (schema v25) ---
  /// One row of `templates` (resources/templates spec §5.1). members_json
  /// and csv_json are parsed by core/resource_templates.hpp.
  struct ResourceTemplateRecord {
    int64_t id = 0;
    std::string name;
    std::string members_json = "[]";
    std::string csv_json = "{}";
    std::optional<std::string> seed; // "materialepas" | "screening"
    std::string created_at;          // ISO 8601 UTC
    std::string updated_at;
  };
  struct ResourceTemplatePatch {
    std::optional<std::string> name, members_json, csv_json;
  };
  /// By id. Empty on a read-only open of a pre-v25 project.
  [[nodiscard]] std::vector<ResourceTemplateRecord> resource_templates() const;
  [[nodiscard]] std::optional<ResourceTemplateRecord>
  resource_template(int64_t id) const;
  /// id/timestamps are ignored. @throws core::NameConflictError
  ResourceTemplateRecord
  add_resource_template(const ResourceTemplateRecord &rec);
  /// @throws std::out_of_range, core::NameConflictError
  ResourceTemplateRecord
  update_resource_template(int64_t id, const ResourceTemplatePatch &patch);
  bool delete_resource_template(int64_t id); // false when absent

  // --- Export templates: legacy view over `templates` (schema v25) ---
  /// The schema v21 shape, kept for the /export-templates routes (rux gui
  /// until GUI Phase 4, and ruxd). `config_json` is the template's CSV
  /// options plus `columns`: legacy-member names and user-column labels. A
  /// write maps `columns` back with core::legacy_column_member; other config
  /// fields become the CSV options (only when the body has any).
  struct ExportTemplateRecord {
    int64_t id = 0;
    std::string name;
    std::string config_json; // JSON: {"columns": [...], ...csv options}
    std::string created_at;  // ISO 8601 UTC
    std::string updated_at;  // ISO 8601 UTC
  };

  /// @throws core::NameConflictError
  ExportTemplateRecord add_export_template(const std::string &name,
                                           const std::string &config_json);
  [[nodiscard]] std::vector<ExportTemplateRecord> list_export_templates() const;
  [[nodiscard]] std::optional<ExportTemplateRecord>
  export_template(int64_t id) const;
  /// @throws std::runtime_error when @p id is unknown, core::NameConflictError
  ExportTemplateRecord update_export_template(int64_t id,
                                              const std::string &name,
                                              const std::string &config_json);
  bool delete_export_template(int64_t id);
```

- [ ] **Step 5: Add the migration.** In `ProjectDB.cpp`:
  - Add `#include "core/resource_keys.hpp"` and `#include "core/resource_templates.hpp"` to the includes.
  - Add this after `prepare_or_throw` in the top anonymous namespace:

```cpp
/// Spec §5.1, verbatim. IF NOT EXISTS because tests that roll a project
/// back past v25 without dropping the table re-run migrateToV25.
constexpr const char *kTemplatesSchema = R"(
  CREATE TABLE IF NOT EXISTS templates (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    name       TEXT NOT NULL UNIQUE,
    members    TEXT NOT NULL DEFAULT '[]',
    csv        TEXT NOT NULL DEFAULT '{}',
    seed       TEXT,
    created_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),
    updated_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
  );
)";
```

  - Change `LATEST_SCHEMA_VERSION = 24` to `25`, and after `if (current < 24) migrateToV24();` add `if (current < 25) { migrateToV25(); }`.
  - Move the body of `ProjectDB::list_property_definitions()` into a new `Impl` member `std::vector<ProjectDB::PropertyDefinition> listPropertyDefinitions() const` (replace `impl_->db` with `db`). The public method becomes `return impl_->listPropertyDefinitions();`.
  - Add these `Impl` members after `migrateToV24()`:

```cpp
  int64_t queryInt(const char *sql) const {
    sqlite3_stmt *s = prepare_or_throw(db, sql, "queryInt");
    StmtGuard guard(s);
    if (sqlite3_step(s) != SQLITE_ROW)
      throw std::runtime_error(std::string("query failed: ") + sql + ": " +
                               sqlite3_errmsg(db));
    return sqlite3_column_int64(s, 0);
  }

  /// Insert one templates row. @throws core::NameConflictError on a taken name.
  int64_t insertTemplate(const std::string &name, const std::string &members,
                         const std::string &csv,
                         const std::optional<std::string> &seed) {
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "INSERT INTO templates (name, members, csv, seed) VALUES (?,?,?,?) "
        "RETURNING id;",
        "insert template");
    StmtGuard guard(s);
    bind_text(s, 1, name);
    bind_text(s, 2, members);
    bind_text(s, 3, csv);
    if (seed)
      bind_text(s, 4, *seed);
    else
      sqlite3_bind_null(s, 4);
    if (sqlite3_step(s) != SQLITE_ROW) {
      if (sqlite3_extended_errcode(db) == SQLITE_CONSTRAINT_UNIQUE)
        throw core::NameConflictError("a template named '" + name +
                                      "' already exists");
      throw std::runtime_error("insert template '" + name +
                               "': " + sqlite3_errmsg(db));
    }
    return sqlite3_column_int64(s, 0);
  }

  std::vector<std::string> templateNames() const {
    sqlite3_stmt *s =
        prepare_or_throw(db, "SELECT name FROM templates;", "template names");
    StmtGuard guard(s);
    std::vector<std::string> out;
    while (sqlite3_step(s) == SQLITE_ROW)
      out.push_back(column_text(s, 0));
    return out;
  }

  std::size_t seedTemplates() {
    std::size_t n = 0;
    for (const auto &seed : core::seed_templates()) {
      insertTemplate(seed.name, core::members_json(seed.members).dump(), "{}",
                     seed.tag);
      ++n;
    }
    return n;
  }

  struct MovedTemplates {
    std::size_t moved = 0;
    std::vector<std::string> unmatched; // "<template>: <column>"
  };
  /// Spec §5.4: every export_templates row becomes a templates row; its
  /// columns become key members, the rest of its config the CSV options.
  MovedTemplates moveExportTemplates() {
    MovedTemplates out;
    if (!tableExists("export_templates"))
      return out;
    const auto catalogue = core::key_catalogue(listPropertyDefinitions());
    auto taken = templateNames();
    std::vector<std::pair<std::string, std::string>> rows;
    {
      sqlite3_stmt *s = prepare_or_throw(
          db, "SELECT name, config FROM export_templates ORDER BY id;",
          "migrateToV25");
      StmtGuard guard(s);
      while (sqlite3_step(s) == SQLITE_ROW)
        rows.emplace_back(column_text(s, 0), column_text(s, 1));
    }
    for (const auto &[name, config] : rows) {
      auto cfg = nlohmann::json::parse(config, nullptr, false);
      if (cfg.is_discarded() || !cfg.is_object()) {
        reusex::warn("v25: export template '{}' had unreadable config; moved "
                     "with no columns",
                     name);
        cfg = nlohmann::json::object();
      }
      std::vector<core::TemplateMember> members;
      if (cfg.contains("columns") && cfg["columns"].is_array())
        for (const auto &col : cfg["columns"]) {
          if (!col.is_string())
            continue;
          auto m = core::legacy_column_member(col.get<std::string>(), catalogue);
          if (m.ref.rfind(core::kLegacyKeyPrefix, 0) == 0)
            out.unmatched.push_back(name + ": " + col.get<std::string>());
          members.push_back(std::move(m));
        }
      cfg.erase("columns");
      const bool clash =
          std::find(taken.begin(), taken.end(), name) != taken.end();
      const auto final_name =
          clash ? core::unique_name(name, "eksport", taken) : name;
      insertTemplate(final_name, core::members_json(members).dump(),
                     cfg.dump(), std::nullopt);
      taken.push_back(final_name);
      ++out.moved;
    }
    execOrThrow("DROP TABLE export_templates;");
    if (!out.unmatched.empty()) {
      std::string list;
      for (const auto &u : out.unmatched)
        list += (list.empty() ? "" : "; ") + u;
      reusex::warn("v25: {} export-template column(s) match no resource key "
                   "and are kept as missing members: {}",
                   out.unmatched.size(), list);
    }
    return out;
  }

  void migrateToV25() {
    reusex::info("Migrating database to schema version 25");
    // Resources and templates (docs/superpowers/specs/
    // 2026-10-02-resources-templates-ia-design.md §4.2): one transaction, so
    // a v24 project upgrades whole or stays v24. Every step is guarded so a
    // test that rolls the version back and reopens re-runs it safely.
    execOrThrow("BEGIN TRANSACTION;");
    try {
      // (Task 5 adds the resource-passport steps here.)
      execOrThrow(kTemplatesSchema);
      // Seeds first: the table is new, so this is "if templates is empty",
      // and moved export templates can then be suffixed against the seeds.
      std::size_t seeded = 0;
      if (queryInt("SELECT COUNT(*) FROM templates;") == 0)
        seeded = seedTemplates();
      const auto moved = moveExportTemplates();
      insertSchemaVersion(25, "Resources and templates: passport per part, "
                              "templates table, no On-site part_code");
      execOrThrow("COMMIT;");
      reusex::info("Migration to schema version 25 complete: {} seed "
                   "template(s), {} export template(s) moved, {} column(s) "
                   "unmatched",
                   seeded, moved.moved, moved.unmatched.size());
    } catch (...) {
      if (sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr) !=
          SQLITE_OK)
        reusex::warn("migrateToV25: ROLLBACK failed: {}", sqlite3_errmsg(db));
      throw;
    }
  }
```

- [ ] **Step 6: Replace the export-template storage.** In `ProjectDB.cpp`, delete everything from `// --- Export Templates (schema v21) ---` up to the end of `ProjectDB::delete_export_template`, and put this in its place:

```cpp
// --- Resource templates (schema v25) ---

namespace {
constexpr const char *kTemplateColumns =
    "id, name, members, csv, seed, created_at, updated_at";

ProjectDB::ResourceTemplateRecord read_template(sqlite3_stmt *s) {
  ProjectDB::ResourceTemplateRecord r;
  r.id = sqlite3_column_int64(s, 0);
  r.name = column_text(s, 1);
  r.members_json = column_text(s, 2);
  r.csv_json = column_text(s, 3);
  if (sqlite3_column_type(s, 4) != SQLITE_NULL)
    r.seed = column_text(s, 4);
  r.created_at = column_text(s, 5);
  r.updated_at = column_text(s, 6);
  return r;
}
} // namespace

std::vector<ProjectDB::ResourceTemplateRecord>
ProjectDB::resource_templates() const {
  if (!impl_->tableExists("templates"))
    return {};
  const std::string sql =
      std::string("SELECT ") + kTemplateColumns + " FROM templates ORDER BY id;";
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, sql.c_str(), "resource_templates");
  StmtGuard guard(stmt);
  std::vector<ResourceTemplateRecord> out;
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(read_template(stmt));
  return out;
}

std::optional<ProjectDB::ResourceTemplateRecord>
ProjectDB::resource_template(int64_t id) const {
  if (!impl_->tableExists("templates"))
    return std::nullopt;
  const std::string sql = std::string("SELECT ") + kTemplateColumns +
                          " FROM templates WHERE id = ?;";
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, sql.c_str(), "resource_template");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    return std::nullopt;
  return read_template(stmt);
}

ProjectDB::ResourceTemplateRecord
ProjectDB::add_resource_template(const ResourceTemplateRecord &rec) {
  impl_->checkWritable();
  const auto id = impl_->insertTemplate(rec.name, rec.members_json,
                                        rec.csv_json, rec.seed);
  return *resource_template(id);
}

ProjectDB::ResourceTemplateRecord
ProjectDB::update_resource_template(int64_t id,
                                    const ResourceTemplatePatch &p) {
  impl_->checkWritable();
  if (!resource_template(id))
    throw std::out_of_range("no template " + std::to_string(id));
  std::vector<std::string> sets;
  if (p.name)
    sets.emplace_back("name = ?");
  if (p.members_json)
    sets.emplace_back("members = ?");
  if (p.csv_json)
    sets.emplace_back("csv = ?");
  if (sets.empty())
    return *resource_template(id);
  std::string sql = "UPDATE templates SET ";
  for (const auto &s : sets)
    sql += s + ", ";
  sql += "updated_at = strftime('%Y-%m-%dT%H:%M:%SZ','now') WHERE id = ?;";
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, sql.c_str(), "update_resource_template");
  StmtGuard guard(stmt);
  int i = 1;
  if (p.name)
    bind_text(stmt, i++, *p.name);
  if (p.members_json)
    bind_text(stmt, i++, *p.members_json);
  if (p.csv_json)
    bind_text(stmt, i++, *p.csv_json);
  sqlite3_bind_int64(stmt, i, id);
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    if (sqlite3_extended_errcode(impl_->db) == SQLITE_CONSTRAINT_UNIQUE)
      throw core::NameConflictError("a template named '" + p.name.value_or("") +
                                    "' already exists");
    throw std::runtime_error("update_resource_template: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  return *resource_template(id);
}

bool ProjectDB::delete_resource_template(int64_t id) {
  impl_->checkWritable();
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db, "DELETE FROM templates WHERE id = ?;", "delete template");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("delete_resource_template: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  return sqlite3_changes(impl_->db) > 0;
}

// --- Export templates: legacy view over `templates` (schema v25) ---

namespace {
ProjectDB::ExportTemplateRecord
as_export_template(const ProjectDB::ResourceTemplateRecord &t,
                   const std::vector<core::ResourceKey> &catalogue) {
  auto config = nlohmann::json::parse(t.csv_json, nullptr, false);
  if (config.is_discarded() || !config.is_object())
    config = nlohmann::json::object();
  config["columns"] = core::legacy_columns(
      core::read_members(t.members_json, t.name), catalogue);
  return {t.id, t.name, config.dump(), t.created_at, t.updated_at};
}

/// An export config split into (members_json, csv_json). Either is nullopt
/// when the config does not carry it, so an update keeps what is stored.
std::pair<std::optional<std::string>, std::optional<std::string>>
from_export_config(const std::string &config_json,
                   const std::vector<core::ResourceKey> &catalogue) {
  auto config = nlohmann::json::parse(config_json, nullptr, false);
  if (config.is_discarded() || !config.is_object())
    config = nlohmann::json::object();
  std::optional<std::string> members;
  if (config.contains("columns") && config["columns"].is_array()) {
    std::vector<core::TemplateMember> m;
    for (const auto &c : config["columns"])
      if (c.is_string())
        m.push_back(core::legacy_column_member(c.get<std::string>(), catalogue));
    members = core::members_json(m).dump();
  }
  config.erase("columns");
  std::optional<std::string> csv;
  if (!config.empty())
    csv = config.dump();
  return {members, csv};
}
} // namespace

ProjectDB::ExportTemplateRecord
ProjectDB::add_export_template(const std::string &name,
                               const std::string &config_json) {
  const auto catalogue = core::key_catalogue(list_property_definitions());
  const auto [members, csv] = from_export_config(config_json, catalogue);
  ResourceTemplateRecord rec;
  rec.name = name;
  rec.members_json = members.value_or("[]");
  rec.csv_json = csv.value_or("{}");
  return as_export_template(add_resource_template(rec), catalogue);
}

std::vector<ProjectDB::ExportTemplateRecord>
ProjectDB::list_export_templates() const {
  const auto catalogue = core::key_catalogue(list_property_definitions());
  std::vector<ExportTemplateRecord> out;
  for (const auto &t : resource_templates())
    out.push_back(as_export_template(t, catalogue));
  return out;
}

std::optional<ProjectDB::ExportTemplateRecord>
ProjectDB::export_template(int64_t id) const {
  const auto t = resource_template(id);
  if (!t)
    return std::nullopt;
  return as_export_template(*t,
                            core::key_catalogue(list_property_definitions()));
}

ProjectDB::ExportTemplateRecord
ProjectDB::update_export_template(int64_t id, const std::string &name,
                                  const std::string &config_json) {
  if (!resource_template(id))
    throw std::runtime_error("update_export_template: id not found: " +
                             std::to_string(id));
  const auto catalogue = core::key_catalogue(list_property_definitions());
  const auto [members, csv] = from_export_config(config_json, catalogue);
  ResourceTemplatePatch p;
  p.name = name;
  p.members_json = members;
  p.csv_json = csv;
  return as_export_template(update_resource_template(id, p), catalogue);
}

bool ProjectDB::delete_export_template(int64_t id) {
  return delete_resource_template(id);
}
```

`config.erase("columns")` on a json object returns the count erased and is harmless when the key is absent. `ExportTemplateRecord` stays an aggregate, so the brace return works.

- [ ] **Step 7: Map the 409 in the rux gui export-template handlers.** In `apps/rux/src/gui/api.cpp`, change the record call in `create_export_template_json` to:

```cpp
  try {
    return template_record_json(db.add_export_template(name, config_json));
  } catch (const reusex::core::NameConflictError &e) {
    throw HttpError(409, e.what());
  }
```

Do the same around `db.update_export_template(...)` in `update_export_template_json`.

- [ ] **Step 8: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests ruxd && cd build && ctest --output-on-failure --parallel $(nproc) -R 'MigrationV25_|TemplatesStore_|ExportTemplates_|ProjectDB|Survey|Samples_'`
Expected: all pass. The older rollback tests (`SurveySchema_MigratesFromV21`, the v10/v12/v23 migration tests) re-run `migrateToV25` on their rolled-back databases. That works because every step is guarded, so a failure here means a step lacks its guard.

- [ ] **Step 9: Commit.**

```bash
git add libs/reusex apps/rux/src/gui/api.cpp tests/unit/core/test_project_db_migration_v25.cpp tests/unit/core/test_project_db_exports.cpp
git commit -m "feat(core): schema v25 templates table, seeds and export-template view

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 5: Schema v25 part 2 — one passport per part, `Transaction`, resource storage

**Files:**
- Create: `tests/support/survey_fixture.hpp`
- Modify: `libs/reusex/include/core/ProjectDB.hpp` (`Transaction`, `SurveyPartRecord::material_guid` doc, new methods)
- Modify: `libs/reusex/src/core/ProjectDB.cpp` (`migrateToV25` resource steps, `Impl::linkResourcePassports`, `Impl::copyPassport`, `Impl::setInstanceMaterial` sync, `kSurveyPartSelect` → `survey_part_select`, new methods)
- Test: `tests/unit/core/test_project_db_resources.cpp` (new), `tests/unit/core/test_project_db_migration_v25.cpp` (extended)

**Interfaces:**
- Produces:

```cpp
class ProjectDB::Transaction {
 public:
  explicit Transaction(ProjectDB &db); // BEGIN IMMEDIATE; throws on a read-only db
  ~Transaction();                      // ROLLBACK unless committed; warns if that fails
  void commit();
  Transaction(const Transaction &) = delete;
  Transaction &operator=(const Transaction &) = delete;
};
/// Call inside a Transaction. The part's passport guid, created (and linked
/// in instance_materials for an instance-backed part) on first use.
std::string ensure_resource_passport(std::string_view code);   // out_of_range
void delete_survey_part(std::string_view code);                 // out_of_range
/// Call inside a Transaction. Moves a user column's stored values.
void rename_passport_field(std::string_view old_name, std::string_view new_name); // NameConflictError
[[nodiscard]] bool is_passport_linked(std::string_view guid) const; // any part or instance link
```
`SurveyPartRecord::material_guid` now means `COALESCE(passport_guid, instance_materials.material_guid)`.

- [ ] **Step 1: Write the shared fixture.** Create `tests/support/survey_fixture.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Survey/resource test fixtures shared by the resource tests: an instance
// cloud, survey types, parts (manual or instance-backed) and bare passports.
// Include as "../../support/survey_fixture.hpp" from tests/unit/<module>/.

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>

#include <pcl/point_types.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace reusex::test_support {

/// An "instances" cloud with instances 1..n (one point each, class 3, guid
/// "guid-inst-<i>").
inline void make_instance_cloud(ProjectDB &db, std::uint32_t n) {
  CloudL labels;
  std::vector<ProjectDB::InstanceRecord> rows;
  for (std::uint32_t i = 1; i <= n; ++i) {
    pcl::Label p;
    p.label = i;
    labels.push_back(p);
    rows.push_back({i, "guid-inst-" + std::to_string(i), 3, 1});
  }
  db.save_point_cloud("instances", labels, "test", "{}");
  db.save_instances("instances", rows);
}

inline int64_t make_type(ProjectDB &db, const std::string &name,
                         core::Treatment t = core::Treatment::genbrug) {
  ProjectDB::SurveyTypeRecord rec;
  rec.name = name;
  rec.treatment = t;
  rec.unit = "stk";
  return db.add_survey_type(rec).id;
}

/// A part, instance-backed when @p instance_id is given.
inline void make_part(ProjectDB &db, const std::string &code, int64_t type_id,
                      std::optional<std::uint32_t> instance_id = std::nullopt,
                      double quantity = 1.0) {
  ProjectDB::SurveyPartRecord p;
  p.code = code;
  p.type_id = type_id;
  if (instance_id) {
    p.cloud_name = "instances";
    p.instance_id = *instance_id;
  }
  p.quantity = quantity;
  db.add_survey_part(p);
}

inline void make_passport(ProjectDB &db, const std::string &guid) {
  core::MaterialPassport p;
  p.metadata.document_guid = guid;
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  db.add_material_passport(p, "");
}

} // namespace reusex::test_support
```

- [ ] **Step 2: Write the failing storage test.** Create `tests/unit/core/test_project_db_resources.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Resource storage (schema v25): transactions, the lazy per-part passport
// and its instance_materials link, part delete, stored-field rename, and
// set_instance_material keeping a part's passport in step.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resource_keys.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <string>

using reusex::ProjectDB;
namespace core = reusex::core;
using namespace reusex::test_support;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_resources") {}
};
std::string guid_of_field(const char *field) {
  for (const auto &f : core::leksikon_fields())
    if (f.field_name == field)
      return f.guid;
  FAIL("no leksikon field " << field);
  return {};
}
} // namespace

TEST_CASE("ResourceStore_Transaction_CommitsOrRollsBack",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  ProjectDB::SurveyPartPatch p;
  p.note = "rolled back";
  {
    ProjectDB::Transaction tx(db);
    db.update_survey_part("RX-001", p);
  }
  CHECK(db.survey_part("RX-001")->note.empty());
  p.note = "kept";
  {
    ProjectDB::Transaction tx(db);
    db.update_survey_part("RX-001", p);
    tx.commit();
  }
  CHECK(db.survey_part("RX-001")->note == "kept");
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK_THROWS_AS(ProjectDB::Transaction(ro), std::runtime_error);
}

TEST_CASE("ResourceStore_EnsurePassport_ManualPart_OnceAndLeksikonReady",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  std::string guid;
  {
    ProjectDB::Transaction tx(db);
    guid = db.ensure_resource_passport("RX-001");
    CHECK(db.ensure_resource_passport("RX-001") == guid);
    // A leksikon field lands under its leksikon definition, not "custom:".
    db.set_passport_property(guid, "width_mm", "600");
    tx.commit();
  }
  CHECK(db.survey_part("RX-001")->material_guid == guid);
  CHECK(db.passport_stored_properties(guid).at("width_mm") == "600");
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  sqlite3_prepare_v2(raw, "SELECT property_id FROM passport_property_values;",
                     -1, &s, nullptr);
  REQUIRE(sqlite3_step(s) == SQLITE_ROW);
  CHECK(std::string(reinterpret_cast<const char *>(
            sqlite3_column_text(s, 0))) == guid_of_field("width_mm"));
  sqlite3_finalize(s);
  sqlite3_close(raw);
  CHECK_THROWS_AS(db.ensure_resource_passport("RX-404"), std::out_of_range);
}

TEST_CASE("ResourceStore_EnsurePassport_InstancePart_UpsertsLink",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 2);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  std::string guid;
  {
    ProjectDB::Transaction tx(db);
    guid = db.ensure_resource_passport("RX-001");
    tx.commit();
  }
  CHECK(db.instance_material_guid("instances", 1) == guid);
  CHECK(db.is_passport_linked(guid));
}

TEST_CASE("ResourceStore_EnsurePassport_AdoptsUnownedInstanceLink",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  make_passport(db, "guid-cli");
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  db.set_instance_material("instances", 1, "guid-cli");
  ProjectDB::Transaction tx(db);
  CHECK(db.ensure_resource_passport("RX-001") == "guid-cli");
  tx.commit();
}

TEST_CASE("ResourceStore_SetInstanceMaterial_MovesPartPassportUnlessOwned",
          "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 2);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  make_part(db, "RX-002", t, 2u);
  std::string a, c;
  {
    ProjectDB::Transaction tx(db);
    a = db.ensure_resource_passport("RX-001");
    c = db.ensure_resource_passport("RX-002");
    tx.commit();
  }
  make_passport(db, "guid-b");
  db.set_instance_material("instances", 1, "guid-b");
  CHECK(db.survey_part("RX-001")->material_guid == "guid-b");
  // guid-b now belongs to RX-001: linking it to instance 2 cannot move
  // RX-002's passport (warn), so RX-002 keeps its own.
  db.set_instance_material("instances", 2, "guid-b");
  CHECK(db.survey_part("RX-002")->material_guid == c);
  CHECK(db.instance_material_guid("instances", 2) == "guid-b");
}

TEST_CASE("ResourceStore_DeletePart_AndRenameField", "[ProjectDB][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  std::string guid;
  {
    ProjectDB::Transaction tx(db);
    guid = db.ensure_resource_passport("RX-001");
    db.set_passport_property(guid, "Stand", "God");
    db.set_passport_property(guid, "Gammel", "x");
    tx.commit();
  }
  {
    ProjectDB::Transaction tx(db);
    db.rename_passport_field("Stand", "Tilstand");
    tx.commit();
  }
  const auto props = db.passport_stored_properties(guid);
  CHECK(props.at("Tilstand") == "God");
  CHECK(props.count("Stand") == 0);
  // The old name is free again: a new value under it does not collide.
  db.set_passport_property(guid, "Stand", "Ny");
  CHECK(db.passport_stored_properties(guid).at("Stand") == "Ny");
  CHECK_THROWS_AS(db.rename_passport_field("Gammel", "Tilstand"),
                  core::NameConflictError);
  db.rename_passport_field("Findes ikke", "Andet"); // nothing stored: no-op
  db.delete_survey_part("RX-001");
  CHECK_FALSE(db.survey_part("RX-001").has_value());
  CHECK_FALSE(db.is_passport_linked(guid));
  CHECK_THROWS_AS(db.delete_survey_part("RX-001"), std::out_of_range);
}
```

In the migration test file, replace `roll_back_to_v24` with this version. It also undoes the passport column and restores `samples.part_code`:

```cpp
/// Undo everything v25 adds, so the next read-write open migrates again.
void roll_back_to_v24(const fs::path &path) {
  exec_raw(path, R"sql(
    DROP TABLE templates;
    CREATE TABLE export_templates (
      id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL,
      config TEXT NOT NULL DEFAULT '{}',
      created_at TEXT NOT NULL DEFAULT (datetime('now')),
      updated_at TEXT NOT NULL DEFAULT (datetime('now')));
    DROP INDEX IF EXISTS idx_survey_parts_passport;
    CREATE TABLE survey_parts_v24 (
      code TEXT PRIMARY KEY,
      type_id INTEGER NOT NULL REFERENCES survey_types(id) ON DELETE CASCADE,
      instance_guid TEXT UNIQUE, room_id INTEGER,
      room_name TEXT NOT NULL DEFAULT '', quantity REAL NOT NULL DEFAULT 1,
      starred INTEGER NOT NULL DEFAULT 0, note TEXT NOT NULL DEFAULT '');
    INSERT INTO survey_parts_v24 SELECT code, type_id, instance_guid, room_id,
      room_name, quantity, starred, note FROM survey_parts;
    DROP TABLE survey_parts;
    ALTER TABLE survey_parts_v24 RENAME TO survey_parts;
    CREATE INDEX IF NOT EXISTS idx_survey_parts_type ON survey_parts(type_id);
    ALTER TABLE samples ADD COLUMN part_code TEXT;
    DELETE FROM schema_version WHERE version >= 25;
  )sql");
}

bool column_exists(const fs::path &path, const char *table, const char *col) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  const std::string sql = std::string("PRAGMA table_info(") + table + ");";
  sqlite3_prepare_v2(raw, sql.c_str(), -1, &s, nullptr);
  bool found = false;
  while (sqlite3_step(s) == SQLITE_ROW)
    found = found || std::string(reinterpret_cast<const char *>(
                         sqlite3_column_text(s, 1))) == col;
  sqlite3_finalize(s);
  sqlite3_close(raw);
  return found;
}
```

Add `#include "../../support/survey_fixture.hpp"` and `using namespace reusex::test_support;`, then append:

```cpp
TEST_CASE("MigrationV25_LinksAndSplitsPassports_DropsPartCode",
          "[ProjectDB][migration]") {
  TempDB tmp;
  int64_t sample_id = 0;
  int64_t type_id = 0;
  {
    ProjectDB db(tmp.path);
    make_instance_cloud(db, 3);
    make_passport(db, "guid-shared");
    db.set_passport_property("guid-shared", "width_mm", "600");
    db.set_material_thumbnail("guid-shared", {0xFF, 0xD8, 0xFF}, "image/jpeg");
    make_passport(db, "guid-orphan");
    // `rux create materials` style: one passport on two instances.
    db.set_instance_material("instances", 1, "guid-shared");
    db.set_instance_material("instances", 2, "guid-shared");
    type_id = make_type(db, "Døre");
    make_part(db, "RX-001", type_id, 1u);
    make_part(db, "RX-002", type_id, 2u);
    make_part(db, "RX-003", type_id); // manual
    const auto s = db.add_sample("PCB", "Fuge", {type_id});
    sample_id = s.id;
  }
  roll_back_to_v24(tmp.path);
  exec_raw(tmp.path, "UPDATE samples SET part_code = 'RX-001';");
  ProjectDB db(tmp.path);
  CHECK(db.schema_version() == 25);
  CHECK(db.survey_part("RX-001")->material_guid ==
        std::optional<std::string>("guid-shared")); // first by code keeps it
  const auto copy = db.survey_part("RX-002")->material_guid;
  REQUIRE(copy.has_value());
  CHECK(*copy != "guid-shared");
  CHECK(db.passport_stored_properties(*copy).at("width_mm") == "600");
  CHECK(db.material_thumbnail(*copy).has_value());
  CHECK(db.instance_material_guid("instances", 2) == copy);
  CHECK(db.instance_material_guid("instances", 1) ==
        std::optional<std::string>("guid-shared"));
  CHECK_FALSE(db.survey_part("RX-003")->material_guid.has_value());
  const auto guids = db.list_passport_guids();
  CHECK(std::find(guids.begin(), guids.end(), "guid-orphan") != guids.end());
  CHECK_FALSE(column_exists(tmp.path, "samples", "part_code"));
  CHECK(column_exists(tmp.path, "survey_parts", "passport_guid"));
  REQUIRE(db.samples().size() == 1);
  CHECK(db.samples()[0].id == sample_id);
  CHECK(db.samples()[0].type_ids == std::vector<int64_t>{type_id});
  // A second open changes nothing.
  const auto count = db.list_passport_guids().size();
  ProjectDB again(tmp.path);
  CHECK(again.list_passport_guids().size() == count);
}
```

Add `#include <algorithm>` to that test file.

- [ ] **Step 3: Run the build to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests`
Expected: compile errors. `ProjectDB::Transaction`, `ensure_resource_passport`, `delete_survey_part`, `rename_passport_field` and `is_passport_linked` are undeclared.

- [ ] **Step 4: Declare the API.** In `ProjectDB.hpp`, add this right after `static int latest_schema_version() noexcept;`:

```cpp
  /// One write transaction (BEGIN IMMEDIATE ... COMMIT), rolled back unless
  /// commit() is called. ProjectDB methods that open their own transaction
  /// (add_material_passport, set_survey_part_quantities, add_sample, …)
  /// cannot be called inside one; the resource/template methods can.
  class Transaction {
      public:
    explicit Transaction(ProjectDB &db); ///< throws on a read-only project
    ~Transaction();
    void commit();
    Transaction(const Transaction &) = delete;
    Transaction &operator=(const Transaction &) = delete;

      private:
    ProjectDB *db_;
    bool open_ = true;
  };
```

Change the `material_guid` member comment in `SurveyPartRecord` to:

```cpp
    std::optional<std::string>
        material_guid; // read-only: the part's passport (survey_parts.
                       // passport_guid, schema v25), else its instance's
                       // instance_materials link
```

Add these after `max_survey_part_number()`:

```cpp
  /// Resource storage (schema v25). Call inside a Transaction. Returns the
  /// part's passport guid; on first use creates a bare passport (or adopts
  /// the instance's linked passport when no other part owns it), sets
  /// survey_parts.passport_guid, upserts instance_materials for an
  /// instance-backed part, and makes sure the leksikon property_definitions
  /// exist. @throws std::out_of_range when @p code is unknown.
  std::string ensure_resource_passport(std::string_view code);
  /// @throws std::out_of_range when @p code is unknown.
  void delete_survey_part(std::string_view code);
  /// Call inside a Transaction. Moves the values stored under a user
  /// column's field name to @p new_name; a no-op when nothing is stored.
  /// @throws core::NameConflictError when values already exist under
  ///         @p new_name (they would merge).
  void rename_passport_field(std::string_view old_name,
                             std::string_view new_name);
  /// True when a survey part or an instance link still references @p guid.
  [[nodiscard]] bool is_passport_linked(std::string_view guid) const;
```

- [ ] **Step 5: Implement the migration steps.** In `Impl`, add these after `moveExportTemplates`:

```cpp
  /// A copy of passport @p src under a fresh guid: row, property values and
  /// thumbnail (not the log). Spec §4.2 step 2.
  std::string copyPassport(const std::string &src) {
    const std::string dst = core::generate_guid();
    auto run = [&](const char *sql) {
      sqlite3_stmt *s = prepare_or_throw(db, sql, "migrateToV25");
      StmtGuard guard(s);
      bind_text(s, 1, dst);
      bind_text(s, 2, src);
      if (sqlite3_step(s) != SQLITE_DONE)
        throw std::runtime_error("Migration to v25 failed copying passport " +
                                 src + ": " + sqlite3_errmsg(db));
    };
    run("INSERT INTO material_passports (id, project_id, document_guid, "
        "created_at, revised_at, version_number, version_date) "
        "SELECT ?1, project_id, ?1, created_at, revised_at, version_number, "
        "version_date FROM material_passports WHERE document_guid = ?2;");
    run("INSERT INTO passport_property_values (id, passport_id, property_id, "
        "leksikon_guid, sort_order, value) "
        "SELECT ?1 || '_' || v.leksikon_guid, ?1, v.property_id, "
        "v.leksikon_guid, v.sort_order, v.value "
        "FROM passport_property_values v "
        "JOIN material_passports mp ON mp.id = v.passport_id "
        "WHERE mp.document_guid = ?2;");
    run("INSERT INTO material_thumbnails (material_guid, blob, mime_type) "
        "SELECT ?1, blob, mime_type FROM material_thumbnails "
        "WHERE material_guid = ?2;");
    return dst;
  }

  struct LinkCounts {
    std::size_t linked = 0, split = 0, unlinked = 0;
  };
  /// Spec §4.2 steps 2–3: link each instance-backed part to its instance's
  /// passport; when several parts share one, the first by code keeps it and
  /// every other gets a copy (its instance link repointed). Passports no
  /// part references are left alone.
  LinkCounts linkResourcePassports() {
    LinkCounts c;
    if (tableExists("instances") && tableExists("instance_materials")) {
      struct Row {
        std::string code;
        int64_t cloud_id = 0, instance_id = 0;
        std::string guid;
      };
      std::vector<Row> rows;
      {
        sqlite3_stmt *s = prepare_or_throw(
            db,
            "SELECT p.code, i.cloud_id, i.instance_id, im.material_guid "
            "FROM survey_parts p "
            "JOIN instances i ON i.guid = p.instance_guid "
            "JOIN instance_materials im ON im.cloud_id = i.cloud_id AND "
            "im.instance_id = i.instance_id "
            "WHERE p.passport_guid IS NULL "
            "ORDER BY im.material_guid, p.code;",
            "migrateToV25");
        StmtGuard guard(s);
        while (sqlite3_step(s) == SQLITE_ROW)
          rows.push_back({column_text(s, 0), sqlite3_column_int64(s, 1),
                          sqlite3_column_int64(s, 2), column_text(s, 3)});
      }
      std::set<std::string> claimed;
      {
        sqlite3_stmt *s = prepare_or_throw(
            db,
            "SELECT passport_guid FROM survey_parts WHERE passport_guid IS "
            "NOT NULL;",
            "migrateToV25");
        StmtGuard guard(s);
        while (sqlite3_step(s) == SQLITE_ROW)
          claimed.insert(column_text(s, 0));
      }
      for (const auto &r : rows) {
        std::string guid = r.guid;
        if (claimed.count(guid)) {
          guid = copyPassport(r.guid);
          sqlite3_stmt *s = prepare_or_throw(
              db,
              "UPDATE instance_materials SET material_guid = ? WHERE "
              "cloud_id = ? AND instance_id = ?;",
              "migrateToV25");
          StmtGuard guard(s);
          bind_text(s, 1, guid);
          sqlite3_bind_int64(s, 2, r.cloud_id);
          sqlite3_bind_int64(s, 3, r.instance_id);
          if (sqlite3_step(s) != SQLITE_DONE)
            throw std::runtime_error("Migration to v25 failed repointing " +
                                     r.code + ": " + sqlite3_errmsg(db));
          ++c.split;
        }
        sqlite3_stmt *s = prepare_or_throw(
            db, "UPDATE survey_parts SET passport_guid = ? WHERE code = ?;",
            "migrateToV25");
        StmtGuard guard(s);
        bind_text(s, 1, guid);
        bind_text(s, 2, r.code);
        if (sqlite3_step(s) != SQLITE_DONE)
          throw std::runtime_error("Migration to v25 failed linking " +
                                   r.code + ": " + sqlite3_errmsg(db));
        claimed.insert(guid);
        ++c.linked;
      }
      if (c.split > 0)
        reusex::warn("v25: {} survey part(s) shared a passport with another "
                     "part and now each have their own copy (values and "
                     "thumbnail copied, log not copied)",
                     c.split);
    }
    c.unlinked = static_cast<std::size_t>(
        queryInt("SELECT COUNT(*) FROM material_passports WHERE document_guid "
                 "NOT IN (SELECT passport_guid FROM survey_parts WHERE "
                 "passport_guid IS NOT NULL);"));
    return c;
  }
```

In `migrateToV25`, replace `// (Task 5 adds the resource-passport steps here.)` with:

```cpp
      // §4.2 step 1. ADD COLUMN cannot carry UNIQUE; the index does (NULLs
      // stay distinct, so parts without a passport are fine).
      if (!columnExists("survey_parts", "passport_guid"))
        execOrThrow("ALTER TABLE survey_parts ADD COLUMN passport_guid TEXT "
                    "REFERENCES material_passports(document_guid) "
                    "ON DELETE SET NULL;");
      execOrThrow("CREATE UNIQUE INDEX IF NOT EXISTS idx_survey_parts_passport "
                  "ON survey_parts(passport_guid);");
      const auto links = linkResourcePassports(); // steps 2–3
      // Step 4: On-site is gone. Plain TEXT column: no rebuild needed.
      if (columnExists("samples", "part_code"))
        execOrThrow("ALTER TABLE samples DROP COLUMN part_code;");
```

Then replace the final `reusex::info(...)` with:

```cpp
      reusex::info("Migration to schema version 25 complete: passports "
                   "linked {}, split {}, left unlinked {}; {} seed "
                   "template(s); {} export template(s) moved, {} column(s) "
                   "unmatched",
                   links.linked, links.split, links.unlinked, seeded,
                   moved.moved, moved.unmatched.size());
```

`links` is declared inside the `try` before `seeded`, so it is in scope there.

- [ ] **Step 6: Keep instance links and part passports in step.** In `Impl::setInstanceMaterial`, add this after the upsert `sqlite3_step` succeeds:

```cpp
    // Schema v25: the part backed by this instance follows the new link,
    // unless another part owns that passport — then it keeps its own and
    // the two disagree until someone relinks (warn, STANDARDS §5).
    if (columnExists("survey_parts", "passport_guid")) {
      sqlite3_stmt *up = prepare_or_throw(
          db,
          "UPDATE survey_parts SET passport_guid = ?1 "
          "WHERE instance_guid = (SELECT guid FROM instances WHERE "
          "cloud_id = ?2 AND instance_id = ?3) "
          "AND NOT EXISTS (SELECT 1 FROM survey_parts WHERE "
          "passport_guid = ?1);",
          "set_instance_material");
      StmtGuard up_guard(up);
      bind_text(up, 1, materialGuid);
      sqlite3_bind_int(up, 2, cloudId);
      sqlite3_bind_int(up, 3, instanceId);
      if (sqlite3_step(up) != SQLITE_DONE)
        throw std::runtime_error("Failed to sync the part's passport: " +
                                 std::string(sqlite3_errmsg(db)));
      sqlite3_stmt *chk = prepare_or_throw(
          db,
          "SELECT p.code, p.passport_guid, (SELECT code FROM survey_parts "
          "WHERE passport_guid = ?1) FROM survey_parts p "
          "WHERE p.instance_guid = (SELECT guid FROM instances WHERE "
          "cloud_id = ?2 AND instance_id = ?3) AND p.passport_guid IS NOT "
          "NULL AND p.passport_guid <> ?1;",
          "set_instance_material");
      StmtGuard chk_guard(chk);
      bind_text(chk, 1, materialGuid);
      sqlite3_bind_int(chk, 2, cloudId);
      sqlite3_bind_int(chk, 3, instanceId);
      if (sqlite3_step(chk) == SQLITE_ROW)
        reusex::warn("set_instance_material: passport '{}' belongs to survey "
                     "part '{}'; part '{}' keeps its own passport '{}'",
                     materialGuid, column_text(chk, 2), column_text(chk, 0),
                     column_text(chk, 1));
    }
```

- [ ] **Step 7: Read the part's passport through the new column.** Replace `constexpr const char *kSurveyPartSelect = …;` with the function below. In `survey_parts()` and `survey_part()`, build the SQL from `survey_part_select(impl_->columnExists("survey_parts", "passport_guid"))`, so a read-only open of a pre-v25 project still works:

```cpp
std::string survey_part_select(bool has_passport_column) {
  return std::string("SELECT p.code, p.type_id, pc.name, i.instance_id, "
                     "p.room_id, p.room_name, p.quantity, p.starred, p.note, ") +
         (has_passport_column ? "COALESCE(p.passport_guid, im.material_guid)"
                              : "im.material_guid") +
         ", p.instance_guid FROM survey_parts p "
         "LEFT JOIN instances i ON i.guid = p.instance_guid "
         "LEFT JOIN point_clouds pc ON pc.id = i.cloud_id "
         "LEFT JOIN instance_materials im ON im.cloud_id = i.cloud_id AND "
         "im.instance_id = i.instance_id ";
}
```

Keep the existing doc comment above it, and add one line to it: `material_guid is the part's own passport (v25) and falls back to its instance's link.`

- [ ] **Step 8: Implement `Transaction` and the storage methods.** Add these to `ProjectDB.cpp` after `max_survey_part_number`:

```cpp
ProjectDB::Transaction::Transaction(ProjectDB &db) : db_(&db) {
  db_->impl_->checkWritable();
  db_->impl_->execOrThrow("BEGIN IMMEDIATE;");
}

ProjectDB::Transaction::~Transaction() {
  if (!open_)
    return;
  // Non-throwing on purpose; a failed rollback leaves the transaction open
  // on this connection, so at least say so (STANDARDS §5).
  if (sqlite3_exec(db_->impl_->db, "ROLLBACK;", nullptr, nullptr, nullptr) !=
      SQLITE_OK)
    reusex::warn("ProjectDB::Transaction: ROLLBACK failed: {}",
                 sqlite3_errmsg(db_->impl_->db));
}

void ProjectDB::Transaction::commit() {
  db_->impl_->execOrThrow("COMMIT;");
  open_ = false;
}

std::string ProjectDB::ensure_resource_passport(std::string_view code) {
  impl_->checkWritable();
  const auto part = survey_part(code);
  if (!part)
    throw std::out_of_range("no survey part '" + std::string(code) + "'");
  {
    sqlite3_stmt *s = prepare_or_throw(
        impl_->db, "SELECT passport_guid FROM survey_parts WHERE code = ?;",
        "ensure_resource_passport");
    StmtGuard guard(s);
    bind_text(s, 1, code);
    if (sqlite3_step(s) == SQLITE_ROW && sqlite3_column_type(s, 0) != SQLITE_NULL)
      return column_text(s, 0);
  }
  // set_passport_property files an unknown name_en under an auto-created
  // "custom:" definition; leksikon names must find their real one.
  impl_->ensureAllPropertyDefinitions();
  std::string guid;
  if (part->material_guid && !is_passport_linked_by_part(*part->material_guid)) {
    guid = *part->material_guid; // adopt the instance's unowned passport
  } else {
    guid = core::generate_guid();
    sqlite3_stmt *s = prepare_or_throw(
        impl_->db,
        "INSERT INTO material_passports (id, document_guid, created_at, "
        "version_number) VALUES (?1, ?1, "
        "strftime('%Y-%m-%dT%H:%M:%SZ','now'), '0.1.0');",
        "ensure_resource_passport");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("ensure_resource_passport: " +
                               std::string(sqlite3_errmsg(impl_->db)));
  }
  {
    sqlite3_stmt *s = prepare_or_throw(
        impl_->db, "UPDATE survey_parts SET passport_guid = ? WHERE code = ?;",
        "ensure_resource_passport");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    bind_text(s, 2, code);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("ensure_resource_passport: " +
                               std::string(sqlite3_errmsg(impl_->db)));
  }
  if (part->instance_guid) {
    // The pipeline's link (Viewport, create materials, MaterialEPAS
    // export) follows in the same transaction (spec §4.1).
    sqlite3_stmt *s = prepare_or_throw(
        impl_->db,
        "INSERT INTO instance_materials (cloud_id, instance_id, material_guid) "
        "SELECT cloud_id, instance_id, ?1 FROM instances WHERE guid = ?2 "
        "ON CONFLICT(cloud_id, instance_id) DO UPDATE SET "
        "material_guid = excluded.material_guid;",
        "ensure_resource_passport");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    bind_text(s, 2, *part->instance_guid);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("ensure_resource_passport: " +
                               std::string(sqlite3_errmsg(impl_->db)));
  }
  return guid;
}

void ProjectDB::delete_survey_part(std::string_view code) {
  impl_->checkWritable();
  sqlite3_stmt *s = prepare_or_throw(
      impl_->db, "DELETE FROM survey_parts WHERE code = ?;",
      "delete_survey_part");
  StmtGuard guard(s);
  bind_text(s, 1, code);
  if (sqlite3_step(s) != SQLITE_DONE)
    throw std::runtime_error("delete_survey_part: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  if (sqlite3_changes(impl_->db) == 0)
    throw std::out_of_range("no survey part '" + std::string(code) + "'");
}

void ProjectDB::rename_passport_field(std::string_view old_name,
                                      std::string_view new_name) {
  impl_->checkWritable();
  if (old_name == new_name)
    return;
  const std::string old_id = "custom:" + std::string(old_name);
  const std::string new_id = "custom:" + std::string(new_name);
  auto exists = [&](const char *sql, std::string_view value) {
    sqlite3_stmt *s = prepare_or_throw(impl_->db, sql, "rename_passport_field");
    StmtGuard guard(s);
    bind_text(s, 1, value);
    return sqlite3_step(s) == SQLITE_ROW;
  };
  if (!exists("SELECT 1 FROM property_definitions WHERE id = ?;", old_id))
    return; // nothing was ever stored under the old name
  if (exists("SELECT 1 FROM property_definitions WHERE name_en = ?;", new_name))
    throw core::NameConflictError(
        "values are already stored under the field name '" +
        std::string(new_name) + "'; renaming onto it would merge them");
  auto run = [&](const char *sql) {
    sqlite3_stmt *s = prepare_or_throw(impl_->db, sql, "rename_passport_field");
    StmtGuard guard(s);
    bind_text(s, 1, new_id);
    bind_text(s, 2, old_id);
    bind_text(s, 3, new_name);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("rename_passport_field: " +
                               std::string(sqlite3_errmsg(impl_->db)));
  };
  // New definition first (values reference it), then the values, then the
  // old definition: foreign keys stay satisfied at every statement.
  run("INSERT INTO property_definitions (id, leksikon_guid, name_en, "
      "category, data_type) SELECT ?1, ?1, ?3, category, data_type "
      "FROM property_definitions WHERE id = ?2;");
  run("UPDATE passport_property_values SET property_id = ?1, "
      "leksikon_guid = ?1, id = passport_id || '_' || ?1 "
      "WHERE property_id = ?2 AND ?3 IS NOT NULL;");
  run("DELETE FROM property_definitions WHERE id = ?2 AND ?1 IS NOT NULL "
      "AND ?3 IS NOT NULL;");
}

bool ProjectDB::is_passport_linked(std::string_view guid) const {
  sqlite3_stmt *s = prepare_or_throw(
      impl_->db,
      "SELECT 1 FROM instance_materials WHERE material_guid = ?1 "
      "UNION ALL SELECT 1 FROM survey_parts WHERE passport_guid = ?1 LIMIT 1;",
      "is_passport_linked");
  StmtGuard guard(s);
  bind_text(s, 1, guid);
  return sqlite3_step(s) == SQLITE_ROW;
}
```

The `?3 IS NOT NULL` / `?1 IS NOT NULL` terms are there only so that every statement uses all three bound parameters. Binding an index a statement does not contain returns `SQLITE_RANGE`. This code ignores that return value, but don't rely on it.

`ensure_resource_passport` uses a private helper. Declare it in the `private:` section of `ProjectDB.hpp` as `bool is_passport_linked_by_part(std::string_view guid) const;` and implement it:

```cpp
bool ProjectDB::is_passport_linked_by_part(std::string_view guid) const {
  sqlite3_stmt *s = prepare_or_throw(
      impl_->db, "SELECT 1 FROM survey_parts WHERE passport_guid = ?;",
      "is_passport_linked_by_part");
  StmtGuard guard(s);
  bind_text(s, 1, guid);
  return sqlite3_step(s) == SQLITE_ROW;
}
```

- [ ] **Step 9: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests ruxd && cd build && ctest --output-on-failure --parallel $(nproc) -R 'ResourceStore_|MigrationV25_|ProjectDB|Survey|Samples_|InstanceMaterial'`
Expected: all pass, including the existing `SurveyParts_MaterialGuid_JoinsFromLinkedInstance`, which goes through the `COALESCE` fallback.

- [ ] **Step 10: Commit.**

```bash
git add libs/reusex tests/support/survey_fixture.hpp tests/unit/core/test_project_db_resources.cpp tests/unit/core/test_project_db_migration_v25.cpp
git commit -m "feat(core): v25 passport per survey part, transactions and resource storage

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 6: Resources service — read values, route writes, add/delete, user columns

**Files:**
- Create: `libs/reusex/include/core/resources.hpp`, `libs/reusex/src/core/resources.cpp`
- Test: `tests/unit/core/test_resources.cpp`

**Interfaces:**
- Consumes: Task 1 (catalogue, `normalise_value`, `format_number`, `parse_number`, `KeyValueError`, `leksikon_fields`). Task 5 (`Transaction`, `ensure_resource_passport`, `delete_survey_part`, `rename_passport_field`, `is_passport_linked`). `environment_statuses` (survey_service).
- Produces (exact):

```cpp
namespace reusex::core {
struct ResourceValue { std::string key; std::optional<std::string> value; };
struct Resource { std::string code; int64_t type_id = 0; bool manual = false; std::vector<ResourceValue> values; };
std::optional<std::string> value_of(const Resource &r, std::string_view key);
std::vector<Resource> list_resources(const ProjectDB &db, const std::optional<std::vector<std::string>> &keys = std::nullopt);
Resource resource(const ProjectDB &db, std::string_view code, const std::optional<std::vector<std::string>> &keys = std::nullopt);
struct ResourceWrite { std::string key; std::optional<std::string> value; };
struct ResourcePatchResult { Resource resource; std::vector<Resource> siblings; };
ResourcePatchResult patch_resource(ProjectDB &db, std::string_view code, const std::vector<ResourceWrite> &writes, const std::optional<std::vector<std::string>> &keys = std::nullopt);
inline constexpr std::string_view kDesignationField = "designation";
Resource create_resource(ProjectDB &db, int64_t type_id, const std::optional<std::string> &name = std::nullopt);
class ResourceConflictError : public std::runtime_error { public: using std::runtime_error::runtime_error; };
void delete_resource(ProjectDB &db, std::string_view code);
struct ColumnPatch { std::optional<std::string> name, type; std::optional<std::vector<std::string>> options; std::optional<int> sort_order, width; };
ProjectDB::PropertyDefinition create_column(ProjectDB &db, ProjectDB::PropertyDefinition def);
ProjectDB::PropertyDefinition update_column(ProjectDB &db, const std::string &id, const ColumnPatch &patch);
}
```

- [ ] **Step 1: Write the failing test.** Create `tests/unit/core/test_resources.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Resources (spec §4): values by key id, writes routed by scope (type,
// part, passport), lazy passports, add/delete, and user columns whose
// rename carries their stored values.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resource_keys.hpp>
#include <core/resources.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <string>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::core;
using namespace reusex::test_support;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_resources") {}
};
std::string lex(const char *field) {
  for (const auto &f : leksikon_fields())
    if (f.field_name == field)
      return "lex:" + f.guid;
  FAIL("no leksikon field " << field);
  return {};
}
} // namespace

TEST_CASE("Resources_List_NoTemplate_BuiltinsPlusStored", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-002", t, std::nullopt, 2.5);
  make_part(db, "RX-001", t);
  const auto list = list_resources(db);
  REQUIRE(list.size() == 2);
  CHECK(list[0].code == "RX-001");
  CHECK(list[0].manual);
  CHECK(list[0].values.size() == builtin_keys().size());
  CHECK(value_of(list[1], "sys:quantity") == "2.5");
  CHECK(value_of(list[1], "sys:name") == "Døre");
  CHECK(value_of(list[1], "sys:treatment") == "genbrug");
  CHECK(value_of(list[1], "sys:environment") == "ren_screening");
  CHECK(value_of(list[1], "sys:starred") == "false");
  CHECK_FALSE(value_of(list[1], "sys:mass_t").has_value());
  patch_resource(db, "RX-001", {{lex("width_mm"), "600"}});
  const auto r = resource(db, "RX-001");
  CHECK(r.values.size() == builtin_keys().size() + 1);
  CHECK(value_of(r, lex("width_mm")) == "600");
}

TEST_CASE("Resources_List_WithKeys_OrderAndNullForMissing", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  const auto r =
      resource(db, "RX-001", std::vector<std::string>{lex("width_mm"),
                                                      "sys:name", "col:gone"});
  REQUIRE(r.values.size() == 3);
  CHECK(r.values[0].key == lex("width_mm"));
  CHECK_FALSE(r.values[0].value.has_value());
  CHECK(r.values[1].value == "Døre");
  CHECK_FALSE(r.values[2].value.has_value());
  CHECK_THROWS_AS(resource(db, "RX-404"), std::out_of_range);
}

TEST_CASE("Resources_Patch_RoutesByScope_ReturnsSiblings", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  make_part(db, "RX-002", t);
  const auto res = patch_resource(
      db, "RX-001",
      {{"sys:eak", "17.02.01"}, {"sys:mass_t", "0,9"}, {"sys:note", "ved trappen"},
       {"sys:starred", "true"}, {"sys:quantity", "4"}});
  CHECK(db.survey_type(t)->eak_code == "17.02.01");
  CHECK(db.survey_type(t)->mass_t == 0.9);
  CHECK(db.survey_part("RX-001")->note == "ved trappen");
  CHECK(db.survey_part("RX-001")->starred);
  CHECK(db.survey_part("RX-001")->quantity == 4.0);
  CHECK(db.survey_part("RX-002")->note.empty());
  REQUIRE(res.siblings.size() == 1);
  CHECK(res.siblings[0].code == "RX-002");
  CHECK(value_of(res.siblings[0], "sys:eak") == "17.02.01");
  // Part-only writes have no siblings to refresh.
  CHECK(patch_resource(db, "RX-001", {{"sys:room", "Kælder"}}).siblings.empty());
  // Clearing: mass_t to NULL, eak to "".
  patch_resource(db, "RX-001", {{"sys:mass_t", std::nullopt}, {"sys:eak", ""}});
  CHECK_FALSE(db.survey_type(t)->mass_t.has_value());
  CHECK(db.survey_type(t)->eak_code.empty());
}

TEST_CASE("Resources_Patch_LazyPassport_InstanceLinkFollows", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  make_part(db, "RX-002", t);
  // Clearing an absent value succeeds and creates no passport.
  patch_resource(db, "RX-002", {{lex("width_mm"), std::nullopt}});
  CHECK_FALSE(db.survey_part("RX-002")->material_guid.has_value());
  patch_resource(db, "RX-001", {{lex("width_mm"), "600"}});
  const auto guid = db.survey_part("RX-001")->material_guid;
  REQUIRE(guid.has_value());
  CHECK(db.instance_material_guid("instances", 1) == guid);
  patch_resource(db, "RX-001", {{lex("width_mm"), std::nullopt}});
  CHECK(db.passport_stored_properties(*guid).count("width_mm") == 0);
}

TEST_CASE("Resources_Patch_ValidatesAllBeforeWriting", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  for (const std::vector<ResourceWrite> &w :
       std::vector<std::vector<ResourceWrite>>{
           {{"sys:note", "x"}, {"sys:environment", "forurenet"}},
           {{"sys:note", "x"}, {"sys:nope", "1"}},
           {{"sys:note", "x"}, {"sys:treatment", "smid ud"}},
           {{"sys:note", "x"}, {"sys:quantity", "mange"}}}) {
    CHECK_THROWS_AS(patch_resource(db, "RX-001", w), KeyValueError);
  }
  CHECK(db.survey_part("RX-001")->note.empty());
  CHECK_THROWS_AS(patch_resource(db, "RX-404", {{"sys:note", "x"}}),
                  std::out_of_range);
}

TEST_CASE("Resources_CreateAndDelete", "[resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  const auto r = create_resource(db, t, std::string("Branddør"));
  CHECK(r.code == "RX-002");
  CHECK(r.manual);
  CHECK(value_of(r, lex("designation")) == "Branddør");
  const auto guid = db.survey_part("RX-002")->material_guid;
  REQUIRE(guid.has_value());
  CHECK(create_resource(db, t).code == "RX-003");
  CHECK_THROWS_AS(create_resource(db, 999), std::out_of_range);
  CHECK_THROWS_AS(create_resource(db, t, std::string("")),
                  std::invalid_argument);
  delete_resource(db, "RX-002");
  CHECK_FALSE(db.survey_part("RX-002").has_value());
  const auto guids = db.list_passport_guids();
  CHECK(std::find(guids.begin(), guids.end(), *guid) == guids.end());
  CHECK_THROWS_AS(delete_resource(db, "RX-001"), ResourceConflictError);
  CHECK_THROWS_AS(delete_resource(db, "RX-404"), std::out_of_range);
}

TEST_CASE("Resources_RenameColumn_CarriesValues", "[resources][columns]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  ProjectDB::PropertyDefinition def;
  def.name = "Stand";
  def.type = "select";
  def.options = {"God", "Dårlig"};
  const auto col = create_column(db, def);
  CHECK_FALSE(col.id.empty());
  patch_resource(db, "RX-001", {{"col:" + col.id, "God"}});
  ColumnPatch p;
  p.name = "Tilstand";
  CHECK(update_column(db, col.id, p).name == "Tilstand");
  CHECK(value_of(resource(db, "RX-001"), "col:" + col.id) == "God");
  ProjectDB::PropertyDefinition dup;
  dup.name = "Tilstand";
  dup.type = "text";
  CHECK_THROWS_AS(create_column(db, dup), reusex::core::NameConflictError);
  ProjectDB::PropertyDefinition lexname;
  lexname.name = "width_mm";
  lexname.type = "text";
  CHECK_THROWS_AS(create_column(db, lexname), reusex::core::NameConflictError);
  CHECK_THROWS_AS(update_column(db, "nope", p), std::out_of_range);
}

TEST_CASE("Resources_RenameColumn_RefusesNameWithStoredValues",
          "[resources][columns]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  ProjectDB::PropertyDefinition a;
  a.name = "Gammel";
  a.type = "text";
  const auto old_col = create_column(db, a);
  patch_resource(db, "RX-001", {{"col:" + old_col.id, "rest"}});
  db.delete_property_definition(old_col.id); // values stay, column gone
  ProjectDB::PropertyDefinition b;
  b.name = "Ny";
  b.type = "text";
  const auto new_col = create_column(db, b);
  patch_resource(db, "RX-001", {{"col:" + new_col.id, "frisk"}});
  ColumnPatch p;
  p.name = "Gammel";
  CHECK_THROWS_AS(update_column(db, new_col.id, p),
                  reusex::core::NameConflictError);
  CHECK(value_of(resource(db, "RX-001"), "col:" + new_col.id) == "frisk");
  p.name = "Ny"; // same name: no-op
  CHECK(update_column(db, new_col.id, p).name == "Ny");
  p.name = "Ny 2";
  update_column(db, new_col.id, p);
  p.name = "Ny"; // and back again
  update_column(db, new_col.id, p);
  CHECK(value_of(resource(db, "RX-001"), "col:" + new_col.id) == "frisk");
}
```

Add `#include <algorithm>` (for `std::find` in `Resources_CreateAndDelete`).

- [ ] **Step 2: Run the build to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error `core/resources.hpp: No such file or directory`.

- [ ] **Step 3: Write the header.** Create `libs/reusex/include/core/resources.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Resources (docs/superpowers/specs/2026-10-02-resources-templates-ia-design.md
/// §4): a resource is one survey part; its values are addressed by key id
/// (core/resource_keys.hpp) and routed by scope — the type, the part, or
/// the part's own passport, created on the first passport write.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/resource_keys.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

struct ResourceValue {
  std::string key;
  std::optional<std::string> value; ///< nullopt: not set
};
struct Resource {
  std::string code;
  int64_t type_id = 0;
  bool manual = false; ///< added by hand: no instance behind it
  std::vector<ResourceValue> values;
};
/// The value stored under @p key in @p r, or nullopt (absent or unset).
std::optional<std::string> value_of(const Resource &r, std::string_view key);

/// One per survey part, by code. With @p keys, `values` holds exactly those
/// keys in that order (null when unset or unknown); without, every
/// built-in key plus each catalogue key the part's passport stores, in
/// catalogue order. Passport fields with no catalogue key are skipped.
std::vector<Resource>
list_resources(const ProjectDB &db,
               const std::optional<std::vector<std::string>> &keys =
                   std::nullopt);
/// @throws std::out_of_range when @p code is unknown.
Resource resource(const ProjectDB &db, std::string_view code,
                  const std::optional<std::vector<std::string>> &keys =
                      std::nullopt);

struct ResourceWrite {
  std::string key;
  std::optional<std::string> value; ///< nullopt clears
};
struct ResourcePatchResult {
  Resource resource;
  /// The type's other parts, when a type-scoped key was written (they
  /// changed too); empty otherwise.
  std::vector<Resource> siblings;
};
/// Validate every write (normalise_value), then apply them in one
/// transaction: type-scoped keys to the survey type, part-scoped built-ins
/// to the part, leksikon/column keys to the part's passport (created on
/// the first non-null write). Clearing an absent value succeeds.
/// @throws KeyValueError (nothing written), std::out_of_range (no part).
ResourcePatchResult
patch_resource(ProjectDB &db, std::string_view code,
               const std::vector<ResourceWrite> &writes,
               const std::optional<std::vector<std::string>> &keys =
                   std::nullopt);

/// The leksikon field a hand-added resource's `name` is stored under.
inline constexpr std::string_view kDesignationField = "designation";
/// A manual part with the next RX code; @p name becomes its designation.
/// @throws std::out_of_range (no type), std::invalid_argument (empty name).
Resource create_resource(ProjectDB &db, int64_t type_id,
                         const std::optional<std::string> &name =
                             std::nullopt);

/// Deleting a resource the scan produced. The GUI answers 409.
class ResourceConflictError : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};
/// Delete a manual part and its passport (unless something else links it).
/// @throws ResourceConflictError for an instance-backed part,
///         std::out_of_range for an unknown code.
void delete_resource(ProjectDB &db, std::string_view code);

/// Sparse update of a user column definition.
struct ColumnPatch {
  std::optional<std::string> name, type;
  std::optional<std::vector<std::string>> options;
  std::optional<int> sort_order, width;
};
/// Add a user column (def.id is ignored and returned filled in).
/// @throws NameConflictError when a user column or a leksikon field already
///         has that name, std::invalid_argument for an empty name.
ProjectDB::PropertyDefinition create_column(ProjectDB &db,
                                            ProjectDB::PropertyDefinition def);
/// A rename moves the stored values in the same transaction.
/// @throws std::out_of_range (no column), NameConflictError (name taken, or
///         values already stored under it), std::invalid_argument (empty).
ProjectDB::PropertyDefinition update_column(ProjectDB &db,
                                            const std::string &id,
                                            const ColumnPatch &patch);

} // namespace reusex::core
```

- [ ] **Step 4: Write the implementation.** Create `libs/reusex/src/core/resources.cpp`:

```cpp
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

std::optional<std::string>
builtin_value(const ResourceKey &k, const ProjectDB::SurveyPartRecord &p,
              const ProjectDB::SurveyTypeRecord &t, EnvironmentStatus e) {
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
    throw ResourceConflictError("resource '" + std::string(code) +
                                "' comes from the scan (instance " +
                                *part->instance_guid +
                                ") and cannot be deleted");
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

ProjectDB::PropertyDefinition update_column(ProjectDB &db,
                                            const std::string &id,
                                            const ColumnPatch &p) {
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
```

`delete_material_passport` binds with `SQLITE_STATIC` and runs no `BEGIN`, so it is safe inside the transaction. Clearing a value on an instance-backed part that only has an instance link (no `passport_guid` yet) deletes from that linked passport. After v25 that passport is never shared between parts.

- [ ] **Step 5: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests && cd build && ctest --output-on-failure --parallel $(nproc) -R 'Resources_'`
Expected: 8 tests pass.

- [ ] **Step 6: Commit.**

```bash
git add libs/reusex/include/core/resources.hpp libs/reusex/src/core/resources.cpp tests/unit/core/test_resources.cpp
git commit -m "feat(core): resources service — values by key, scope routing, columns

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 7: Template service (create, edit, delete, duplicate, restore seeds, views)

**Files:**
- Modify: `libs/reusex/include/core/resource_templates.hpp` (append the DB section), `libs/reusex/src/core/resource_templates.cpp`
- Test: `tests/unit/core/test_resource_template_service.cpp` (new)

**Interfaces:**
- Consumes: Task 2 (pure functions). Task 4 (`resource_templates`, `add/update/delete_resource_template`, `NameConflictError`).
- Produces (appended to `resource_templates.hpp`, which now also includes `"reusex/core/ProjectDB.hpp"` through `resource_keys.hpp`):

```cpp
struct TemplateView { ProjectDB::ResourceTemplateRecord record; std::vector<TemplateMember> members; CsvOptions csv; ResolvedTemplate resolved; };
std::vector<TemplateView> template_views(const ProjectDB &db);
TemplateView template_view(const ProjectDB &db, int64_t id);                 // out_of_range
struct TemplateInput { std::optional<std::string> name; std::optional<nlohmann::json> members, csv; };
TemplateView create_template(ProjectDB &db, const TemplateInput &input);    // invalid_argument, NameConflictError
TemplateView update_template(ProjectDB &db, int64_t id, const TemplateInput &input); // + out_of_range
void delete_template(ProjectDB &db, int64_t id);                            // out_of_range
TemplateView duplicate_template(ProjectDB &db, int64_t id);                 // out_of_range
std::vector<TemplateView> restore_seed_templates(ProjectDB &db);            // the ones inserted
```

- [ ] **Step 1: Write the failing test.** Create `tests/unit/core/test_resource_template_service.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// The template service (spec §5.5): CRUD with unique names, duplicate as
// "(kopi)", restore seeds, and views resolved against the live catalogue.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resource_templates.hpp>
#include <core/survey.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::core;
using json = nlohmann::json;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_template_service") {}
};
TemplateInput named(const char *name) {
  TemplateInput in;
  in.name = name;
  return in;
}
} // namespace

TEST_CASE("TemplateService_Create_ValidatesAndResolves", "[templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  auto in = named("Mit valg");
  in.members = json::parse(R"([{"key":"sys:note"},{"key":"col:gone"}])");
  in.csv = json::parse(R"({"delimiter":","})");
  const auto v = create_template(db, in);
  CHECK(v.record.name == "Mit valg");
  CHECK_FALSE(v.record.seed.has_value());
  CHECK(v.resolved.keys == std::vector<std::string>{"sys:note"});
  CHECK(v.resolved.missing.size() == 1);
  CHECK(v.csv.delimiter == ",");
  CHECK_THROWS_AS(create_template(db, named("Mit valg")), NameConflictError);
  CHECK_THROWS_AS(create_template(db, TemplateInput{}), std::invalid_argument);
  CHECK_THROWS_AS(create_template(db, named("")), std::invalid_argument);
  auto bad = named("Andet");
  bad.members = json::parse(R"([{"key":1}])");
  CHECK_THROWS_AS(create_template(db, bad), std::invalid_argument);
  bad.members.reset();
  bad.csv = json::parse(R"({"header":"kolonne"})");
  CHECK_THROWS_AS(create_template(db, bad), std::invalid_argument);
  CHECK(template_views(db).size() == 3);
}

TEST_CASE("TemplateService_Update_Delete", "[templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = create_template(db, named("A")).record.id;
  TemplateInput in;
  in.members = json::parse(R"([{"category":"Kortlægning"}])");
  const auto v = update_template(db, id, in);
  CHECK(v.record.name == "A");
  CHECK(v.resolved.keys.size() == 11);
  in.members.reset();
  in.name = "Hurtig genbrugsscreening";
  CHECK_THROWS_AS(update_template(db, id, in), NameConflictError);
  CHECK_THROWS_AS(update_template(db, 999, named("x")), std::out_of_range);
  delete_template(db, id);
  CHECK_THROWS_AS(delete_template(db, id), std::out_of_range);
  CHECK_THROWS_AS(template_view(db, id), std::out_of_range);
}

TEST_CASE("TemplateService_Duplicate_NumbersTheCopy", "[templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto seed = template_views(db)[1];
  const auto a = duplicate_template(db, seed.record.id);
  CHECK(a.record.name == "Hurtig genbrugsscreening (kopi)");
  CHECK_FALSE(a.record.seed.has_value());
  CHECK(a.record.members_json == seed.record.members_json);
  CHECK(duplicate_template(db, seed.record.id).record.name ==
        "Hurtig genbrugsscreening (kopi 2)");
  CHECK_THROWS_AS(duplicate_template(db, 999), std::out_of_range);
}

TEST_CASE("TemplateService_RestoreSeeds_SuffixOnNameClash_Idempotent",
          "[templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  for (const auto &v : template_views(db))
    delete_template(db, v.record.id);
  create_template(db, named("Materialepas (fuld)")); // user's own
  const auto restored = restore_seed_templates(db);
  REQUIRE(restored.size() == 2);
  CHECK(restored[0].record.name == "Materialepas (fuld) (standard)");
  CHECK(restored[0].record.seed == std::optional<std::string>("materialepas"));
  CHECK(restored[1].record.name == "Hurtig genbrugsscreening");
  CHECK(restore_seed_templates(db).empty());
  CHECK(template_views(db).size() == 3);
}
```

- [ ] **Step 2: Run the build to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests`
Expected: compile errors. `TemplateInput`, `create_template` and the other service names are undeclared.

- [ ] **Step 3: Append to the header.** Add this before the closing `} // namespace reusex::core` in `resource_templates.hpp`, and add `#include <cstdint>` and `#include <optional>` at the top:

```cpp
// ---- Template service over ProjectDB --------------------------------------

/// One template as the API shows it: its row, parsed members and CSV
/// options, and its resolution against the live key catalogue.
struct TemplateView {
  ProjectDB::ResourceTemplateRecord record;
  std::vector<TemplateMember> members;
  CsvOptions csv;
  ResolvedTemplate resolved;
};
/// Every template by id. Logs warn once per template with missing members.
std::vector<TemplateView> template_views(const ProjectDB &db);
/// @throws std::out_of_range when @p id is unknown.
TemplateView template_view(const ProjectDB &db, int64_t id);

/// A create or sparse edit; absent fields are left alone.
struct TemplateInput {
  std::optional<std::string> name;
  std::optional<nlohmann::json> members, csv;
};
/// @throws std::invalid_argument (no/empty name, bad members or csv),
///         NameConflictError (name taken).
TemplateView create_template(ProjectDB &db, const TemplateInput &input);
/// @throws as create_template, plus std::out_of_range for an unknown id.
TemplateView update_template(ProjectDB &db, int64_t id,
                             const TemplateInput &input);
/// @throws std::out_of_range when @p id is unknown.
void delete_template(ProjectDB &db, int64_t id);
/// Copy as "<name> (kopi)", "<name> (kopi 2)", … (never a seed).
/// @throws std::out_of_range when @p id is unknown.
TemplateView duplicate_template(ProjectDB &db, int64_t id);
/// Insert every seed whose tag no template carries ("Gendan
/// standardskabeloner"); a seed whose name is taken gets " (standard)".
/// Returns the inserted ones.
std::vector<TemplateView> restore_seed_templates(ProjectDB &db);
```

- [ ] **Step 4: Append the implementation** to `resource_templates.cpp`, before its closing namespace brace:

```cpp
namespace {
std::vector<std::string> template_names(const ProjectDB &db) {
  std::vector<std::string> out;
  for (const auto &t : db.resource_templates())
    out.push_back(t.name);
  return out;
}

TemplateView make_view(const ProjectDB::ResourceTemplateRecord &rec,
                       const std::vector<ResourceKey> &catalogue) {
  TemplateView v{rec, read_members(rec.members_json, rec.name),
                 read_csv_options(rec.csv_json, rec.name), {}};
  v.resolved = resolve_template(v.members, catalogue);
  if (!v.resolved.missing.empty()) {
    std::string refs;
    for (const auto &m : v.resolved.missing)
      refs += (refs.empty() ? "" : ", ") + m.ref;
    reusex::warn("template '{}' (id {}): {} member(s) no longer resolve and "
                 "are skipped: {}",
                 rec.name, rec.id, v.resolved.missing.size(), refs);
  }
  return v;
}
} // namespace

std::vector<TemplateView> template_views(const ProjectDB &db) {
  const auto catalogue = key_catalogue(db);
  std::vector<TemplateView> out;
  for (const auto &rec : db.resource_templates())
    out.push_back(make_view(rec, catalogue));
  return out;
}

TemplateView template_view(const ProjectDB &db, int64_t id) {
  const auto rec = db.resource_template(id);
  if (!rec)
    throw std::out_of_range("no template " + std::to_string(id));
  return make_view(*rec, key_catalogue(db));
}

TemplateView create_template(ProjectDB &db, const TemplateInput &in) {
  if (!in.name || in.name->empty())
    throw std::invalid_argument("'name' is required and must be non-empty");
  ProjectDB::ResourceTemplateRecord rec;
  rec.name = *in.name;
  if (in.members)
    rec.members_json = members_json(parse_members(*in.members)).dump();
  if (in.csv)
    rec.csv_json = csv_options_json(parse_csv_options(*in.csv)).dump();
  return template_view(db, db.add_resource_template(rec).id);
}

TemplateView update_template(ProjectDB &db, int64_t id,
                             const TemplateInput &in) {
  if (!db.resource_template(id))
    throw std::out_of_range("no template " + std::to_string(id));
  ProjectDB::ResourceTemplatePatch p;
  if (in.name) {
    if (in.name->empty())
      throw std::invalid_argument("'name' must be non-empty");
    p.name = *in.name;
  }
  if (in.members)
    p.members_json = members_json(parse_members(*in.members)).dump();
  if (in.csv)
    p.csv_json = csv_options_json(parse_csv_options(*in.csv)).dump();
  db.update_resource_template(id, p);
  return template_view(db, id);
}

void delete_template(ProjectDB &db, int64_t id) {
  if (!db.delete_resource_template(id))
    throw std::out_of_range("no template " + std::to_string(id));
}

TemplateView duplicate_template(ProjectDB &db, int64_t id) {
  const auto src = db.resource_template(id);
  if (!src)
    throw std::out_of_range("no template " + std::to_string(id));
  ProjectDB::ResourceTemplateRecord copy;
  copy.name = unique_name(src->name, "kopi", template_names(db));
  copy.members_json = src->members_json;
  copy.csv_json = src->csv_json;
  return template_view(db, db.add_resource_template(copy).id);
}

std::vector<TemplateView> restore_seed_templates(ProjectDB &db) {
  std::vector<TemplateView> out;
  const auto existing = db.resource_templates();
  auto names = template_names(db);
  for (const auto &seed : seed_templates()) {
    const bool present =
        std::any_of(existing.begin(), existing.end(),
                    [&](const auto &r) { return r.seed == seed.tag; });
    if (present)
      continue;
    ProjectDB::ResourceTemplateRecord rec;
    rec.name = std::find(names.begin(), names.end(), seed.name) == names.end()
                   ? seed.name
                   : unique_name(seed.name, "standard", names);
    rec.members_json = members_json(seed.members).dump();
    rec.seed = seed.tag;
    names.push_back(rec.name);
    out.push_back(template_view(db, db.add_resource_template(rec).id));
  }
  return out;
}
```

- [ ] **Step 5: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests && cd build && ctest --output-on-failure --parallel $(nproc) -R 'TemplateService_|ResourceTemplates_'`
Expected: all pass.

- [ ] **Step 6: Commit.**

```bash
git add libs/reusex/include/core/resource_templates.hpp libs/reusex/src/core/resource_templates.cpp tests/unit/core/test_resource_template_service.cpp
git commit -m "feat(core): template service — CRUD, duplicate, restore seeds, views

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 8: Resource export — CSV builder and PDF table chunking

**Files:**
- Create: `libs/reusex/include/core/resource_export.hpp`, `libs/reusex/src/core/resource_export.cpp`
- Test: `tests/unit/core/test_resource_export.cpp`

**Interfaces:**
- Consumes: Task 1, Task 6 (`Resource`, `value_of`, `list_resources`), Task 7 (`template_view`, `CsvOptions`).
- Produces (exact):

```cpp
namespace reusex::core {
std::string display_value(const ResourceKey &key, const std::optional<std::string> &value);
std::string csv_cell(std::string_view value, std::string_view delimiter);
std::string build_resource_csv(const std::vector<ResourceKey> &columns, const std::vector<Resource> &rows, const CsvOptions &options);
std::string export_resources_csv(const ProjectDB &db, int64_t template_id); // out_of_range
struct ResourceTable { std::vector<std::string> headers; std::vector<std::vector<std::string>> rows; };
inline constexpr std::size_t kResourceTableMaxColumns = 8;
std::vector<ResourceTable> resource_tables(const std::vector<ResourceKey> &columns, const std::vector<Resource> &rows, std::size_t max_columns = kResourceTableMaxColumns);
struct ResourceReportSection { std::string template_name; std::vector<ResourceTable> tables; };
ResourceReportSection resource_report_section(const ProjectDB &db, int64_t template_id); // out_of_range
}
```

- [ ] **Step 1: Write the failing test.** Create `tests/unit/core/test_resource_export.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// CSV and PDF output through a template (spec §6.3): the formula-injection
// guard, quoting, header modes, display values, and the PDF's tables of at
// most 8 columns led by Betegnelse.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/resource_export.hpp>
#include <core/resource_templates.hpp>
#include <core/resources.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using reusex::ProjectDB;
using namespace reusex::core;
using namespace reusex::test_support;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_resource_export") {}
};
} // namespace

TEST_CASE("ResourceExport_CsvCell_GuardAndQuote", "[resources][csv]") {
  for (const char *risky : {"=SUM(A1)", "+1", "-3", "@x", "\tx", "\rx"}) {
    INFO(risky);
    CHECK(csv_cell(risky, ";").rfind("'", 0) == 0 ||
          csv_cell(risky, ";").rfind("\"'", 0) == 0);
  }
  CHECK(csv_cell("-3", ";") == "'-3");
  CHECK(csv_cell("a;b", ";") == "\"a;b\"");
  CHECK(csv_cell("a,b", ";") == "a,b");
  CHECK(csv_cell("a,b", ",") == "\"a,b\"");
  CHECK(csv_cell("say \"hi\"", ";") == "\"say \"\"hi\"\"\"");
  CHECK(csv_cell("two\nlines", ";") == "\"two\nlines\"");
  CHECK(csv_cell("\rx", ";") == "\"'\rx\"");
  CHECK(csv_cell("plain", ";") == "plain");
}

TEST_CASE("ResourceExport_Csv_HeaderModesDisplayValuesBom",
          "[resources][csv]") {
  const auto cat = key_catalogue({});
  const std::vector<ResourceKey> cols{*find_key(cat, "sys:name"),
                                      *find_key(cat, "sys:treatment"),
                                      *find_key(cat, "sys:starred"),
                                      *find_key(cat, "sys:mass_t")};
  const std::vector<Resource> rows{
      {"RX-001", 1, true,
       {{"sys:name", "=Døre"},
        {"sys:treatment", "nyttiggoerelse"},
        {"sys:starred", "true"},
        {"sys:mass_t", std::nullopt}}}};
  CsvOptions o; // ";", utf-8-bom, label
  CHECK(build_resource_csv(cols, rows, o) ==
        "\xEF\xBB\xBF"
        "Betegnelse;Behandling;Vigtig;Tons\r\n"
        "'=Døre;Nyttiggørelse;Ja;\r\n");
  o.header = "key";
  o.encoding = "utf-8";
  o.delimiter = ",";
  CHECK(build_resource_csv(cols, rows, o) ==
        "sys:name,sys:treatment,sys:starred,sys:mass_t\r\n"
        "'=Døre,nyttiggoerelse,true,\r\n");
}

TEST_CASE("ResourceExport_Tables_ChunkedLedByBetegnelse", "[resources][pdf]") {
  const auto cat = key_catalogue({});
  std::vector<ResourceKey> cols{*find_key(cat, "sys:name")};
  const auto lex = leksikon_keys();
  for (std::size_t i = 0; i < 15; ++i)
    cols.push_back(lex[i]);
  Resource r{"RX-007", 1, false, {{"sys:name", "Døre"}}};
  const auto tables = resource_tables(cols, {r});
  REQUIRE(tables.size() == 3); // 15 others / 7 per table
  CHECK(tables[0].headers.size() == 8);
  CHECK(tables[0].headers.front() == "Betegnelse");
  CHECK(tables[1].headers.size() == 8);
  CHECK(tables[2].headers.size() == 2);
  CHECK(tables[2].headers[1] == lex[14].label);
  for (const auto &t : tables) {
    REQUIRE(t.rows.size() == 1);
    CHECK(t.rows[0].front() == "Døre · RX-007");
    CHECK(t.rows[0].size() == t.headers.size());
  }
  CHECK(resource_tables(cols, {}).empty());
  const auto only_name = resource_tables({*find_key(cat, "sys:name")}, {r});
  REQUIRE(only_name.size() == 1);
  CHECK(only_name[0].headers == std::vector<std::string>{"Betegnelse"});
}

TEST_CASE("ResourceExport_ThroughATemplate", "[resources][csv][pdf]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto keep = make_type(db, "Døre");
  const auto drop = make_type(db, "Fejl");
  make_part(db, "RX-001", keep);
  make_part(db, "RX-002", drop);
  ProjectDB::SurveyTypePatch rej;
  rej.review_status = ReviewStatus::rejected;
  db.update_survey_type(drop, rej);
  TemplateInput in;
  in.name = "Kort";
  in.members = nlohmann::json::parse(R"([{"key":"sys:quantity"}])");
  in.csv = nlohmann::json::parse(R"({"encoding":"utf-8"})");
  const auto id = create_template(db, in).record.id;
  CHECK(export_resources_csv(db, id) ==
        "Mængde\r\n1\r\n1\r\n"); // CSV: every resource
  const auto section = resource_report_section(db, id);
  CHECK(section.template_name == "Kort");
  REQUIRE(section.tables.size() == 1);
  REQUIRE(section.tables[0].rows.size() == 1); // PDF: rejected type left out
  CHECK(section.tables[0].rows[0] ==
        std::vector<std::string>{"Døre · RX-001", "1"});
  CHECK_THROWS_AS(export_resources_csv(db, 999), std::out_of_range);
  CHECK_THROWS_AS(resource_report_section(db, 999), std::out_of_range);
}
```

- [ ] **Step 2: Run the build to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error `core/resource_export.hpp: No such file or directory`.

- [ ] **Step 3: Write the header.** Create `libs/reusex/include/core/resource_export.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Resources out through a template (docs/superpowers/specs/
/// 2026-10-02-resources-templates-ia-design.md §6.3): the backend CSV
/// (GET /resources/export.csv) and the PDF report's Ressourcetabel.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/resource_keys.hpp"
#include "reusex/core/resource_templates.hpp"
#include "reusex/core/resources.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

/// A value as a reader sees it: treatment and miljøstatus as their Danish
/// labels, booleans as Ja/Nej, unset as "".
std::string display_value(const ResourceKey &key,
                          const std::optional<std::string> &value);
/// One CSV cell: a cell starting with = + - @ TAB or CR gets a leading '
/// (formula injection), then RFC 4180 quoting when it holds the delimiter,
/// a quote, CR or LF.
std::string csv_cell(std::string_view value, std::string_view delimiter);
/// Header + one CRLF-terminated row per resource. header "label" writes key
/// labels and display values; "key" writes key ids and raw values. A
/// "utf-8-bom" encoding prefixes the UTF-8 byte-order mark.
std::string build_resource_csv(const std::vector<ResourceKey> &columns,
                               const std::vector<Resource> &rows,
                               const CsvOptions &options);
/// The CSV for one template: its resolved keys, every resource by code,
/// its CSV options. @throws std::out_of_range when the template is unknown.
std::string export_resources_csv(const ProjectDB &db, int64_t template_id);

struct ResourceTable {
  std::vector<std::string> headers;
  std::vector<std::vector<std::string>> rows;
};
inline constexpr std::size_t kResourceTableMaxColumns = 8;
/// Wrap @p columns into tables of at most @p max_columns, each led by a
/// "Betegnelse" column whose cell is "<sys:name> · <code>" (sys:name is not
/// repeated among the others). No rows: no tables.
std::vector<ResourceTable>
resource_tables(const std::vector<ResourceKey> &columns,
                const std::vector<Resource> &rows,
                std::size_t max_columns = kResourceTableMaxColumns);
struct ResourceReportSection {
  std::string template_name;
  std::vector<ResourceTable> tables;
};
/// The PDF's Ressourcetabel for one template: every resource whose type is
/// not rejected, by code. @throws std::out_of_range for an unknown template.
ResourceReportSection resource_report_section(const ProjectDB &db,
                                              int64_t template_id);

} // namespace reusex::core
```

- [ ] **Step 4: Write the implementation.** Create `libs/reusex/src/core/resource_export.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/resource_export.hpp"

#include "reusex/core/survey.hpp"

#include <algorithm>
#include <set>

namespace reusex::core {
namespace {
std::vector<ResourceKey> columns_of(const std::vector<std::string> &ids,
                                    const std::vector<ResourceKey> &catalogue) {
  std::vector<ResourceKey> out;
  for (const auto &id : ids)
    if (const auto *k = find_key(catalogue, id))
      out.push_back(*k);
  return out;
}
} // namespace

std::string display_value(const ResourceKey &key,
                          const std::optional<std::string> &value) {
  if (!value)
    return {};
  if (key.id == "sys:treatment")
    if (const auto t = treatment_from_string(*value))
      return std::string(treatment_label_da(*t));
  if (key.id == "sys:environment")
    for (auto e : {EnvironmentStatus::ren_screening, EnvironmentStatus::afventer,
                   EnvironmentStatus::forurenet,
                   EnvironmentStatus::ren_proevesvar})
      if (to_string(e) == *value)
        return std::string(environment_label_da(e));
  if (key.data_type == "boolean") {
    if (*value == "true")
      return "Ja";
    if (*value == "false")
      return "Nej";
  }
  return *value;
}

std::string csv_cell(std::string_view value, std::string_view delimiter) {
  std::string s(value);
  if (!s.empty() && (s[0] == '=' || s[0] == '+' || s[0] == '-' ||
                     s[0] == '@' || s[0] == '\t' || s[0] == '\r'))
    s.insert(s.begin(), '\'');
  const bool quote = s.find(delimiter) != std::string::npos ||
                     s.find_first_of("\"\n\r") != std::string::npos;
  if (!quote)
    return s;
  std::string out = "\"";
  for (char c : s)
    out += c == '"' ? std::string("\"\"") : std::string(1, c);
  return out + "\"";
}

std::string build_resource_csv(const std::vector<ResourceKey> &columns,
                               const std::vector<Resource> &rows,
                               const CsvOptions &o) {
  const bool by_key = o.header == "key";
  std::string out = o.encoding == "utf-8-bom" ? "\xEF\xBB\xBF" : "";
  for (std::size_t i = 0; i < columns.size(); ++i)
    out += (i ? o.delimiter : std::string()) +
           csv_cell(by_key ? columns[i].id : columns[i].label, o.delimiter);
  out += "\r\n";
  for (const auto &r : rows) {
    for (std::size_t i = 0; i < columns.size(); ++i) {
      const auto v = value_of(r, columns[i].id);
      out += (i ? o.delimiter : std::string()) +
             csv_cell(by_key ? v.value_or("") : display_value(columns[i], v),
                      o.delimiter);
    }
    out += "\r\n";
  }
  return out;
}

std::string export_resources_csv(const ProjectDB &db, int64_t template_id) {
  const auto view = template_view(db, template_id);
  const auto columns = columns_of(view.resolved.keys, key_catalogue(db));
  return build_resource_csv(columns, list_resources(db, view.resolved.keys),
                            view.csv);
}

std::vector<ResourceTable>
resource_tables(const std::vector<ResourceKey> &columns,
                const std::vector<Resource> &rows, std::size_t max_columns) {
  if (max_columns < 2)
    throw std::invalid_argument("resource_tables: max_columns must leave room "
                                "for Betegnelse and one more column");
  if (rows.empty())
    return {};
  std::vector<const ResourceKey *> others;
  for (const auto &k : columns)
    if (k.id != "sys:name")
      others.push_back(&k);
  const std::size_t per = max_columns - 1;
  std::vector<ResourceTable> out;
  for (std::size_t first = 0;; first += per) {
    const std::size_t last = std::min(first + per, others.size());
    ResourceTable t;
    t.headers.push_back("Betegnelse");
    for (std::size_t i = first; i < last; ++i)
      t.headers.push_back(others[i]->label);
    for (const auto &r : rows) {
      std::vector<std::string> cells{value_of(r, "sys:name").value_or("") +
                                     " · " + r.code};
      for (std::size_t i = first; i < last; ++i)
        cells.push_back(display_value(*others[i], value_of(r, others[i]->id)));
      t.rows.push_back(std::move(cells));
    }
    out.push_back(std::move(t));
    if (last >= others.size())
      break;
  }
  return out;
}

ResourceReportSection resource_report_section(const ProjectDB &db,
                                              int64_t template_id) {
  const auto view = template_view(db, template_id);
  const auto columns = columns_of(view.resolved.keys, key_catalogue(db));
  std::vector<std::string> keys{"sys:name"};
  for (const auto &id : view.resolved.keys)
    if (id != "sys:name")
      keys.push_back(id);
  std::set<int64_t> rejected;
  for (const auto &t : db.survey_types())
    if (t.review_status == ReviewStatus::rejected)
      rejected.insert(t.id);
  std::vector<Resource> rows;
  for (auto &r : list_resources(db, keys))
    if (rejected.count(r.type_id) == 0)
      rows.push_back(std::move(r));
  return {view.record.name, resource_tables(columns, rows)};
}

} // namespace reusex::core
```
- [ ] **Step 5: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests && cd build && ctest --output-on-failure --parallel $(nproc) -R 'ResourceExport_'`
Expected: 4 tests pass.

- [ ] **Step 6: Commit.**

```bash
git add libs/reusex/include/core/resource_export.hpp libs/reusex/src/core/resource_export.cpp tests/unit/core/test_resource_export.cpp
git commit -m "feat(core): resource CSV builder with formula guard and PDF table chunking

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 9: HTTP — resources, keys, columns path, CSV export

**Files:**
- Create: `apps/rux/include/gui/resources.hpp`, `apps/rux/src/gui/resources.cpp`
- Modify: `apps/rux/src/gui/api.cpp` (endpoint table rows; `create_material_column` / `patch_material_column` through `core::create_column` / `core::update_column`)
- Modify: `apps/rux/src/gui/Server.cpp` (registrations, `#include "gui/resources.hpp"`)
- Modify: `docs/gui/openapi.yaml`
- Test: `tests/unit/rux_gui/test_gui_resources.cpp` (new), `tests/unit/rux_gui/test_gui_api.cpp`, `tests/unit/rux_gui/test_gui_server_socket.cpp`

**Interfaces:**
- Consumes: Tasks 6–8.
- Produces (in `rux::gui`):

```cpp
nlohmann::json resource_keys_json(const reusex::ProjectDB &db);  // JSON array
nlohmann::json resources_json(const reusex::ProjectDB &db, const Params &params);
nlohmann::json patch_resource_json(reusex::ProjectDB &db, const std::string &code, const Params &params, const std::string &body);
nlohmann::json create_resource_json(reusex::ProjectDB &db, const std::string &body);
void delete_resource(reusex::ProjectDB &db, const std::string &code);
Blob resources_csv_blob(const reusex::ProjectDB &db, const Params &params);
```
- Wire: `ResourceKey {id,label,category,scope,data_type,unit|null,options,editable}`, `Resource {code,type_id,manual,values:{<key id>: string|null}}`.

- [ ] **Step 1: Write the failing handler test.** Create `tests/unit/rux_gui/test_gui_resources.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// The resources HTTP handlers (spec §4.4, §6.3, §7): shapes and status
// codes over a real ProjectDB.

#include <catch2/catch_test_macros.hpp>

#include <gui/api.hpp>
#include <gui/resources.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>
#include <core/resource_keys.hpp>
#include <core/resource_templates.hpp>

#include <nlohmann/json.hpp>

#include <functional>
#include <string>

using namespace rux::gui;
using reusex::ProjectDB;
using json = nlohmann::json;
using namespace reusex::test_support;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_gui_resources") {}
};
int status_of(const std::function<void()> &f) {
  try {
    f();
  } catch (const HttpError &e) {
    return e.status();
  }
  return 200;
}
Params params_of(std::initializer_list<std::pair<std::string, std::string>> kv) {
  Params p;
  for (const auto &[k, v] : kv)
    p.set(k, v);
  return p;
}
int64_t screening_id(const ProjectDB &db) {
  for (const auto &t : db.resource_templates())
    if (t.seed == std::optional<std::string>("screening"))
      return t.id;
  FAIL("no screening seed");
  return 0;
}
} // namespace

TEST_CASE("GuiResources_Keys_IsTheCatalogueArray", "[gui][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto keys = resource_keys_json(db);
  REQUIRE(keys.is_array());
  CHECK(keys.size() == reusex::core::key_catalogue(db).size());
  CHECK(keys[0] == json::parse(R"({"id":"sys:name","label":"Betegnelse",
        "category":"Kortlægning","scope":"type","data_type":"text",
        "unit":null,"options":[],"editable":true})"));
  CHECK(keys[6].at("editable") == false);
  CHECK(keys[8].at("unit") == "t");
  CHECK_FALSE(keys[0].contains("field"));
}

TEST_CASE("GuiResources_List_WithAndWithoutTemplate", "[gui][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  const auto all = resources_json(db, {});
  REQUIRE(all.at("resources").size() == 1);
  CHECK(all.at("resources")[0].at("manual") == true);
  CHECK_FALSE(all.contains("template"));
  const auto id = screening_id(db);
  const auto shaped =
      resources_json(db, params_of({{"template", std::to_string(id)}}));
  CHECK(shaped.at("template").at("id") == id);
  CHECK(shaped.at("template").at("resolved_keys").size() == 11);
  CHECK(shaped.at("template").at("missing") == json::array());
  const auto &values = shaped.at("resources")[0].at("values");
  CHECK(values.size() == 11);
  CHECK(values.at("sys:mass_t").is_null());
  CHECK(values.at("sys:name") == "Døre");
  CHECK(status_of([&] { resources_json(db, params_of({{"template", "999"}})); }) ==
        404);
  CHECK(status_of([&] { resources_json(db, params_of({{"template", "x"}})); }) ==
        400);
}

TEST_CASE("GuiResources_Patch_StatusesAndSiblings", "[gui][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t);
  make_part(db, "RX-002", t);
  const auto r = patch_resource_json(db, "RX-001", {},
                                     R"({"values":{"sys:unit":"m²"}})");
  CHECK(r.at("resource").at("values").at("sys:unit") == "m²");
  REQUIRE(r.at("siblings").size() == 1);
  CHECK(r.at("siblings")[0].at("code") == "RX-002");
  CHECK(status_of([&] {
          patch_resource_json(db, "RX-001", {},
                              R"({"values":{"sys:environment":"ren_screening"}})");
        }) == 400);
  CHECK(status_of([&] {
          patch_resource_json(db, "RX-001", {}, R"({"values":{"sys:x":"1"}})");
        }) == 400);
  CHECK(status_of([&] {
          patch_resource_json(db, "RX-001", {}, R"({"values":{"sys:note":1}})");
        }) == 400);
  CHECK(status_of([&] { patch_resource_json(db, "RX-001", {}, R"({})"); }) ==
        400);
  CHECK(status_of([&] {
          patch_resource_json(db, "RX-404", {}, R"({"values":{}})");
        }) == 404);
  // A 400 names the key.
  try {
    patch_resource_json(db, "RX-001", {},
                        R"({"values":{"sys:treatment":"smid ud"}})");
  } catch (const HttpError &e) {
    CHECK(std::string(e.what()).find("sys:treatment") != std::string::npos);
  }
}

TEST_CASE("GuiResources_CreateDelete", "[gui][resources]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  make_instance_cloud(db, 1);
  const auto t = make_type(db, "Døre");
  make_part(db, "RX-001", t, 1u);
  const auto r = create_resource_json(
      db, R"({"type_id":)" + std::to_string(t) + R"(,"name":"Branddør"})");
  CHECK(r.at("code") == "RX-002");
  CHECK(r.at("manual") == true);
  CHECK(status_of([&] { create_resource_json(db, R"({"type_id":999})"); }) ==
        404);
  CHECK(status_of([&] { create_resource_json(db, R"({"name":"x"})"); }) == 400);
  CHECK(status_of([&] { delete_resource(db, "RX-001"); }) == 409);
  CHECK(status_of([&] { delete_resource(db, "RX-404"); }) == 404);
  delete_resource(db, "RX-002");
  CHECK_FALSE(db.survey_part("RX-002").has_value());
}

TEST_CASE("GuiResources_Csv_RequiresTemplate", "[gui][resources][csv]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = make_type(db, "=Døre");
  make_part(db, "RX-001", t);
  const auto blob = resources_csv_blob(
      db, params_of({{"template", std::to_string(screening_id(db))}}));
  CHECK(blob.content_type == "text/csv; charset=utf-8");
  const std::string csv(blob.data.begin(), blob.data.end());
  CHECK(csv.rfind("\xEF\xBB\xBF" "Betegnelse;Mængde;", 0) == 0);
  CHECK(csv.find("\r\n'=Døre;1;") != std::string::npos);
  CHECK(status_of([&] { resources_csv_blob(db, {}); }) == 400);
  CHECK(status_of([&] {
          resources_csv_blob(db, params_of({{"template", "999"}}));
        }) == 404);
}

TEST_CASE("GuiResources_Columns_ConflictIs409", "[gui][resources][columns]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = create_material_column(db, R"({"name":"Stand","type":"text"})");
  CHECK(status_of([&] {
          create_material_column(db, R"({"name":"Stand","type":"text"})");
        }) == 409);
  const auto b = create_material_column(db, R"({"name":"Andet","type":"text"})");
  CHECK(status_of([&] {
          patch_material_column(db, b.at("id"), R"({"name":"Stand"})");
        }) == 409);
  CHECK(status_of([&] {
          patch_material_column(db, "nope", R"({"name":"x"})");
        }) == 404);
  CHECK(patch_material_column(db, a.at("id"), R"({"width":300})").at("width") ==
        300);
}
```

In `tests/unit/rux_gui/test_gui_api.cpp`, add these to the `expected` set after the `material-columns` entries:

```cpp
      "GET /api/v1/resources/columns",
      "POST /api/v1/resources/columns",
      "PATCH /api/v1/resources/columns/<string>",
      "DELETE /api/v1/resources/columns/<string>",
      "GET /api/v1/resources/keys",
      "GET /api/v1/resources",
      "POST /api/v1/resources",
      "PATCH /api/v1/resources/<string>",
      "DELETE /api/v1/resources/<string>",
      "GET /api/v1/resources/export.csv",
```

In `tests/unit/rux_gui/test_gui_server_socket.cpp`, add this after `RunningServer_KeepAliveVariousRoutes_HonorsRoutingContract`:

```cpp
TEST_CASE("RunningServer_ResourceRoutes_StaticPathsBeatTheCodeParam",
          "[gui][server][socket]") {
  // /resources/keys etc. sit next to /resources/<string>; Crow keeps one
  // trie per method, so GET never reaches the PATCH/DELETE code route.
  TempPath project("test_gui_server_socket", ".rux");
  TempDir assets("test_gui_server_socket_assets");
  write_file(assets.path / "index.html", kIndexBody);
  RunningServer server(options_for(project.path, assets.path, free_port()));
  KeepAliveConnection connection(server.port());
  for (const char *route :
       {"/api/v1/resources/keys", "/api/v1/resources/columns",
        "/api/v1/resources", "/api/v1/resources/export.csv?template=2"}) {
    INFO("route: " << route);
    CHECK(connection.get(route).status == 200);
  }
  CHECK(connection.get("/api/v1/resources/export.csv").status == 400);
}
```

The CSV request uses `template=2`, the screening seed's id on a fresh project.

- [ ] **Step 2: Run the build to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests`
Expected: compile error `gui/resources.hpp: No such file or directory`.

- [ ] **Step 3: Write the handlers.** Create `apps/rux/include/gui/resources.hpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Resources and templates endpoints (docs/superpowers/specs/
// 2026-10-02-resources-templates-ia-design.md §4.4, §5.5, §6.3). Thin: each
// parses, calls one reusex::core function, and maps its exceptions —
// KeyValueError/invalid_argument 400, out_of_range 404, NameConflictError
// and ResourceConflictError 409 — by throwing HttpError (gui/api.hpp).

#include "gui/api.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>

namespace rux::gui {

/// `GET /resources/keys`: the key catalogue as a JSON array.
nlohmann::json resource_keys_json(const reusex::ProjectDB &db);
/// `GET /resources[?template=<id>]`: `{resources:[…], template?:{id,
/// resolved_keys, missing}}`. @throws HttpError 400 (bad id), 404 (unknown).
nlohmann::json resources_json(const reusex::ProjectDB &db,
                              const Params &params);
/// `PATCH /resources/<code>[?template=<id>]`, body `{values:{<key>:
/// string|null}}` → `{resource, siblings}`.
nlohmann::json patch_resource_json(reusex::ProjectDB &db,
                                   const std::string &code,
                                   const Params &params,
                                   const std::string &body);
/// `POST /resources`, body `{type_id, name?}` → the new resource.
nlohmann::json create_resource_json(reusex::ProjectDB &db,
                                    const std::string &body);
/// `DELETE /resources/<code>`: manual parts only (409 otherwise).
void delete_resource(reusex::ProjectDB &db, const std::string &code);
/// `GET /resources/export.csv?template=<id>` (template required → 400).
Blob resources_csv_blob(const reusex::ProjectDB &db, const Params &params);

} // namespace rux::gui
```

Create `apps/rux/src/gui/resources.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/resources.hpp"

#include <reusex/core/resource_export.hpp>
#include <reusex/core/resource_keys.hpp>
#include <reusex/core/resource_templates.hpp>
#include <reusex/core/resources.hpp>
#include <reusex/core/survey.hpp>

#include <optional>
#include <stdexcept>
#include <vector>

namespace rux::gui {
namespace {
using json = nlohmann::json;
namespace core = reusex::core;

/// Runs f, translating library exceptions into the documented statuses.
template <typename F> auto mapped(F &&f) -> decltype(f()) {
  try {
    return f();
  } catch (const HttpError &) {
    throw;
  } catch (const core::KeyValueError &e) {
    throw HttpError(400, e.what());
  } catch (const core::NameConflictError &e) {
    throw HttpError(409, e.what());
  } catch (const core::ResourceConflictError &e) {
    throw HttpError(409, e.what());
  } catch (const std::out_of_range &e) {
    throw HttpError(404, e.what());
  } catch (const std::invalid_argument &e) {
    throw HttpError(400, e.what());
  }
}

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
  return mapped([&] {
    const auto view = template_param(db, params);
    json list = json::array();
    for (const auto &r : core::list_resources(db, keys_of(view)))
      list.push_back(resource_json(r));
    json out{{"resources", std::move(list)}};
    if (view)
      out["template"] = {{"id", view->record.id},
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
  return mapped([&] {
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
  return mapped([&] {
    return resource_json(core::create_resource(db, t->get<int64_t>(), name));
  });
}

void delete_resource(reusex::ProjectDB &db, const std::string &code) {
  mapped([&] {
    core::delete_resource(db, code);
    return 0;
  });
}

Blob resources_csv_blob(const reusex::ProjectDB &db, const Params &params) {
  if (!params.find("template"))
    throw HttpError(400, "'template' is required: the CSV's columns come from "
                         "a template");
  const auto id = params.integer("template", 0);
  return mapped([&] {
    const auto csv = core::export_resources_csv(db, id);
    Blob b;
    b.content_type = "text/csv; charset=utf-8";
    b.data.assign(csv.begin(), csv.end());
    return b;
  });
}

} // namespace rux::gui
```

- [ ] **Step 4: Send the column handlers through core.** In `apps/rux/src/gui/api.cpp`, add `#include <reusex/core/resources.hpp>`. In `create_material_column`, replace everything from `reusex::ProjectDB::PropertyDefinition created;` to the end of the function with:

```cpp
  reusex::ProjectDB::PropertyDefinition def;
  def.name = name;
  def.type = type;
  def.options = options;
  def.sort_order = sort_order;
  def.width = width;
  try {
    return definition_json(reusex::core::create_column(db, def));
  } catch (const reusex::core::NameConflictError &e) {
    throw HttpError(409, e.what());
  } catch (const std::invalid_argument &e) {
    throw HttpError(400, e.what());
  }
```

Replace the body of `patch_material_column` after the parse check with:

```cpp
  reusex::core::ColumnPatch patch;
  if (parsed.contains("name")) {
    if (!parsed["name"].is_string())
      throw HttpError(400, "'name' must be a string");
    patch.name = parsed["name"].get<std::string>();
  }
  if (parsed.contains("type")) {
    if (!parsed["type"].is_string())
      throw HttpError(400, "'type' must be a string");
    patch.type = parsed["type"].get<std::string>();
    if (!is_valid_column_type(*patch.type))
      throw HttpError(400, "'type' must be one of "
                           "text/number/date/boolean/select/multiselect");
  }
  if (parsed.contains("options")) {
    if (!parsed["options"].is_array())
      throw HttpError(400, "'options' must be an array");
    std::vector<std::string> options;
    for (const auto &option : parsed["options"])
      if (option.is_string())
        options.push_back(option.get<std::string>());
    patch.options = std::move(options);
  }
  if (parsed.contains("sort_order")) {
    if (!parsed["sort_order"].is_number_integer())
      throw HttpError(400, "'sort_order' must be an integer");
    patch.sort_order = parsed["sort_order"].get<int>();
  }
  if (parsed.contains("width")) {
    if (!parsed["width"].is_number_integer())
      throw HttpError(400, "'width' must be an integer");
    patch.width = parsed["width"].get<int>();
  }
  // A rename carries the column's stored values (core::update_column).
  try {
    return definition_json(reusex::core::update_column(db, id, patch));
  } catch (const reusex::core::NameConflictError &e) {
    throw HttpError(409, e.what());
  } catch (const std::out_of_range &e) {
    throw HttpError(404, e.what());
  } catch (const std::invalid_argument &e) {
    throw HttpError(400, e.what());
  }
```

Add these endpoint-table rows after the four `material-columns` rows, with the same summaries the openapi will carry:

```cpp
      {"GET", "/api/v1/resources/columns",
       "User-defined resource column definitions"},
      {"POST", "/api/v1/resources/columns",
       "Create a resource column definition"},
      {"PATCH", "/api/v1/resources/columns/<string>",
       "Update a resource column definition"},
      {"DELETE", "/api/v1/resources/columns/<string>",
       "Delete a resource column definition"},
      {"GET", "/api/v1/resources/keys", "The resource key catalogue"},
      {"GET", "/api/v1/resources",
       "Resources (survey parts) with their key values"},
      {"POST", "/api/v1/resources", "Add a resource by hand"},
      {"PATCH", "/api/v1/resources/<string>",
       "Set or clear a resource's key values"},
      {"DELETE", "/api/v1/resources/<string>",
       "Delete a manually added resource"},
      {"GET", "/api/v1/resources/export.csv",
       "Resources as CSV through a template", true},
```

- [ ] **Step 5: Register the routes.** In `Server.cpp`, add `#include "gui/resources.hpp"`. Replace the two `material-columns` route blocks with a loop, so both paths share their handlers:

```cpp
    // /material-columns is the pre-v25 path; Phase 3 of the resources
    // redesign deletes it once its last frontend user is gone.
    for (const std::string base :
         {"/api/v1/material-columns", "/api/v1/resources/columns"}) {
      app_.route_dynamic(base).methods(crow::HTTPMethod::GET,
                                       crow::HTTPMethod::POST)(
          [this](const crow::request &req) {
            if (req.method == crow::HTTPMethod::GET)
              return with_db([&](const reusex::ProjectDB &db) {
                return json_response(200, material_columns_json(db));
              });
            return with_write([&](reusex::ProjectDB &db) {
              return json_response(201, create_material_column(db, req.body));
            });
          });
      app_.route_dynamic(base + "/<string>")
          .methods(crow::HTTPMethod::PATCH, crow::HTTPMethod::DELETE)(
              [this](const crow::request &req, std::string id) {
                if (req.method == crow::HTTPMethod::PATCH)
                  return with_write([&](reusex::ProjectDB &db) {
                    return json_response(
                        200, patch_material_column(db, id, req.body));
                  });
                return with_write([&](reusex::ProjectDB &db) {
                  delete_material_column(db, id);
                  return crow::response(204);
                });
              });
    }

    // ---- resources (schema v25) ----
    // Static paths are registered before /resources/<string>. Crow keeps one
    // trie per method and that route takes only PATCH/DELETE, so a GET can
    // never reach it either way.
    get("/api/v1/resources/keys")([this](const crow::request &) {
      return with_db([](const reusex::ProjectDB &db) {
        return json_response(200, resource_keys_json(db));
      });
    });
    get("/api/v1/resources/export.csv")([this](const crow::request &req) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) {
        const auto b = resources_csv_blob(db, params);
        crow::response res(200);
        res.set_header("Content-Type", b.content_type);
        res.set_header("Content-Disposition",
                       "attachment; filename=\"ressourcer.csv\"");
        res.body.assign(reinterpret_cast<const char *>(b.data.data()),
                        b.data.size());
        return res;
      });
    });
    app_.route_dynamic("/api/v1/resources")
        .methods(crow::HTTPMethod::GET,
                 crow::HTTPMethod::POST)([this](const crow::request &req) {
          if (req.method == crow::HTTPMethod::GET) {
            const Params params = params_of(req);
            return with_db([&](const reusex::ProjectDB &db) {
              return json_response(200, resources_json(db, params));
            });
          }
          return with_write([&](reusex::ProjectDB &db) {
            return json_response(201, create_resource_json(db, req.body));
          });
        });
    app_.route_dynamic("/api/v1/resources/<string>")
        .methods(crow::HTTPMethod::PATCH, crow::HTTPMethod::DELETE)(
            [this](const crow::request &req, std::string code) {
              if (req.method == crow::HTTPMethod::PATCH) {
                const Params params = params_of(req);
                return with_write([&](reusex::ProjectDB &db) {
                  return json_response(
                      200, patch_resource_json(db, code, params, req.body));
                });
              }
              return with_write([&](reusex::ProjectDB &db) {
                delete_resource(db, code);
                return crow::response(204);
              });
            });
```

If `with_db`/`with_write` deduce one return type per lambda, the CSV lambda's `crow::response` is fine: `json_response` already returns `crow::response`.

- [ ] **Step 6: Document the contract.** In `docs/gui/openapi.yaml`:
  - Add two entries to `tags`: `- name: resources` with `description: Resources (survey parts) addressed by key id, their columns and CSV export (schema v25)`, and `- name: templates` with `description: Templates — ordered category/key selections over resource keys (schema v25)`.
  - Prepend `Deprecated alias of /resources/columns; removed in GUI resources Phase 3. ` to the description of `/material-columns` (add a `description:` to its `get` operation).
  - Copy the `/material-columns` and `/material-columns/{id}` path items to `/resources/columns` and `/resources/columns/{id}`. Use `tags: [resources]`, operationIds `listResourceColumns`/`createResourceColumn`/`updateResourceColumn`/`deleteResourceColumn`, and the summaries from the endpoint table. Add a `"409": { $ref: "#/components/responses/WriteConflict" }` response to both POSTs and both PATCHes. In each PATCH description, add this line: `A rename moves the values stored under the old name; 409 when the name is taken by another column or a leksikon field, or already holds stored values.`
  - Add these paths:

```yaml
  /resources/keys:
    get:
      tags: [resources]
      operationId: listResourceKeys
      summary: The resource key catalogue
      description: |
        Built-in keys (sys:), leksikon keys (lex:) and user columns (col:),
        in catalogue order — the order a template category expands in.
      responses:
        "200":
          description: Every key
          content:
            application/json:
              schema:
                type: array
                items: { $ref: "#/components/schemas/ResourceKey" }
        default: { $ref: "#/components/responses/UnexpectedError" }

  /resources:
    get:
      tags: [resources]
      operationId: listResources
      summary: Resources (survey parts) with their key values
      parameters:
        - name: template
          in: query
          required: false
          schema: { type: integer, format: int64 }
          description: |
            When given, `values` holds exactly the template's resolved keys
            (null when unset). Without it, `values` holds every key the
            resource has a value for.
      responses:
        "200":
          description: Resources, by code
          content:
            application/json:
              schema:
                type: object
                required: [resources]
                properties:
                  resources:
                    type: array
                    items: { $ref: "#/components/schemas/Resource" }
                  template:
                    type: object
                    description: Present when `template` was given.
                    required: [id, resolved_keys, missing]
                    properties:
                      id: { type: integer, format: int64 }
                      resolved_keys: { type: array, items: { type: string } }
                      missing:
                        type: array
                        items: { $ref: "#/components/schemas/TemplateMember" }
        "400": { $ref: "#/components/responses/BadRequest" }
        "404": { $ref: "#/components/responses/NotFound" }
        default: { $ref: "#/components/responses/UnexpectedError" }
    post:
      tags: [resources]
      operationId: createResource
      summary: Add a resource by hand
      description: |
        Creates a manual survey part with the next RX code under `type_id`.
        `name` is stored as the resource's leksikon `designation`.
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [type_id]
              properties:
                type_id: { type: integer, format: int64 }
                name: { type: string, nullable: true }
      responses:
        "201":
          description: The new resource
          content:
            application/json:
              schema: { $ref: "#/components/schemas/Resource" }
        "400": { $ref: "#/components/responses/BadRequest" }
        "404": { $ref: "#/components/responses/NotFound" }
        "503": { $ref: "#/components/responses/WriterBusy" }
        default: { $ref: "#/components/responses/UnexpectedError" }

  /resources/{code}:
    parameters:
      - name: code
        in: path
        required: true
        schema: { type: string, example: RX-008 }
    patch:
      tags: [resources]
      operationId: patchResource
      summary: Set or clear a resource's key values
      description: |
        Writes are routed by scope: type-scoped keys change the survey type
        (and so every part of it; those parts come back in `siblings`),
        part-scoped keys the part, leksikon and column keys the part's own
        passport (created on the first write). `null` clears; clearing an
        absent value succeeds. Every value is validated before anything is
        written, in one transaction. Optional `template` shapes `values`
        like GET /resources.
      parameters:
        - name: template
          in: query
          required: false
          schema: { type: integer, format: int64 }
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [values]
              properties:
                values: { $ref: "#/components/schemas/ResourceValues" }
      responses:
        "200":
          description: The resource after the write, and its type's other parts when a type-scoped key changed
          content:
            application/json:
              schema:
                type: object
                required: [resource, siblings]
                properties:
                  resource: { $ref: "#/components/schemas/Resource" }
                  siblings:
                    type: array
                    items: { $ref: "#/components/schemas/Resource" }
        "400":
          description: Unknown key, read-only key, or a value its type refuses; the message names the key
          content:
            application/json:
              schema: { $ref: "#/components/schemas/Error" }
        "404": { $ref: "#/components/responses/NotFound" }
        "503": { $ref: "#/components/responses/WriterBusy" }
        default: { $ref: "#/components/responses/UnexpectedError" }
    delete:
      tags: [resources]
      operationId: deleteResource
      summary: Delete a manually added resource
      description: |
        Removes a manual part and its passport (unless something else links
        it). An instance-backed part comes from the scan and is refused.
      responses:
        "204": { description: Deleted }
        "404": { $ref: "#/components/responses/NotFound" }
        "409": { $ref: "#/components/responses/WriteConflict" }
        "503": { $ref: "#/components/responses/WriterBusy" }
        default: { $ref: "#/components/responses/UnexpectedError" }

  /resources/export.csv:
    get:
      tags: [resources]
      operationId: exportResourcesCsv
      summary: Resources as CSV through a template
      description: |
        One row per resource, columns from the template's resolved keys,
        delimiter/encoding/header from its `csv` options. Cells starting
        with = + - @ TAB or CR are prefixed with an apostrophe (formula
        injection guard). Lines end in CRLF.
      parameters:
        - name: template
          in: query
          required: true
          schema: { type: integer, format: int64 }
      responses:
        "200":
          description: The CSV file
          content:
            text/csv:
              schema: { type: string }
        "400": { $ref: "#/components/responses/BadRequest" }
        "404": { $ref: "#/components/responses/NotFound" }
        default: { $ref: "#/components/responses/UnexpectedError" }
```

  - Add these schemas. Put `TemplateMember` here because `/resources` uses it, and Task 10 adds the rest:

```yaml
    ResourceKey:
      type: object
      required: [id, label, category, scope, data_type, unit, options, editable]
      properties:
        id: { type: string, example: "sys:quantity", description: "sys:<name>, lex:<leksikon guid> or col:<column id>" }
        label: { type: string }
        category: { type: string, description: "Kortlægning, Egne felter, or a leksikon category" }
        scope: { type: string, enum: [type, part], description: "What a write changes" }
        data_type: { type: string, enum: [text, number, enum, boolean, date] }
        unit: { type: string, nullable: true }
        options: { type: array, items: { type: string }, description: "Choices of an enum key" }
        editable: { type: boolean, description: "false for derived keys (sys:environment)" }
    ResourceValues:
      type: object
      description: Key id -> value; null means unset (GET) or clear (PATCH).
      additionalProperties: { type: string, nullable: true }
    Resource:
      type: object
      required: [code, type_id, manual, values]
      properties:
        code: { type: string, example: RX-008 }
        type_id: { type: integer, format: int64 }
        manual: { type: boolean, description: "Added by hand (no instance); only these can be deleted" }
        values: { $ref: "#/components/schemas/ResourceValues" }
    TemplateMember:
      type: object
      description: Exactly one of `category` or `key`.
      properties:
        category: { type: string }
        key: { type: string }
```

  If `components.responses.BadRequest` does not exist, grep for it. It is referenced at the `/export-templates` POST, so it does exist.

- [ ] **Step 7: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests rux && cd build && ctest --output-on-failure --parallel $(nproc) -R 'GuiResources_|EndpointTable|EndpointsJson|RunningServer_|gui_api_contract_parses'`
Expected: all pass.

- [ ] **Step 8: Commit.**

```bash
git add apps/rux/include/gui/resources.hpp apps/rux/src/gui/resources.cpp apps/rux/src/gui/api.cpp apps/rux/src/gui/Server.cpp docs/gui/openapi.yaml tests/unit/rux_gui
git commit -m "feat(gui): resources API — keys, values, add/delete, columns path, CSV

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 10: HTTP — templates CRUD, duplicate, restore seeds

**Files:**
- Modify: `apps/rux/include/gui/resources.hpp`, `apps/rux/src/gui/resources.cpp`, `apps/rux/src/gui/api.cpp` (endpoint rows), `apps/rux/src/gui/Server.cpp`, `docs/gui/openapi.yaml`
- Test: `tests/unit/rux_gui/test_gui_resources.cpp`, `test_gui_api.cpp`, `test_gui_server_socket.cpp`

**Interfaces:**
- Consumes: Task 7.
- Produces (in `rux::gui`):

```cpp
nlohmann::json templates_json(const reusex::ProjectDB &db);                       // {templates:[…]}
nlohmann::json create_template_json(reusex::ProjectDB &db, const std::string &body);
nlohmann::json patch_template_json(reusex::ProjectDB &db, int64_t id, const std::string &body);
void delete_template(reusex::ProjectDB &db, int64_t id);
nlohmann::json duplicate_template_json(reusex::ProjectDB &db, int64_t id);
nlohmann::json restore_seed_templates_json(reusex::ProjectDB &db);                // {restored:[names], templates:[…]}
```
- Wire `Template`: `{id, name, members:[TemplateMember], csv:CsvOptions, seed:string|null, resolved_keys:[string], missing:[TemplateMember], created_at, updated_at}`.

- [ ] **Step 1: Write the failing tests.** Append to `tests/unit/rux_gui/test_gui_resources.cpp`:

```cpp
TEST_CASE("GuiTemplates_ListShape", "[gui][templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto body = templates_json(db);
  REQUIRE(body.at("templates").size() == 2);
  const auto &s = body.at("templates")[1];
  CHECK(s.at("name") == "Hurtig genbrugsscreening");
  CHECK(s.at("seed") == "screening");
  CHECK(s.at("members").size() == 11);
  CHECK(s.at("members")[0] == json::parse(R"({"key":"sys:name"})"));
  CHECK(s.at("resolved_keys").size() == 11);
  CHECK(s.at("missing") == json::array());
  CHECK(s.at("csv") == json::parse(R"({"delimiter":";","encoding":"utf-8-bom",
        "header":"label"})"));
  CHECK(body.at("templates")[0].at("seed") == "materialepas");
}

TEST_CASE("GuiTemplates_CrudAndStatuses", "[gui][templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = create_template_json(
      db, R"({"name":"Mit","members":[{"key":"sys:note"},{"key":"col:x"}]})");
  const int64_t id = t.at("id");
  CHECK(t.at("seed").is_null());
  CHECK(t.at("missing") == json::parse(R"([{"key":"col:x"}])"));
  CHECK(status_of([&] { create_template_json(db, R"({"name":"Mit"})"); }) ==
        409);
  CHECK(status_of([&] { create_template_json(db, R"({})"); }) == 400);
  CHECK(status_of([&] {
          create_template_json(db, R"({"name":"X","members":{}})");
        }) == 400);
  CHECK(status_of([&] {
          create_template_json(db, R"({"name":"X","csv":{"delimiter":"|"}})");
        }) == 400);
  const auto p = patch_template_json(db, id, R"({"csv":{"header":"key"}})");
  CHECK(p.at("csv").at("header") == "key");
  CHECK(p.at("name") == "Mit");
  CHECK(status_of([&] { patch_template_json(db, 999, R"({"name":"Y"})"); }) ==
        404);
  CHECK(status_of([&] {
          patch_template_json(db, id, R"({"name":"Hurtig genbrugsscreening"})");
        }) == 409);
  CHECK(duplicate_template_json(db, id).at("name") == "Mit (kopi)");
  CHECK(status_of([&] { duplicate_template_json(db, 999); }) == 404);
  delete_template(db, id);
  CHECK(status_of([&] { delete_template(db, id); }) == 404);
}

TEST_CASE("GuiTemplates_RestoreSeeds", "[gui][templates]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  delete_template(db, screening_id(db));
  const auto r = restore_seed_templates_json(db);
  CHECK(r.at("restored") == json::parse(R"(["Hurtig genbrugsscreening"])"));
  CHECK(r.at("templates").size() == 2);
  CHECK(restore_seed_templates_json(db).at("restored") == json::array());
}
```

In `test_gui_api.cpp`, add these to `expected`:

```cpp
      "GET /api/v1/templates",
      "POST /api/v1/templates",
      "POST /api/v1/templates/restore-seeds",
      "PATCH /api/v1/templates/<int>",
      "DELETE /api/v1/templates/<int>",
      "POST /api/v1/templates/<int>/duplicate",
```

In the socket test `RunningServer_ResourceRoutes_StaticPathsBeatTheCodeParam`, append:

```cpp
  CHECK(connection.get("/api/v1/templates").status == 200);
  CHECK(connection.send_json("POST", "/api/v1/templates/restore-seeds", "{}")
            .status == 200);
  CHECK(connection.send_json("POST", "/api/v1/templates/1/duplicate", "{}")
            .status == 201);
```

If `send_json` does not set `Content-Type: application/json`, read `send_request` in that file. The server answers 415 without it. The existing PATCH tests in the same file show which header line to use.

- [ ] **Step 2: Run the build to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests`
Expected: compile errors. `templates_json` and the other handlers are undeclared.

- [ ] **Step 3: Write the handlers.** Append these declarations to `apps/rux/include/gui/resources.hpp`, inside the namespace:

```cpp
/// `GET /templates`: `{templates:[Template…]}`, by id.
nlohmann::json templates_json(const reusex::ProjectDB &db);
/// `POST /templates`, body `{name, members?, csv?}` (201).
nlohmann::json create_template_json(reusex::ProjectDB &db,
                                    const std::string &body);
/// `PATCH /templates/<id>`, body any of `{name, members, csv}`.
nlohmann::json patch_template_json(reusex::ProjectDB &db, int64_t id,
                                   const std::string &body);
/// `DELETE /templates/<id>` (204).
void delete_template(reusex::ProjectDB &db, int64_t id);
/// `POST /templates/<id>/duplicate` (201): "<name> (kopi)", numbered.
nlohmann::json duplicate_template_json(reusex::ProjectDB &db, int64_t id);
/// `POST /templates/restore-seeds`: `{restored:[names], templates:[…]}`.
nlohmann::json restore_seed_templates_json(reusex::ProjectDB &db);
```

Add these to the anonymous namespace in `resources.cpp`:

```cpp
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
```

Add these after `resources_csv_blob`:

```cpp
json templates_json(const reusex::ProjectDB &db) {
  return {{"templates", template_list(db)}};
}

json create_template_json(reusex::ProjectDB &db, const std::string &body) {
  const auto in = template_input(parse_body(body));
  return mapped([&] { return template_json(core::create_template(db, in)); });
}

json patch_template_json(reusex::ProjectDB &db, int64_t id,
                         const std::string &body) {
  const auto in = template_input(parse_body(body));
  return mapped(
      [&] { return template_json(core::update_template(db, id, in)); });
}

void delete_template(reusex::ProjectDB &db, int64_t id) {
  mapped([&] {
    core::delete_template(db, id);
    return 0;
  });
}

json duplicate_template_json(reusex::ProjectDB &db, int64_t id) {
  return mapped(
      [&] { return template_json(core::duplicate_template(db, id)); });
}

json restore_seed_templates_json(reusex::ProjectDB &db) {
  return mapped([&] {
    json names = json::array();
    for (const auto &v : core::restore_seed_templates(db))
      names.push_back(v.record.name);
    return json{{"restored", std::move(names)},
                {"templates", template_list(db)}};
  });
}
```

Add these to the endpoint table after the resources rows:

```cpp
      {"GET", "/api/v1/templates", "Templates with their resolved keys"},
      {"POST", "/api/v1/templates", "Create a template"},
      {"POST", "/api/v1/templates/restore-seeds",
       "Re-insert missing standard templates"},
      {"PATCH", "/api/v1/templates/<int>", "Rename or edit a template"},
      {"DELETE", "/api/v1/templates/<int>", "Delete a template"},
      {"POST", "/api/v1/templates/<int>/duplicate", "Copy a template"},
```

Register them in `Server.cpp` after the resources block:

```cpp
    // ---- templates (schema v25) ----
    app_.route_dynamic("/api/v1/templates")
        .methods(crow::HTTPMethod::GET,
                 crow::HTTPMethod::POST)([this](const crow::request &req) {
          if (req.method == crow::HTTPMethod::GET)
            return with_db([](const reusex::ProjectDB &db) {
              return json_response(200, templates_json(db));
            });
          return with_write([&](reusex::ProjectDB &db) {
            return json_response(201, create_template_json(db, req.body));
          });
        });
    app_.route_dynamic("/api/v1/templates/restore-seeds")
        .methods(crow::HTTPMethod::POST)([this](const crow::request &) {
          return with_write([](reusex::ProjectDB &db) {
            return json_response(200, restore_seed_templates_json(db));
          });
        });
    app_.route_dynamic("/api/v1/templates/<int>")
        .methods(crow::HTTPMethod::PATCH, crow::HTTPMethod::DELETE)(
            [this](const crow::request &req, int id) {
              if (req.method == crow::HTTPMethod::PATCH)
                return with_write([&](reusex::ProjectDB &db) {
                  return json_response(200,
                                       patch_template_json(db, id, req.body));
                });
              return with_write([&](reusex::ProjectDB &db) {
                delete_template(db, id);
                return crow::response(204);
              });
            });
    app_.route_dynamic("/api/v1/templates/<int>/duplicate")
        .methods(crow::HTTPMethod::POST)([this](const crow::request &, int id) {
          return with_write([&](reusex::ProjectDB &db) {
            return json_response(201, duplicate_template_json(db, id));
          });
        });
```

- [ ] **Step 4: Document the contract.** In `docs/gui/openapi.yaml`:
  - Prepend this to the `/export-templates` GET description (add the `description:` key if the operation has none): `Legacy view over /templates (schema v25): config.columns lists the template's legacy members and user-column labels; category members are not listed. Removed from rux gui in resources Phase 4. Duplicate names are 409.` Add `"409": { $ref: "#/components/responses/WriteConflict" }` to the POST and PATCH there.
  - Add these paths:

```yaml
  /templates:
    get:
      tags: [templates]
      operationId: listTemplates
      summary: Templates with their resolved keys
      description: |
        Every template, by id, resolved against the live key catalogue: a
        category member expands to every key now in that category. Members
        that resolve to nothing are listed in `missing`.
      responses:
        "200":
          description: Templates
          content:
            application/json:
              schema:
                type: object
                required: [templates]
                properties:
                  templates:
                    type: array
                    items: { $ref: "#/components/schemas/Template" }
        default: { $ref: "#/components/responses/UnexpectedError" }
    post:
      tags: [templates]
      operationId: createTemplate
      summary: Create a template
      requestBody:
        required: true
        content:
          application/json:
            schema: { $ref: "#/components/schemas/TemplateInput" }
      responses:
        "201":
          description: The new template
          content:
            application/json:
              schema: { $ref: "#/components/schemas/Template" }
        "400": { $ref: "#/components/responses/BadRequest" }
        "409": { $ref: "#/components/responses/WriteConflict" }
        "503": { $ref: "#/components/responses/WriterBusy" }
        default: { $ref: "#/components/responses/UnexpectedError" }

  /templates/restore-seeds:
    post:
      tags: [templates]
      operationId: restoreSeedTemplates
      summary: Re-insert missing standard templates
      description: |
        Inserts every seed (materialepas, screening) whose tag no template
        carries. A seed whose name is taken gets " (standard)". Running it
        again inserts nothing.
      responses:
        "200":
          description: Names inserted, and the full list afterwards
          content:
            application/json:
              schema:
                type: object
                required: [restored, templates]
                properties:
                  restored: { type: array, items: { type: string } }
                  templates:
                    type: array
                    items: { $ref: "#/components/schemas/Template" }
        "503": { $ref: "#/components/responses/WriterBusy" }
        default: { $ref: "#/components/responses/UnexpectedError" }

  /templates/{id}:
    parameters:
      - name: id
        in: path
        required: true
        schema: { type: integer, format: int64 }
    patch:
      tags: [templates]
      operationId: patchTemplate
      summary: Rename or edit a template
      requestBody:
        required: true
        content:
          application/json:
            schema: { $ref: "#/components/schemas/TemplateInput" }
      responses:
        "200":
          description: The template after the edit
          content:
            application/json:
              schema: { $ref: "#/components/schemas/Template" }
        "400": { $ref: "#/components/responses/BadRequest" }
        "404": { $ref: "#/components/responses/NotFound" }
        "409": { $ref: "#/components/responses/WriteConflict" }
        "503": { $ref: "#/components/responses/WriterBusy" }
        default: { $ref: "#/components/responses/UnexpectedError" }
    delete:
      tags: [templates]
      operationId: deleteTemplate
      summary: Delete a template
      responses:
        "204": { description: Deleted }
        "404": { $ref: "#/components/responses/NotFound" }
        "503": { $ref: "#/components/responses/WriterBusy" }
        default: { $ref: "#/components/responses/UnexpectedError" }

  /templates/{id}/duplicate:
    parameters:
      - name: id
        in: path
        required: true
        schema: { type: integer, format: int64 }
    post:
      tags: [templates]
      operationId: duplicateTemplate
      summary: Copy a template
      description: |
        Copies members and CSV options as "<name> (kopi)", then
        "<name> (kopi 2)", … until the name is free. The copy is never a seed.
      responses:
        "201":
          description: The copy
          content:
            application/json:
              schema: { $ref: "#/components/schemas/Template" }
        "404": { $ref: "#/components/responses/NotFound" }
        "503": { $ref: "#/components/responses/WriterBusy" }
        default: { $ref: "#/components/responses/UnexpectedError" }
```

  - Add these schemas:

```yaml
    CsvOptions:
      type: object
      description: A template's CSV export options; unknown fields are kept.
      properties:
        delimiter: { type: string, enum: [";", ",", "\t"], default: ";" }
        encoding: { type: string, enum: [utf-8, utf-8-bom], default: utf-8-bom }
        header: { type: string, enum: [label, key], default: label, description: "label writes key labels and display values; key writes key ids and raw values" }
      additionalProperties: true
    Template:
      type: object
      required: [id, name, members, csv, seed, resolved_keys, missing, created_at, updated_at]
      properties:
        id: { type: integer, format: int64 }
        name: { type: string }
        members:
          type: array
          items: { $ref: "#/components/schemas/TemplateMember" }
        csv: { $ref: "#/components/schemas/CsvOptions" }
        seed: { type: string, enum: [materialepas, screening], nullable: true }
        resolved_keys: { type: array, items: { type: string } }
        missing:
          type: array
          items: { $ref: "#/components/schemas/TemplateMember" }
        created_at: { type: string }
        updated_at: { type: string }
    TemplateInput:
      type: object
      description: Create (name required) or sparse edit.
      properties:
        name: { type: string }
        members:
          type: array
          items: { $ref: "#/components/schemas/TemplateMember" }
        csv: { $ref: "#/components/schemas/CsvOptions" }
```

  A delimiter of `"\t"` in a YAML flow sequence is a real TAB inside double quotes. That is valid YAML.

- [ ] **Step 5: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests rux && cd build && ctest --output-on-failure --parallel $(nproc) -R 'GuiTemplates_|GuiResources_|EndpointTable|EndpointsJson|RunningServer_|gui_api_contract_parses'`
Expected: all pass.

- [ ] **Step 6: Commit.**

```bash
git add apps/rux/include/gui/resources.hpp apps/rux/src/gui/resources.cpp apps/rux/src/gui/api.cpp apps/rux/src/gui/Server.cpp docs/gui/openapi.yaml tests/unit/rux_gui
git commit -m "feat(gui): templates API — CRUD, duplicate, restore seeds

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 11: PDF report — Ressourcetabel section and the template request field

**Files:**
- Modify: `libs/reusex/include/core/report_generator.hpp`, `libs/reusex/src/core/report_generator.cpp`, `apps/rux/resources/report.typ`
- Modify: `apps/rux/include/gui/edits.hpp:85-92`, `apps/rux/src/gui/edits.cpp` (`generate_report_pdf_json`), `apps/rux/src/gui/Server.cpp` (report POST), `docs/gui/openapi.yaml`
- Test: `tests/unit/core/test_report_survey.cpp`, `tests/unit/rux_gui/test_gui_reports.cpp`

**Interfaces:**
- Consumes: Task 8 `resource_report_section`.
- Produces: `std::vector<std::uint8_t> generate_ressourcekortlaegning_pdf(ProjectDB &db, std::optional<std::int64_t> resource_template_id = std::nullopt);` and `nlohmann::json rux::gui::generate_report_pdf_json(reusex::ProjectDB &db, const std::string &body);`. The request body is `{"resource_template_id": int|null}`. ruxd's call (`apps/ruxd/src/handlers/reports.cpp:71`) is unchanged and uses the default.

- [ ] **Step 1: Write the failing tests.** Append to `tests/unit/core/test_report_survey.cpp`, and add `#include <core/resource_templates.hpp>` to its includes:

```cpp
TEST_CASE("ReportPdf_UnknownResourceTemplate_ThrowsBeforeTypst",
          "[report][resources]") {
  // Checked before any typst work, so it holds on machines without typst.
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK_THROWS_AS(reusex::generate_ressourcekortlaegning_pdf(db, 999),
                  std::out_of_range);
}

TEST_CASE("ReportPdf_WithResourceTable_Compiles", "[report][typst]") {
  if (std::system("command -v typst > /dev/null 2>&1") != 0)
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t =
      add_type(db, "#panic(\"x\") *y*", core::Treatment::genanvendelse, 4,
               core::ReviewStatus::approved);
  add_part(db, "RX-001", t, 3);
  add_part(db, "RX-002", t, 5);
  int64_t screening = 0;
  for (const auto &rec : db.resource_templates())
    if (rec.seed == std::optional<std::string>("screening"))
      screening = rec.id;
  REQUIRE(screening > 0);
  // 11 screening keys: Betegnelse + 10 others -> two stacked tables.
  const auto pdf = reusex::generate_ressourcekortlaegning_pdf(db, screening);
  REQUIRE(pdf.size() > 4);
  CHECK(std::string(pdf.begin(), pdf.begin() + 4) == "%PDF");
}
```

Append to `tests/unit/rux_gui/test_gui_reports.cpp`, adding `#include <gui/edits.hpp>`, `#include <gui/api.hpp>` and `#include "../../support/temp_path.hpp"` if they are missing:

```cpp
TEST_CASE("GuiReport_TemplateField_400And404BeforeGenerating",
          "[gui][reports]") {
  reusex::test_support::TempPath tmp("test_gui_reports_template");
  reusex::ProjectDB db(tmp.path);
  auto status = [&](const std::string &body) {
    try {
      rux::gui::generate_report_pdf_json(db, body);
    } catch (const rux::gui::HttpError &e) {
      return e.status();
    }
    return 201;
  };
  CHECK(status(R"({"resource_template_id":"2"})") == 400);
  CHECK(status(R"({"resource_template_id":999})") == 404);
  CHECK(status("[]") == 400);
  CHECK(db.list_report_pdfs().empty());
}
```

- [ ] **Step 2: Run the build to verify it fails.**
Run: `cmake --build build --target reusex_unit_tests`
Expected: compile errors. `generate_ressourcekortlaegning_pdf(db, 999)` takes too many arguments, and `generate_report_pdf_json(db, body)` does not match.

- [ ] **Step 3: Extend the generator.** In `report_generator.hpp`, add `#include <optional>`. Replace the declaration and its doc with:

```cpp
/// Generate a Ressourcekortlægning PDF for the given project.
///
/// Assembles material passports, user-defined column definitions and
/// thumbnail blobs from @p db into a temporary directory, writes the
/// Typst template there, and executes:
///   typst compile report.typ out.pdf --root <tmpdir>
///
/// With @p resource_template_id the PDF also gets a Ressourcetabel: every
/// non-rejected resource through that template's keys, in tables of at
/// most 8 columns each led by Betegnelse (core::resource_report_section).
///
/// @returns Raw PDF bytes on success.
/// @throws std::out_of_range when @p resource_template_id names no
///         template (checked before any typst work).
/// @throws std::runtime_error if typst is not in PATH, data assembly fails,
///         or the compilation exits non-zero (the error text from typst is
///         included in the message).
std::vector<std::uint8_t> generate_ressourcekortlaegning_pdf(
    ProjectDB &db, std::optional<std::int64_t> resource_template_id =
                       std::nullopt);
```

In `report_generator.cpp`, add `#include "reusex/core/resource_export.hpp"` (match the file's existing include style). Add this to its anonymous namespace before `assemble_report_data`:

```cpp
nlohmann::json resources_json(const core::ResourceReportSection &s) {
  nlohmann::json tables = nlohmann::json::array();
  for (const auto &t : s.tables)
    tables.push_back({{"headers", t.headers}, {"rows", t.rows}});
  return {{"name", s.template_name}, {"tables", std::move(tables)}};
}
```

Replace `generate_ressourcekortlaegning_pdf` with:

```cpp
std::vector<std::uint8_t>
generate_ressourcekortlaegning_pdf(ProjectDB &db,
                                   std::optional<std::int64_t> template_id) {
  // Resolve the template first: an unknown id fails fast, typst or not.
  std::optional<core::ResourceReportSection> resources;
  if (template_id)
    resources = core::resource_report_section(db, *template_id);

  TempDir tmpdir;

  auto data = assemble_report_data(db, tmpdir.path);
  data["resources"] =
      resources ? resources_json(*resources) : nlohmann::json(nullptr);
  {
    std::ofstream f(tmpdir.path / "data.json");
    f << data.dump();
    if (!f.good())
      throw std::runtime_error("report_generator: failed to write data.json");
  }

  reusex::info(
      "generate_ressourcekortlaegning_pdf: invoking typst ({} materials, "
      "{} columns, {} resource table(s))",
      data.at("materials").size(), data.at("columns").size(),
      resources ? resources->tables.size() : 0);

  const auto pdf = run_typst(tmpdir.path);

  reusex::info("generate_ressourcekortlaegning_pdf: {} bytes", pdf.size());
  return pdf;
}
```

- [ ] **Step 4: Add the Typst section to both copies.** Make these two edits in `apps/rux/resources/report.typ` and **identically** inside `kTypstTemplate` in `report_generator.cpp`. The test `ReportTemplate_CopiesInSync` compares the two.
  - In the header comment's data.json structure, add this after the `"survey": {…}` entry (and its closing brace line):

```
//     "resources": null | {
//       "name": "template name",
//       "tables": [{"headers": ["Betegnelse", ...], "rows": [["...", ...]]}]
//     }
```

  - Insert this section immediately before the line `#text(size: 13pt, weight: "bold")[Materialepas]` (and the `#v(0.6cm)` above it stays where it is):

```typst
// ── Ressourcetabel: resources through a chosen template ──────────────────────

#let res = data.at("resources", default: none)
#if res != none [
  #text(size: 13pt, weight: "bold")[Ressourcetabel — #res.name]
  #v(0.2cm)
  #if res.tables.len() == 0 [
    _Ingen ressourcer i projektet._
  ] else {
    for t in res.tables {
      table(
        columns: t.headers.len(),
        stroke: 0.3pt + luma(190),
        inset: (x: 5pt, y: 5pt),
        fill: (col, row) => if row == 0 { luma(215) } else { white },
        table.header(..t.headers.map(h => [*#h*])),
        ..t.rows.flatten(),
      )
      v(0.3cm)
    }
  }
  #v(0.6cm)
]
```

  Put it after the existing `#v(0.6cm)` that follows the circularity line, so the order is: Kortlægning, Ressourcetabel (when chosen), Materialepas. Template values arrive as JSON strings and print literally. The Typst-markup type name in the compile test checks that.

- [ ] **Step 5: Change the route.** Replace `generate_report_pdf_json` in `apps/rux/src/gui/edits.cpp` with:

```cpp
json generate_report_pdf_json(reusex::ProjectDB &db, const std::string &body) {
  auto parsed = json::parse(body.empty() ? "{}" : body, nullptr,
                            /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    throw HttpError(400, "request body must be a JSON object");
  std::optional<std::int64_t> template_id;
  if (const auto it = parsed.find("resource_template_id");
      it != parsed.end() && !it->is_null()) {
    if (!it->is_number_integer())
      throw HttpError(400, "'resource_template_id' must be an integer or null");
    template_id = it->get<std::int64_t>();
    // Checked here, not left to the generator: its errors map to 500.
    if (!db.resource_template(*template_id))
      throw HttpError(404, "no template " + std::to_string(*template_id));
  }
  // Counted before generation, from the same state the PDF is built from.
  const int blocking = reusex::report_blocking_types(db);
  std::vector<std::uint8_t> pdf;
  try {
    pdf = reusex::generate_ressourcekortlaegning_pdf(db, template_id);
  } catch (const std::exception &e) {
    throw HttpError(500, std::string("PDF generation failed: ") + e.what());
  }
  // Storing stays outside the try, so a locked database still maps to 503 in
  // with_write, not 500.
  return report_version_json(
      db.add_report_pdf(pdf, "Ressourcekortlægning", blocking));
}
```

In `edits.hpp`, change the declaration to `nlohmann::json generate_report_pdf_json(reusex::ProjectDB &db, const std::string &body);`. Extend its doc with the following two lines: `/// Body: optional \`{"resource_template_id": int|null}\` — adds the Ressourcetabel section.` and `/// @throws HttpError(400) bad body/field, HttpError(404) unknown template.`

In `Server.cpp`, change the report POST lambda to `return with_write([&](reusex::ProjectDB &db) { return json_response(201, generate_report_pdf_json(db, req.body)); });`. The `[&]` capture includes `req`.

- [ ] **Step 6: Document the contract.** Add this to the `/reports/ressourcekortlaegning` `post` in `docs/gui/openapi.yaml`:

```yaml
      requestBody:
        required: false
        content:
          application/json:
            schema:
              type: object
              properties:
                resource_template_id:
                  type: integer
                  format: int64
                  nullable: true
                  description: |
                    Adds a Ressourcetabel section: every resource whose type
                    is not rejected, through this template's keys, in tables
                    of at most 8 columns each led by Betegnelse. Null or
                    absent leaves the section out.
```

Add `"400": { $ref: "#/components/responses/BadRequest" }` and `"404": { $ref: "#/components/responses/NotFound" }` to its responses. Then extend its description with one sentence: `With resource_template_id the PDF also carries a Ressourcetabel between Kortlægning and Materialepas.`

- [ ] **Step 7: Run the tests to verify they pass.**
Run: `cmake --build build --target reusex_unit_tests rux ruxd && cd build && ctest --output-on-failure --parallel $(nproc) -R 'ReportPdf_|ReportTemplate_|GuiReport_|ReportVersions|gui_api_contract_parses'`
Expected: all pass. `ReportPdf_WithResourceTable_Compiles` either passes or is SKIPPED when typst is absent. In the devshell typst is on PATH, so it must pass there. If Typst reports a syntax error, the message names the line in `report.typ`.

- [ ] **Step 8: Commit.**

```bash
git add libs/reusex/include/core/report_generator.hpp libs/reusex/src/core/report_generator.cpp apps/rux/resources/report.typ apps/rux/include/gui/edits.hpp apps/rux/src/gui/edits.cpp apps/rux/src/gui/Server.cpp docs/gui/openapi.yaml tests/unit/core/test_report_survey.cpp tests/unit/rux_gui/test_gui_reports.cpp
git commit -m "feat(report): Ressourcetabel PDF section through a chosen template

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 12: Whole-phase verification

**Files:** none new. Fix whatever this step finds, in the task's own files.

- [ ] **Step 1: Full build.**
Run: `cmake --build build` (timeout 600000; re-run on a timeout)
Expected: success, including `rux`, `ruxd`, `reusex_unit_tests`, `reusex_unit_tests_vision` and the Python bindings.

- [ ] **Step 2: Full test suite.**
Run: `cd build && ctest --output-on-failure --parallel $(nproc)`
Expected: 100% pass (GPU-tagged tests may SKIP).

- [ ] **Step 3: Contract and hygiene checks.**
Run each and expect a clean result:
- `python3 scripts/check-openapi.py` → exit 0.
- `grep -rn "part_code" libs apps/rux/src apps/rux/include docs/gui` → only `migrateToV24`/`migrateToV25` in `ProjectDB.cpp` and the `core::part_code()` RX-code generator (`survey.hpp`, `survey.cpp`, `survey_service.cpp`, `resources.cpp`).
- `grep -rn "export_templates" libs apps --include=*.cpp --include=*.hpp` → only `migrateToV21` and `moveExportTemplates` in `ProjectDB.cpp`.
- `grep -rln "On-site\|onsite" apps/rux/src apps/rux/include libs` → nothing.
- `reuse lint` if it is installed (the lint workflow runs it). Otherwise check that every new file starts with the SPDX header: `head -3` on each file created in Tasks 1–11.
- `git diff --name-only main -- apps/rux/frontend/src` → empty (no frontend source changed in this phase).

- [ ] **Step 4: The current frontend still builds and tests against the unchanged client.**
Run: `npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend test -- --run && npm --prefix apps/rux/frontend run build`
Expected: all pass. Nothing in the frontend changed. This proves that the generated types and tests do not depend on removed backend fields in a way that breaks the build.

- [ ] **Step 5: Commit any fixes** (only if Steps 1–4 required changes):

```bash
git add -A
git commit -m "fix: resources phase 1 verification fixes

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

## Notes for the later phases (frontend plans read this)

- **Phase 2 (nav + On-site frontend):** `Sample.part_code` is no longer sent by the server, and `POST /samples` ignores `part_code`/`stage`. Remove them from `api/types.ts`. `miljoe/model.ts` `takenAt` already treats a missing value as null.
- **Phase 3 (Kortlægning):** use the envelopes in ruling 11 and the `manual` field. `PATCH /resources/<code>?template=<id>` returns `{resource, siblings}`, with siblings present only after a type-scoped write. `GET /resources/keys` is a bare array, and `unit` is null when there is none. Column CRUD is at `/api/v1/resources/columns` (same body and response as `/material-columns`). Column create/rename can now 409. Delete `/material-columns` from `Server.cpp`, `api.cpp`, `test_gui_api.cpp` and `openapi.yaml`.
- **Phase 4 (Skabeloner + Rapport):** template members whose key starts with `legacy:` came from old export templates and are always in `missing`. The CSV options are exactly ruling 12, and the report request field is `resource_template_id`. When ExportPage is deleted, remove the five `/export-templates` routes from `rux gui` (Server.cpp, api.cpp, test_gui_api.cpp, openapi.yaml). The `ProjectDB::*_export_template` view stays for ruxd.
