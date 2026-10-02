<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Resources, templates and GUI navigation cleanup — design

Date: 2026-10-02 · Status: draft for review · Follows
`docs/design/gui-kortlaegning-redesign.md` (phases 1–6, merged locally).

## 1. Intent

What the user asked for:

- On-site belongs to the future mobile app. Remove it from `rux gui` **and** the
  backend.
- Remove the pages that duplicate each other between Værktøjer and the main
  section. Eksport merges into Rapport. Materialedata is retired, and Kortlægning
  takes over its job: adding materials and adding columns.
- Move Viewport into the main section.
- Add a **Skabeloner** section, so a recurring field selection isn't rebuilt
  from scratch every time.
- Start calling materials **resources** (ressourcer). Every resource is one
  instance/part, and it can carry any number of key/value pairs. The
  material passport is just one particular set of those keys.
- A **template** works like a filter over keys. It is built from whole
  categories and/or individual keys, and it is stored in the project (.rux).
  A global template library may come later.
- A key the template asks for but the resource doesn't have yet shows as
  blank. Filling it in saves the pair on the resource.
- Seed two templates:
  - **Materialepas (fuld)**: every leksikon property;
  - **Hurtig genbrugsscreening**: the fields the current Kortlægning page uses.
  The user will later supply an Excel file with another category selection.
- Rapport exports with a chosen template to CSV and to a section of the PDF.
  xlsx comes later.

**Assumptions (mine, open to correction):**

- Built-in fields that belong to the *type* (EAK, BIM7AA, behandling, enhed,
  tons) keep living on the type. In a template they read and write the
  type's value, so the approval rules and the Indberetning rules don't change.
  The user confirmed this.
- Miljøstatus is derived from the samples, so it is read-only in every view.

**Success criteria:**

- The navigation shows each job exactly once.
- A Kortlægning user can switch template, fill in blanks, add a resource,
  and add a column, without leaving the page.
- Rapport exports CSV and PDF limited to a chosen template.
- No On-site code or columns are left.
- An existing v24 project upgrades with no data loss.

## 2. Approach

**The approach is A: reuse the existing passport store and rename it at the
API and UI boundary.**

The store is already free-form key/value:

- `set_passport_property(guid, field, value)` writes
  `passport_property_values` keyed by field name.
- Leksikon fields (`property_definitions`) and user columns
  (`material_property_definitions`) already share that one table.

Inside the database the table names stay as they are (`material_passports`, …).
Everything a person sees and every new API path says "resource" or "ressource".

**Rejected alternatives:**

- **B: new tables plus migration.** This would mean rewriting MaterialEPAS
  import/export, `create attributes`, CSV and the history log, for no change a
  user can see.
- **C: a view only.** This keeps one passport shared by many instances, which
  contradicts "the part is the resource".

## 3. Navigation

| Group | Entries, in order |
|---|---|
| Sag | Overblik · Kortlægning · Viewport · Miljø & prøver · Rapport · Indberetning · Skabeloner |
| Værktøjer | Projektdata · Posegraf · Pipeline · Kørselslog · Billeder · Geometri · Instanser · Labels |

**Removed entries:** On-site (`/onsite`), Materialedata (`/materials`),
Eksport (`/export`).

**Redirects:** the old paths still resolve, so bookmarks and the cross-links in
`app/links.ts` keep working.

| Old path | Goes to |
|---|---|
| `/materials` | `/kortlaegning` |
| `/export` | `/rapport` |
| `/onsite` | `/kortlaegning` |

The new route is `/skabeloner`.

## 4. Resource model

### 4.1 Identity

**A resource is one row in `survey_parts`.** That row is either:

- backed by a scan instance (`instance_guid` set), or
- added by hand (`instance_guid` NULL).

**Its values live in exactly one passport.** Schema **v25** adds the link:
`survey_parts.passport_guid TEXT UNIQUE REFERENCES material_passports(document_guid) ON DELETE SET NULL`.

**The passport is created lazily** on the first value write. A part with no
values has no passport.

**`instance_materials` stays the pipeline's link.** Whenever an
instance-backed part gets or changes its passport, the
`(cloud, instance) → passport` row is upserted in the same transaction. That
keeps the Viewport, `create materials` and MaterialEPAS export consistent.

### 4.2 Migration v24 → v25

This runs in one transaction.

1. **Add `survey_parts.passport_guid`.**
2. **Link existing passports.** For each instance-backed part whose instance
   already has a row in `instance_materials`:
   - if no other part uses that passport, link it;
   - **if several parts share the passport, the first part, ordered by code,
     keeps the original. Each other part gets a copy** with a fresh GUID: the
     property values, plus the thumbnail if there is one. The log is not
     copied. Each copied part's `instance_materials` row is repointed to its
     copy.
   - The migration logs at `warn` how many passports were split.
3. **Leave unlinked passports alone.** A passport no part references keeps
   existing, so MaterialEPAS imports are not lost. The GUI does not list it.
   It stays reachable through the CLI and through MaterialEPAS export.
4. **Drop On-site** (§8) with `ALTER TABLE samples DROP COLUMN part_code`. The
   column is plain TEXT with no index or foreign key, so the drop works without
   rebuilding the table, and sample ids and `sample_links` are untouched.
5. **Create `templates`** and move the existing `export_templates` rows into it
   (§5.4). Then drop `export_templates`.
6. **Seed templates** (§5.3) if `templates` is empty.

### 4.3 Keys

The key catalogue is the union of three sources:

| Source | Key id | Storage | Category |
|---|---|---|---|
| Leksikon | `lex:<leksikon_guid>` | passport value | `property_definitions.category` |
| User column | `col:<material_property_definitions.id>` | passport value | "Egne felter" |
| Built-in | `sys:<name>` | survey column | "Kortlægning" |

**Built-in keys.** The scope column says what a write changes:

| Key | Label | Scope | Backing column | Editable |
|---|---|---|---|---|
| `sys:name` | Betegnelse | type | `survey_types.name` | yes |
| `sys:quantity` | Mængde | part | `survey_parts.quantity` | yes |
| `sys:unit` | Enhed | type | `survey_types.unit` | yes |
| `sys:eak` | EAK | type | `survey_types.eak_code` | yes |
| `sys:bim7aa` | BIM7AA | type | `survey_types.bim7aa_code` | yes |
| `sys:treatment` | Behandling | type | `survey_types.treatment` | yes (enum) |
| `sys:environment` | Miljøstatus | type | derived from samples | no |
| `sys:room` | Rum | part | `survey_parts.room_name` | yes |
| `sys:mass_t` | Tons | type | `survey_types.mass_t` | yes |
| `sys:note` | Note | part | `survey_parts.note` | yes |
| `sys:starred` | Vigtig | part | `survey_parts.starred` | yes |

**How passport values are stored.** Values for `lex:` and `col:` keys go
through the existing `set_passport_property`, under the same field name the
current editor uses: `name_en` for leksikon keys, and the column's display name
for user columns.

This name mapping happens in one place, the key catalogue in `core`. The API
and the frontend only ever see key ids.

**Renaming a user column.** Its stored values are renamed in the same
transaction. Today a rename silently disconnects the values; this fixes that.

### 4.4 Values API

**`GET /api/v1/resources/keys`** returns the catalogue:
`[{id, label, category, scope, data_type, unit, options, editable}]`.

**`GET /api/v1/resources?template=<id>`** returns, for each part, an object of
the form `{code, type_id, values: {<key id>: string|null}}`.

- `values` holds exactly the template's resolved keys.
- A missing value is `null`. The frontend shows null as blank.
- Without `template`, `values` holds every key the resource actually has.

**`PATCH /api/v1/resources/<code>`** takes
`{values: {<key id>: string|null}}`, and the rules are:

- Writes are routed by scope.
- `null` clears the value, and clearing an absent value succeeds.
- Writing a key with `editable: false` returns **400**.
- Enum validation and number validation are the same rules the survey routes
  already use.
- Type-scoped writes change every part of that type. The response includes the
  type's other parts, so the client can refresh them.
- The handler validates everything first and writes second, inside one
  transaction. That is the same pattern `patch_material` uses.

**`POST /api/v1/resources`** takes `{type_id, name?}`. It creates a manual part
with the next code for that type, and the response is the new resource.

**`DELETE /api/v1/resources/<code>`** works only on a manual part; on an
instance-backed part it returns **409**, because those come from the scan. It
also removes the passport, unless the passport is linked elsewhere.

**Column definitions** keep their existing endpoints, renamed to
`/api/v1/resources/columns`. The old `/api/v1/materials/columns` path is
removed.

**The other `/api/v1/materials*` routes stay.** The Instanser tool page's
`InstanceList` still uses them to link and view an instance's passport.
`MaterialTable.tsx` and `MaterialsPage.tsx` are deleted. `PeekPanel.tsx` is
kept only if something other than `MaterialTable` imports it. The CLI is not
affected. `docs/gui/openapi.yaml` is updated to match.

## 5. Templates

### 5.1 Table

```sql
CREATE TABLE templates (
  id         INTEGER PRIMARY KEY AUTOINCREMENT,
  name       TEXT NOT NULL UNIQUE,
  members    TEXT NOT NULL DEFAULT '[]',  -- JSON, ordered
  csv        TEXT NOT NULL DEFAULT '{}',  -- JSON: delimiter, header style, …
  seed       TEXT,                        -- 'materialepas' | 'screening' | NULL
  created_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),
  updated_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
);
```

**Members.** Each member is either `{"category": "<name>"}` or
`{"key": "<key id>"}`.

### 5.2 Resolution

`resolve_template(members) → ordered key ids` works through the members in
order.

- **A category member** expands to every key currently in that category, in
  catalogue order.
- **A key member** adds that one key.
- **Duplicates** keep their first position.
- **A member whose key or category no longer exists** is skipped and reported
  in the API response as `missing: [...]`. The Skabeloner editor shows those
  members struck through, with a remove action.

Because categories are resolved live, a new leksikon key or a new user column
in that category shows up without anyone editing the template. Resolution lives
in `core` and is unit-tested there.

### 5.3 Seeds

| Name | Seed tag | Members |
|---|---|---|
| Materialepas (fuld) | `materialepas` | one category member per distinct `property_definitions.category`, in leksikon order |
| Hurtig genbrugsscreening | `screening` | `sys:name, sys:quantity, sys:unit, sys:eak, sys:bim7aa, sys:treatment, sys:environment, sys:room, sys:mass_t, sys:note, sys:starred` |

- **When seeding runs:** in the v25 migration, and when a project is created,
  in both cases only if `templates` is empty.
- **Seeds are ordinary rows**, so they can be renamed, edited and deleted.
- **Seeds come back only by hand.** Nothing re-seeds a project in which the
  user deleted them. The Skabeloner page offers "Gendan standardskabeloner",
  which inserts any missing seed tag.
- **The Excel-supplied selection** will later become a third seed, or an
  import. That is out of scope here.

### 5.4 Migrating `export_templates`

- **Each `export_templates` row becomes a `templates` row.**
  - Its CSV column list becomes `key` members. Columns are matched by display
    name against the catalogue; the ones that don't match go into
    `missing` and are logged at `warn`.
  - Its remaining config becomes `csv`.
- **Name clashes with a seed** get the suffix " (eksport)".

### 5.5 API

| Method and path | Does |
|---|---|
| `GET /api/v1/templates` | list, each with `resolved_keys` and `missing` |
| `POST /api/v1/templates` | `{name, members?, csv?}` |
| `PATCH /api/v1/templates/<id>` | name / members / csv |
| `DELETE /api/v1/templates/<id>` | delete |
| `POST /api/v1/templates/<id>/duplicate` | copy as "<name> (kopi)", with a numeric suffix until unique |
| `POST /api/v1/templates/restore-seeds` | re-insert missing seeds |

A duplicate name returns **409**.

## 6. Frontend

### 6.1 Kortlægning

**Template picker** (`SelectDropdown`), in the table head. It defaults to the
`screening` seed, falls back to the first template, and is remembered per
project in `localStorage`.

**The parts table builds its columns from the template:**

- Columns are `resolved_keys`, in order. The type grouping, review actions and
  filters stay as they are.
- **Cells are editable inline** using the existing editor building blocks:
  `editorKeys`, `useTextDraft`, `useMutationQueue` and the app write chain.
- **The editor widget depends on the key's `data_type`:** text, number, enum,
  boolean or date.
- **Blank cells** show an empty field with a muted placeholder `—`.
- **Type-scoped columns** have a small "type" marker in the header. The tooltip
  reads "Gælder alle dele af typen". After a save, the type's other rows
  update.
- **Read-only keys** (`sys:environment`) are not editable.

**"Tilføj ressource"** opens a small dialog: choose the type, with an optional
name. It creates a manual part. A manual part has a "Manuel" marker, and only a
manual part has "Slet ressource".

**"Tilføj kolonne"** opens a dialog asking for name, type and options. It
creates the user column and appends a key member to the selected template.

**If the selected template is a seed,** the dialog says the column will be
added to that seed. It offers "Opret kopi af skabelonen i stedet", which
duplicates the template first.

**The detail panel gains "Alle egenskaber".** It lists every key the resource
has a value for, grouped by category and collapsible, and every value there is
editable.

The thumbnail and peek from Materialedata are **not** carried over: Kortlægning
already shows evidence photos (phase 3).

### 6.2 Skabeloner (`/skabeloner`)

**Layout:** the template list sits on the left, the editor on the right. At
phone width the two stack.

**List actions:** new, rename, duplicate, delete (with a confirmation), and
"Gendan standardskabeloner".

**Editor sections:**

- **Kategorier:** a checkbox per category, with its key count.
- **Enkelte felter:** search the catalogue and add keys.
- **Rækkefølge:** an ordered list of members that can be reordered by drag and
  by keyboard (up/down buttons), with a remove action on each.
- **Count line:** "N felter", the resolved count.
- **Missing members:** shown struck through.

**Saving:** the editor saves on change, through the write chain.

### 6.3 Rapport

Rapport takes over everything Eksport did.

**The PDF versions panel is unchanged,** apart from a new
**"Ressourcetabel"** option. It picks a template (or "Ingen"), and the
generated PDF then gets a section that tables the resources with that
template's columns.

- **Where it is built:** the Typst template `apps/rux/resources/report.typ`
  gains that section. The data comes from the same resolver.
- **Width:** long templates wrap their columns into several stacked tables of
  at most 8 columns each, with Betegnelse repeated in each table.

**A new "Data-eksport" panel:**

- template picker;
- CSV options (delimiter, encoding, header label vs key id), taken from the
  template's `csv` and saved back to it;
- "Download CSV".

**The CSV is built in the backend** at
`GET /api/v1/resources/export.csv?template=<id>`. It has one row per resource
and **applies the formula-injection guard**: cells starting with `= + - @`, tab
or CR are prefixed with `'`. This closes issue draft 29 for this path.

`ExportPage.tsx` is deleted.

## 7. Error handling

**Every write is validated first and written second, in one transaction.**

| Situation | Response |
|---|---|
| Unknown resource code or template id | 404 |
| Unknown key id, read-only key, bad enum or bad number | 400, naming the key |
| Duplicate template name | 409 |
| Deleting an instance-backed part | 409 |
| Backend busy | 503 busy (the existing path); the write chain retries |

**Logging rules:**

- The migration logs counts at `info`: passports linked, split, left unlinked;
  export templates moved; members that didn't match.
- Anything lost or skipped is logged at `warn` (STANDARDS §5).
- Template resolution with missing members logs `warn` once per template per
  request. It is not an error.

## 8. On-site removal

**Frontend:**

- `routes/OnsitePage.*`, `components/onsite/*`, `onsite/model.ts`,
  `test/onsite.model.test.ts`;
- the nav entry;
- the On-site helpers in `app/links.ts` and their tests;
- any `samples` field used only by On-site (`part_code`).

**Backend:**

- `samples.part_code` (dropped in v25, §4.2);
- the `part_code` and `stage` parameters of the sample-create route, which only
  On-site sends;
- the On-site samples in the dev and demo fixtures;
- the On-site note in `rux gui --help`.

**Unchanged:** Miljø still creates samples with its own fields, and the
`samples.stage` column stays because Miljø's workflow uses it.

**Docs:** CLAUDE.md's `gui` row, `docs/gui/openapi.yaml`, `docs/DIRECTION.md`
(changelog entry) and the phase spec are updated. The phase spec gets a note
that On-site moved to the mobile-app track.

`--bind` and `--allow-origin` stay, as generic options. Their help text no
longer mentions phones.

## 9. Testing

**C++ (Catch2, `reusex_unit_tests`):**

- **v24 → v25 migration on a built fixture:**
  - one shared passport split into copies, with values and thumbnail copied and
    `instance_materials` repointed;
  - a manual part;
  - an orphan passport left untouched;
  - `samples.part_code` gone, with sample ids and links preserved;
  - export templates moved, including an unmatched column and a name clash;
  - both seeds present;
  - a second open is a no-op.
- **Key catalogue:** ids, categories, scopes. Renaming a column renames its
  stored values.
- **`resolve_template`:** category expansion, dedup, order, missing members,
  and a new key appearing under an existing category member.
- **Value routing:** type versus part versus passport writes; a lazy passport
  creating its `instance_materials` row; null clears; read-only rejected.
- **CSV builder:** columns follow the template; the formula guard; quoting.

**API (the existing gui route tests):** resources GET/PATCH/POST/DELETE,
templates CRUD plus duplicate plus restore-seeds, and status codes per §7.

**Frontend (vitest, Node):**

- nav entries and redirects;
- template column building from `resolved_keys` plus the catalogue;
- the cell-value draft and save model for a blank-to-filled cell;
- Skabeloner member reorder and remove logic.

**Visual:** Kortlægning (screening and full templates), Skabeloner and Rapport,
in both themes, at desktop and phone width, via `shot.sh`. Run `token_lint`
on the changed CSS.

## 10. Out of scope

- Global or shared template library.
- xlsx export.
- The Excel category selection (it arrives later as a seed or import).
- Migrating unlinked passports into resources.
- Renaming database tables.
- Mobile On-site.

## 11. Delivery

Four phases, each merged locally on completion (standing instruction):

1. **Backend model.** v25 migration, key catalogue, resolver, resources and
   templates API, CSV builder, On-site backend removal, openapi.
2. **Navigation and On-site frontend removal.** Nav regrouping, redirects,
   deleting On-site, Materialedata and Eksport.
3. **Kortlægning.** Template picker, template-built editable columns, Tilføj
   ressource, Tilføj kolonne, Alle egenskaber.
4. **Skabeloner page and Rapport.** Data-eksport panel, Ressourcetabel PDF
   section, docs and DIRECTION changelog.
