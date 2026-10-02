<!--

> **Controller reconciliation with the Phase 1 plan (binding, overrides assumptions below):**
> `GET /api/v1/resources/keys` returns a **bare array** of keys (not `{keys}`); `unit` is null when none.
> `GET /api/v1/resources[?template=]` returns `{resources, template?: {id, resolved_keys, missing}}`; each resource has
> `{code, type_id, manual, values}`; without `template`, `values` holds **every built-in `sys:` key** plus each stored catalogue key.
> `PATCH /api/v1/resources/<code>[?template=]` returns `{resource, siblings}`. `GET /api/v1/templates` returns `{templates}`.
> Column create/rename can return 409 (duplicate or leksikon-name clash). The old column path is `/api/v1/material-columns`.
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Resources & Templates Phase 3: Kortlægning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Load the project skill `design-studio` (`.claude/skills/design-studio/SKILL.md`) before touching any `.tsx`/`.css`.

**Goal:** Kortlægning becomes the one place a surveyor works with resources. They can switch template, fill in blank cells inline, add a resource, add a column (or a copy of a seed template with the column), and edit every property of a part under "Alle egenskaber". Materialedata (`MaterialsPage`/`MaterialTable` and its component cluster) and the old `/api/v1/material-columns` backend route are retired.

**Architecture:**
- **Pure modules, tested in Node:**
  - `src/kortlaegning/resources.ts`: the columns a template builds, the model of each cell, merging resources, the "Alle egenskaber" groups.
  - `src/kortlaegning/cellDraft.ts`: how a key's value is shown, how it is edited, and when a draft is sent.
  - `src/kortlaegning/templatePick.ts`: the default template, `localStorage` per project, appending a member.
  - `src/kortlaegning/columnDraft.ts`: validating the Tilføj kolonne dialog and planning the seed copy.
- **Components** are presentational: `ResourceCell`, `FormDialog`, `AddResourceDialog`, `AddColumnDialog`, and the extended `SurveyTable` / `DetailPanel`.
- **`KortlaegningPage`** owns the state. It loads the key catalogue, the templates and **all resources without a template** once. It derives each template's cells on the client, so a template switch is instant and costs no request.
- **Every write** goes through `useMutationQueue`, on the app write chain.

**Tech Stack:** React 19, TypeScript, CSS Modules (tokens only), vitest in Node (no DOM), Crow C++ for the one backend route removal, Catch2 v3.

**Spec:** `docs/superpowers/specs/2026-10-02-resources-templates-ia-design.md` (§6.1 Kortlægning, §4.4 Values API, §5.5 Templates API, §4.4 column-route rename). Phase split: `/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad/rt-plan-context.md`.

**Preconditions:** Phase 1 (backend model, v25, `/api/v1/resources*`, `/api/v1/templates*`, `/api/v1/resources/columns`) and Phase 2 (nav regroup, On-site removed, the Materialedata nav entry gone, the `/materials` route still rendering `MaterialsPage`) are merged to `main`.

## Rulings made in this plan

These rulings are deliberate. Reviewers should check that the implementation follows them, not the alternative.

- **R1 — One resources load, no template.**
  - The page loads `GET /resources` (no `template`) once and reads a template's cells as `values[key] ?? null`.
  - Under spec §4.4 this is the same result `?template=` returns: missing values are null.
  - `api.resources(templateId?)` still takes the parameter, for Phase 4.
- **R2 — Editors only in the selected row.**
  - Every row shows formatted text; a blank shows a muted `—`.
  - Only the selected row (type or part) renders editors: inputs, selects, checkboxes. Blank fields there have the `—` placeholder.
  - This keeps a several-hundred-part survey from mounting thousands of draft hooks, and keeps the existing arrow-key and fold behaviour. A click on a row selects it; a second click lands in the field.
- **R3 — What a type row shows.**
  - **Type-scoped columns:** the value of the type's first part (by code). Edits go to that part's `PATCH /resources/<code>`, which under spec §4.4 changes every part of the type.
  - **A type with no parts:** those cells are read-only and blank.
  - **The Mængde column:** the existing aggregate (`formatQuantity` + tonnes), read-only. The aggregate stays editable in the detail panel.
  - **Other part-scoped columns:** empty.
- **R4 — The fixed columns.**
  - The first column is always the tree column (Betegnelse: chevron, name, part label). It is not inline-editable, because a click there selects or folds.
  - `sys:name` is therefore never a dynamic column; the name is edited under "Alle egenskaber".
  - The trailing **Status** column (review state) stays fixed. Review actions are not keys.
- **R5 — Which reads follow which writes.** Every refresh runs inside the same `mutate` call, so the write chain keeps them in order.
  - A resource PATCH that writes a `sys:` key re-reads `GET /survey`, because the type state behind the detail panel, filters and dialog changed.
  - A survey PATCH from the detail panel or the dialog (mængde, behandling, note, ★) re-reads `GET /resources`.
  - Create and delete re-read both.
  - Approve, reject and reopen re-read nothing new.
- **R6 — The template picker is a native `<select>`.**
  - It is styled like the table's existing filter selects. The existing `SelectDropdown` is not used: it is a Notion-style tag *creator* bound to `PropertyDefinition`, with a "Create" row and a clear `×`, and neither fits choosing a template.
  - Enum cells are also native `<select>`s.
  - `SelectDropdown.tsx` and `MultiSelectDropdown.tsx` are **kept**: the design-studio skill names them as shared components. They become unused, which is flagged in the phase exit notes.
- **R7 — What is deleted with Materialedata.**
  - Everything only `MaterialTable` imports: `PeekPanel`, `EditableCell`, `ColumnHeaderMenu`, `ColumnFilterCell`, `columnFilterHelpers` (+ its test), `SortableColumnHeader`, `tableNav`, `ThumbnailCell`, and their CSS.
  - `.claude/skills/design-studio/references/reusex-frontend.md` still lists `PeekPanel`. It is **not** edited here (agent config); this is reported in the exit notes.
- **R8 — Renaming the column client.**
  - `propertyDefinitions` / `createPropertyDefinition` / `updatePropertyDefinition` / `deletePropertyDefinition` become `resourceColumns` / `createResourceColumn` / `updateResourceColumn` / `deleteResourceColumn`, on `/resources/columns`.
  - `ExportPage` (alive until Phase 4) switches to `api.resourceColumns`.
- **R9 — ruxd is untouched.** `apps/ruxd/src/handlers/material_columns.cpp` is a separate binary that the spec does not cover. Only `rux gui`'s route, its endpoint table, the route test and `docs/gui/openapi.yaml` lose `/material-columns`.
- **R10 — Booleans travel as `"true"` / `"false"`.** Reads also accept `"1"`, `"ja"` and `"yes"`.
- **R11 — "Tilføj kolonne" on a seed has two submit buttons.**
  - **"Tilføj kolonne"** adds the column to the seed, and a note says so.
  - **"Opret kopi af skabelonen i stedet"** duplicates the template first (`POST /templates/<id>/duplicate`), appends the column to the copy, and selects the copy.
- **R12 — A new resource is always shown.** If the type it is filed under is hidden by the current tab or filters, the page switches to the Alle tab with no filters, then selects the new part.

## Global Constraints

- SPDX header on every new file: `// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `// SPDX-License-Identifier: GPL-3.0-or-later`. CSS uses the `/* … */` block form.
- **Tokens only.** No literal colour, radius, spacing or font-size in `.module.css`, inline `style`, or TS constants; only `var(--…)`.
  - Never edit `tokens.css`.
  - Check with `python .claude/skills/design-studio/scripts/token_lint.py <changed css> --tsx`.
- Reuse the shared rules:
  - `components/controls.module.css` (`btnGhost`, `btnPrimary`, `btnDanger`, `input`, `checkbox`, `field`, `fieldLabel`) via `composes:`;
  - the `scrim` from `EditDialog.module.css`;
  - `Pill` for miljøstatus and behandling.
- All UI copy is Danish. Key ids (`sys:`, `lex:`, `col:`) never appear in the UI; the label does.
- Wire contract: spec §4.4 and §5.5 exactly, as Phase 1 documented them in `docs/gui/openapi.yaml`. Task 1 Step 1 checks the response envelopes against that file. If an envelope or field name differs, change **only** the client unwrapping and the `types.ts` shape in Task 1; every later task consumes the typed client.
- No DOM test environment may be added: vitest runs in Node. Testable logic is a pure exported function. Components are verified by screenshot (`shot.sh`) in **both themes**, at **desktop and phone** width.
- Frontend commands run from the worktree root:
  - `npm --prefix apps/rux/frontend run typecheck`
  - `npm --prefix apps/rux/frontend test -- <file>`
  - `npm --prefix apps/rux/frontend run build`
- **Backend build** (Task 11 only), inside the worktree with the devshell active:

  ```bash
  cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
  cmake --build build --target reusex_unit_tests rux
  ```

  - Run builds in the foreground with a 600000 ms timeout, and re-run the same command on a timeout (ccache resumes it).
  - Never touch `/home/mephisto/repos/ReUseX/build`.
- **Workspace:**
  - Worktree: `/home/mephisto/repos/ReUseX/.worktrees/rt-phase3`, branch `rt-phase3-kortlaegning`.
  - Dev servers: gui port **8433**, vite port **5183**.
  - Scratch: `SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad`.
  - `$SP/corridor-clouds.rux` is the source project. Never serve a tracked fixture in place.
- Every commit message ends with these two lines. Never use `--no-verify`.

  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf
  ```

## Review Focus

- **A blank cell filled and then emptied again.**
  - Typing `12,5` into a blank number cell must send `"12.5"`.
  - Clearing it must send `null`.
  - Focusing and leaving a blank or filled cell untouched must send **nothing**.
  - A draft that will not parse (`abc` in a number cell) must snap back and toast, never send.
  - Pinned by `cellCommit` tests in Task 3.
- **A type-scoped edit from a part row.** Changing Behandling on `RX-008` must update `RX-009`'s cell, the type row and the detail panel (R3, R5). Pinned by `replaceResources` / `patchedResources` tests in Task 2 and by the Task 6 browser check.
- **A stale or vanished template.**
  - A stored template id that was deleted, a template whose `resolved_keys` names a key the catalogue lacks, and an empty template list must each still render the table.
  - The fallbacks are: screening seed → first template → no dynamic columns.
  - Pinned by the `pickTemplate` / `templateColumns` tests in Tasks 2 and 4.
- **A double tap on "Tilføj ressource".** It must create exactly one part; every create burns a server-assigned code. `createOnceGuard` is used in Task 7 and checked in its browser step.
- **A Tilføj kolonne name that already exists.** It must be refused in the dialog before any request. If the column is created but appending it to the template fails, the toast must say the column exists but was not added. Pinned by `columnDraftError` in Task 5 and the partial-failure copy in Task 8.

---

## File Structure

| File | Responsibility |
|---|---|
| `apps/rux/frontend/src/api/types.ts` (modify) | `ResourceKey`, `Resource`, `ResourcePatchResult`, `ResourceCreate`, `ResourceColumnCreate`, `TemplateMember`, `TemplateSeed`, `TemplateCsv`, `Template`, `TemplatePatch` |
| `apps/rux/frontend/src/api/client.ts` (modify) | resources, keys, templates, duplicate/patch template, resource columns (renamed) |
| `apps/rux/frontend/src/routes/ExportPage.tsx` (modify, 1 line) | `api.resourceColumns` |
| `apps/rux/frontend/src/test/resources.client.test.ts` (new) | client paths, verbs, bodies, unwrapping |
| `apps/rux/frontend/src/test/resourceFixtures.ts` (new) | `resourceKey`, `resource`, `template` builders |
| `apps/rux/frontend/src/kortlaegning/resources.ts` (new) + `src/test/kortlaegning.resources.test.ts` | columns, cell model, merging, Alle egenskaber groups, new-resource view |
| `apps/rux/frontend/src/kortlaegning/cellDraft.ts` (new) + `src/test/kortlaegning.cellDraft.test.ts` | display, input text, validate, commit, booleans, enum options |
| `apps/rux/frontend/src/kortlaegning/templatePick.ts` (new) + `src/test/kortlaegning.templatePick.test.ts` | default template, storage per project, `appendKeyMember` |
| `apps/rux/frontend/src/kortlaegning/columnDraft.ts` (new) + `src/test/kortlaegning.columnDraft.test.ts` | Tilføj kolonne validation, options parsing, create body, seed plan |
| `apps/rux/frontend/src/components/kortlaegning/ResourceCell.tsx` + `.module.css` (new) | one key's display/editor |
| `apps/rux/frontend/src/components/kortlaegning/SurveyTable.tsx` + `.module.css` (modify) | template picker, toolbar actions, dynamic columns, Manuel marker |
| `apps/rux/frontend/src/components/kortlaegning/FormDialog.tsx` + `.module.css` (new) | small modal form shell |
| `apps/rux/frontend/src/components/kortlaegning/AddResourceDialog.tsx` (new) | type + optional name |
| `apps/rux/frontend/src/components/kortlaegning/AddColumnDialog.tsx` (new) | name, kind, options, seed copy |
| `apps/rux/frontend/src/components/kortlaegning/DetailPanel.tsx` + `.module.css` (modify) | Slet ressource, Alle egenskaber |
| `apps/rux/frontend/src/routes/KortlaegningPage.tsx` (modify) | state, loads, mutations, dialogs |
| `apps/rux/frontend/src/app/App.tsx` (modify) | `/materials` → `/kortlaegning` redirect |
| deletions (Task 10) | `MaterialsPage`, `MaterialTable`, `PeekPanel`, `EditableCell`, `ColumnHeaderMenu`, `ColumnFilterCell`, `columnFilterHelpers`, `SortableColumnHeader`, `tableNav`, `ThumbnailCell` (+css), `test/columnFilter.test.ts` |
| `apps/rux/src/gui/Server.cpp`, `apps/rux/src/gui/api.cpp`, `tests/unit/rux_gui/test_gui_api.cpp`, `docs/gui/openapi.yaml` (modify) | drop `/api/v1/material-columns*` |

---

### Task 0: Worktree

- [ ] **Step 1: Create the worktree from main**

```bash
cd /home/mephisto/repos/ReUseX
git worktree add .worktrees/rt-phase3 -b rt-phase3-kortlaegning main
cd .worktrees/rt-phase3 && direnv allow . && npm --prefix apps/rux/frontend ci
```

- [ ] **Step 2: Confirm the preconditions**

```bash
grep -n "resources/keys\|/resources/{code}\|/resources/columns\|/templates/{id}/duplicate" docs/gui/openapi.yaml | head
grep -n "Materialedata\|/onsite" apps/rux/frontend/src/app/navigation.ts apps/rux/frontend/src/app/App.tsx
```

Expected:
- The first grep finds all four paths (Phase 1).
- `navigation.ts` has no `Materialedata` entry (Phase 2).
- `App.tsx` still has `<Route path="/materials" element={<MaterialsPage />} />`.

If any of these fails, stop and report: the preconditions are not met.

---

### Task 1: API types and client for resources, keys, templates and columns

**Files:**
- Modify: `apps/rux/frontend/src/api/types.ts` (append a section after `SamplePatch`, ~line 1010)
- Modify: `apps/rux/frontend/src/api/client.ts` (replace the `material columns` block at lines 881–911; add `resources` and `templates` blocks before `// websocket`)
- Modify: `apps/rux/frontend/src/routes/ExportPage.tsx:85`
- Create: `apps/rux/frontend/src/test/resources.client.test.ts`

**Interfaces:**
- Produces (types):
  - `ResourceKeyScope`, `ResourceDataType`, `ResourceKey`
  - `Resource`, `ResourcePatchResult`, `ResourceCreate`, `ResourceColumnCreate`
  - `TemplateMember`, `TemplateSeed`, `TemplateCsv`, `Template`, `TemplatePatch`
- Produces (client methods on `RuxApiClient`):
  - `resourceKeys(signal?): Promise<ResourceKey[]>`
  - `resources(templateId?: number, signal?): Promise<Resource[]>`
  - `patchResource(code: string, values: Record<string, string | null>): Promise<ResourcePatchResult>`
  - `createResource(body: ResourceCreate): Promise<Resource>`
  - `deleteResource(code: string): Promise<void>`
  - `resourceColumns(signal?): Promise<PropertyDefinition[]>`
  - `createResourceColumn(body: ResourceColumnCreate): Promise<PropertyDefinition>`
  - `updateResourceColumn(id: string, patch: Partial<ResourceColumnCreate>): Promise<PropertyDefinition>`
  - `deleteResourceColumn(id: string): Promise<void>`
  - `templates(signal?): Promise<Template[]>`
  - `duplicateTemplate(id: number): Promise<Template>`
  - `patchTemplate(id: number, patch: TemplatePatch): Promise<Template>`

  Phase 4 adds `createTemplate`, `deleteTemplate` and `restoreSeedTemplates` next to these.

- [ ] **Step 1: Read the Phase 1 contract and note the envelopes**

```bash
awk '/^  \/resources/,/^  \/[a-qs-z]/' docs/gui/openapi.yaml | grep -n "schema\|properties\|required\|\$ref" | head -60
awk '/^  \/templates/,/^  \/[a-su-z]/' docs/gui/openapi.yaml | grep -n "schema\|properties\|required\|\$ref" | head -60
grep -n "    ResourceKey:\|    Resource:\|    Template:\|    TemplateMember:\|    ResourcePatchResult:" -A24 docs/gui/openapi.yaml
```

This plan assumes:
- `GET /resources/keys` → `{keys: ResourceKey[]}`;
- `GET /resources` → `{resources: Resource[]}`;
- `PATCH /resources/{code}` → `{resource: Resource, siblings: Resource[]}`, where `siblings` holds the type's other parts;
- `POST /resources` → `201 Resource`;
- `GET /templates` → `{templates: Template[]}`;
- `POST /templates/{id}/duplicate` and `PATCH /templates/{id}` → `Template`;
- `Template.missing` → an array of members.

If the file says otherwise, use the file's names in the types below and in the `requestJson<…>` unwrapping, and adjust the test payloads to match. The method signatures listed under **Interfaces** do not change.

- [ ] **Step 2: Write the failing client test**

Create `apps/rux/frontend/src/test/resources.client.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Resource / template client methods: paths, verbs, bodies and envelope
 * unwrapping, against the Phase 1 contract (docs/gui/openapi.yaml §resources,
 * §templates).
 */

import { describe, expect, it } from 'vitest';

import { RuxApiClient, type FetchLike } from '../api/client';

function client(payload: unknown, status = 200) {
  const calls: { url: string; method?: string; body?: string; headers?: Record<string, string> }[] = [];
  const fetchLike: FetchLike = (url, init) => {
    calls.push({ url, method: init?.method, body: init?.body as string | undefined, headers: init?.headers });
    return Promise.resolve(
      new Response(status === 204 ? null : JSON.stringify(payload), {
        status,
        headers: { 'Content-Type': 'application/json' },
      }),
    );
  };
  return { calls, api: new RuxApiClient({ baseUrl: '/api/v1', fetch: fetchLike }) };
}

describe('resources client', () => {
  it('reads the key catalogue out of its envelope', async () => {
    const key = { id: 'sys:eak', label: 'EAK', category: 'Kortlægning', scope: 'type', data_type: 'text', unit: null, options: [], editable: true };
    const { calls, api } = client({ keys: [key] });
    expect(await api.resourceKeys()).toEqual([key]);
    expect(calls[0]).toMatchObject({ url: '/api/v1/resources/keys', method: 'GET' });
  });

  it('reads resources with and without a template', async () => {
    const { calls, api } = client({ resources: [{ code: 'RX-001', type_id: 2, values: {} }] });
    expect(await api.resources()).toHaveLength(1);
    await api.resources(7);
    expect(calls.map((c) => c.url)).toEqual(['/api/v1/resources', '/api/v1/resources?template=7']);
  });

  it('patches values sparsely, null meaning clear', async () => {
    const { calls, api } = client({ resource: { code: 'RX-001', type_id: 2, values: {} }, siblings: [] });
    const out = await api.patchResource('RX-001', { 'sys:note': 'ok', 'col:3': null });
    expect(calls[0]).toMatchObject({ url: '/api/v1/resources/RX-001', method: 'PATCH' });
    expect(calls[0].headers?.['Content-Type']).toBe('application/json');
    expect(JSON.parse(calls[0].body!)).toEqual({ values: { 'sys:note': 'ok', 'col:3': null } });
    expect(out.siblings).toEqual([]);
  });

  it('url-encodes the resource code', async () => {
    const { calls, api } = client({ resource: { code: 'a/b', type_id: 1, values: {} }, siblings: [] });
    await api.patchResource('a/b', {});
    expect(calls[0].url).toBe('/api/v1/resources/a%2Fb');
  });

  it('creates a manual resource and deletes one expecting 204', async () => {
    const { calls, api } = client({ code: 'RX-019', type_id: 2, values: {} }, 201);
    expect((await api.createResource({ type_id: 2, name: 'Ekstra søjle' })).code).toBe('RX-019');
    expect(JSON.parse(calls[0].body!)).toEqual({ type_id: 2, name: 'Ekstra søjle' });
    const del = client(null, 204);
    await del.api.deleteResource('RX-019');
    expect(del.calls[0]).toMatchObject({ url: '/api/v1/resources/RX-019', method: 'DELETE' });
  });

  it('manages resource columns on the renamed path', async () => {
    const { calls, api } = client([]);
    await api.resourceColumns();
    await api.createResourceColumn({ name: 'Stand', type: 'select', options: ['God', 'Dårlig'] });
    await api.updateResourceColumn('4', { options: ['God'] });
    expect(calls.map((c) => `${c.method} ${c.url}`)).toEqual([
      'GET /api/v1/resources/columns',
      'POST /api/v1/resources/columns',
      'PATCH /api/v1/resources/columns/4',
    ]);
    const del = client(null, 204);
    await del.api.deleteResourceColumn('4');
    expect(del.calls[0]).toMatchObject({ url: '/api/v1/resources/columns/4', method: 'DELETE' });
  });
});

describe('templates client', () => {
  const tpl = {
    id: 3, name: 'Hurtig genbrugsscreening', members: [{ key: 'sys:name' }], csv: {}, seed: 'screening',
    resolved_keys: ['sys:name'], missing: [], created_at: '', updated_at: '',
  };

  it('lists templates out of their envelope', async () => {
    const { calls, api } = client({ templates: [tpl] });
    expect(await api.templates()).toEqual([tpl]);
    expect(calls[0].url).toBe('/api/v1/templates');
  });

  it('duplicates and patches a template', async () => {
    const { calls, api } = client(tpl);
    await api.duplicateTemplate(3);
    await api.patchTemplate(3, { members: [{ key: 'sys:name' }, { key: 'col:4' }] });
    expect(calls.map((c) => `${c.method} ${c.url}`)).toEqual([
      'POST /api/v1/templates/3/duplicate',
      'PATCH /api/v1/templates/3',
    ]);
    expect(JSON.parse(calls[1].body!)).toEqual({ members: [{ key: 'sys:name' }, { key: 'col:4' }] });
  });
});
```

- [ ] **Step 3: Run it to verify it fails**

Run: `npm --prefix apps/rux/frontend test -- src/test/resources.client.test.ts`
Expected: FAIL with `api.resourceKeys is not a function` (and type errors for the others).

- [ ] **Step 4: Add the wire types**

Append to `apps/rux/frontend/src/api/types.ts` after `SamplePatch`:

```ts
// ------------------------------------------------- resources & templates ----

/** Where a key's write lands (spec §4.3): `type` changes every part of the type. */
export type ResourceKeyScope = 'type' | 'part';

/** Which editor a key gets in Kortlægning (spec §6.1). */
export type ResourceDataType = 'text' | 'number' | 'enum' | 'boolean' | 'date';

/**
 * `ResourceKey` — one row of `GET /resources/keys`. `id` is `lex:<guid>`
 * (leksikon), `col:<id>` (user column) or `sys:<name>` (built-in); the UI
 * shows `label`, never the id.
 */
export interface ResourceKey {
  id: string;
  label: string;
  category: string;
  scope: ResourceKeyScope;
  data_type: ResourceDataType;
  unit: string | null;
  /** Allowed values of an `enum` key, wire values (e.g. `genbrug`). */
  options: string[];
  /** False for derived keys (`sys:environment`): a write is a 400. */
  editable: boolean;
}

/** `Resource` — one survey part's values by key id. `null` = no value. */
export interface Resource {
  code: string;
  type_id: number;
  values: Record<string, string | null>;
}

/** Response of `PATCH /resources/{code}`: the resource, plus the type's other parts after a type-scoped write. */
export interface ResourcePatchResult {
  resource: Resource;
  siblings: Resource[];
}

/** Body of `POST /resources` — a manual part under an existing type. */
export interface ResourceCreate {
  type_id: number;
  name?: string;
}

/** Body of `POST /resources/columns` (a `PropertyDefinition` without its id). */
export interface ResourceColumnCreate {
  name: string;
  type: PropertyType;
  options?: string[];
  sort_order?: number;
  width?: number;
}

/** A template member: a whole category, or one key (spec §5.1). */
export type TemplateMember = { category: string } | { key: string };

/** The tag of a seeded template (spec §5.3). */
export type TemplateSeed = 'materialepas' | 'screening';

/** CSV export options saved on a template (Phase 4 edits them). */
export interface TemplateCsv {
  delimiter?: string;
  encoding?: string;
  header?: 'label' | 'key';
}

/** `Template` — one row of `GET /templates`, resolved against the live catalogue. */
export interface Template {
  id: number;
  name: string;
  members: TemplateMember[];
  csv: TemplateCsv;
  seed: TemplateSeed | null;
  /** Ordered key ids the members resolve to now (spec §5.2). */
  resolved_keys: string[];
  /** Members whose key or category no longer exists. */
  missing: TemplateMember[];
  created_at: string;
  updated_at: string;
}

/** Body of `PATCH /templates/{id}` — sparse. */
export interface TemplatePatch {
  name?: string;
  members?: TemplateMember[];
  csv?: TemplateCsv;
}
```

- [ ] **Step 5: Replace the material-columns block and add resources and templates to the client**

In `client.ts`, extend the `import type { … } from './types'` list with `Resource, ResourceColumnCreate, ResourceCreate, ResourceKey, ResourcePatchResult, Template, TemplatePatch`.

Then replace the whole `// ------------------------------------------------- material columns ----` block (lines 881–911) with:

```ts
  // ------------------------------------------------- resource columns ----

  /** All user-defined resource column definitions (`col:<id>` keys). */
  resourceColumns(signal?: AbortSignal): Promise<PropertyDefinition[]> {
    return this.requestJson<PropertyDefinition[]>('/resources/columns', undefined, signal);
  }

  /** Create a user column. Returns the created definition; its key id is `col:<id>`. */
  createResourceColumn(body: ResourceColumnCreate): Promise<PropertyDefinition> {
    return this.postJson<PropertyDefinition>('/resources/columns', body);
  }

  /** Sparsely update a user column. A rename also renames its stored values (spec §4.3). */
  updateResourceColumn(id: string, patch: Partial<ResourceColumnCreate>): Promise<PropertyDefinition> {
    return this.patchJson<PropertyDefinition>(`/resources/columns/${encodeURIComponent(id)}`, patch);
  }

  /** Delete a user column. The server answers 204 (no body). */
  async deleteResourceColumn(id: string): Promise<void> {
    await this.deleteNoContent(`/resources/columns/${encodeURIComponent(id)}`);
  }

  // -------------------------------------------------------- resources ----

  /** The key catalogue: leksikon, user columns and built-ins (spec §4.3). */
  async resourceKeys(signal?: AbortSignal): Promise<ResourceKey[]> {
    const body = await this.requestJson<{ keys: ResourceKey[] }>('/resources/keys', undefined, signal);
    return body.keys;
  }

  /**
   * Every resource. With `templateId`, `values` holds exactly that template's
   * resolved keys (missing = null); without, every key the resource has.
   */
  async resources(templateId?: number, signal?: AbortSignal): Promise<Resource[]> {
    const body = await this.requestJson<{ resources: Resource[] }>(
      '/resources',
      templateId === undefined ? undefined : { template: templateId },
      signal,
    );
    return body.resources;
  }

  /**
   * Set or clear values (`null` clears; clearing an absent value succeeds).
   * Writes route by key scope; a type-scoped write changes every part of the
   * type, and those parts come back in `siblings`. 400 names a bad key/value.
   */
  patchResource(code: string, values: Record<string, string | null>): Promise<ResourcePatchResult> {
    return this.patchJson<ResourcePatchResult>(`/resources/${encodeURIComponent(code)}`, { values });
  }

  /** Create a manual part with the next code for the type. */
  createResource(body: ResourceCreate): Promise<Resource> {
    return this.postJson<Resource>('/resources', body);
  }

  /** Delete a manual part (an instance-backed one is a 409). 204, no body. */
  async deleteResource(code: string): Promise<void> {
    await this.deleteNoContent(`/resources/${encodeURIComponent(code)}`);
  }

  // -------------------------------------------------------- templates ----

  /** Every template, each resolved (`resolved_keys`, `missing`). */
  async templates(signal?: AbortSignal): Promise<Template[]> {
    const body = await this.requestJson<{ templates: Template[] }>('/templates', undefined, signal);
    return body.templates;
  }

  /** Copy a template as "<name> (kopi)" (numeric suffix until unique). */
  duplicateTemplate(id: number): Promise<Template> {
    return this.postJson<Template>(`/templates/${id}/duplicate`);
  }

  /** Sparse edit of name / members / csv. A duplicate name is a 409. */
  patchTemplate(id: number, patch: TemplatePatch): Promise<Template> {
    return this.patchJson<Template>(`/templates/${id}`, patch);
  }
```

Then add the shared DELETE helper next to `patchJson` (inside the class, after it):

```ts
  /** A DELETE that answers 204 with no body. */
  private async deleteNoContent(path: string): Promise<void> {
    const url = this.url(path);
    const response = await this.doFetch(url, { method: 'DELETE' });
    if (!response.ok) {
      throw new ApiRequestError(response.status, await describeFailure(response), url);
    }
  }
```

Leave the existing `deleteMaterial`, `deleteSample` and `deleteExportTemplate` bodies alone (no drive-by refactor).

- [ ] **Step 6: Point ExportPage at the renamed method**

In `apps/rux/frontend/src/routes/ExportPage.tsx:85` change `(signal) => api.propertyDefinitions(signal),` to `(signal) => api.resourceColumns(signal),`.

Then check for remaining callers of the old names:

```bash
grep -rn "propertyDefinitions\|createPropertyDefinition\|updatePropertyDefinition\|deletePropertyDefinition" apps/rux/frontend/src
```

Expected: hits only in `components/MaterialTable.tsx`, plus doc comments in `SelectDropdown.tsx` / `MultiSelectDropdown.tsx`. In `MaterialTable.tsx`, rename the calls to the new names so typecheck passes until Task 10 deletes the file:
- `api.propertyDefinitions(` → `api.resourceColumns(`
- `api.createPropertyDefinition(` → `api.createResourceColumn(`
- `api.updatePropertyDefinition(` → `api.updateResourceColumn(`
- `api.deletePropertyDefinition(` → `api.deleteResourceColumn(`

In the two dropdown doc comments, change `api.updatePropertyDefinition` to `api.updateResourceColumn`.

- [ ] **Step 7: Run tests and typecheck**

Run: `npm --prefix apps/rux/frontend test -- src/test/resources.client.test.ts && npm --prefix apps/rux/frontend run typecheck`
Expected: PASS, no type errors.

- [ ] **Step 8: Commit**

```bash
git add apps/rux/frontend/src/api/types.ts apps/rux/frontend/src/api/client.ts apps/rux/frontend/src/routes/ExportPage.tsx apps/rux/frontend/src/components/MaterialTable.tsx apps/rux/frontend/src/components/SelectDropdown.tsx apps/rux/frontend/src/components/MultiSelectDropdown.tsx apps/rux/frontend/src/test/resources.client.test.ts
git commit -m "feat(gui): resources, keys, templates and resource-columns client

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 2: Template columns, cell model and resource merging (pure)

**Files:**
- Create: `apps/rux/frontend/src/test/resourceFixtures.ts`
- Create: `apps/rux/frontend/src/kortlaegning/resources.ts`
- Test: `apps/rux/frontend/src/test/kortlaegning.resources.test.ts`

**Interfaces:**
- Consumes: the `ResourceKey`, `Resource`, `ResourcePatchResult` and `Template` types from Task 1; `Row`, `Tab`, `Filters`, `NO_FILTERS`, `visibleTypes` from `kortlaegning/model.ts`; `SurveyType`, `SurveyPart`.
- Produces:
  - `interface ResourceColumn { key: ResourceKey; typeScoped: boolean }`
  - `FIXED_COLUMN_KEYS: ReadonlySet<string>`
  - `templateColumns(template: Template | null, catalogue: readonly ResourceKey[]): ResourceColumn[]`
  - `type ResourceIndex = ReadonlyMap<string, Resource>`; `resourceIndex(list: readonly Resource[]): ResourceIndex`
  - `valueOf(index: ResourceIndex, code: string, keyId: string): string | null`
  - `typeCarrier(type: SurveyType): string | null`
  - `interface CellModel { kind: 'value' | 'aggregate' | 'none'; value: string | null; target: string | null }`
  - `cellModel(row: Row, type: SurveyType, column: ResourceColumn, index: ResourceIndex): CellModel`
  - `patchedResources(result: ResourcePatchResult): Resource[]`
  - `replaceResources(list: readonly Resource[], updated: readonly Resource[]): Resource[]`
  - `touchesSurvey(keyIds: readonly string[]): boolean`
  - `isManual(part: SurveyPart): boolean`
  - `resourceColumnKeyId(columnId: string): string`
  - `interface PropertyGroup { category: string; keys: ResourceKey[] }`
  - `allPropertyGroups(resource: Resource | null, catalogue: readonly ResourceKey[]): PropertyGroup[]`
  - `addableTypes(types: readonly SurveyType[]): SurveyType[]`
  - `viewForNewResource(types: SurveyType[], typeId: number, tab: Tab, filters: Filters): { tab: Tab; filters: Filters }`

- [ ] **Step 1: Write the fixtures**

Create `apps/rux/frontend/src/test/resourceFixtures.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/** Builders for resource keys, resources and templates in the Phase 3 (resources) tests. */

import type { Resource, ResourceKey, Template } from '../api/types';

export function resourceKey(over: Partial<ResourceKey> = {}): ResourceKey {
  return {
    id: 'sys:note',
    label: 'Note',
    category: 'Kortlægning',
    scope: 'part',
    data_type: 'text',
    unit: null,
    options: [],
    editable: true,
    ...over,
  };
}

export function resource(over: Partial<Resource> = {}): Resource {
  return { code: 'RX-008', type_id: 6, values: {}, ...over };
}

export function template(over: Partial<Template> = {}): Template {
  return {
    id: 1,
    name: 'Hurtig genbrugsscreening',
    members: [],
    csv: {},
    seed: 'screening',
    resolved_keys: [],
    missing: [],
    created_at: '',
    updated_at: '',
    ...over,
  };
}

/** The screening seed's built-in keys, in catalogue order (spec §4.3). */
export const SYS_KEYS: ResourceKey[] = [
  resourceKey({ id: 'sys:name', label: 'Betegnelse', scope: 'type' }),
  resourceKey({ id: 'sys:quantity', label: 'Mængde', scope: 'part', data_type: 'number' }),
  resourceKey({ id: 'sys:unit', label: 'Enhed', scope: 'type' }),
  resourceKey({ id: 'sys:eak', label: 'EAK', scope: 'type' }),
  resourceKey({ id: 'sys:treatment', label: 'Behandling', scope: 'type', data_type: 'enum', options: ['bevaring', 'genbrug', 'genanvendelse', 'nyttiggoerelse', 'bortskaffelse'] }),
  resourceKey({ id: 'sys:environment', label: 'Miljøstatus', scope: 'type', data_type: 'enum', editable: false }),
  resourceKey({ id: 'sys:room', label: 'Rum', scope: 'part' }),
  resourceKey({ id: 'sys:mass_t', label: 'Tons', scope: 'type', data_type: 'number', unit: 't' }),
  resourceKey({ id: 'sys:note', label: 'Note', scope: 'part' }),
  resourceKey({ id: 'sys:starred', label: 'Vigtig', scope: 'part', data_type: 'boolean' }),
];
```

- [ ] **Step 2: Write the failing test**

Create `apps/rux/frontend/src/test/kortlaegning.resources.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { NO_FILTERS } from '../kortlaegning/model';
import {
  addableTypes,
  allPropertyGroups,
  cellModel,
  isManual,
  patchedResources,
  replaceResources,
  resourceColumnKeyId,
  resourceIndex,
  templateColumns,
  touchesSurvey,
  typeCarrier,
  valueOf,
  viewForNewResource,
} from '../kortlaegning/resources';
import { resource, resourceKey, SYS_KEYS, template } from './resourceFixtures';
import { surveyPart, surveyType } from './surveyFixtures';

const lex = resourceKey({ id: 'lex:abc', label: 'Producent', category: 'Identifikation', scope: 'part' });
const col = resourceKey({ id: 'col:4', label: 'Stand', category: 'Egne felter', scope: 'part', data_type: 'enum', options: ['God', 'Dårlig'] });
const catalogue = [...SYS_KEYS, lex, col];
const byId = (id: string) => catalogue.find((k) => k.id === id)!;

describe('templateColumns', () => {
  it('follows resolved_keys in order, skipping the tree column (sys:name)', () => {
    const cols = templateColumns(template({ resolved_keys: ['sys:name', 'sys:eak', 'col:4', 'sys:quantity'] }), catalogue);
    expect(cols.map((c) => c.key.id)).toEqual(['sys:eak', 'col:4', 'sys:quantity']);
  });

  it('marks type-scoped columns', () => {
    const cols = templateColumns(template({ resolved_keys: ['sys:eak', 'sys:note'] }), catalogue);
    expect(cols.map((c) => c.typeScoped)).toEqual([true, false]);
  });

  it('skips keys the catalogue does not know and duplicates', () => {
    const cols = templateColumns(template({ resolved_keys: ['lex:gone', 'sys:eak', 'sys:eak'] }), catalogue);
    expect(cols.map((c) => c.key.id)).toEqual(['sys:eak']);
  });

  it('is empty without a template', () => {
    expect(templateColumns(null, catalogue)).toEqual([]);
  });
});

describe('cellModel', () => {
  const t = surveyType({ id: 6, parts: [surveyPart({ code: 'RX-009' }), surveyPart({ code: 'RX-008' })] });
  const index = resourceIndex([
    resource({ code: 'RX-008', values: { 'sys:eak': '17.04.02', 'sys:note': 'Ren fuge' } }),
    resource({ code: 'RX-009', values: { 'sys:eak': '17.04.02' } }),
  ]);
  const column = (id: string) => ({ key: byId(id), typeScoped: byId(id).scope === 'type' });

  it('a part row edits its own value; a blank is null', () => {
    expect(cellModel({ kind: 'part', typeId: 6, partCode: 'RX-008' }, t, column('sys:note'), index)).toEqual({ kind: 'value', value: 'Ren fuge', target: 'RX-008' });
    expect(cellModel({ kind: 'part', typeId: 6, partCode: 'RX-009' }, t, column('sys:note'), index)).toEqual({ kind: 'value', value: null, target: 'RX-009' });
  });

  it('a read-only key has no target', () => {
    expect(cellModel({ kind: 'part', typeId: 6, partCode: 'RX-008' }, t, column('sys:environment'), index).target).toBeNull();
  });

  it("a type row reads and writes a type-scoped key through its first part by code", () => {
    expect(typeCarrier(t)).toBe('RX-008');
    expect(cellModel({ kind: 'type', typeId: 6 }, t, column('sys:eak'), index)).toEqual({ kind: 'value', value: '17.04.02', target: 'RX-008' });
  });

  it('a type row shows the Mængde aggregate and nothing for other part keys', () => {
    expect(cellModel({ kind: 'type', typeId: 6 }, t, column('sys:quantity'), index).kind).toBe('aggregate');
    expect(cellModel({ kind: 'type', typeId: 6 }, t, column('sys:note'), index)).toEqual({ kind: 'none', value: null, target: null });
  });

  it('a type without parts is read-only and blank', () => {
    const empty = surveyType({ id: 9, parts: [] });
    expect(typeCarrier(empty)).toBeNull();
    expect(cellModel({ kind: 'type', typeId: 9 }, empty, column('sys:eak'), index)).toEqual({ kind: 'value', value: null, target: null });
  });

  it('valueOf is null for an unknown code', () => {
    expect(valueOf(index, 'RX-999', 'sys:eak')).toBeNull();
  });
});

describe('merging patch responses', () => {
  it('replaces the patched resource and its type siblings, keeping the rest', () => {
    const list = [resource({ code: 'RX-008' }), resource({ code: 'RX-009' }), resource({ code: 'RX-001', type_id: 2 })];
    const result = {
      resource: resource({ code: 'RX-008', values: { 'sys:treatment': 'genbrug' } }),
      siblings: [resource({ code: 'RX-009', values: { 'sys:treatment': 'genbrug' } })],
    };
    const next = replaceResources(list, patchedResources(result));
    expect(next.map((r) => r.code)).toEqual(['RX-008', 'RX-009', 'RX-001']);
    expect(next[1].values['sys:treatment']).toBe('genbrug');
    expect(next[2]).toBe(list[2]);
  });

  it('appends a resource it did not have', () => {
    expect(replaceResources([], [resource({ code: 'RX-019' })]).map((r) => r.code)).toEqual(['RX-019']);
  });

  it('only sys: writes need the survey re-read', () => {
    expect(touchesSurvey(['col:4', 'lex:abc'])).toBe(false);
    expect(touchesSurvey(['col:4', 'sys:treatment'])).toBe(true);
  });
});

describe('resource helpers', () => {
  it('a part without an instance is manual', () => {
    expect(isManual(surveyPart({ instance_guid: null }))).toBe(true);
    expect(isManual(surveyPart({ instance_guid: 'g-1', orphaned: true }))).toBe(false);
  });

  it('a user column id becomes its key id', () => {
    expect(resourceColumnKeyId('4')).toBe('col:4');
  });

  it('Alle egenskaber groups the keys with a value by category, in catalogue order', () => {
    const r = resource({ values: { 'col:4': 'God', 'sys:note': 'x', 'lex:abc': 'Velux', 'sys:eak': null } });
    const groups = allPropertyGroups(r, catalogue);
    expect(groups.map((g) => [g.category, g.keys.map((k) => k.id)])).toEqual([
      ['Kortlægning', ['sys:note']],
      ['Identifikation', ['lex:abc']],
      ['Egne felter', ['col:4']],
    ]);
    expect(allPropertyGroups(null, catalogue)).toEqual([]);
  });

  it('a resource can be added to any type that is not rejected, by name', () => {
    const types = [surveyType({ id: 1, name: 'Ø' }), surveyType({ id: 2, name: 'A' }), surveyType({ id: 3, name: 'B', review_status: 'rejected' })];
    expect(addableTypes(types).map((t) => t.id)).toEqual([2, 1]);
  });

  it('shows a new resource whose type the current tab or filters hide', () => {
    const types = [surveyType({ id: 1, review_status: 'approved' })];
    expect(viewForNewResource(types, 1, 'queue', NO_FILTERS)).toEqual({ tab: 'all', filters: NO_FILTERS });
    const filters = { ...NO_FILTERS, search: 'vindue' };
    expect(viewForNewResource(types, 1, 'approved', filters)).toEqual({ tab: 'approved', filters });
  });
});
```

- [ ] **Step 3: Run it to verify it fails**

Run: `npm --prefix apps/rux/frontend test -- src/test/kortlaegning.resources.test.ts`
Expected: FAIL, `Failed to resolve import "../kortlaegning/resources"`.

- [ ] **Step 4: Implement**

Create `apps/rux/frontend/src/kortlaegning/resources.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Resources in the Kortlægning table, as data: which columns a template
 * builds, what each cell shows and where its write goes, and how patch
 * responses fold back in. Pure, so it is testable without a DOM.
 *
 * Rulings (plan R1–R4): the page holds every resource with all its values
 * and reads a template's cells as `values[key] ?? null`; a type row shows its
 * type-scoped keys through its first part by code (a type-scoped write
 * changes every part of the type, spec §4.4); `sys:name` is the fixed tree
 * column, never a dynamic one.
 */

import type { Resource, ResourceKey, ResourcePatchResult, SurveyPart, SurveyType, Template } from '../api/types';
import { NO_FILTERS, visibleTypes, type Filters, type Row, type Tab } from './model';

/** One template-built column of the parts table. */
export interface ResourceColumn {
  key: ResourceKey;
  /** A write changes every part of the type — the header shows a "type" marker. */
  typeScoped: boolean;
}

/** Keys drawn by a fixed column instead of a dynamic one (R4). */
export const FIXED_COLUMN_KEYS: ReadonlySet<string> = new Set(['sys:name']);

/**
 * The template's columns, in `resolved_keys` order. A key the catalogue does
 * not know (resolved after the catalogue was read) is skipped until the next
 * load; duplicates keep their first position.
 */
export function templateColumns(template: Template | null, catalogue: readonly ResourceKey[]): ResourceColumn[] {
  if (!template) return [];
  const byId = new Map(catalogue.map((k) => [k.id, k]));
  const seen = new Set<string>();
  const out: ResourceColumn[] = [];
  for (const id of template.resolved_keys) {
    if (seen.has(id) || FIXED_COLUMN_KEYS.has(id)) continue;
    seen.add(id);
    const key = byId.get(id);
    if (key) out.push({ key, typeScoped: key.scope === 'type' });
  }
  return out;
}

export type ResourceIndex = ReadonlyMap<string, Resource>;

export function resourceIndex(list: readonly Resource[]): ResourceIndex {
  return new Map(list.map((r) => [r.code, r]));
}

/** A resource's value for a key; a missing resource or key is null (blank). */
export function valueOf(index: ResourceIndex, code: string, keyId: string): string | null {
  return index.get(code)?.values[keyId] ?? null;
}

/** The part a type row reads and writes type-scoped keys through: its first by code (R3). */
export function typeCarrier(type: SurveyType): string | null {
  if (type.parts.length === 0) return null;
  return [...type.parts].map((p) => p.code).sort((a, b) => a.localeCompare(b))[0];
}

export interface CellModel {
  /** `aggregate`: the type row's Mængde sum; `none`: nothing to show. */
  kind: 'value' | 'aggregate' | 'none';
  /** The value shown and edited; null = blank. */
  value: string | null;
  /** The resource code a write goes to, or null when the cell is read-only. */
  target: string | null;
}

export function cellModel(row: Row, type: SurveyType, column: ResourceColumn, index: ResourceIndex): CellModel {
  const { key } = column;
  if (row.kind === 'part') {
    return { kind: 'value', value: valueOf(index, row.partCode, key.id), target: key.editable ? row.partCode : null };
  }
  if (key.id === 'sys:quantity') return { kind: 'aggregate', value: null, target: null };
  if (!column.typeScoped) return { kind: 'none', value: null, target: null };
  const carrier = typeCarrier(type);
  if (carrier === null) return { kind: 'value', value: null, target: null };
  return { kind: 'value', value: valueOf(index, carrier, key.id), target: key.editable ? carrier : null };
}

/** Every resource a PATCH response carries: the patched one, then its type siblings. */
export function patchedResources(result: ResourcePatchResult): Resource[] {
  return [result.resource, ...result.siblings];
}

/** `list` with each of `updated` replacing the resource of the same code (or appended). */
export function replaceResources(list: readonly Resource[], updated: readonly Resource[]): Resource[] {
  const fresh = new Map(updated.map((r) => [r.code, r]));
  const out = list.map((r) => fresh.get(r.code) ?? r);
  const known = new Set(list.map((r) => r.code));
  for (const r of updated) if (!known.has(r.code)) out.push(r);
  return out;
}

/** A write to a built-in key changed survey state: the page re-reads `/survey` (R5). */
export function touchesSurvey(keyIds: readonly string[]): boolean {
  return keyIds.some((id) => id.startsWith('sys:'));
}

/** A part added by hand (spec §4.1): no scan instance behind it. Only these can be deleted. */
export function isManual(part: SurveyPart): boolean {
  return part.instance_guid === null;
}

/** The key id of a user column definition (spec §4.3). */
export function resourceColumnKeyId(columnId: string): string {
  return `col:${columnId}`;
}

export interface PropertyGroup {
  category: string;
  keys: ResourceKey[];
}

/**
 * "Alle egenskaber": every key the resource has a value for, grouped by
 * category. Categories and keys keep catalogue order.
 */
export function allPropertyGroups(resource: Resource | null, catalogue: readonly ResourceKey[]): PropertyGroup[] {
  if (!resource) return [];
  const groups = new Map<string, ResourceKey[]>();
  for (const key of catalogue) {
    const v = resource.values[key.id];
    if (v === null || v === undefined) continue;
    const list = groups.get(key.category) ?? [];
    list.push(key);
    groups.set(key.category, list);
  }
  return [...groups].map(([category, keys]) => ({ category, keys }));
}

/** The types "Tilføj ressource" offers: every type not rejected, by Danish name order. */
export function addableTypes(types: readonly SurveyType[]): SurveyType[] {
  return types.filter((t) => t.review_status !== 'rejected').sort((a, b) => a.name.localeCompare(b.name, 'da'));
}

/** The tab and filters that show a just-created resource's type (R12). */
export function viewForNewResource(
  types: SurveyType[],
  typeId: number,
  tab: Tab,
  filters: Filters,
): { tab: Tab; filters: Filters } {
  const shown = visibleTypes(types, tab, filters).some((t) => t.id === typeId);
  return shown ? { tab, filters } : { tab: 'all', filters: NO_FILTERS };
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `npm --prefix apps/rux/frontend test -- src/test/kortlaegning.resources.test.ts`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add apps/rux/frontend/src/kortlaegning/resources.ts apps/rux/frontend/src/test/kortlaegning.resources.test.ts apps/rux/frontend/src/test/resourceFixtures.ts
git commit -m "feat(gui): template columns and resource cell model

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 3: The draft and save model for a cell (pure)

**Files:**
- Create: `apps/rux/frontend/src/kortlaegning/cellDraft.ts`
- Test: `apps/rux/frontend/src/test/kortlaegning.cellDraft.test.ts`

**Interfaces:**
- Consumes: `draftCommit`, `DraftCommit`, `DraftValidate` from `app/textDraft.ts`; `ENV_LABEL`, `TREATMENT_LABEL`, `formatNumber`, `formatQuantityInput`, `parseDanishNumber` from `kortlaegning/vocab.ts`; `ResourceKey`.
- Produces:
  - `BLANK = '—'`
  - `isTrue(value: string | null): boolean`
  - `toggleValue(value: string | null): 'true' | 'false'`
  - `cellDisplay(key: ResourceKey, value: string | null): string` (`''` when blank)
  - `optionLabel(key: ResourceKey, option: string): string`
  - `enumOptions(key: ResourceKey, value: string | null): string[]`
  - `cellInputText(key: ResourceKey, value: string | null): string`
  - `cellValidate(key: ResourceKey): DraftValidate<string | null>`
  - `cellCommit(key: ResourceKey, draft: string, value: string | null): DraftCommit<string | null>`

- [ ] **Step 1: Write the failing test**

Create `apps/rux/frontend/src/test/kortlaegning.cellDraft.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  BLANK,
  cellCommit,
  cellDisplay,
  cellInputText,
  enumOptions,
  isTrue,
  optionLabel,
  toggleValue,
} from '../kortlaegning/cellDraft';
import { resourceKey } from './resourceFixtures';

const text = resourceKey({ id: 'col:1', label: 'Producent' });
const num = resourceKey({ id: 'sys:mass_t', label: 'Tons', scope: 'type', data_type: 'number', unit: 't' });
const date = resourceKey({ id: 'col:2', data_type: 'date' });
const treat = resourceKey({ id: 'sys:treatment', data_type: 'enum', options: ['genbrug', 'bortskaffelse'] });
const env = resourceKey({ id: 'sys:environment', data_type: 'enum', editable: false });
const stand = resourceKey({ id: 'col:4', data_type: 'enum', options: ['God', 'Dårlig'] });
const starred = resourceKey({ id: 'sys:starred', data_type: 'boolean' });
const flag = resourceKey({ id: 'col:5', data_type: 'boolean' });

describe('blank-to-filled and back', () => {
  it('a blank text cell filled sends the trimmed text', () => {
    expect(cellCommit(text, '  Velux ', null)).toEqual({ send: true, value: 'Velux' });
  });

  it('an untouched blank never sends', () => {
    expect(cellCommit(text, '', null)).toEqual({ send: false, invalid: false });
    expect(cellCommit(num, '   ', null)).toEqual({ send: false, invalid: false });
  });

  it('emptying a filled cell sends null (clear)', () => {
    expect(cellCommit(text, '', 'Velux')).toEqual({ send: true, value: null });
    expect(cellCommit(num, '', '3.1')).toEqual({ send: true, value: null });
  });

  it('a Danish number becomes a wire number', () => {
    expect(cellCommit(num, '12,5', null)).toEqual({ send: true, value: '12.5' });
    expect(cellCommit(num, '1.240', null)).toEqual({ send: true, value: '1240' });
  });

  it('an untouched filled number never sends, nor does an equal one', () => {
    expect(cellCommit(num, cellInputText(num, '3.1'), '3.1')).toEqual({ send: false, invalid: false });
    expect(cellCommit(num, '3,10', '3.1')).toEqual({ send: false, invalid: false });
  });

  it('a draft that does not parse is invalid and sends nothing', () => {
    expect(cellCommit(num, 'abc', null)).toEqual({ send: false, invalid: true });
    expect(cellCommit(num, '3.1', '2')).toEqual({ send: false, invalid: true }); // English decimal
    expect(cellCommit(date, '1/2/2026', null)).toEqual({ send: false, invalid: true });
  });

  it('a date sends ISO text', () => {
    expect(cellCommit(date, '2026-10-02', null)).toEqual({ send: true, value: '2026-10-02' });
  });

  it('an enum sends a listed option or null, nothing for the same value', () => {
    expect(cellCommit(stand, 'God', null)).toEqual({ send: true, value: 'God' });
    expect(cellCommit(stand, '', 'God')).toEqual({ send: true, value: null });
    expect(cellCommit(stand, 'God', 'God')).toEqual({ send: false, invalid: false });
    expect(cellCommit(stand, 'Middel', null)).toEqual({ send: false, invalid: true });
  });
});

describe('input text', () => {
  it('shows a wire number with a Danish comma, and keeps legacy text', () => {
    expect(cellInputText(num, '12.5')).toBe('12,5');
    expect(cellInputText(num, 'ca. 5')).toBe('ca. 5');
    expect(cellInputText(num, null)).toBe('');
    expect(cellInputText(text, 'x')).toBe('x');
  });
});

describe('display', () => {
  it('is empty for a blank, so the cell can draw the muted placeholder', () => {
    expect(cellDisplay(text, null)).toBe('');
    expect(cellDisplay(text, '  ')).toBe('');
    expect(BLANK).toBe('—');
  });

  it('formats numbers in Danish with the key unit', () => {
    expect(cellDisplay(num, '1240.5')).toBe('1.240,5 t');
  });

  it('labels treatment and miljøstatus in Danish', () => {
    expect(cellDisplay(treat, 'genbrug')).toBe('Genbrug');
    expect(cellDisplay(env, 'afventer')).toBe('Afventer prøve');
    expect(optionLabel(treat, 'bortskaffelse')).toBe('Bortskaffelse');
    expect(optionLabel(stand, 'God')).toBe('God');
  });

  it('shows ★ for Vigtig and Ja/Nej for other booleans', () => {
    expect(cellDisplay(starred, 'true')).toBe('★');
    expect(cellDisplay(starred, 'false')).toBe('');
    expect(cellDisplay(flag, '1')).toBe('Ja');
    expect(cellDisplay(flag, 'false')).toBe('Nej');
  });
});

describe('booleans and enum options', () => {
  it('reads tolerant truthy strings and toggles to wire booleans', () => {
    expect(['true', '1', 'Ja', 'YES'].map(isTrue)).toEqual([true, true, true, true]);
    expect([null, 'false', '0', ''].map(isTrue)).toEqual([false, false, false, false]);
    expect(toggleValue(null)).toBe('true');
    expect(toggleValue('true')).toBe('false');
  });

  it('keeps a legacy value the options no longer list, so the select can show it', () => {
    expect(enumOptions(stand, 'Middel')).toEqual(['God', 'Dårlig', 'Middel']);
    expect(enumOptions(stand, 'God')).toEqual(['God', 'Dårlig']);
    expect(enumOptions(stand, null)).toEqual(['God', 'Dårlig']);
  });
});
```

- [ ] **Step 2: Run it to verify it fails**

Run: `npm --prefix apps/rux/frontend test -- src/test/kortlaegning.cellDraft.test.ts`
Expected: FAIL, `Failed to resolve import "../kortlaegning/cellDraft"`.

- [ ] **Step 3: Implement**

Create `apps/rux/frontend/src/kortlaegning/cellDraft.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * One resource value as the Kortlægning table shows and edits it. Pure, so
 * "blank → filled sends the value, an untouched blur sends nothing, emptying
 * sends null" is tested here; `ResourceCell` wires it to `useTextDraft`,
 * which runs exactly `draftCommit(draft, cellInputText(...), cellValidate(...))`.
 *
 * Wire format (spec §4.4): every value is a string or null. Numbers travel
 * with a `.` decimal; booleans as "true"/"false" (plan R10); dates ISO.
 */

import type { EnvironmentStatus, ResourceKey, Treatment } from '../api/types';
import { draftCommit, type DraftCommit, type DraftValidate } from '../app/textDraft';
import { ENV_LABEL, TREATMENT_LABEL, formatNumber, formatQuantityInput, parseDanishNumber } from './vocab';

/** The muted placeholder of a blank cell. */
export const BLANK = '—';

const TRUTHY = new Set(['true', '1', 'ja', 'yes']);

export function isTrue(value: string | null): boolean {
  return value !== null && TRUTHY.has(value.trim().toLowerCase());
}

export function toggleValue(value: string | null): 'true' | 'false' {
  return isTrue(value) ? 'false' : 'true';
}

/** An enum option's label: Danish for behandling and miljøstatus, the option itself otherwise. */
export function optionLabel(key: ResourceKey, option: string): string {
  if (key.id === 'sys:treatment' && option in TREATMENT_LABEL) return TREATMENT_LABEL[option as Treatment];
  if (key.id === 'sys:environment' && option in ENV_LABEL) return ENV_LABEL[option as EnvironmentStatus];
  return option;
}

/** The options a select lists: the key's, plus a stored value they no longer include. */
export function enumOptions(key: ResourceKey, value: string | null): string[] {
  const options = key.options ?? [];
  return value !== null && value !== '' && !options.includes(value) ? [...options, value] : [...options];
}

/** What a cell shows; `''` for a blank (the cell then draws `BLANK`, muted). */
export function cellDisplay(key: ResourceKey, value: string | null): string {
  if (value === null || value.trim() === '') return '';
  if (key.id === 'sys:starred') return isTrue(value) ? '★' : '';
  switch (key.data_type) {
    case 'number': {
      const n = Number(value);
      const shown = Number.isFinite(n) ? formatNumber(n, 3) : value;
      return key.unit ? `${shown} ${key.unit}` : shown;
    }
    case 'boolean':
      return isTrue(value) ? 'Ja' : 'Nej';
    case 'enum':
      return optionLabel(key, value);
    default:
      return value;
  }
}

/** The text an editor starts from: a wire number in Danish input form; anything else as stored. */
export function cellInputText(key: ResourceKey, value: string | null): string {
  if (value === null) return '';
  if (key.data_type === 'number') {
    const n = Number(value);
    return value.trim() !== '' && Number.isFinite(n) ? formatQuantityInput(n) : value;
  }
  return value;
}

const ISO_DATE = /^\d{4}-\d{2}-\d{2}$/;

/** Parses a draft into the wire value to send: `null` clears, a failed parse is invalid. */
export function cellValidate(key: ResourceKey): DraftValidate<string | null> {
  return (draft) => {
    const t = draft.trim();
    if (t === '') return { value: null };
    switch (key.data_type) {
      case 'number': {
        const n = parseDanishNumber(t);
        return n === null ? null : { value: String(n) };
      }
      case 'date':
        return ISO_DATE.test(t) ? { value: t } : null;
      case 'enum':
        return (key.options ?? []).includes(t) ? { value: t } : null;
      case 'boolean':
        return { value: isTrue(t) ? 'true' : 'false' };
      default:
        return { value: t };
    }
  };
}

/** A cell's blur (or a select's change): what to send, if anything. */
export function cellCommit(key: ResourceKey, draft: string, value: string | null): DraftCommit<string | null> {
  return draftCommit(draft, cellInputText(key, value), cellValidate(key));
}
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `npm --prefix apps/rux/frontend test -- src/test/kortlaegning.cellDraft.test.ts`
Expected: PASS.

Check one case by hand: `cellDisplay(num, '1240.5')`. `formatNumber(1240.5, 3)` gives `1.240,5` in `da-DK`. If Node's ICU groups differently (small-icu builds), the existing `kortlaegning.vocab.test.ts` would already fail the same way. Keep the assertion and make it match whatever `formatNumber` produces there.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/src/kortlaegning/cellDraft.ts apps/rux/frontend/src/test/kortlaegning.cellDraft.test.ts
git commit -m "feat(gui): resource cell draft and save model

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 4: Template selection and storage per project (pure)

**Files:**
- Create: `apps/rux/frontend/src/kortlaegning/templatePick.ts`
- Test: `apps/rux/frontend/src/test/kortlaegning.templatePick.test.ts`

**Interfaces:**
- Consumes: `Template` and `TemplateMember` (Task 1).
- Produces:
  - `TEMPLATE_STORAGE_PREFIX = 'rux.kortlaegning.template:'`
  - `interface StorageLike { getItem(key: string): string | null; setItem(key: string, value: string): void }`
  - `templateStorageKey(project: string): string`
  - `readStoredTemplateId(project: string, storage?: StorageLike): number | null`
  - `writeStoredTemplateId(project: string, id: number, storage?: StorageLike): void`
  - `pickTemplate(templates: readonly Template[], storedId: number | null): Template | null`
  - `appendKeyMember(members: readonly TemplateMember[], keyId: string): TemplateMember[]`

- [ ] **Step 1: Write the failing test**

Create `apps/rux/frontend/src/test/kortlaegning.templatePick.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  appendKeyMember,
  pickTemplate,
  readStoredTemplateId,
  templateStorageKey,
  writeStoredTemplateId,
  type StorageLike,
} from '../kortlaegning/templatePick';
import { template } from './resourceFixtures';

function memoryStorage(): StorageLike & { data: Map<string, string> } {
  const data = new Map<string, string>();
  return { data, getItem: (k) => data.get(k) ?? null, setItem: (k, v) => void data.set(k, v) };
}

const full = template({ id: 1, name: 'Materialepas (fuld)', seed: 'materialepas' });
const screening = template({ id: 2, name: 'Hurtig genbrugsscreening', seed: 'screening' });
const own = template({ id: 5, name: 'Min', seed: null });

describe('pickTemplate', () => {
  it('keeps the stored template while it exists', () => {
    expect(pickTemplate([full, screening, own], 5)).toBe(own);
  });

  it('falls back to the screening seed, then the first template, then nothing', () => {
    expect(pickTemplate([full, screening, own], 99)).toBe(screening);
    expect(pickTemplate([full, screening], null)).toBe(screening);
    expect(pickTemplate([own, full], null)).toBe(own);
    expect(pickTemplate([], 2)).toBeNull();
  });
});

describe('stored template per project', () => {
  it('round-trips under a per-project key', () => {
    const s = memoryStorage();
    writeStoredTemplateId('malov.rux', 5, s);
    expect(s.data.get(templateStorageKey('malov.rux'))).toBe('5');
    expect(readStoredTemplateId('malov.rux', s)).toBe(5);
    expect(readStoredTemplateId('other.rux', s)).toBeNull();
  });

  it('ignores junk and a storage that throws', () => {
    const s = memoryStorage();
    s.data.set(templateStorageKey('p'), 'abc');
    expect(readStoredTemplateId('p', s)).toBeNull();
    const broken: StorageLike = {
      getItem: () => {
        throw new Error('denied');
      },
      setItem: () => {
        throw new Error('denied');
      },
    };
    expect(readStoredTemplateId('p', broken)).toBeNull();
    expect(() => writeStoredTemplateId('p', 1, broken)).not.toThrow();
  });
});

describe('appendKeyMember', () => {
  it('appends a key member once', () => {
    const members = [{ category: 'Egne felter' }, { key: 'sys:name' }];
    expect(appendKeyMember(members, 'col:4')).toEqual([...members, { key: 'col:4' }]);
    expect(appendKeyMember([{ key: 'col:4' }], 'col:4')).toEqual([{ key: 'col:4' }]);
  });
});
```

- [ ] **Step 2: Run it to verify it fails**

Run: `npm --prefix apps/rux/frontend test -- src/test/kortlaegning.templatePick.test.ts`
Expected: FAIL, `Failed to resolve import "../kortlaegning/templatePick"`.

- [ ] **Step 3: Implement**

Create `apps/rux/frontend/src/kortlaegning/templatePick.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Which template Kortlægning shows (spec §6.1): the one remembered for this
 * project, else the `screening` seed, else the first. Remembered in
 * localStorage per project; storage that is missing or throws (sandbox,
 * privacy mode) is treated as empty, never as an error.
 */

import type { Template, TemplateMember } from '../api/types';

export const TEMPLATE_STORAGE_PREFIX = 'rux.kortlaegning.template:';

export interface StorageLike {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
}

function defaultStorage(): StorageLike | undefined {
  try {
    return typeof localStorage !== 'undefined' ? localStorage : undefined;
  } catch {
    return undefined;
  }
}

export function templateStorageKey(project: string): string {
  return `${TEMPLATE_STORAGE_PREFIX}${project}`;
}

export function readStoredTemplateId(project: string, storage = defaultStorage()): number | null {
  try {
    const raw = storage?.getItem(templateStorageKey(project));
    if (raw === null || raw === undefined || !/^\d+$/.test(raw)) return null;
    return Number(raw);
  } catch {
    return null;
  }
}

export function writeStoredTemplateId(project: string, id: number, storage = defaultStorage()): void {
  try {
    storage?.setItem(templateStorageKey(project), String(id));
  } catch {
    // A refused write only means the choice is not remembered.
  }
}

export function pickTemplate(templates: readonly Template[], storedId: number | null): Template | null {
  return (
    templates.find((t) => t.id === storedId) ??
    templates.find((t) => t.seed === 'screening') ??
    templates[0] ??
    null
  );
}

/** `members` plus a key member for `keyId`, unless one is already there. */
export function appendKeyMember(members: readonly TemplateMember[], keyId: string): TemplateMember[] {
  if (members.some((m) => 'key' in m && m.key === keyId)) return [...members];
  return [...members, { key: keyId }];
}
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `npm --prefix apps/rux/frontend test -- src/test/kortlaegning.templatePick.test.ts`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/src/kortlaegning/templatePick.ts apps/rux/frontend/src/test/kortlaegning.templatePick.test.ts
git commit -m "feat(gui): Kortlægning template selection, remembered per project

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 5: The Tilføj kolonne draft (pure)

**Files:**
- Create: `apps/rux/frontend/src/kortlaegning/columnDraft.ts`
- Test: `apps/rux/frontend/src/test/kortlaegning.columnDraft.test.ts`

**Interfaces:**
- Consumes: `ResourceColumnCreate` and `Template` (Task 1).
- Produces:
  - `type ColumnKind = 'text' | 'number' | 'date' | 'boolean' | 'select'`
  - `COLUMN_KINDS: readonly ColumnKind[]`; `COLUMN_KIND_LABEL: Record<ColumnKind, string>`
  - `interface ColumnDraft { name: string; kind: ColumnKind; optionsText: string }`
  - `EMPTY_COLUMN_DRAFT: ColumnDraft`
  - `parseOptions(text: string): string[]`
  - `columnDraftError(draft: ColumnDraft, existingLabels: readonly string[]): string | null`
  - `columnCreateBody(draft: ColumnDraft): ResourceColumnCreate`
  - `seedNote(template: Template | null): string | null`
  - `duplicateFirst(template: Template | null, copyInstead: boolean): boolean`
  - `columnPartialFailureMessage(name: string, detail: string): string`

- [ ] **Step 1: Write the failing test**

Create `apps/rux/frontend/src/test/kortlaegning.columnDraft.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  COLUMN_KIND_LABEL,
  columnCreateBody,
  columnDraftError,
  columnPartialFailureMessage,
  duplicateFirst,
  EMPTY_COLUMN_DRAFT,
  parseOptions,
  seedNote,
} from '../kortlaegning/columnDraft';
import { template } from './resourceFixtures';

describe('parseOptions', () => {
  it('splits on commas and new lines, trims, drops blanks and case-insensitive repeats', () => {
    expect(parseOptions(' God, Dårlig\nGod\n\n middel ,god')).toEqual(['God', 'Dårlig', 'middel']);
  });
});

describe('columnDraftError', () => {
  it('needs a name', () => {
    expect(columnDraftError({ ...EMPTY_COLUMN_DRAFT, name: '  ' }, [])).toBe('Angiv et navn.');
  });

  it('refuses a name a key already uses, ignoring case and spaces', () => {
    expect(columnDraftError({ ...EMPTY_COLUMN_DRAFT, name: ' stand ' }, ['Stand'])).toBe(
      'Der findes allerede et felt med det navn.',
    );
  });

  it('needs an option for a select list', () => {
    expect(columnDraftError({ name: 'Stand', kind: 'select', optionsText: ' , ' }, [])).toBe(
      'Angiv mindst én valgmulighed.',
    );
    expect(columnDraftError({ name: 'Stand', kind: 'select', optionsText: 'God' }, [])).toBeNull();
  });
});

describe('columnCreateBody', () => {
  it('sends options only for a select list', () => {
    expect(columnCreateBody({ name: ' Stand ', kind: 'select', optionsText: 'God,Dårlig' })).toEqual({
      name: 'Stand',
      type: 'select',
      options: ['God', 'Dårlig'],
    });
    expect(columnCreateBody({ name: 'Leverandør', kind: 'text', optionsText: 'ignored' })).toEqual({
      name: 'Leverandør',
      type: 'text',
    });
  });

  it('labels every kind in Danish', () => {
    expect(COLUMN_KIND_LABEL).toEqual({ text: 'Tekst', number: 'Tal', date: 'Dato', boolean: 'Ja/nej', select: 'Valgliste' });
  });
});

describe('seed templates', () => {
  it('warns that the column goes into a seed, and only then offers a copy', () => {
    expect(seedNote(template({ name: 'Hurtig genbrugsscreening', seed: 'screening' }))).toBe(
      'Kolonnen føjes til standardskabelonen »Hurtig genbrugsscreening«.',
    );
    expect(seedNote(template({ seed: null }))).toBeNull();
    expect(seedNote(null)).toBeNull();
    expect(duplicateFirst(template({ seed: 'screening' }), true)).toBe(true);
    expect(duplicateFirst(template({ seed: 'screening' }), false)).toBe(false);
    expect(duplicateFirst(template({ seed: null }), true)).toBe(false);
  });

  it('says the column exists when only the template update failed', () => {
    expect(columnPartialFailureMessage('Stand', 'busy')).toBe(
      'Kolonnen »Stand« er oprettet, men kunne ikke føjes til skabelonen: busy',
    );
  });
});
```

- [ ] **Step 2: Run it to verify it fails**

Run: `npm --prefix apps/rux/frontend test -- src/test/kortlaegning.columnDraft.test.ts`
Expected: FAIL, `Failed to resolve import "../kortlaegning/columnDraft"`.

- [ ] **Step 3: Implement**

Create `apps/rux/frontend/src/kortlaegning/columnDraft.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The "Tilføj kolonne" dialog as data (spec §6.1): validate the draft before
 * any request, build the column-create body, and decide whether a seed
 * template is copied first (plan R11).
 */

import type { ResourceColumnCreate, Template } from '../api/types';

export type ColumnKind = 'text' | 'number' | 'date' | 'boolean' | 'select';
export const COLUMN_KINDS: readonly ColumnKind[] = ['text', 'number', 'date', 'boolean', 'select'];
export const COLUMN_KIND_LABEL: Record<ColumnKind, string> = {
  text: 'Tekst',
  number: 'Tal',
  date: 'Dato',
  boolean: 'Ja/nej',
  select: 'Valgliste',
};

export interface ColumnDraft {
  name: string;
  kind: ColumnKind;
  /** Comma- or line-separated; used by `select` only. */
  optionsText: string;
}

export const EMPTY_COLUMN_DRAFT: ColumnDraft = { name: '', kind: 'text', optionsText: '' };

export function parseOptions(text: string): string[] {
  const seen = new Set<string>();
  const out: string[] = [];
  for (const raw of text.split(/[\n,]/)) {
    const option = raw.trim();
    const folded = option.toLocaleLowerCase('da');
    if (option === '' || seen.has(folded)) continue;
    seen.add(folded);
    out.push(option);
  }
  return out;
}

/** Why the draft cannot be submitted, or null. `existingLabels`: every catalogue key's label. */
export function columnDraftError(draft: ColumnDraft, existingLabels: readonly string[]): string | null {
  const name = draft.name.trim();
  if (name === '') return 'Angiv et navn.';
  const folded = name.toLocaleLowerCase('da');
  if (existingLabels.some((l) => l.trim().toLocaleLowerCase('da') === folded)) {
    return 'Der findes allerede et felt med det navn.';
  }
  if (draft.kind === 'select' && parseOptions(draft.optionsText).length === 0) {
    return 'Angiv mindst én valgmulighed.';
  }
  return null;
}

export function columnCreateBody(draft: ColumnDraft): ResourceColumnCreate {
  const body: ResourceColumnCreate = { name: draft.name.trim(), type: draft.kind };
  if (draft.kind === 'select') body.options = parseOptions(draft.optionsText);
  return body;
}

/** The dialog's note when the selected template is a seed, else null. */
export function seedNote(template: Template | null): string | null {
  return template?.seed ? `Kolonnen føjes til standardskabelonen »${template.name}«.` : null;
}

/** Whether to duplicate the template before appending: only a seed, only on request. */
export function duplicateFirst(template: Template | null, copyInstead: boolean): boolean {
  return copyInstead && template !== null && template.seed !== null;
}

/** The toast when the column was created but appending it to the template failed. */
export function columnPartialFailureMessage(name: string, detail: string): string {
  return `Kolonnen »${name}« er oprettet, men kunne ikke føjes til skabelonen: ${detail}`;
}
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `npm --prefix apps/rux/frontend test -- src/test/kortlaegning.columnDraft.test.ts`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/src/kortlaegning/columnDraft.ts apps/rux/frontend/src/test/kortlaegning.columnDraft.test.ts
git commit -m "feat(gui): Tilføj kolonne draft validation and seed plan

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 6: ResourceCell, the template-driven SurveyTable, and page wiring for the picker and inline edits

**Files:**
- Create: `apps/rux/frontend/src/components/kortlaegning/ResourceCell.tsx`, `ResourceCell.module.css`
- Modify: `apps/rux/frontend/src/components/kortlaegning/SurveyTable.tsx` (props, tools row, `<thead>`, `<tbody>`, lines 30–50 and 136–362), `SurveyTable.module.css`
- Modify: `apps/rux/frontend/src/routes/KortlaegningPage.tsx`

**Interfaces:**
- Consumes:
  - from Task 2: `templateColumns`, `resourceIndex`, `cellModel`, `patchedResources`, `replaceResources`, `touchesSurvey`, `isManual`, `ResourceColumn`, `ResourceIndex`;
  - from Task 3: `BLANK`, `cellDisplay`, `cellInputText`, `cellValidate`, `cellCommit`, `enumOptions`, `optionLabel`, `isTrue`, `toggleValue`;
  - from Task 4: `pickTemplate`, `readStoredTemplateId`, `writeStoredTemplateId`;
  - from Task 1: `api.resourceKeys`, `api.templates`, `api.resources`, `api.patchResource`, `api.health`;
  - `useTextDraft`, `fieldKeys` (app/useTextDraft).
- Produces:
  - `ResourceCell` props: `{ resourceKey: ResourceKey; value: string | null; editing: boolean; onCommit: (value: string | null) => void; onInvalid: (label: string) => void; home: RefObject<HTMLElement | null>; variant?: 'table' | 'field' }`.
  - New `SurveyTableProps`: `columns: ResourceColumn[]`, `resources: ResourceIndex`, `templates: Template[]`, `templateId: number | null`, `onTemplate: (id: number) => void`, `onCellCommit: (code: string, keyId: string, value: string | null) => void`, `onInvalid: (label: string) => void`, `onAddResource: () => void`, `onAddColumn: () => void`.
  - Page state: `keys`, `templates`, `templateId`, `resources`; helpers `commitCell(code, keyId, value)` and `refreshResources()`, used by Tasks 7–9.

- [ ] **Step 1: Create ResourceCell**

`apps/rux/frontend/src/components/kortlaegning/ResourceCell.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * One resource value: formatted text, or — when `editing` — the editor its
 * `data_type` asks for (spec §6.1): a text/number/date field that commits on
 * blur (`useTextDraft` + `cellDraft`), a select for an enum, a checkbox for a
 * boolean. Blank shows a muted `—` (plan R2: only the selected row edits).
 *
 * The editor swallows click/double-click so a click in a field never selects,
 * folds or opens the row underneath; Esc reverts and parks focus on `home`
 * (the table), Enter commits and does the same.
 */

import type { RefObject } from 'react';

import type { EnvironmentStatus, ResourceKey, Treatment } from '../../api/types';
import { TREATMENTS } from '../../api/types';
import { fieldKeys, useTextDraft } from '../../app/useTextDraft';
import {
  BLANK,
  cellCommit,
  cellDisplay,
  cellInputText,
  cellValidate,
  enumOptions,
  isTrue,
  optionLabel,
  toggleValue,
} from '../../kortlaegning/cellDraft';
import { ENV_TONE } from '../../kortlaegning/vocab';
import { Pill } from '../Pill';
import styles from './ResourceCell.module.css';

export interface ResourceCellProps {
  resourceKey: ResourceKey;
  value: string | null;
  editing: boolean;
  onCommit: (value: string | null) => void;
  /** A blur found a draft that does not parse; the field has snapped back. */
  onInvalid: (label: string) => void;
  /** Where focus goes after Esc/Enter in a field. */
  home: RefObject<HTMLElement | null>;
  /** `table`: compact, in a row; `field`: full width, in the detail panel. */
  variant?: 'table' | 'field';
}

function isTreatment(v: string): v is Treatment {
  return (TREATMENTS as readonly string[]).includes(v);
}

function CellText({ resourceKey: key, value }: { resourceKey: ResourceKey; value: string | null }) {
  const text = cellDisplay(key, value);
  if (text === '') return <span className={styles.blank}>{BLANK}</span>;
  if (key.id === 'sys:environment' && value !== null && value in ENV_TONE) {
    return <Pill tone={ENV_TONE[value as EnvironmentStatus]}>{text}</Pill>;
  }
  if (key.id === 'sys:treatment' && value !== null && isTreatment(value)) {
    return <Pill treatment={value}>{text}</Pill>;
  }
  return (
    <span className={key.data_type === 'number' ? `${styles.text} mono` : styles.text} title={text}>
      {text}
    </span>
  );
}

function DraftInput({ resourceKey: key, value, onCommit, onInvalid, home }: ResourceCellProps) {
  const draft = useTextDraft<string | null>(cellInputText(key, value), onCommit, {
    validate: cellValidate(key),
    onInvalid: () => onInvalid(key.label),
  });
  const keys = fieldKeys(draft, home);
  return (
    <input
      type={key.data_type === 'date' ? 'date' : 'text'}
      inputMode={key.data_type === 'number' ? 'decimal' : undefined}
      className={key.data_type === 'number' ? `${styles.input} mono` : styles.input}
      aria-label={key.label}
      placeholder={BLANK}
      {...draft.props}
      onKeyDown={(e) => {
        keys(e);
        // Enter committed by blurring; give the table its keys back.
        if (e.key === 'Enter' && e.defaultPrevented) home.current?.focus({ preventScroll: true });
      }}
    />
  );
}

export function ResourceCell(props: ResourceCellProps) {
  const { resourceKey: key, value, editing, variant = 'table' } = props;
  if (!editing) return <CellText resourceKey={key} value={value} />;

  let editor;
  if (key.data_type === 'boolean') {
    editor = (
      <input
        type="checkbox"
        className={styles.checkbox}
        aria-label={key.label}
        checked={isTrue(value)}
        onChange={() => props.onCommit(toggleValue(value))}
      />
    );
  } else if (key.data_type === 'enum') {
    editor = (
      <select
        className={styles.select}
        aria-label={key.label}
        value={value ?? ''}
        onChange={(e) => {
          const c = cellCommit(key, e.target.value, value);
          if (c.send) props.onCommit(c.value);
        }}
      >
        <option value="">{BLANK}</option>
        {enumOptions(key, value).map((o) => (
          <option key={o} value={o}>
            {optionLabel(key, o)}
          </option>
        ))}
      </select>
    );
  } else {
    editor = <DraftInput {...props} />;
  }

  return (
    <div
      className={styles.editor}
      data-variant={variant}
      onClick={(e) => e.stopPropagation()}
      onDoubleClick={(e) => e.stopPropagation()}
    >
      {editor}
    </div>
  );
}
```

`apps/rux/frontend/src/components/kortlaegning/ResourceCell.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.blank {
  color: var(--color-text-faint);
}

/* Long notes stay one line in a row; the title carries the full text. */
.text {
  display: inline-block;
  max-width: calc(var(--space-7) * 5);
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  vertical-align: bottom;
}

.editor {
  display: flex;
  align-items: center;
  min-width: calc(var(--space-7) * 2);
}

.editor[data-variant='field'] {
  width: 100%;
}

.input,
.select {
  composes: input from '../controls.module.css';
  width: 100%;
}

.input::placeholder {
  color: var(--color-text-faint);
}

.checkbox {
  composes: checkbox from '../controls.module.css';
}

/* Phone width: 44px tap targets for the row editors. */
@media (max-width: 56.25rem) {
  .input,
  .select {
    min-height: var(--layout-titlebar-height);
  }
}
```

- [ ] **Step 2: Rebuild SurveyTable's columns from the template**

In `SurveyTable.tsx`:

1. Add the imports:

   ```tsx
   import type { RefObject } from 'react';
   import type { Template } from '../../api/types';
   import { cellModel, isManual, type ResourceColumn, type ResourceIndex } from '../../kortlaegning/resources';
   import { ResourceCell } from './ResourceCell';
   ```

   Drop `ENV_LABEL`, `ENV_TONE` and `TREATMENT_LABEL` from the vocab import; they are no longer used here.

2. Add these props to `SurveyTableProps`:

   ```ts
     /** Template-built columns (resolved_keys order, sys:name excluded — plan R4). */
     columns: ResourceColumn[];
     resources: ResourceIndex;
     templates: Template[];
     templateId: number | null;
     onTemplate: (id: number) => void;
     onCellCommit: (code: string, keyId: string, value: string | null) => void;
     onInvalid: (label: string) => void;
     onAddResource: () => void;
     onAddColumn: () => void;
   ```

   Destructure them in the component.

3. In the `.tools` div, before the search input, add the picker:

   ```tsx
           <label className={styles.picker}>
             <span className={styles.pickerLabel}>Skabelon</span>
             <select
               className={styles.select}
               aria-label="Skabelon"
               value={templateId === null ? '' : String(templateId)}
               onChange={(e) => onTemplate(Number(e.target.value))}
               disabled={templates.length === 0}
             >
               {templates.length === 0 && <option value="">Ingen skabeloner</option>}
               {templates.map((t) => (
                 <option key={t.id} value={t.id}>
                   {t.name}
                 </option>
               ))}
             </select>
           </label>
   ```

   After the `Kun vigtige ★` label, add the two actions:

   ```tsx
           <div className={styles.toolActions}>
             <button type="button" className={styles.btnGhost} onClick={onAddResource}>
               + Tilføj ressource
             </button>
             <button type="button" className={styles.btnGhost} onClick={onAddColumn} disabled={templateId === null}>
               + Tilføj kolonne
             </button>
           </div>
   ```

4. Replace `<thead>…</thead>` with:

   ```tsx
             <thead>
               <tr>
                 <th>Betegnelse</th>
                 {columns.map((c) => (
                   <th key={c.key.id}>
                     <span className={styles.headLabel}>
                       {c.key.label}
                       {c.typeScoped && (
                         <span className={styles.typeMark} title="Gælder alle dele af typen">
                           type
                         </span>
                       )}
                     </span>
                   </th>
                 ))}
                 <th>Status</th>
               </tr>
             </thead>
   ```

5. Change the empty row's `colSpan={7}` to `colSpan={columns.length + 2}`.

6. Add a cell renderer above `export function SurveyTable`:

   ```tsx
   function Cells(props: {
     row: Row;
     type: SurveyType;
     columns: ResourceColumn[];
     resources: ResourceIndex;
     selected: boolean;
     onCellCommit: SurveyTableProps['onCellCommit'];
     onInvalid: SurveyTableProps['onInvalid'];
     home: RefObject<HTMLElement | null>;
   }) {
     const { row, type, columns, resources, selected } = props;
     return (
       <>
         {columns.map((column) => {
           const m = cellModel(row, type, column, resources);
           let content = null;
           if (m.kind === 'aggregate') {
             content = (
               <>
                 {formatQuantity(type.quantity, type.unit)} <span className={styles.faint}>{formatTonnes(type.mass_t)}</span>
               </>
             );
           } else if (m.kind === 'value') {
             const target = m.target;
             content = (
               <ResourceCell
                 resourceKey={column.key}
                 value={m.value}
                 editing={selected && target !== null}
                 onCommit={(v) => target !== null && props.onCellCommit(target, column.key.id, v)}
                 onInvalid={props.onInvalid}
                 home={props.home}
               />
             );
           }
           return <td key={column.key.id}>{content}</td>;
         })}
       </>
     );
   }
   ```

   Add `type Row` to the `../../kortlaegning/model` import, and `SurveyType` is already imported.

7. In the type row, replace the six `<td>` after the name cell, from `formatQuantity(type.quantity…` through the miljø `<Pill>`, with:

   ```tsx
                         <Cells row={row} type={type} columns={columns} resources={resources} selected={selected}
                           onCellCommit={onCellCommit} onInvalid={onInvalid} home={tableRef} />
   ```

   Keep the Status `<td>` (Godkendt / ConfidenceBar) as it is.

8. In the part row, replace the cells from `<td>{formatQuantity(part.quantity, type.unit)}</td>` through the four empty `<td>`s with the same `<Cells …/>`, keeping the final `<td className={styles.faint}>—</td>` (Status).
   In the part cell, after `{partLabel(part)}`, add:

   ```tsx
                           {isManual(part) && (
                             <Pill tone="accent" title="Tilføjet manuelt — ikke fra scanningen">
                               Manuel
                             </Pill>
                           )}
                         ```

9. Append to `SurveyTable.module.css`:

```css
/* -------------------------------------------------------- picker/actions -- */

.picker {
  display: inline-flex;
  align-items: center;
  gap: var(--space-2);
}

.pickerLabel {
  color: var(--color-text-muted);
  font-size: var(--font-size-xs);
  font-weight: var(--font-weight-bold);
  text-transform: uppercase;
  letter-spacing: var(--tracking-caps);
}

.toolActions {
  display: inline-flex;
  gap: var(--space-2);
  margin-left: auto;
}

.btnGhost {
  composes: btnGhost from '../controls.module.css';
}

/* ------------------------------------------------------- column headers -- */

.headLabel {
  display: inline-flex;
  align-items: center;
  gap: var(--space-1);
  white-space: nowrap;
}

.typeMark {
  padding: 0 var(--space-1);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-sm);
  color: var(--color-text-faint);
  font-size: var(--font-size-2xs);
  font-weight: var(--font-weight-regular);
  text-transform: none;
  cursor: help;
}

@media (max-width: 56.25rem) {
  .toolActions {
    margin-left: 0;
    width: 100%;
  }

  .toolActions > button {
    flex: 1 1 0;
    min-height: var(--layout-titlebar-height);
  }
}
```

The table keeps its own horizontal scroll in `.wrap` (`overflow: auto`); a wide template scrolls inside the panel, and the page itself does not.

- [ ] **Step 3: Load keys, templates and resources in the page and wire the picker and inline commits**

In `KortlaegningPage.tsx`:

1. Imports. Add:

   ```tsx
   import type { Resource, ResourceKey, Template } from '../api/types';
   import { patchedResources, replaceResources, resourceIndex, templateColumns, touchesSurvey } from '../kortlaegning/resources';
   import { pickTemplate, readStoredTemplateId, writeStoredTemplateId } from '../kortlaegning/templatePick';
   ```

2. Load. Replace the `useAsync` body with:

   ```tsx
     const { data, error, loading, reload } = useAsync(
       (s) =>
         appWriteChain
           .idle()
           .then(() =>
             Promise.all([
               api.survey(s),
               api.samples(s),
               api.surveySummary(s),
               api.resourceKeys(s),
               api.templates(s),
               api.resources(undefined, s),
               api.health(s),
             ]),
           ),
       [],
     );
   ```

3. State. Add after `setTypes`:

   ```tsx
     const [keys, setKeys] = useState<ResourceKey[]>([]);
     const [templates, setTemplates] = useState<Template[]>([]);
     const [templateId, setTemplateId] = useState<number | null>(null);
     const [resources, setResourcesState] = useState<Resource[]>([]);
     const resourcesRef = useRef<Resource[]>([]);
     const setResources = useCallback((update: (prev: Resource[]) => Resource[]) => {
       resourcesRef.current = update(resourcesRef.current);
       setResourcesState(resourcesRef.current);
     }, []);
     const projectRef = useRef('');
   ```

4. Seeding. In the seeding effect, after `setTypes(() => data[0].types);`, add:

   ```tsx
       setKeys(data[3]);
       setTemplates(data[4]);
       setResources(() => data[5]);
       projectRef.current = data[6].project.name;
       setTemplateId((current) => pickTemplate(data[4], current ?? readStoredTemplateId(projectRef.current))?.id ?? null);
   ```

   Add `setResources` to the effect's dependency list.

5. Derived values. Next to `selType` / `selPart`:

   ```tsx
     const template = templates.find((t) => t.id === templateId) ?? null;
     const columns = templateColumns(template, keys);
     const index = resourceIndex(resources);
   ```

6. Mutations. Add these to the mutations section:

   ```tsx
     function chooseTemplate(id: number) {
       setTemplateId(id);
       writeStoredTemplateId(projectRef.current, id);
     }

     /** Re-read every resource (after a survey PATCH changed sys: values — plan R5). */
     async function refreshResources() {
       const list = await api.resources();
       setResources(() => list);
     }

     /** Re-read the survey (after a resource write changed type state — plan R5). */
     async function refreshSurvey() {
       const s = await api.survey();
       setTypes(() => s.types);
     }

     function commitCell(code: string, keyId: string, value: string | null) {
       void mutate(async () => {
         const body = await api.patchResource(code, { [keyId]: value });
         setResources((prev) => replaceResources(prev, patchedResources(body)));
         if (touchesSurvey([keyId])) await refreshSurvey();
       });
     }

     function invalidValue(label: string) {
       toast.show(`Ugyldig værdi for ${label} — ikke gemt`);
     }
   ```

7. Make `patchType` and `patchPart` refresh resources after they replace the type or part:

   ```tsx
     function patchType(t: SurveyType, patch: Parameters<typeof api.patchSurveyType>[1]) {
       void mutate(async () => {
         const body = await api.patchSurveyType(t.id, patch);
         setTypes((prev) => replaceType(prev, body));
         await refreshResources();
       });
     }

     function patchPart(p: SurveyPart, patch: Parameters<typeof api.patchSurveyPart>[1]) {
       void mutate(async () => {
         const body = await api.patchSurveyPart(p.code, patch);
         setTypes((prev) => replacePart(prev, body));
         await refreshResources();
       });
     }
   ```

   Approve, reject and reopen keep calling `api.patchSurveyType` directly and do **not** refresh resources: review status is not a key.

8. SurveyTable props. Pass to `<SurveyTable>`:

   ```tsx
               columns={columns}
               resources={index}
               templates={templates}
               templateId={templateId}
               onTemplate={chooseTemplate}
               onCellCommit={commitCell}
               onInvalid={invalidValue}
               onAddResource={() => {}}
               onAddColumn={() => {}}
   ```

   The two no-op handlers are replaced in Tasks 7 and 8; the buttons do nothing until then.

- [ ] **Step 4: Typecheck, test, token lint**

```bash
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend test
python .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/kortlaegning/ResourceCell.module.css apps/rux/frontend/src/components/kortlaegning/SurveyTable.module.css --tsx
```

Expected:
- No type errors.
- All tests pass. `kortlaegning.surveyTable.test.ts` still imports only `typeRowClick`.
- The token lint is clean.

- [ ] **Step 5: Seed a demo project and check visually (both themes, desktop and phone)**

```bash
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
cmake --build build --target rux   # the worktree's own build; Phase 1+2 code
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/rt3-demo.rux"
RUX_BIN="$PWD/build/apps/rux/rux" bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/rt3-demo.rux" 8433 5183
curl -s localhost:8433/api/v1/templates | python3 -c 'import json,sys; print([(t["id"],t["name"],len(t["resolved_keys"])) for t in json.load(sys.stdin)["templates"]])'
curl -s localhost:8433/api/v1/resources | python3 -c 'import json,sys; r=json.load(sys.stdin)["resources"]; print(len(r), sorted(r[0]["values"])[:12])'
for theme in light dark; do
  bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5183/kortlaegning" --out "$SP/shots/rt3-t6" --theme $theme --viewports desktop,mobile
done
```

Expected:
- Both seeds are listed.
- Each resource's `values` includes `sys:` keys. If it does **not**, R1's reading is wrong. Stop and switch the page load to `api.resources(templateId)`, re-fetching on template change. The Task 2 functions are unchanged by that switch.
- Screenshots:
  - the Skabelon picker on "Hurtig genbrugsscreening";
  - columns Mængde · Enhed · EAK · BIM7AA · Behandling · Miljøstatus · Rum · Tons · Note · Vigtig, then Status;
  - "type" markers on the type-scoped headers;
  - muted `—` in blank cells;
  - the selected first type row showing editors in its type-scoped cells;
  - on phone width, the table scrolling inside its panel with no page-level horizontal scroll, and the two action buttons full-width.

  Read every screenshot and fix any overlap, clipping, or contrast problem in either theme before moving on.

Then, in a browser at `http://localhost:5183/kortlaegning` (or a short Playwright script under `$SP`), check:
- Switching to "Materialepas (fuld)" shows its leksikon columns, and reloading the page keeps that choice.
- Expanding a type and selecting a part, then typing `2,5` in a blank Tons cell and pressing Enter, sends `PATCH /api/v1/resources/<code>` with `{"values":{"sys:mass_t":"2.5"}}`. The type's other parts show `2,5 t` without a reload.
- Focusing and leaving a cell untouched sends nothing.
- `abc` in Tons shows the toast "Ugyldig værdi for Tons — ikke gemt" and snaps back.

Take one more shot of each theme on the full template. Then stop the servers: `bash .claude/skills/design-studio/scripts/dev_env.sh stop`.

- [ ] **Step 6: Commit**

```bash
git add apps/rux/frontend/src/components/kortlaegning/ResourceCell.tsx apps/rux/frontend/src/components/kortlaegning/ResourceCell.module.css apps/rux/frontend/src/components/kortlaegning/SurveyTable.tsx apps/rux/frontend/src/components/kortlaegning/SurveyTable.module.css apps/rux/frontend/src/routes/KortlaegningPage.tsx
git commit -m "feat(gui): template picker and template-built editable Kortlægning columns

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 7: FormDialog, Tilføj ressource, and Slet ressource

**Files:**
- Create: `apps/rux/frontend/src/components/kortlaegning/FormDialog.tsx`, `FormDialog.module.css`, `AddResourceDialog.tsx`
- Modify: `apps/rux/frontend/src/components/kortlaegning/DetailPanel.tsx`, `DetailPanel.module.css`
- Modify: `apps/rux/frontend/src/routes/KortlaegningPage.tsx`

**Interfaces:**
- Consumes:
  - from Task 2: `addableTypes`, `isManual`, `viewForNewResource`;
  - from Task 1: `api.createResource`, `api.deleteResource`;
  - `formKeyDown` (app/editorKeys), `createOnceGuard` (app/onceGuard);
  - from Task 6: `refreshSurvey` and `refreshResources`.
- Produces:
  - `FormDialog` props: `{ title: string; onCancel: () => void; onSubmit: () => void; children: ReactNode; actions: ReactNode; error?: string | null }`. Reused by Task 8.
  - `AddResourceDialog` props: `{ types: SurveyType[]; defaultTypeId: number | null; busy: boolean; onCancel: () => void; onSubmit: (body: ResourceCreate) => void }`.
  - New `DetailPanel` props: `manual: boolean`, `onDeleteResource: () => void`.

- [ ] **Step 1: Create FormDialog**

`FormDialog.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A small modal form over Kortlægning (Tilføj ressource, Tilføj kolonne).
 * Keys follow `formKeyDown`: Esc anywhere cancels (nothing is saved yet),
 * Ctrl/⌘+Enter submits; plain Enter submits natively. The page returns focus
 * to the table when the dialog closes.
 */

import { useId, type ReactNode } from 'react';

import { formKeyDown } from '../../app/editorKeys';
import styles from './FormDialog.module.css';

export interface FormDialogProps {
  title: string;
  onCancel: () => void;
  onSubmit: () => void;
  children: ReactNode;
  /** The footer buttons; the primary one is `type="submit"`. */
  actions: ReactNode;
  error?: string | null;
}

export function FormDialog({ title, onCancel, onSubmit, children, actions, error }: FormDialogProps) {
  const titleId = useId();
  return (
    <div className={styles.scrim} onMouseDown={(e) => e.target === e.currentTarget && onCancel()}>
      <form
        className={styles.dialog}
        role="dialog"
        aria-modal="true"
        aria-labelledby={titleId}
        onSubmit={(e) => {
          e.preventDefault();
          onSubmit();
        }}
        onKeyDown={(e) => formKeyDown(e, { onCancel, onSubmit })}
      >
        <h2 id={titleId} className={styles.title}>
          {title}
        </h2>
        <div className={styles.body}>{children}</div>
        {error && (
          <p className={styles.error} role="alert">
            {error}
          </p>
        )}
        <div className={styles.actions}>{actions}</div>
      </form>
    </div>
  );
}
```

`FormDialog.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.scrim {
  composes: scrim from './EditDialog.module.css';
}

.dialog {
  display: flex;
  flex-direction: column;
  gap: var(--space-3);
  width: min(calc(var(--space-7) * 11), calc(100vw - var(--space-4) * 2));
  max-height: 92vh;
  overflow: auto;
  padding: var(--space-5);
  background: var(--color-surface);
  border-radius: var(--radius-xl);
  box-shadow: var(--shadow-lg);
}

.title {
  margin: 0;
  font-size: var(--font-size-lg);
  font-weight: var(--font-weight-bold);
}

.body {
  composes: fields from '../controls.module.css';
}

.error {
  margin: 0;
  color: var(--tone-crit-ink);
  font-size: var(--font-size-sm);
}

.actions {
  display: flex;
  flex-wrap: wrap;
  justify-content: flex-end;
  gap: var(--space-2);
}

@media (max-width: 56.25rem) {
  .actions > button {
    flex: 1 1 100%;
    min-height: var(--layout-titlebar-height);
  }
}
```

Before relying on it, check that `controls.module.css` `.fields` is a vertical field stack (line 98). If it is a grid meant for a different layout, replace the `composes` with `display: flex; flex-direction: column; gap: var(--space-3);`.

- [ ] **Step 2: Create AddResourceDialog**

`AddResourceDialog.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/** "Tilføj ressource" (spec §6.1): a manual part under an existing type, with an optional name. */

import { useState } from 'react';

import type { ResourceCreate, SurveyType } from '../../api/types';
import { addableTypes } from '../../kortlaegning/resources';
import controls from '../controls.module.css';
import { FormDialog } from './FormDialog';

export interface AddResourceDialogProps {
  types: SurveyType[];
  /** The selected type, preselected. */
  defaultTypeId: number | null;
  busy: boolean;
  onCancel: () => void;
  onSubmit: (body: ResourceCreate) => void;
}

export function AddResourceDialog({ types, defaultTypeId, busy, onCancel, onSubmit }: AddResourceDialogProps) {
  const options = addableTypes(types);
  const [typeId, setTypeId] = useState<number | null>(
    options.some((t) => t.id === defaultTypeId) ? defaultTypeId : (options[0]?.id ?? null),
  );
  const [name, setName] = useState('');
  const [error, setError] = useState<string | null>(null);

  function submit() {
    if (typeId === null) {
      setError('Vælg en type.');
      return;
    }
    const trimmed = name.trim();
    onSubmit(trimmed ? { type_id: typeId, name: trimmed } : { type_id: typeId });
  }

  return (
    <FormDialog
      title="Tilføj ressource"
      onCancel={onCancel}
      onSubmit={submit}
      error={error}
      actions={
        <>
          <button type="button" className={controls.btnGhost} onClick={onCancel}>
            Annuller
          </button>
          <button type="submit" className={controls.btnPrimary} disabled={busy || typeId === null}>
            Tilføj ressource
          </button>
        </>
      }
    >
      <label className={controls.field}>
        <span className={controls.fieldLabel}>Type</span>
        <select
          className={controls.input}
          value={typeId === null ? '' : String(typeId)}
          onChange={(e) => setTypeId(Number(e.target.value))}
          autoFocus
        >
          {options.map((t) => (
            <option key={t.id} value={t.id}>
              {t.name}
            </option>
          ))}
        </select>
      </label>
      <label className={controls.field}>
        <span className={controls.fieldLabel}>Navn (valgfrit)</span>
        <input className={controls.input} value={name} onChange={(e) => setName(e.target.value)} />
      </label>
    </FormDialog>
  );
}
```

Check that `controls.module.css` exports `field`, `fieldLabel`, `input`, `btnGhost` and `btnPrimary` (it does, lines 28–134). Importing a shared module directly as `controls` is the pattern other case screens use. Confirm with `grep -rn "from '../controls.module.css'\|from '../../components/controls.module.css'" apps/rux/frontend/src | head -3`. If no TSX imports it directly, add local classes that `composes:` those rules in `FormDialog.module.css` and import from there instead.

- [ ] **Step 3: Add Slet ressource to DetailPanel**

In `DetailPanel.tsx`:

1. Add `import { useEffect, useState } from 'react';`.
2. Add to `DetailPanelProps`:

   ```ts
     /** The selected part was added by hand: it can be deleted (spec §4.4). */
     manual: boolean;
     onDeleteResource: () => void;
   ```

3. Add `manual` and `onDeleteResource` to the destructuring.
4. Inside the component, before the early return:

   ```tsx
     // Two-step delete: the first click arms, the second deletes. Re-armed per selection.
     const [armed, setArmed] = useState(false);
     const partCode = part?.code ?? null;
     useEffect(() => setArmed(false), [partCode]);
   ```

5. In `.actions`, after the star button, add:

   ```tsx
           {part && manual && (
             <button
               type="button"
               className={styles.danger}
               disabled={busy}
               onClick={() => {
                 if (!armed) {
                   setArmed(true);
                   return;
                 }
                 setArmed(false);
                 onDeleteResource();
               }}
             >
               {armed ? `Bekræft: slet ${part.code}` : 'Slet ressource'}
             </button>
           )}
   ```

6. Add the `Manuel` pill to `.head` after the BIM7AA pill: `{part && manual && <Pill tone="accent">Manuel</Pill>}`.
7. Append to `DetailPanel.module.css`:

```css
.danger {
  composes: btnDanger from '../controls.module.css';
}
```

The Phase 3 detail-panel tests import pure functions only, so they are unaffected.

- [ ] **Step 4: Wire both into the page**

In `KortlaegningPage.tsx`:

1. Add the imports:

   ```tsx
   import type { ResourceCreate } from '../api/types';
   import { createOnceGuard } from '../app/onceGuard';
   import { AddResourceDialog } from '../components/kortlaegning/AddResourceDialog';
   import { isManual, viewForNewResource } from '../kortlaegning/resources';
   ```

   Merge the last import into the existing `resources` import.

2. Add the state and the create/delete functions:

   ```tsx
     const [addResourceOpen, setAddResourceOpen] = useState(false);
     // A create burns a server-assigned code: a double tap must send once (Review Focus).
     const createGuard = useRef(createOnceGuard());

     function addResource(body: ResourceCreate) {
       createGuard.current.run(() =>
         mutate(async () => {
           const created = await api.createResource(body);
           const [s, list] = await Promise.all([api.survey(), api.resources()]);
           const next = setTypes(() => s.types);
           setResources(() => list);
           const view = viewForNewResource(next, created.type_id, viewRef.current.tab, viewRef.current.filters);
           setTab(view.tab);
           setFilters(view.filters);
           select({ typeId: created.type_id, partCode: created.code });
           setAddResourceOpen(false);
           toast.show(`${created.code} tilføjet`);
         }),
       );
     }

     function deleteResource() {
       const p = selPart;
       if (!p || !isManual(p)) return;
       void mutate(async () => {
         await api.deleteResource(p.code);
         const [s, list] = await Promise.all([api.survey(), api.resources()]);
         setTypes(() => s.types);
         setResources(() => list);
         select({ typeId: p.type_id, partCode: null });
         toast.show(`${p.code} slettet`);
       });
     }
   ```

   The dialog closes only on success. On failure `mutate`'s `onError` shows the toast and the dialog stays open with its input.

3. Pass `onAddResource={() => setAddResourceOpen(true)}` to `SurveyTable`, and `manual={selPart !== null && isManual(selPart)}` and `onDeleteResource={deleteResource}` to `DetailPanel`.
4. Render the dialog next to `EditDialog`:

   ```tsx
         {addResourceOpen && (
           <AddResourceDialog
             types={types}
             defaultTypeId={selType?.id ?? null}
             busy={busy}
             onCancel={() => setAddResourceOpen(false)}
             onSubmit={addResource}
           />
         )}
   ```

5. Focus return. Extend the existing `wasOpen` effect so any dialog closing hands focus back to the table:

   ```tsx
     const anyDialog = dialogOpen || addResourceOpen;
     const wasOpen = useRef(false);
     useEffect(() => {
       if (wasOpen.current && !anyDialog) tableRef.current?.focus({ preventScroll: true });
       wasOpen.current = anyDialog;
     }, [anyDialog]);
   ```

   Task 8 adds `addColumnOpen` to `anyDialog`.

6. The empty state (`types.length === 0`) keeps only "Opret kortlægning fra instanser". Adding a resource needs a type, so it is not offered there.

- [ ] **Step 5: Typecheck, test, token lint**

```bash
npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend test
python .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/kortlaegning/FormDialog.module.css apps/rux/frontend/src/components/kortlaegning/DetailPanel.module.css --tsx
```

Expected: no type errors, all tests pass, and the token lint is clean.

- [ ] **Step 6: Visual and behaviour check (both themes, desktop and phone)**

Start the servers as in Task 6 Step 5 on a fresh copy (`dev_env.sh start "$SP/rt3-demo.rux" 8433 5183`). Use a Playwright script in `$SP/rt3_t7.py` that waits with `expect`/`expect_response`, never with fixed sleeps:

1. Open `/kortlaegning`, click "+ Tilføj ressource" and take a screenshot of the dialog in both themes at desktop and mobile.
2. Pick "Indvendige døre, træ", type the name `Ekstra dør`, and **double-click** the submit button.
   - Assert exactly **one** `POST /api/v1/resources` request was captured.
   - Then `expect_response` for it, and assert the dialog closed.
   - The new part row is selected with a `Manuel` pill (shot).
3. In the detail panel, click "Slet ressource". It must read `Bekræft: slet RX-0NN`. Click again and assert one `DELETE /api/v1/resources/RX-0NN`. The type row is selected.
4. Select an instance-backed or seeded part whose `instance_guid` is set, if the demo has any, and assert that no "Slet ressource" button is shown.
5. Esc in the dialog closes it, and focus is back on the table: `document.activeElement` is the `.wrap` div.

Read every screenshot and fix any problem in either theme before committing. Then stop the servers.

- [ ] **Step 7: Commit**

```bash
git add apps/rux/frontend/src/components/kortlaegning/FormDialog.tsx apps/rux/frontend/src/components/kortlaegning/FormDialog.module.css apps/rux/frontend/src/components/kortlaegning/AddResourceDialog.tsx apps/rux/frontend/src/components/kortlaegning/DetailPanel.tsx apps/rux/frontend/src/components/kortlaegning/DetailPanel.module.css apps/rux/frontend/src/routes/KortlaegningPage.tsx
git commit -m "feat(gui): Tilføj ressource and Slet ressource for manual parts

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 8: Tilføj kolonne, with "Opret kopi af skabelonen i stedet"

**Files:**
- Create: `apps/rux/frontend/src/components/kortlaegning/AddColumnDialog.tsx`
- Modify: `apps/rux/frontend/src/routes/KortlaegningPage.tsx`

**Interfaces:**
- Consumes:
  - from Task 5: `COLUMN_KINDS`, `COLUMN_KIND_LABEL`, `EMPTY_COLUMN_DRAFT`, `ColumnDraft`, `columnDraftError`, `columnCreateBody`, `seedNote`, `duplicateFirst`, `columnPartialFailureMessage`;
  - from Task 4: `appendKeyMember`;
  - from Task 2: `resourceColumnKeyId`;
  - from Task 1: `api.createResourceColumn`, `api.duplicateTemplate`, `api.patchTemplate`, `api.resourceKeys`, `api.templates`;
  - `FormDialog` (Task 7); `errorMessage` (app/saveError).
- Produces: `AddColumnDialog` props `{ template: Template | null; existingLabels: string[]; busy: boolean; onCancel: () => void; onSubmit: (draft: ColumnDraft, copyInstead: boolean) => void }`.

- [ ] **Step 1: Create AddColumnDialog**

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * "Tilføj kolonne" (spec §6.1): create a user column and append it to the
 * selected template. On a seed template the dialog says so and offers
 * "Opret kopi af skabelonen i stedet" (plan R11). The draft is validated
 * here, before any request (`columnDraftError`).
 */

import { useState } from 'react';

import type { Template } from '../../api/types';
import {
  COLUMN_KIND_LABEL,
  COLUMN_KINDS,
  columnDraftError,
  EMPTY_COLUMN_DRAFT,
  seedNote,
  type ColumnDraft,
  type ColumnKind,
} from '../../kortlaegning/columnDraft';
import controls from '../controls.module.css';
import { FormDialog } from './FormDialog';

export interface AddColumnDialogProps {
  template: Template | null;
  /** Every catalogue key's label: a new column may not reuse one. */
  existingLabels: string[];
  busy: boolean;
  onCancel: () => void;
  onSubmit: (draft: ColumnDraft, copyInstead: boolean) => void;
}

export function AddColumnDialog({ template, existingLabels, busy, onCancel, onSubmit }: AddColumnDialogProps) {
  const [draft, setDraft] = useState<ColumnDraft>(EMPTY_COLUMN_DRAFT);
  const [error, setError] = useState<string | null>(null);
  const note = seedNote(template);

  function submit(copyInstead: boolean) {
    const problem = columnDraftError(draft, existingLabels);
    setError(problem);
    if (!problem) onSubmit(draft, copyInstead);
  }

  return (
    <FormDialog
      title="Tilføj kolonne"
      onCancel={onCancel}
      onSubmit={() => submit(false)}
      error={error}
      actions={
        <>
          <button type="button" className={controls.btnGhost} onClick={onCancel}>
            Annuller
          </button>
          {note && (
            <button type="button" className={controls.btnGhost} disabled={busy} onClick={() => submit(true)}>
              Opret kopi af skabelonen i stedet
            </button>
          )}
          <button type="submit" className={controls.btnPrimary} disabled={busy}>
            Tilføj kolonne
          </button>
        </>
      }
    >
      <label className={controls.field}>
        <span className={controls.fieldLabel}>Navn</span>
        <input
          className={controls.input}
          value={draft.name}
          onChange={(e) => setDraft({ ...draft, name: e.target.value })}
          autoFocus
        />
      </label>
      <label className={controls.field}>
        <span className={controls.fieldLabel}>Type</span>
        <select
          className={controls.input}
          value={draft.kind}
          onChange={(e) => setDraft({ ...draft, kind: e.target.value as ColumnKind })}
        >
          {COLUMN_KINDS.map((k) => (
            <option key={k} value={k}>
              {COLUMN_KIND_LABEL[k]}
            </option>
          ))}
        </select>
      </label>
      {draft.kind === 'select' && (
        <label className={controls.field}>
          <span className={controls.fieldLabel}>Valgmuligheder (komma eller ny linje)</span>
          <textarea
            className={controls.input}
            rows={3}
            value={draft.optionsText}
            onChange={(e) => setDraft({ ...draft, optionsText: e.target.value })}
          />
        </label>
      )}
      {note && <p className={controls.fieldLabel}>{note}</p>}
    </FormDialog>
  );
}
```

`formKeyDown` treats plain Enter in the textarea as a new line: its `kind` is text, and only Ctrl/⌘+Enter submits. Native form submission does not fire from a textarea on Enter.

- [ ] **Step 2: Wire it into the page**

1. Add the imports:

   ```tsx
   import { AddColumnDialog } from '../components/kortlaegning/AddColumnDialog';
   import { columnCreateBody, columnPartialFailureMessage, duplicateFirst, type ColumnDraft } from '../kortlaegning/columnDraft';
   import { appendKeyMember } from '../kortlaegning/templatePick';
   import { errorMessage } from '../app/saveError';
   ```

   Add `resourceColumnKeyId` to the `resources` import.

2. Add the state and the function:

   ```tsx
     const [addColumnOpen, setAddColumnOpen] = useState(false);
     const columnGuard = useRef(createOnceGuard());

     function addColumn(draft: ColumnDraft, copyInstead: boolean) {
       const base = template;
       if (!base) return;
       columnGuard.current.run(() =>
         mutate(async () => {
           const def = await api.createResourceColumn(columnCreateBody(draft));
           try {
             const target = duplicateFirst(base, copyInstead) ? await api.duplicateTemplate(base.id) : base;
             await api.patchTemplate(target.id, { members: appendKeyMember(target.members, resourceColumnKeyId(def.id)) });
             chooseTemplate(target.id);
             toast.show(`Kolonnen »${def.name}« er tilføjet til »${target.name}«`);
             setAddColumnOpen(false);
           } catch (cause) {
             toast.show(columnPartialFailureMessage(def.name, errorMessage(cause)));
             setAddColumnOpen(false);
           } finally {
             const [k, t] = await Promise.all([api.resourceKeys(), api.templates()]);
             setKeys(k);
             setTemplates(t);
           }
         }),
       );
     }
   ```

   A failure of `createResourceColumn` itself, such as a 400 or a 409 on a duplicate name, goes to `mutate`'s `onError` toast, and the dialog stays open. A failure after the column exists closes the dialog with the partial-failure copy (Review Focus), then re-reads the catalogue so the new column shows under "Egne felter".

3. Pass `onAddColumn={() => setAddColumnOpen(true)}` to `SurveyTable`.

4. Render:

   ```tsx
         {addColumnOpen && (
           <AddColumnDialog
             template={template}
             existingLabels={keys.map((k) => k.label)}
             busy={busy}
             onCancel={() => setAddColumnOpen(false)}
             onSubmit={addColumn}
           />
         )}
   ```

5. Extend focus return: `const anyDialog = dialogOpen || addResourceOpen || addColumnOpen;`.

The user-column write itself is passport-scoped. A new column's cells are blank (`null`) until filled, and a fill goes through `commitCell` like any other.

- [ ] **Step 3: Typecheck and test**

Run: `npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend test`
Expected: PASS.

- [ ] **Step 4: Visual and behaviour check (both themes, desktop and phone)**

On a fresh served copy, using a Playwright script `$SP/rt3_t8.py`:

1. With "Hurtig genbrugsscreening" selected, open "+ Tilføj kolonne".
   - Take a shot in both themes, desktop and mobile. It must show the seed note and both submit buttons.
   - Submit with an empty name: "Angiv et navn." appears, and no request is captured.
   - Type `Note`, the label of `sys:note`: "Der findes allerede et felt med det navn.", and no request.
2. Name `Stand`, type Valgliste, options `God, Dårlig`, then "Tilføj kolonne". Assert the captured order:
   - `POST /api/v1/resources/columns`
   - `PATCH /api/v1/templates/<screening id>` with `members` ending in `{"key":"col:<id>"}`
   - `GET /api/v1/resources/keys` and `GET /api/v1/templates`

   A "Stand" column appears last, before Status.
3. Select a part, choose `God` in its Stand cell, and assert `PATCH /api/v1/resources/<code>` with `{"values":{"col:<id>":"God"}}`.
4. Open the dialog again, name it `Leverandør`, and click "Opret kopi af skabelonen i stedet". Assert:
   - `POST /api/v1/templates/<id>/duplicate` comes **before** the `PATCH` of the copy's id;
   - the picker now shows "Hurtig genbrugsscreening (kopi)" with a Leverandør column;
   - switching back to the seed shows Stand but **not** Leverandør.

Shot the table with the new columns in both themes. Read every shot and fix any problem before committing. Then stop the servers.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/src/components/kortlaegning/AddColumnDialog.tsx apps/rux/frontend/src/routes/KortlaegningPage.tsx
git commit -m "feat(gui): Tilføj kolonne appends a user column to the template (or a copy of a seed)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 9: Alle egenskaber in the detail panel

**Files:**
- Modify: `apps/rux/frontend/src/components/kortlaegning/DetailPanel.tsx`, `DetailPanel.module.css`
- Modify: `apps/rux/frontend/src/routes/KortlaegningPage.tsx`

**Interfaces:**
- Consumes: `allPropertyGroups` and `PropertyGroup` (Task 2); `ResourceCell` (Task 6); `commitCell` and `invalidValue` (Task 6).
- Produces: new `DetailPanel` props `resource: Resource | null`, `catalogue: ResourceKey[]`, `onCellCommit: (code: string, keyId: string, value: string | null) => void`, `onInvalid: (label: string) => void`, `home: RefObject<HTMLElement | null>`.

- [ ] **Step 1: Render the groups**

In `DetailPanel.tsx`:

1. Add the imports:

   ```tsx
   import type { RefObject } from 'react';
   import type { Resource, ResourceKey } from '../../api/types';
   import { allPropertyGroups } from '../../kortlaegning/resources';
   import { ResourceCell } from './ResourceCell';
   ```

2. Add the five props above to `DetailPanelProps`, with doc comments:
   - `resource`: "the selected part's values; null for a type";
   - `home`: "where Esc/Enter in a property field returns focus".

   Destructure them.

3. After the `Proces / håndtering` field and before `.actions`, add:

   ```tsx
         <section className={styles.props} aria-label="Alle egenskaber">
           <h3 className={styles.propsHeading}>Alle egenskaber</h3>
           {!part ? (
             <p className={styles.propsEmpty}>Vælg en bygningsdel for at se alle dens egenskaber.</p>
           ) : groups.length === 0 ? (
             <p className={styles.propsEmpty}>Ingen udfyldte egenskaber endnu — udfyld felter i tabellen.</p>
           ) : (
             groups.map((g, i) => (
               <details key={g.category} className={styles.group} open={i === 0}>
                 <summary className={styles.groupSummary}>
                   {g.category} <span className={styles.groupCount}>{g.keys.length}</span>
                 </summary>
                 <div className={styles.groupBody}>
                   {g.keys.map((key) => (
                     <div key={key.id} className={styles.field}>
                       <span className={styles.label}>{key.label}</span>
                       <ResourceCell
                         resourceKey={key}
                         value={resource?.values[key.id] ?? null}
                         editing={key.editable}
                         variant="field"
                         onCommit={(v) => onCellCommit(part.code, key.id, v)}
                         onInvalid={onInvalid}
                         home={home}
                       />
                     </div>
                   ))}
                 </div>
               </details>
             ))
           )}
         </section>
   ```

   with `const groups = allPropertyGroups(part ? resource : null, catalogue);` computed after the early return.

   Clearing a value here sends `null`, and the key then leaves the list on the next render: it no longer has a value. That is the spec's definition of "Alle egenskaber".

4. Append to `DetailPanel.module.css`:

```css
/* ----------------------------------------------------- Alle egenskaber -- */

.props {
  display: flex;
  flex-direction: column;
  gap: var(--space-2);
  padding-top: var(--space-3);
  border-top: 1px solid var(--color-border);
}

.propsHeading {
  margin: 0;
  color: var(--color-text-muted);
  font-size: var(--font-size-xs);
  font-weight: var(--font-weight-bold);
  letter-spacing: var(--tracking-caps);
  text-transform: uppercase;
}

.propsEmpty {
  margin: 0;
  color: var(--color-text-faint);
  font-size: var(--font-size-sm);
}

.group {
  border: 1px solid var(--color-border);
  border-radius: var(--radius-md);
  background: var(--color-surface-sunken);
}

.groupSummary {
  padding: var(--space-2) var(--space-3);
  font-size: var(--font-size-sm);
  font-weight: var(--font-weight-bold);
  cursor: pointer;
}

.groupSummary:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: -2px;
}

.groupCount {
  color: var(--color-text-faint);
  font-weight: var(--font-weight-regular);
}

.groupBody {
  display: flex;
  flex-direction: column;
  gap: var(--space-2);
  padding: 0 var(--space-3) var(--space-3);
}

@media (max-width: 56.25rem) {
  .groupSummary {
    min-height: var(--layout-titlebar-height);
    display: flex;
    align-items: center;
  }
}
```

The `.field` and `.label` classes already exist in `DetailPanel.module.css`; reuse them.

- [ ] **Step 2: Pass the props from the page**

```tsx
              resource={selPart ? (index.get(selPart.code) ?? null) : null}
              catalogue={keys}
              onCellCommit={commitCell}
              onInvalid={invalidValue}
              home={tableRef}
```

- [ ] **Step 3: Typecheck, test, token lint**

```bash
npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend test
python .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/kortlaegning/DetailPanel.module.css --tsx
```

Expected: PASS and a clean lint.

- [ ] **Step 4: Visual check (both themes, desktop and phone)**

On a fresh served copy:
1. Fill a Producent value on one part by switching to "Materialepas (fuld)" and typing into a leksikon cell. Fill a Stand value too, if Task 8's column is present on this copy; otherwise create it as in Task 8.
2. Select that part, then take shots of the detail panel in both themes, desktop and mobile. Expected:
   - "Alle egenskaber" shows the Kortlægning group open, plus the leksikon and "Egne felter" groups, each with a count.
   - Each value is editable; Miljøstatus is shown as a pill, not editable.
3. Edit Producent there and assert `PATCH /api/v1/resources/<code>`. The table cell under the full template shows the new value.
4. Select a type row: the copy "Vælg en bygningsdel …" shows.

Read the shots and fix problems. Then stop the servers.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/src/components/kortlaegning/DetailPanel.tsx apps/rux/frontend/src/components/kortlaegning/DetailPanel.module.css apps/rux/frontend/src/routes/KortlaegningPage.tsx
git commit -m "feat(gui): Alle egenskaber — every value a resource has, grouped and editable

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 10: Retire Materialedata (MaterialsPage, MaterialTable cluster) and redirect `/materials`

**Files:**
- Delete:
  - `src/routes/MaterialsPage.tsx`
  - `src/components/MaterialTable.tsx`, `MaterialTable.module.css`
  - `PeekPanel.tsx`, `PeekPanel.module.css`
  - `EditableCell.tsx`, `EditableCell.module.css`
  - `ColumnHeaderMenu.tsx`, `ColumnHeaderMenu.module.css`
  - `ColumnFilterCell.tsx`, `columnFilterHelpers.ts`, `SortableColumnHeader.tsx`, `tableNav.ts`
  - `ThumbnailCell.tsx`, `ThumbnailCell.module.css`
  - `src/test/columnFilter.test.ts`

  All paths are under `apps/rux/frontend/`.
- Keep: `SelectDropdown.tsx` / `MultiSelectDropdown.tsx` (+css) (R6).
- Modify: `apps/rux/frontend/src/app/App.tsx`

- [ ] **Step 1: Prove each file has no importer outside the cluster**

```bash
cd apps/rux/frontend
CLUSTER='MaterialTable|MaterialsPage|PeekPanel|EditableCell|ColumnHeaderMenu|ColumnFilterCell|columnFilterHelpers|SortableColumnHeader|tableNav|ThumbnailCell'
grep -rnE "from '[^']*/($CLUSTER)(\.module\.css)?'" src --include=*.ts --include=*.tsx \
  | grep -vE "^src/(components/($CLUSTER)|routes/MaterialsPage|test/columnFilter)" ; echo "exit=$?"
cd -
```

Expected: the only remaining hit is `src/app/App.tsx` importing `MaterialsPage`, followed by `exit=0`. If anything else imports a cluster file, keep that file, and record why in the commit message.

- [ ] **Step 2: Delete the files and redirect the route**

```bash
cd apps/rux/frontend/src
git rm routes/MaterialsPage.tsx \
  components/MaterialTable.tsx components/MaterialTable.module.css \
  components/PeekPanel.tsx components/PeekPanel.module.css \
  components/EditableCell.tsx components/EditableCell.module.css \
  components/ColumnHeaderMenu.tsx components/ColumnHeaderMenu.module.css \
  components/ColumnFilterCell.tsx components/columnFilterHelpers.ts \
  components/SortableColumnHeader.tsx components/tableNav.ts \
  components/ThumbnailCell.tsx components/ThumbnailCell.module.css \
  test/columnFilter.test.ts
cd -
```

If a listed `.module.css` does not exist, drop it from the command. `ls` the directory first.

In `apps/rux/frontend/src/app/App.tsx`:
- Delete `import { MaterialsPage } from '../routes/MaterialsPage';`.
- Replace `<Route path="/materials" element={<MaterialsPage />} />` with `<Route path="/materials" element={<Navigate to={KORTLAEGNING_PATH} replace />} />`.

If Phase 2 introduced a redirect table instead of inline `<Navigate>` routes (check with `grep -rn "'/onsite'" apps/rux/frontend/src/app`), add `/materials → KORTLAEGNING_PATH` there in the same form, and extend that table's test in `navigation.test.ts` / `links.test.ts` with:

```ts
  it('sends the retired Materialedata path to Kortlægning', () => {
    expect(REDIRECTS['/materials']).toBe(KORTLAEGNING_PATH);
  });
```

Use the table's real exported name.

Also update the keep-alive comment in `App.tsx` that names `/materials` as an example: change "`/geometry`, `/materials`, etc." to "`/geometry`, `/kortlaegning`, etc.".

- [ ] **Step 3: Typecheck, test, build**

```bash
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run build
grep -rn "MaterialTable\|MaterialsPage\|PeekPanel\|EditableCell\|ThumbnailCell\|tableNav" apps/rux/frontend/src ; echo "exit=$?"
```

Expected:
- typecheck, test and build pass;
- the grep prints nothing, then `exit=1`. A doc comment in `SelectDropdown.tsx` that mentions `MaterialTable` is fine to reword to "a user column".

- [ ] **Step 4: Check the redirect in the browser**

With the servers up, run `shot.sh "http://localhost:5183/materials" --out "$SP/shots/rt3-t10" --theme light --viewports desktop`. The shot must be Kortlægning, and the URL must end in `/kortlaegning`; check `page.url` in a two-line Playwright snippet if `shot.sh` does not print it.

- [ ] **Step 5: Commit**

```bash
git add -A apps/rux/frontend/src
git commit -m "refactor(gui): retire Materialedata — Kortlægning takes over; /materials redirects

Deletes MaterialsPage, MaterialTable and the components only it used
(PeekPanel, EditableCell, column header/filter/sort, tableNav, ThumbnailCell).
SelectDropdown and MultiSelectDropdown stay as shared components.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 11: Remove the old `/api/v1/material-columns` route from `rux gui`

**Files:**
- Modify: `apps/rux/src/gui/Server.cpp:1035-1059` (the two `route_dynamic("/api/v1/material-columns…")` blocks)
- Modify: `apps/rux/src/gui/api.cpp:677-684` (four endpoint-table rows)
- Modify: `tests/unit/rux_gui/test_gui_api.cpp:150-153` (four expected rows)
- Modify: `docs/gui/openapi.yaml:1637-1743` (the `# material columns` comment, `/material-columns` and `/material-columns/{id}`)

**Interfaces:**
- Consumes: Phase 1's `/api/v1/resources/columns` routes, which must already call the same `material_columns_json` / `create_material_column` / `patch_material_column` / `delete_material_column` handlers. Those handlers stay.
- Produces: no `material-columns` path in `rux gui`.

- [ ] **Step 1: Update the route-contract test first (failing)**

In `tests/unit/rux_gui/test_gui_api.cpp`, delete these four lines from the `expected` set in `EndpointTable_DocumentedRoutes_MatchesContract`:

```cpp
      "GET /api/v1/material-columns",
      "POST /api/v1/material-columns",
      "PATCH /api/v1/material-columns/<string>",
      "DELETE /api/v1/material-columns/<string>",
```

Confirm the Phase 1 `"… /api/v1/resources/columns…"` rows are present in the same set (`grep -n "resources/columns" tests/unit/rux_gui/test_gui_api.cpp`).

Build and run it:

```bash
cmake --build build --target reusex_unit_tests
cd build && ctest --output-on-failure -R "EndpointTable_DocumentedRoutes_MatchesContract" ; cd -
```

Expected: FAIL. `actual` still holds the four `material-columns` rows.

- [ ] **Step 2: Remove the endpoint rows and the routes**

In `apps/rux/src/gui/api.cpp`, delete the four `{"…", "/api/v1/material-columns…", "…"}` rows (lines 677–684).

In `apps/rux/src/gui/Server.cpp`, delete both `app_.route_dynamic("/api/v1/material-columns")` and `app_.route_dynamic("/api/v1/material-columns/<string>")` blocks, about lines 1035–1059. Leave the handler functions in `api.cpp` alone: the `/resources/columns` routes call them. Confirm:

```bash
grep -n "material_columns_json\|create_material_column\|patch_material_column\|delete_material_column" apps/rux/src/gui/Server.cpp
```

Expected: hits only inside the `/api/v1/resources/columns` route blocks.

- [ ] **Step 3: Remove the paths from the contract**

In `docs/gui/openapi.yaml`, delete everything from the line `  # ---------------------------------------------------- material columns ----` through the `default:` response of `deleteMaterialColumn`: that is the `/material-columns` and `/material-columns/{id}` path items, ending right before `  # ----------------------------------------------------------- instances ----`.

Keep `components.schemas.PropertyDefinition`: `/resources/columns` uses it. Check:

```bash
grep -n "material-columns\|MaterialColumn" docs/gui/openapi.yaml ; echo "exit=$?"
python3 -c "import yaml,sys; yaml.safe_load(open('docs/gui/openapi.yaml')); print('yaml ok')"
```

Expected: the grep prints nothing, then `exit=1`; then `yaml ok`.

- [ ] **Step 4: Build and run the GUI tests**

```bash
cmake --build build --target reusex_unit_tests rux
cd build && ctest --output-on-failure --parallel $(nproc) -R "\[gui\]|gui|Gui|EndpointTable|EndpointsJson" ; cd -
```

Expected: PASS.

Then check the live server on a scratch copy:

```bash
./build/apps/rux/rux -p "$SP/rt3-demo.rux" gui --port 8434 --no-browser & S=$!
for i in $(seq 1 30); do curl -sf -o /dev/null localhost:8434/api/v1/health && break; sleep 1; done
curl -s -o /dev/null -w "%{http_code}\n" localhost:8434/api/v1/material-columns   # expect 404
curl -s -o /dev/null -w "%{http_code}\n" localhost:8434/api/v1/resources/columns  # expect 200
kill $S
```

- [ ] **Step 5: Commit**

```bash
git add apps/rux/src/gui/Server.cpp apps/rux/src/gui/api.cpp tests/unit/rux_gui/test_gui_api.cpp docs/gui/openapi.yaml
git commit -m "refactor(gui): drop /api/v1/material-columns — /resources/columns replaces it

The last frontend user (MaterialTable/ExportPage) moved to the new path.
ruxd keeps its own handlers (separate binary, out of scope).

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 12: Phase verification

- [ ] **Step 1: The whole frontend gate**

```bash
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run build
python .claude/skills/design-studio/scripts/token_lint.py $(git diff --name-only main -- 'apps/rux/frontend/src/**/*.css') --tsx
```

Expected: all green, and the lint is clean.

- [ ] **Step 2: The end-to-end journey and final shots (both themes, desktop and phone)**

On a fresh served copy (`dev_env.sh start "$SP/rt3-demo.rux" 8433 5183`), run one Playwright script, `$SP/rt3_journey.py`. It waits only on `expect` and captured responses. Steps:

1. Open `/materials`; it lands on `/kortlaegning`.
2. Switch template to "Materialepas (fuld)", reload, and assert it is still selected.
3. Fill a blank leksikon cell on a part, then clear it. Assert `"x"` and then `null` were sent.
4. Change Behandling on a part. The sibling part row, the type row's Behandling pill and the detail panel's Behandling select all show the new value.
5. Tilføj ressource → Slet ressource.
6. Tilføj kolonne on the seed, then a copy.
7. Edit under Alle egenskaber.
8. Reject a type with A and approve one with G, to check the review flow is intact: the 422 toast for an `afventer` type is unchanged.

Final shots: `/kortlaegning` on the screening and on the full template, the detail panel with Alle egenskaber, and both dialogs. Each in `light` and `dark`, `desktop` and `mobile`, into `$SP/shots/rt3-final`. Read every shot. There must be no horizontal page scroll at phone width (the table scrolls in its panel) and no unreadable text in dark mode.

- [ ] **Step 3: Exit notes**

Report to the controller, not as a file:
- **Phase 4 should reuse:**
  - the client: `api.templates`, `api.duplicateTemplate`, `api.patchTemplate`, `api.resourceKeys`, `api.resources(templateId?)`;
  - the types: `Template`, `TemplateMember`, `TemplateSeed`, `TemplateCsv`, `TemplatePatch`, `ResourceKey`;
  - `appendKeyMember` from `kortlaegning/templatePick.ts`.

  Phase 4 still has to add `createTemplate`, `deleteTemplate` and `restoreSeedTemplates`.
- `SelectDropdown` / `MultiSelectDropdown` are now unused (R6). `.claude/skills/design-studio/references/reusex-frontend.md` still lists the deleted `PeekPanel` (R7).
- ruxd still serves `/material-columns` (R9).

---

## Phase exit criteria

- Kortlægning shows a Skabelon picker. It defaults to the screening seed, falls back to the first template, and is remembered per project.
- The parts table's columns are the template's `resolved_keys` in order: the tree column first, then the dynamic columns, then Status.
  - Type-scoped columns carry a "type" marker with the tooltip "Gælder alle dele af typen".
  - Blanks show a muted `—`.
  - Miljøstatus is read-only.
- Inline edits in the selected row:
  - send `PATCH /resources/<code>`;
  - a type-scoped edit updates the type's other parts;
  - an untouched blur sends nothing;
  - emptying a cell sends `null`.
- Tilføj ressource creates exactly one manual part with a "Manuel" marker. Only manual parts offer Slet ressource.
- Tilføj kolonne creates a user column and appends it to the template. On a seed it says so, and "Opret kopi af skabelonen i stedet" duplicates the seed first.
- The detail panel's "Alle egenskaber" lists every key with a value, grouped by category, collapsible and editable.
- No Materialedata code is left, `/materials` redirects to `/kortlaegning`, and `rux gui` no longer serves `/api/v1/material-columns`.
- vitest, typecheck, build, token lint and the gui ctest set are green. Screenshots in both themes at desktop and phone width have been read and are clean.
