<!--

> **Controller reconciliation with the Phase 1 plan (binding, overrides assumptions below):**
> The report-generate request field is **`resource_template_id`** (int; absent = no Ressourcetabel).
> Template `csv` = `{delimiter: ";"|","|"\t", encoding: "utf-8"|"utf-8-bom", header: "label"|"key"}` plus kept extras;
> backend defaults are **`;`, `utf-8-bom`, `label`** — use these as `CSV_DEFAULTS`. `POST /templates/restore-seeds` returns
> `{restored: [names], templates}`; `GET /templates` returns `{templates}`. Old `/export-templates` routes are backed by the
> `templates` table after Phase 1 (ProjectDB `*_export_template` is a view over it).
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Resources & templates Phase 4 — Skabeloner page and Rapport Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Load the project skill `design-studio` (`.claude/skills/design-studio/SKILL.md`) before touching any `.tsx`/`.css`.

**Goal:** Finish the resources/templates redesign:
- a **Skabeloner** page at `/skabeloner`, where templates are created, renamed, duplicated, deleted, restored from seeds and edited (categories, single keys, order);
- **Rapport** takes over Eksport: a **Ressourcetabel** option on PDF generation and a **Data-eksport** panel (template picker, CSV options saved to the template, "Download CSV");
- `ExportPage`, its `/export` route and the `rux gui` export-template routes are deleted (`/export` redirects to `/rapport`);
- docs: CLAUDE.md `gui` row, `docs/DIRECTION.md` dated changelog, the On-site note in the phase spec.

**Architecture:** Everything that can be pure is pure and unit-tested in Node (vitest, no DOM):
- `src/skabeloner/members.ts`: member identity, category toggle, add/remove/move, local template resolution (a mirror of the server's §5.2 resolver, so the count line and struck-through members react instantly), catalogue search;
- `src/skabeloner/model.ts`: Danish copy, error mapping, new-name and selection rules, the "latest edit wins" gate for save-on-change, list-state replacement helpers;
- `src/rapport/csvOptions.ts`: reading/writing the template's `csv` JSON as typed CSV options, option lists, file name;
- `src/rapport/model.ts` (existing): the Ressourcetabel choice and the default export template.

Components are presentational; each route owns its state and writes through `useMutationQueue` (app chain for template writes, the existing page chain for PDF generation). Every value on screen comes from a response body or from the pure resolver applied to response bodies.

**Tech Stack:** React 19, react-router-dom 7, TypeScript, CSS Modules (tokens only), vitest (Node). C++20 `rux_gui_lib` (route deletion only), Catch2 v3. Playwright via the design-studio scripts.

**Spec:** `docs/superpowers/specs/2026-10-02-resources-templates-ia-design.md` — §3 (nav, redirects), §4.4 (keys), §5 (templates, resolution, seeds, API), §6.2 (Skabeloner), §6.3 (Rapport), §7 (errors), §8 Docs bullet, §9 Frontend/Visual. Phases 1–3 of the same spec are merged on `main` before this plan starts.

## Rulings (spec silent, or Phase 1–3 left a choice)

- **R1 — Client names are whatever Phase 3 chose.** Task 1 records the real names in the **Name map** below and every later task uses the map. Where this plan writes `api.listTemplates`, `api.resourceKeys`, `api.duplicateTemplate`, `Template`, `ResourceKey`, `TemplateMember`, read "the name in the map".
- **R2 — Count and missing members are resolved locally.** The editor resolves `members` against the key catalogue with `resolveMembers` (a port of spec §5.2). The server's `resolved_keys`/`missing` replace nothing in the editor; they are used by Rapport (field counts) and stay the source of truth for the CSV/PDF. This keeps the count line correct while edits are queued.
- **R3 — Save on change = full `members` snapshot per PATCH, serial, latest wins.** Each edit updates local state at once and queues `PATCH /templates/<id> {members}` with the whole array. The serial chain guarantees the last snapshot lands last; a response is applied only if no newer edit for that template was made since (`createLatestGate`). A failed write toasts and re-reads the list inside the same queued task, so optimistic state never outlives a failure.
- **R4 — Rename lives in the editor's name field** (`useTextDraft`, required, commits on blur/Enter, Esc reverts). The list's "Omdøb" action selects the template and focuses that field. A 409 on rename snaps the field back and toasts "Der findes allerede en skabelon med det navn."
- **R5 — 409 is ambiguous on this server.** `with_write` answers 409 for a running pipeline job (`Server.cpp:509-513`, message contains "pipeline job"); the templates API answers 409 for a duplicate name. `templateErrorMessage` tells them apart by that message.
- **R6 — "Ny skabelon" creates immediately** with the first free name of `Ny skabelon`, `Ny skabelon 2`, … and `members: []`, selects it and focuses the name field. No dialog.
- **R7 — Delete confirms with `window.confirm`** (the pattern `SampleCard.tsx:150-155` uses). After a delete the next template in the list is selected, else the previous, else none.
- **R8 — "Gendan standardskabeloner" is disabled when both seed tags exist** (title says so), and re-lists afterwards. Its response body is not relied on.
- **R9 — Template pickers are native `<select>`s styled with `controls.module.css` `.input`,** unless Phase 3 built a reusable template picker component (grep `TemplatePicker`/`TemplateSelect` in `src/components`), in which case reuse it. `SelectDropdown.tsx` is not a generic picker — it edits a `PropertyDefinition` select column.
- **R10 — Ressourcetabel defaults to "Ingen" and is not persisted**, so a plain "Generér ny version" behaves exactly as before. The Data-eksport picker defaults to the `screening` seed, then the first template (the same rule as Kortlægning's picker; reuse Phase 3's helper if one exists). Neither is stored in `localStorage`.
- **R11 — CSV options are read with defaults and written back merged.** Unknown keys in `csv` (e.g. what Phase 1 migrated out of `export_templates`) are preserved. The JSON field names and the defaults must equal Phase 1's backend CSV builder; `CSV_FIELDS`/`CSV_DEFAULTS` in `rapport/csvOptions.ts` are the one place to change them.
- **R12 — "Download CSV" is disabled while a CSV-option write is queued or in flight,** so the file always reflects the options on screen, and while the template resolves to 0 fields.
- **R13 — The Inventarliste row in `VersionList` and `GET /api/v1/exports/csv` stay.** The spec keeps the PDF versions panel unchanged; `/exports/csv` is the element CSV (`rux export csv` round-trip, also used by Kortlægning's export link), not an export-template route.
- **R14 — Only `rux gui`'s export-template routes are deleted.** `apps/ruxd`'s `/export-templates` handlers and any `ProjectDB` methods they still call are out of scope (they are whatever Phase 1 left). `ProjectDB` methods are deleted only if nothing references them after Task 8.
- **R15 — Nav:** Skabeloner is the last `sag` entry (spec §3), with no `pending`. If Phase 2 added a pending placeholder entry/route, Task 5 replaces it; if Phase 2 omitted it, Task 5 adds it.

## Name map (filled in by Task 1 Step 1 — do this first)

| This plan says | Phase 3 actual name | Notes |
|---|---|---|
| `ResourceKey` (type) | | `{id, label, category, scope, data_type, unit, options, editable}` |
| `Template` (type) | | `{id, name, members, csv, seed, resolved_keys, missing, created_at, updated_at}` |
| `TemplateMember` (type) | | `{category: string} \| {key: string}` |
| `api.resourceKeys()` | | `GET /resources/keys` |
| `api.listTemplates()` | | `GET /templates` |
| `api.duplicateTemplate(id)` | | `POST /templates/<id>/duplicate` |
| default-template helper | | screening seed → first (Kortlægning picker) |
| `api.createTemplate(body)` | added in Task 1 if absent | |
| `api.patchTemplate(id, patch)` | added in Task 1 if absent | |
| `api.deleteTemplate(id)` | added in Task 1 if absent | |
| `api.restoreSeedTemplates()` | added in Task 1 if absent | |
| `api.resourcesExportCsvUrl(id)` | added in Task 1 if absent | `GET /resources/export.csv?template=<id>` |
| `api.generateReport(templateId)` | changed in Task 1 | sends Phase 1's field (see Task 1 Step 1) |
| Phase 1 PDF field | | name of the report-generate body field selecting a template |
| Phase 1 CSV option fields + defaults | | from the backend CSV builder / openapi `Template.csv` |

Do not edit this plan file: write the filled-in map at the top of your Task 1 report (and in the Task 10 report), and use the real names in every task below.

## Global Constraints

- Danish UI copy everywhere a user reads it.
- CSS Modules with tokens only (`var(--…)` from `tokens.css`); never edit `tokens.css`. Lint every changed CSS/TSX with `python3 .claude/skills/design-studio/scripts/token_lint.py <files> --tsx`.
- vitest runs in Node with no DOM: tests import pure modules only (`src/skabeloner/*.ts`, `src/rapport/*.ts`, `src/app/*.ts`, `src/api/client.ts`).
- SPDX header on every new file (the three-line `// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `//` / `// SPDX-License-Identifier: GPL-3.0-or-later` block for TS; the `/* … */` form for CSS).
- Paths stay extensionless (`App.tsx` header comment): `/skabeloner`, `/rapport`.
- HTTP contract exactly as spec §4.4 and §5.5: `GET/POST /api/v1/templates`, `PATCH/DELETE /api/v1/templates/<id>`, `POST /api/v1/templates/<id>/duplicate`, `POST /api/v1/templates/restore-seeds`, `GET /api/v1/resources/keys`, `GET /api/v1/resources/export.csv?template=<id>`. A duplicate name returns **409**; unknown id **404**.
- Members are `{"category": "<name>"}` or `{"key": "<key id>"}`, ordered. Resolution: category → every key in that category in catalogue order; key → that key; duplicates keep their first position; a member whose key/category no longer exists is skipped and reported as missing (spec §5.2).
- Seeds: tags `materialepas` ("Materialepas (fuld)") and `screening` ("Hurtig genbrugsscreening"); seeds are ordinary rows (renamable, editable, deletable).
- Every mutating request carries `Content-Type: application/json` (the client's `postJson`/`patchJson` do; a DELETE has no body).
- Work happens in `/home/mephisto/repos/ReUseX/.worktrees/rt-phase4` (branch `rt-phase4-skabeloner`, from `main`). Frontend commands run from the worktree root: `npm --prefix apps/rux/frontend test -- --run`, `npm --prefix apps/rux/frontend run typecheck`, `npm --prefix apps/rux/frontend run build`.
- **Backend builds run inside `nix develop`** with exactly this configure line, in the foreground, re-running `cmake --build …` on a tool timeout (it resumes):

  ```bash
  cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DCMAKE_CUDA_COMPILER=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9/bin/nvcc -DCUDAToolkit_ROOT=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9 -DCUDA_TOOLKIT_ROOT_DIR=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9
  ```

  **Never touch `/home/mephisto/repos/ReUseX/build`**; the worktree has its own `build/`.
- Dev servers: **gui port 8429, vite port 5182** — `dev_env.sh start <project> 8429 5182`. Scratch: `SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad`; `$SP/corridor-clouds.rux` is the source project (`dev_env.sh` serves a copy, which the v25 migration upgrades on open). Never serve a tracked fixture in place.
- Every commit ends with the trailers `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and `Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf`. Never `--no-verify`.

## Review Focus

1. **Rapid edits in the Skabeloner editor** (tick three categories, then drag a member, within a second): the server must end with the last state on screen, and an earlier PATCH response must not flash old members back. → `createLatestGate` tests in Task 3; the editor wires it in Task 5.
2. **Download right after changing a CSV option**: the file must use the new delimiter/encoding/header. → R12; `downloadState` test in Task 4, wired in Task 6.
3. **A template whose members all went missing** (a deleted user column, a renamed category): the editor shows them struck through with a working remove, the count reads "0 felter", Data-eksport disables Download with a reason. → `resolveMembers` missing tests (Task 2), `downloadState` zero-field test (Task 4).
4. **Duplicate names** on rename, "Ny skabelon" and duplicate while a pipeline job is running: a 409 from a job must not read as "navnet findes allerede". → `templateErrorMessage` tests (Task 3).
5. **The selected template disappears** (deleted on Skabeloner, then Rapport opened with it as the remembered choice in state, or deleted in another tab): Rapport falls back to "Ingen"/default, Skabeloner selects a neighbour. → `validChoice` and `defaultExportTemplateId` tests (Task 4), `selectAfterDelete` tests (Task 3).

---

## File structure

| File | Responsibility |
|---|---|
| `apps/rux/frontend/src/api/client.ts` (modify) | template create/patch/delete/restore-seeds, resources CSV URL, `generateReport(templateId)`; remove export-template functions (Task 7) |
| `apps/rux/frontend/src/api/types.ts` (modify) | `TemplateMember`/`TemplateCreate`/`TemplatePatch` if absent; remove `ExportTemplate` (Task 7) |
| `apps/rux/frontend/src/skabeloner/members.ts` (create) | pure member-list editing + local resolution + search |
| `apps/rux/frontend/src/skabeloner/model.ts` (create) | pure copy, errors, names, selection, latest-wins gate, list-state helpers |
| `apps/rux/frontend/src/rapport/csvOptions.ts` (create) | pure CSV option mapping, option lists, file name, download state |
| `apps/rux/frontend/src/rapport/model.ts` (modify) | Ressourcetabel choice, default export template |
| `apps/rux/frontend/src/routes/SkabelonerPage.tsx` + `.module.css` (create) | route: loads templates + keys, owns state and writes |
| `apps/rux/frontend/src/components/skabeloner/TemplateList.tsx` + css (create) | list + list actions |
| `apps/rux/frontend/src/components/skabeloner/TemplateEditor.tsx` + css (create) | name, Kategorier, Enkelte felter, Rækkefølge, count |
| `apps/rux/frontend/src/components/rapport/TemplateSelect.tsx` (create, unless Phase 3 has one — R9) | native select over templates (+ optional "Ingen") |
| `apps/rux/frontend/src/components/rapport/DataExportPanel.tsx` + css (create) | Data-eksport panel |
| `apps/rux/frontend/src/routes/RapportPage.tsx` + css (modify) | Ressourcetabel picker, Data-eksport panel |
| `apps/rux/frontend/src/app/{links,navigation,App}.ts(x)` (modify) | `SKABELONER_PATH`, nav entry, route, `/export` redirect |
| `apps/rux/frontend/src/routes/ExportPage.*`, `src/data/csvExport.ts`, `src/test/csvExport.test.ts` (delete) | dead after Rapport takes over |
| `apps/rux/src/gui/{Server,api}.cpp`, `apps/rux/include/gui/api.hpp`, `tests/unit/rux_gui/test_gui_api.cpp`, `docs/gui/openapi.yaml` (modify) | delete `rux gui` export-template routes |
| `CLAUDE.md`, `docs/DIRECTION.md`, `docs/design/gui-kortlaegning-redesign.md` (modify) | docs |
| tests: `src/test/templates.client.test.ts`, `skabeloner.members.test.ts`, `skabeloner.model.test.ts`, `rapport.csvOptions.test.ts` (create); `rapport.model.test.ts`, `navigation.test.ts` (modify) | |

---

### Task 1: Template CRUD client functions and the report-generate field

**Files:**
- Modify: `apps/rux/frontend/src/api/client.ts` (reports section ~`:1019-1055`; add a `// ---- templates ----` block next to Phase 3's template functions)
- Modify: `apps/rux/frontend/src/api/types.ts` (only types that are missing)
- Create: `apps/rux/frontend/src/test/templates.client.test.ts`

**Interfaces:**
- Consumes: Phase 3's `Template`, `ResourceKey`, `api.listTemplates`, `api.duplicateTemplate`, `api.resourceKeys` (real names → Name map).
- Produces:
  - `createTemplate(body: TemplateCreate, signal?): Promise<Template>` — `POST /templates`
  - `patchTemplate(id: number, patch: TemplatePatch, signal?): Promise<Template>` — `PATCH /templates/<id>`
  - `deleteTemplate(id: number, signal?): Promise<void>` — `DELETE /templates/<id>`
  - `restoreSeedTemplates(signal?): Promise<void>` — `POST /templates/restore-seeds`
  - `resourcesExportCsvUrl(templateId: number): string`
  - `generateReport(templateId?: number | null, signal?): Promise<ReportPdfVersion>`
  - types `TemplateMember = { category: string } | { key: string }`, `TemplateCreate = { name: string; members?: TemplateMember[]; csv?: Record<string, unknown> }`, `TemplatePatch = { name?: string; members?: TemplateMember[]; csv?: Record<string, unknown> }`

- [ ] **Step 1: Inventory what Phase 1–3 left, fill the Name map**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/rt-phase4
grep -n "interface Template\|type Template\|TemplateMember\|interface ResourceKey\|interface Resource\b" apps/rux/frontend/src/api/types.ts
grep -n "templates\|resources\|export.csv\|generateReport" apps/rux/frontend/src/api/client.ts
grep -rn "TemplatePicker\|TemplateSelect\|defaultTemplate" apps/rux/frontend/src --include=*.ts --include=*.tsx | grep -v test/
grep -n "template" apps/rux/src/gui/edits.cpp apps/rux/src/gui/Server.cpp | grep -i "report\|pdf"
grep -n "/reports/ressourcekortlaegning:" -A60 docs/gui/openapi.yaml | grep -n -i "template"
grep -n "delimiter\|encoding\|header" docs/gui/openapi.yaml | head -20
```

Fill in the Name map (in your notes, not in this file). For the **Phase 1 PDF field**: read the report-generate handler (`generate_report_pdf_json` in `apps/rux/src/gui/edits.cpp`, and how `Server.cpp`'s `/api/v1/reports/ressourcekortlaegning` POST passes `req.body`) and the openapi request body. This plan assumes `{"template_id": <int>}` (absent or `null` = no Ressourcetabel). If Phase 1 chose another name, use it in Step 3 and in the test. For the **CSV option fields**: read the `Template.csv` schema in openapi and the backend CSV builder's parsing of `csv`; note field names, allowed values and defaults for Task 4.

If a function in Produces already exists under another name, do not add a second one: record the name and skip its part of Steps 3–4 (keep its test, renamed).

- [ ] **Step 2: Write the failing tests**

Create `apps/rux/frontend/src/test/templates.client.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/** Contract tests for the template CRUD calls and the report-generate body (spec §5.5, §6.3). */

import { describe, expect, it } from 'vitest';
import { ApiRequestError, RuxApiClient, type FetchLike } from '../api/client';

interface RecordedRequest {
  url: string;
  method?: string;
  headers?: Record<string, string>;
  body?: string;
}

function clientFor(payload: unknown, status = 200) {
  const calls: RecordedRequest[] = [];
  const fetchLike: FetchLike = (url, options) => {
    calls.push({ url, ...(options as Omit<RecordedRequest, 'url'>) });
    const body = status === 204 ? null : JSON.stringify(payload);
    return Promise.resolve(
      new Response(body, { status, headers: { 'Content-Type': 'application/json' } }),
    );
  };
  return { calls, api: new RuxApiClient({ baseUrl: '/api/v1', fetch: fetchLike }) };
}

const TEMPLATE = {
  id: 7,
  name: 'Ny skabelon',
  members: [],
  csv: {},
  seed: null,
  resolved_keys: [],
  missing: [],
  created_at: '2026-10-02T10:00:00Z',
  updated_at: '2026-10-02T10:00:00Z',
};

describe('template CRUD', () => {
  it('creates with a JSON POST', async () => {
    const { calls, api } = clientFor(TEMPLATE, 201);
    const t = await api.createTemplate({ name: 'Ny skabelon', members: [] });
    expect(t.id).toBe(7);
    expect(calls[0].url).toBe('/api/v1/templates');
    expect(calls[0].method).toBe('POST');
    expect(calls[0].headers?.['Content-Type']).toBe('application/json');
    expect(JSON.parse(calls[0].body!)).toEqual({ name: 'Ny skabelon', members: [] });
  });

  it('patches only the given fields', async () => {
    const { calls, api } = clientFor({ ...TEMPLATE, members: [{ category: 'Egne felter' }] });
    await api.patchTemplate(7, { members: [{ category: 'Egne felter' }] });
    expect(calls[0].url).toBe('/api/v1/templates/7');
    expect(calls[0].method).toBe('PATCH');
    expect(JSON.parse(calls[0].body!)).toEqual({ members: [{ category: 'Egne felter' }] });
  });

  it('deletes and accepts a 204', async () => {
    const { calls, api } = clientFor(null, 204);
    await expect(api.deleteTemplate(7)).resolves.toBeUndefined();
    expect(calls[0].url).toBe('/api/v1/templates/7');
    expect(calls[0].method).toBe('DELETE');
  });

  it('maps a duplicate-name 409 to ApiRequestError', async () => {
    const { api } = clientFor({ error: 'template name already exists' }, 409);
    const err = await api.patchTemplate(7, { name: 'Materialepas (fuld)' }).catch((e) => e);
    expect(err).toBeInstanceOf(ApiRequestError);
    expect((err as ApiRequestError).status).toBe(409);
  });

  it('restores seeds with a JSON POST and ignores the body', async () => {
    const { calls, api } = clientFor({ templates: [] });
    await expect(api.restoreSeedTemplates()).resolves.toBeUndefined();
    expect(calls[0].url).toBe('/api/v1/templates/restore-seeds');
    expect(calls[0].method).toBe('POST');
  });

  it('builds the resources CSV URL for one template', () => {
    const { api } = clientFor(null);
    expect(api.resourcesExportCsvUrl(7)).toBe('/api/v1/resources/export.csv?template=7');
  });
});

describe('report generation', () => {
  it('sends no template when none is chosen', async () => {
    const { calls, api } = clientFor({ id: 1 }, 201);
    await api.generateReport(null);
    expect(JSON.parse(calls[0].body!)).toEqual({});
  });

  it('sends the chosen template id', async () => {
    const { calls, api } = clientFor({ id: 1 }, 201);
    await api.generateReport(7);
    expect(calls[0].url).toBe('/api/v1/reports/ressourcekortlaegning');
    expect(JSON.parse(calls[0].body!)).toEqual({ template_id: 7 });
  });
});
```

(Replace `template_id` with Phase 1's field name if it differs; replace any function name per the Name map. If `buildQuery` encodes differently — e.g. sorted keys — the URL assertion still holds for a single parameter.)

- [ ] **Step 3: Run to verify failure**

Run: `npm --prefix apps/rux/frontend test -- --run src/test/templates.client.test.ts`
Expected: FAIL — `api.createTemplate is not a function` (and the others), and `{}` vs `{ template_id: 7 }`.

- [ ] **Step 4: Implement**

In `api/types.ts`, add only what is missing (next to Phase 3's `Template`):

```ts
/** One member of a template: a whole category, or a single key id (spec §5.1). */
export type TemplateMember = { category: string } | { key: string };

/** Body of `POST /templates`. */
export interface TemplateCreate {
  name: string;
  members?: TemplateMember[];
  csv?: Record<string, unknown>;
}

/** Body of `PATCH /templates/{id}`: any subset. */
export interface TemplatePatch {
  name?: string;
  members?: TemplateMember[];
  csv?: Record<string, unknown>;
}
```

In `api/client.ts`, replace `generateReport` (`:1029-1031`):

```ts
  /**
   * Generate a Ressourcekortlægning PDF server-side and store it.
   *
   * `templateId` adds the Ressourcetabel section built from that template's
   * columns (spec §6.3); null or omitted generates the report without it.
   * Writer-locked: a 409 means a pipeline stage is holding the lock; a 503
   * means a transient busy.
   */
  generateReport(templateId?: number | null, signal?: AbortSignal): Promise<ReportPdfVersion> {
    const body = templateId == null ? {} : { template_id: templateId };
    return this.postJson<ReportPdfVersion>('/reports/ressourcekortlaegning', body, signal);
  }
```

Add, in the templates block (reuse Phase 3's private DELETE helper if it added one; otherwise this inline form is the existing `deleteExportTemplate` pattern):

```ts
  createTemplate(body: TemplateCreate, signal?: AbortSignal): Promise<Template> {
    return this.postJson<Template>('/templates', body, signal);
  }

  patchTemplate(id: number, patch: TemplatePatch, signal?: AbortSignal): Promise<Template> {
    return this.patchJson<Template>(`/templates/${id}`, patch, signal);
  }

  async deleteTemplate(id: number, signal?: AbortSignal): Promise<void> {
    const url = this.url(`/templates/${id}`);
    const response = await this.doFetch(url, { method: 'DELETE', signal });
    if (!response.ok) {
      throw new ApiRequestError(response.status, await describeFailure(response), url);
    }
  }

  /** Re-insert any missing seed template (spec §5.3). Re-list afterwards. */
  async restoreSeedTemplates(signal?: AbortSignal): Promise<void> {
    await this.postJson<unknown>('/templates/restore-seeds', {}, signal);
  }

  /** The resources CSV for one template (spec §6.3), for an `<a href download>`. */
  resourcesExportCsvUrl(templateId: number): string {
    return this.url('/resources/export.csv', { template: templateId });
  }
```

Import `TemplateCreate`, `TemplatePatch` (and `Template` if not yet imported) in the client's type import list. Check `Query` accepts a number value (`grep -n "type Query" api/client.ts`); if it only takes strings, pass `String(templateId)`.

Update the one existing caller: `RapportPage.tsx:76` `api.generateReport()` still type-checks (the argument is optional); Task 6 passes the choice.

- [ ] **Step 5: Run tests and typecheck**

Run: `npm --prefix apps/rux/frontend test -- --run src/test/templates.client.test.ts && npm --prefix apps/rux/frontend run typecheck`
Expected: PASS, no type errors.

- [ ] **Step 6: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/rt-phase4
git add apps/rux/frontend/src/api/client.ts apps/rux/frontend/src/api/types.ts apps/rux/frontend/src/test/templates.client.test.ts
git commit -m "feat(gui): template CRUD client calls and the report template field" -m "Adds create/patch/delete/restore-seeds for /templates, the resources CSV URL, and lets generateReport send the template that builds the PDF's Ressourcetabel (spec §5.5, §6.3)." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 2: Pure member-list editing (`skabeloner/members.ts`)

**Files:**
- Create: `apps/rux/frontend/src/skabeloner/members.ts`
- Test: `apps/rux/frontend/src/test/skabeloner.members.test.ts`

**Interfaces:**
- Consumes: `ResourceKey`, `TemplateMember` (Task 1 / Name map).
- Produces:
  - `memberId(m: TemplateMember): string` — `'category:<name>'` | `'key:<id>'`
  - `isCategory(m): m is { category: string }`
  - `categoryCounts(keys: readonly ResourceKey[]): { category: string; count: number }[]` — catalogue order
  - `hasCategory(members, name): boolean`
  - `toggleCategory(members, name): TemplateMember[]`
  - `addKey(members, keyId): TemplateMember[]`
  - `removeMemberAt(members, index): TemplateMember[]`
  - `moveMember(members, from, to): TemplateMember[]`
  - `resolveMembers(members, keys): { keys: string[]; missing: TemplateMember[] }`
  - `searchKeys(keys, query, members, limit = 20): ResourceKey[]`
  - `memberLabel(m, keys): { label: string; kind: 'Kategori' | 'Felt'; detail: string }`

- [ ] **Step 1: Write the failing tests**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { ResourceKey, TemplateMember } from '../api/types';
import {
  addKey,
  categoryCounts,
  hasCategory,
  memberId,
  memberLabel,
  moveMember,
  removeMemberAt,
  resolveMembers,
  searchKeys,
  toggleCategory,
} from '../skabeloner/members';

function key(id: string, label: string, category: string): ResourceKey {
  return {
    id,
    label,
    category,
    scope: id.startsWith('sys:') ? 'part' : 'passport',
    data_type: 'text',
    unit: null,
    options: [],
    editable: true,
  } as ResourceKey;
}

const KEYS: ResourceKey[] = [
  key('sys:name', 'Betegnelse', 'Kortlægning'),
  key('sys:quantity', 'Mængde', 'Kortlægning'),
  key('lex:a', 'Producent', 'Produkt'),
  key('lex:b', 'Model', 'Produkt'),
  key('lex:c', 'Brandklasse', 'Brand'),
  key('col:1', 'Farve', 'Egne felter'),
];

describe('member identity', () => {
  it('names categories and keys apart', () => {
    expect(memberId({ category: 'Produkt' })).toBe('category:Produkt');
    expect(memberId({ key: 'lex:a' })).toBe('key:lex:a');
  });
});

describe('categories', () => {
  it('counts keys per category in catalogue order', () => {
    expect(categoryCounts(KEYS)).toEqual([
      { category: 'Kortlægning', count: 2 },
      { category: 'Produkt', count: 2 },
      { category: 'Brand', count: 1 },
      { category: 'Egne felter', count: 1 },
    ]);
  });

  it('toggles a category on at the end and off where it was', () => {
    const on = toggleCategory([{ key: 'sys:name' }], 'Produkt');
    expect(on).toEqual([{ key: 'sys:name' }, { category: 'Produkt' }]);
    expect(hasCategory(on, 'Produkt')).toBe(true);
    expect(toggleCategory(on, 'Produkt')).toEqual([{ key: 'sys:name' }]);
  });
});

describe('editing the list', () => {
  const M: TemplateMember[] = [{ key: 'sys:name' }, { category: 'Produkt' }, { key: 'col:1' }];

  it('adds a key once', () => {
    expect(addKey(M, 'lex:c')).toEqual([...M, { key: 'lex:c' }]);
    expect(addKey(M, 'sys:name')).toBe(M);
  });

  it('removes by position, including a missing member', () => {
    expect(removeMemberAt(M, 1)).toEqual([{ key: 'sys:name' }, { key: 'col:1' }]);
    expect(removeMemberAt(M, 9)).toBe(M);
  });

  it('moves a member and leaves out-of-range moves alone', () => {
    expect(moveMember(M, 2, 0)).toEqual([{ key: 'col:1' }, { key: 'sys:name' }, { category: 'Produkt' }]);
    expect(moveMember(M, 0, 1)).toEqual([{ category: 'Produkt' }, { key: 'sys:name' }, { key: 'col:1' }]);
    expect(moveMember(M, 0, -1)).toBe(M);
    expect(moveMember(M, 2, 3)).toBe(M);
    expect(moveMember(M, 1, 1)).toBe(M);
  });

  it('never mutates its input', () => {
    const copy = structuredClone(M);
    toggleCategory(M, 'Brand');
    addKey(M, 'lex:c');
    removeMemberAt(M, 0);
    moveMember(M, 0, 2);
    expect(M).toEqual(copy);
  });
});

describe('resolveMembers (spec §5.2)', () => {
  it('expands categories in catalogue order and keeps the first position of a duplicate', () => {
    const r = resolveMembers([{ key: 'lex:b' }, { category: 'Produkt' }, { key: 'sys:name' }], KEYS);
    expect(r.keys).toEqual(['lex:b', 'lex:a', 'sys:name']);
    expect(r.missing).toEqual([]);
  });

  it('skips and reports members that no longer exist', () => {
    const r = resolveMembers([{ key: 'col:99' }, { category: 'Fjernet' }, { key: 'col:1' }], KEYS);
    expect(r.keys).toEqual(['col:1']);
    expect(r.missing).toEqual([{ key: 'col:99' }, { category: 'Fjernet' }]);
  });

  it('picks up a key added to a category member later', () => {
    const more = [...KEYS, key('lex:d', 'Årgang', 'Produkt')];
    expect(resolveMembers([{ category: 'Produkt' }], more).keys).toEqual(['lex:a', 'lex:b', 'lex:d']);
  });

  it('resolves an empty template to nothing', () => {
    expect(resolveMembers([], KEYS)).toEqual({ keys: [], missing: [] });
  });
});

describe('searchKeys', () => {
  it('matches label, id and category case-insensitively, minus explicit key members', () => {
    expect(searchKeys(KEYS, 'PROD', [], 20).map((k) => k.id)).toEqual(['lex:a', 'lex:b']);
    expect(searchKeys(KEYS, 'col:', [], 20).map((k) => k.id)).toEqual(['col:1']);
    expect(searchKeys(KEYS, 'prod', [{ key: 'lex:a' }], 20).map((k) => k.id)).toEqual(['lex:b']);
  });

  it('returns nothing for a blank query and honours the limit', () => {
    expect(searchKeys(KEYS, '   ', [], 20)).toEqual([]);
    expect(searchKeys(KEYS, 'e', [], 2)).toHaveLength(2);
  });
});

describe('memberLabel', () => {
  it('labels a category with its key count and a key with its category', () => {
    expect(memberLabel({ category: 'Produkt' }, KEYS)).toEqual({ label: 'Produkt', kind: 'Kategori', detail: '2 felter' });
    expect(memberLabel({ key: 'lex:c' }, KEYS)).toEqual({ label: 'Brandklasse', kind: 'Felt', detail: 'Brand' });
  });

  it('falls back to the raw id for a missing member', () => {
    expect(memberLabel({ key: 'col:99' }, KEYS)).toEqual({ label: 'col:99', kind: 'Felt', detail: 'Findes ikke længere' });
    expect(memberLabel({ category: 'Fjernet' }, KEYS)).toEqual({ label: 'Fjernet', kind: 'Kategori', detail: 'Findes ikke længere' });
  });
});
```

If Phase 3's `ResourceKey` has different optional fields (e.g. `unit?: string`), adjust the `key()` factory only; drop the `as ResourceKey` cast if it type-checks without it.

- [ ] **Step 2: Run to verify failure**

Run: `npm --prefix apps/rux/frontend test -- --run src/test/skabeloner.members.test.ts`
Expected: FAIL — cannot resolve `../skabeloner/members`.

- [ ] **Step 3: Implement**

Create `apps/rux/frontend/src/skabeloner/members.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A template's member list as data (spec §5.1–5.2, §6.2): identity, the
 * editor's edits (category toggle, add a key, remove, move), the catalogue
 * search and a local port of the server's resolver. Every function returns a
 * new array, or the same array when nothing changes, and never mutates.
 *
 * `resolveMembers` must stay equal to `resolve_template` in core: the editor
 * shows its count and its struck-through members while writes are queued.
 */

import type { ResourceKey, TemplateMember } from '../api/types';

export function isCategory(m: TemplateMember): m is { category: string } {
  return 'category' in m;
}

export function memberId(m: TemplateMember): string {
  return isCategory(m) ? `category:${m.category}` : `key:${m.key}`;
}

export function categoryCounts(keys: readonly ResourceKey[]): { category: string; count: number }[] {
  const order: string[] = [];
  const counts = new Map<string, number>();
  for (const k of keys) {
    if (!counts.has(k.category)) order.push(k.category);
    counts.set(k.category, (counts.get(k.category) ?? 0) + 1);
  }
  return order.map((category) => ({ category, count: counts.get(category)! }));
}

export function hasCategory(members: readonly TemplateMember[], name: string): boolean {
  return members.some((m) => isCategory(m) && m.category === name);
}

export function toggleCategory(members: readonly TemplateMember[], name: string): TemplateMember[] {
  return hasCategory(members, name)
    ? members.filter((m) => !(isCategory(m) && m.category === name))
    : [...members, { category: name }];
}

export function addKey(members: TemplateMember[], keyId: string): TemplateMember[] {
  return members.some((m) => !isCategory(m) && m.key === keyId) ? members : [...members, { key: keyId }];
}

export function removeMemberAt(members: TemplateMember[], index: number): TemplateMember[] {
  if (index < 0 || index >= members.length) return members;
  return members.filter((_, i) => i !== index);
}

export function moveMember(members: TemplateMember[], from: number, to: number): TemplateMember[] {
  const n = members.length;
  if (from === to || from < 0 || from >= n || to < 0 || to >= n) return members;
  const next = [...members];
  const [moved] = next.splice(from, 1);
  next.splice(to, 0, moved);
  return next;
}

export function resolveMembers(
  members: readonly TemplateMember[],
  keys: readonly ResourceKey[],
): { keys: string[]; missing: TemplateMember[] } {
  const known = new Set(keys.map((k) => k.id));
  const seen = new Set<string>();
  const out: string[] = [];
  const missing: TemplateMember[] = [];
  const push = (id: string) => {
    if (!seen.has(id)) {
      seen.add(id);
      out.push(id);
    }
  };
  for (const m of members) {
    if (isCategory(m)) {
      const inCat = keys.filter((k) => k.category === m.category);
      if (inCat.length === 0) missing.push(m);
      for (const k of inCat) push(k.id);
    } else if (known.has(m.key)) {
      push(m.key);
    } else {
      missing.push(m);
    }
  }
  return { keys: out, missing };
}

export function searchKeys(
  keys: readonly ResourceKey[],
  query: string,
  members: readonly TemplateMember[],
  limit = 20,
): ResourceKey[] {
  const q = query.trim().toLocaleLowerCase('da-DK');
  if (!q) return [];
  const explicit = new Set(members.filter((m) => !isCategory(m)).map((m) => (m as { key: string }).key));
  const hits: ResourceKey[] = [];
  for (const k of keys) {
    if (explicit.has(k.id)) continue;
    const hay = `${k.label}\n${k.id}\n${k.category}`.toLocaleLowerCase('da-DK');
    if (hay.includes(q)) hits.push(k);
    if (hits.length >= limit) break;
  }
  return hits;
}

const GONE = 'Findes ikke længere';

export function memberLabel(
  m: TemplateMember,
  keys: readonly ResourceKey[],
): { label: string; kind: 'Kategori' | 'Felt'; detail: string } {
  if (isCategory(m)) {
    const n = keys.filter((k) => k.category === m.category).length;
    return { label: m.category, kind: 'Kategori', detail: n === 0 ? GONE : n === 1 ? '1 felt' : `${n} felter` };
  }
  const k = keys.find((x) => x.id === m.key);
  return k ? { label: k.label, kind: 'Felt', detail: k.category } : { label: m.key, kind: 'Felt', detail: GONE };
}
```

- [ ] **Step 4: Run tests**

Run: `npm --prefix apps/rux/frontend test -- --run src/test/skabeloner.members.test.ts && npm --prefix apps/rux/frontend run typecheck`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/src/skabeloner/members.ts apps/rux/frontend/src/test/skabeloner.members.test.ts
git commit -m "feat(gui): pure template member editing and local resolution" -m "Category toggle, add/remove/move, catalogue search and a port of the server's template resolver, so the Skabeloner editor's count and missing members react before the write lands (spec §5.2, §6.2)." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 3: Skabeloner page model (`skabeloner/model.ts`)

**Files:**
- Create: `apps/rux/frontend/src/skabeloner/model.ts`
- Test: `apps/rux/frontend/src/test/skabeloner.model.test.ts`

**Interfaces:**
- Consumes: `Template`, `TemplateMember`, `ApiRequestError`, `saveErrorMessage` (`app/saveError.ts`).
- Produces:
  - `SEED_TAGS = ['materialepas', 'screening'] as const`
  - `countLine(n: number): string` — `'1 felt'` / `'N felter'`
  - `nextTemplateName(names: readonly string[], base = 'Ny skabelon'): string`
  - `missingSeeds(templates: readonly Pick<Template,'seed'>[]): string[]`
  - `restoreSeedsTitle(missing: readonly string[]): string`
  - `deleteConfirmText(t: Pick<Template,'name'|'seed'>): string`
  - `templateErrorMessage(cause: unknown): string`
  - `isNameConflict(cause: unknown): boolean`
  - `selectAfterDelete(ids: readonly number[], deletedId: number): number | null`
  - `replaceTemplate<T extends {id:number}>(list: readonly T[], next: T): T[]`
  - `withMembers<T extends {id:number; members: TemplateMember[]}>(list: readonly T[], id: number, members: TemplateMember[]): T[]`
  - `createLatestGate(): { next(id: number): number; isLatest(id: number, ticket: number): boolean }`
  - `seedLabel(seed: string | null): string | null` — `'Standard'` for a seed

- [ ] **Step 1: Write the failing tests**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import {
  countLine,
  createLatestGate,
  deleteConfirmText,
  isNameConflict,
  missingSeeds,
  nextTemplateName,
  replaceTemplate,
  restoreSeedsTitle,
  seedLabel,
  selectAfterDelete,
  templateErrorMessage,
  withMembers,
} from '../skabeloner/model';

const err = (status: number, message: string) => new ApiRequestError(status, message, '/api/v1/templates/1');

describe('copy', () => {
  it('counts fields in Danish', () => {
    expect(countLine(0)).toBe('0 felter');
    expect(countLine(1)).toBe('1 felt');
    expect(countLine(42)).toBe('42 felter');
  });

  it('marks seeds', () => {
    expect(seedLabel('screening')).toBe('Standard');
    expect(seedLabel(null)).toBeNull();
  });

  it('asks before deleting, and says how a seed comes back', () => {
    expect(deleteConfirmText({ name: 'Min skabelon', seed: null })).toBe(
      'Slet skabelonen "Min skabelon"? Det kan ikke fortrydes.',
    );
    expect(deleteConfirmText({ name: 'Hurtig genbrugsscreening', seed: 'screening' })).toBe(
      'Slet skabelonen "Hurtig genbrugsscreening"? Den kan hentes tilbage med "Gendan standardskabeloner".',
    );
  });
});

describe('names', () => {
  it('finds the first free "Ny skabelon"', () => {
    expect(nextTemplateName([])).toBe('Ny skabelon');
    expect(nextTemplateName(['Ny skabelon'])).toBe('Ny skabelon 2');
    expect(nextTemplateName(['Ny skabelon', 'Ny skabelon 2', 'Ny skabelon 4'])).toBe('Ny skabelon 3');
  });
});

describe('seeds', () => {
  it('lists the seed tags no row carries', () => {
    expect(missingSeeds([{ seed: 'screening' }, { seed: null }])).toEqual(['materialepas']);
    expect(missingSeeds([{ seed: 'screening' }, { seed: 'materialepas' }])).toEqual([]);
  });

  it('titles the restore button', () => {
    expect(restoreSeedsTitle([])).toBe('Begge standardskabeloner findes i projektet.');
    expect(restoreSeedsTitle(['materialepas'])).toBe('Genopretter Materialepas (fuld).');
    expect(restoreSeedsTitle(['materialepas', 'screening'])).toBe(
      'Genopretter Materialepas (fuld) og Hurtig genbrugsscreening.',
    );
  });
});

describe('errors (R5)', () => {
  it('reads a name clash as a name clash', () => {
    const e = err(409, 'template name already exists');
    expect(isNameConflict(e)).toBe(true);
    expect(templateErrorMessage(e)).toBe('Der findes allerede en skabelon med det navn.');
  });

  it('reads the pipeline lock as the lock, not as a name clash', () => {
    const e = err(409, 'a pipeline job is running or queued; edits are refused while a stage is writing the project');
    expect(isNameConflict(e)).toBe(false);
    expect(templateErrorMessage(e)).toBe('Kunne ikke gemme — et pipeline-job kører. Prøv igen om lidt.');
  });

  it('explains a vanished template and a bad member', () => {
    expect(templateErrorMessage(err(404, 'no such template'))).toBe(
      'Skabelonen findes ikke længere — listen er hentet igen.',
    );
    expect(templateErrorMessage(err(400, 'unknown key id col:99'))).toBe('Ugyldig skabelon: unknown key id col:99');
  });
});

describe('selection after delete (R7)', () => {
  it('picks the next, else the previous, else none', () => {
    expect(selectAfterDelete([1, 2, 3], 2)).toBe(3);
    expect(selectAfterDelete([1, 2, 3], 3)).toBe(2);
    expect(selectAfterDelete([5], 5)).toBeNull();
    expect(selectAfterDelete([1, 2], 9)).toBe(1);
  });
});

describe('list state', () => {
  const list = [
    { id: 1, name: 'A', members: [] },
    { id: 2, name: 'B', members: [{ key: 'sys:name' }] },
  ];

  it('replaces one template by id and keeps order', () => {
    expect(replaceTemplate(list, { id: 2, name: 'B2', members: [] }).map((t) => t.name)).toEqual(['A', 'B2']);
    expect(replaceTemplate(list, { id: 9, name: 'X', members: [] })).toEqual(list);
  });

  it('sets members on one template', () => {
    expect(withMembers(list, 1, [{ category: 'Brand' }])[0].members).toEqual([{ category: 'Brand' }]);
    expect(list[0].members).toEqual([]);
  });
});

describe('createLatestGate (R3)', () => {
  it('lets only the newest edit per template apply its response', () => {
    const gate = createLatestGate();
    const a = gate.next(1);
    const b = gate.next(1);
    const other = gate.next(2);
    expect(gate.isLatest(1, a)).toBe(false);
    expect(gate.isLatest(1, b)).toBe(true);
    expect(gate.isLatest(2, other)).toBe(true);
  });
});
```

(`ApiRequestError`'s constructor is `(status, message, url)` — `client.ts:99`. If `.message` is prefixed by the class, assert on `isNameConflict` via `cause.message.includes('pipeline job')` as implemented below.)

- [ ] **Step 2: Run to verify failure**

Run: `npm --prefix apps/rux/frontend test -- --run src/test/skabeloner.model.test.ts`
Expected: FAIL — module not found.

- [ ] **Step 3: Implement**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Skabeloner as data (spec §6.2): the copy, how errors read, the new-name and
 * selection rules, list-state replacement and the gate that keeps a stale
 * save-on-change response from overwriting a newer edit (R3).
 */

import { ApiRequestError } from '../api/client';
import type { Template, TemplateMember } from '../api/types';
import { errorMessage, saveErrorMessage } from '../app/saveError';

export const SEED_TAGS = ['materialepas', 'screening'] as const;

const SEED_NAMES: Record<string, string> = {
  materialepas: 'Materialepas (fuld)',
  screening: 'Hurtig genbrugsscreening',
};

export function countLine(n: number): string {
  return n === 1 ? '1 felt' : `${n} felter`;
}

export function seedLabel(seed: string | null): string | null {
  return seed ? 'Standard' : null;
}

export function nextTemplateName(names: readonly string[], base = 'Ny skabelon'): string {
  const taken = new Set(names);
  if (!taken.has(base)) return base;
  for (let i = 2; ; i += 1) if (!taken.has(`${base} ${i}`)) return `${base} ${i}`;
}

export function missingSeeds(templates: readonly Pick<Template, 'seed'>[]): string[] {
  const present = new Set(templates.map((t) => t.seed));
  return SEED_TAGS.filter((s) => !present.has(s));
}

export function restoreSeedsTitle(missing: readonly string[]): string {
  if (missing.length === 0) return 'Begge standardskabeloner findes i projektet.';
  return `Genopretter ${missing.map((s) => SEED_NAMES[s] ?? s).join(' og ')}.`;
}

export function deleteConfirmText(t: Pick<Template, 'name' | 'seed'>): string {
  return t.seed
    ? `Slet skabelonen "${t.name}"? Den kan hentes tilbage med "Gendan standardskabeloner".`
    : `Slet skabelonen "${t.name}"? Det kan ikke fortrydes.`;
}

/** 409 from the templates API (duplicate name), not from `with_write`'s job lock (R5). */
export function isNameConflict(cause: unknown): boolean {
  return cause instanceof ApiRequestError && cause.status === 409 && !cause.message.includes('pipeline job');
}

export function templateErrorMessage(cause: unknown): string {
  if (isNameConflict(cause)) return 'Der findes allerede en skabelon med det navn.';
  if (cause instanceof ApiRequestError) {
    if (cause.status === 404) return 'Skabelonen findes ikke længere — listen er hentet igen.';
    if (cause.status === 400) return `Ugyldig skabelon: ${errorMessage(cause)}`;
  }
  return saveErrorMessage(cause);
}

export function selectAfterDelete(ids: readonly number[], deletedId: number): number | null {
  const i = ids.indexOf(deletedId);
  if (i === -1) return ids[0] ?? null;
  return ids[i + 1] ?? ids[i - 1] ?? null;
}

export function replaceTemplate<T extends { id: number }>(list: readonly T[], next: T): T[] {
  return list.map((t) => (t.id === next.id ? next : t));
}

export function withMembers<T extends { id: number; members: TemplateMember[] }>(
  list: readonly T[],
  id: number,
  members: TemplateMember[],
): T[] {
  return list.map((t) => (t.id === id ? { ...t, members } : t));
}

export function createLatestGate(): { next(id: number): number; isLatest(id: number, ticket: number): boolean } {
  const latest = new Map<number, number>();
  return {
    next(id) {
      const n = (latest.get(id) ?? 0) + 1;
      latest.set(id, n);
      return n;
    },
    isLatest(id, ticket) {
      return latest.get(id) === ticket;
    },
  };
}
```

If `ApiRequestError.message` is not the server's message verbatim (check `describeFailure` in `client.ts:200-223`: it reads the JSON `error` field), the `400` test's expected text follows whatever `errorMessage(cause)` returns; keep the `Ugyldig skabelon: ` prefix.

- [ ] **Step 4: Run tests**

Run: `npm --prefix apps/rux/frontend test -- --run src/test/skabeloner.model.test.ts && npm --prefix apps/rux/frontend run typecheck`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/src/skabeloner/model.ts apps/rux/frontend/src/test/skabeloner.model.test.ts
git commit -m "feat(gui): Skabeloner page model" -m "Danish copy, name-clash vs pipeline-lock 409s, new-name and post-delete selection rules, and a latest-edit gate for save-on-change." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 4: CSV option mapping and Rapport choices (pure)

**Files:**
- Create: `apps/rux/frontend/src/rapport/csvOptions.ts`
- Modify: `apps/rux/frontend/src/rapport/model.ts` (append)
- Test: create `apps/rux/frontend/src/test/rapport.csvOptions.test.ts`; modify `apps/rux/frontend/src/test/rapport.model.test.ts`

**Interfaces:**
- Consumes: `Template` (needs `id`, `name`, `seed`, `csv`, `resolved_keys`); Phase 1's CSV field names/defaults (Task 1 Step 1).
- Produces (`rapport/csvOptions.ts`):
  - `type CsvDelimiter = ',' | ';' | '\t'`, `type CsvEncoding = 'utf-8' | 'utf-8-bom'`, `type CsvHeader = 'label' | 'key'`, `interface CsvOptions { delimiter; encoding; header }`
  - `CSV_FIELDS`, `CSV_DEFAULTS`, `DELIMITER_OPTIONS`, `ENCODING_OPTIONS`, `HEADER_OPTIONS` (`{ value, label }[]`)
  - `readCsvOptions(csv: unknown): CsvOptions`
  - `writeCsvOptions(csv: unknown, patch: Partial<CsvOptions>): Record<string, unknown>`
  - `csvFileName(templateName: string): string`
  - `downloadState(t: Pick<Template,'resolved_keys'> | null, writing: boolean): { enabled: boolean; reason: string | null }`
- Produces (`rapport/model.ts`):
  - `NO_TEMPLATE = ''`
  - `parseTemplateChoice(value: string): number | null`
  - `validChoice(templates: readonly Pick<Template,'id'>[], id: number | null): number | null`
  - `defaultExportTemplateId(templates: readonly Pick<Template,'id'|'seed'>[]): number | null` (or re-export Phase 3's helper under this name)
  - `ressourcetabelHint(t: Pick<Template,'name'|'resolved_keys'> | null): string`

- [ ] **Step 1: Write the failing tests**

`apps/rux/frontend/src/test/rapport.csvOptions.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  CSV_DEFAULTS,
  DELIMITER_OPTIONS,
  csvFileName,
  downloadState,
  readCsvOptions,
  writeCsvOptions,
} from '../rapport/csvOptions';

describe('readCsvOptions', () => {
  it('fills defaults for an empty or foreign csv object', () => {
    expect(readCsvOptions({})).toEqual(CSV_DEFAULTS);
    expect(readCsvOptions(null)).toEqual(CSV_DEFAULTS);
    expect(readCsvOptions('nonsense')).toEqual(CSV_DEFAULTS);
  });

  it('reads valid values and drops invalid ones field by field', () => {
    expect(readCsvOptions({ delimiter: ';', encoding: 'utf-8-bom', header: 'key' })).toEqual({
      delimiter: ';',
      encoding: 'utf-8-bom',
      header: 'key',
    });
    expect(readCsvOptions({ delimiter: '|', header: 'key' })).toEqual({ ...CSV_DEFAULTS, header: 'key' });
  });
});

describe('writeCsvOptions (R11)', () => {
  it('merges the change and keeps keys it does not own', () => {
    expect(writeCsvOptions({ columns: ['a'], delimiter: ',' }, { delimiter: ';' })).toEqual({
      columns: ['a'],
      delimiter: ';',
    });
  });

  it('starts from an empty object when csv is not one', () => {
    expect(writeCsvOptions(undefined, { header: 'key' })).toEqual({ header: 'key' });
  });
});

describe('option lists', () => {
  it('offers semicolon, comma and tab', () => {
    expect(DELIMITER_OPTIONS.map((o) => o.value)).toEqual([';', ',', '\t']);
  });
});

describe('csvFileName', () => {
  it('makes an ASCII file name from a Danish template name', () => {
    expect(csvFileName('Hurtig genbrugsscreening')).toBe('ressourcer-hurtig-genbrugsscreening.csv');
    expect(csvFileName('Materialepas (fuld)')).toBe('ressourcer-materialepas-fuld.csv');
    expect(csvFileName('Ærø/Øst Å')).toBe('ressourcer-aeroe-oest-aa.csv');
    expect(csvFileName('  ')).toBe('ressourcer.csv');
  });
});

describe('downloadState (R12)', () => {
  it('needs a template with fields and no write in flight', () => {
    expect(downloadState(null, false)).toEqual({ enabled: false, reason: 'Vælg en skabelon.' });
    expect(downloadState({ resolved_keys: [] }, false)).toEqual({
      enabled: false,
      reason: 'Skabelonen har ingen felter — tilføj nogle under Skabeloner.',
    });
    expect(downloadState({ resolved_keys: ['sys:name'] }, true)).toEqual({
      enabled: false,
      reason: 'Gemmer CSV-indstillingerne…',
    });
    expect(downloadState({ resolved_keys: ['sys:name'] }, false)).toEqual({ enabled: true, reason: null });
  });
});
```

(If Phase 1's encoding values or defaults differ — Task 1 Step 1 — change the literals here and in Step 3 together. `CSV_DEFAULTS` is imported, so the default tests follow automatically.)

Append to `apps/rux/frontend/src/test/rapport.model.test.ts` (extend the import list with `defaultExportTemplateId, NO_TEMPLATE, parseTemplateChoice, ressourcetabelHint, validChoice`):

```ts
describe('template choices (R10)', () => {
  const T = [
    { id: 3, seed: 'materialepas' },
    { id: 5, seed: null },
    { id: 8, seed: 'screening' },
  ];

  it('parses the select value', () => {
    expect(parseTemplateChoice(NO_TEMPLATE)).toBeNull();
    expect(parseTemplateChoice('8')).toBe(8);
    expect(parseTemplateChoice('x')).toBeNull();
  });

  it('drops a choice whose template is gone', () => {
    expect(validChoice(T, 5)).toBe(5);
    expect(validChoice(T, 99)).toBeNull();
    expect(validChoice(T, null)).toBeNull();
  });

  it('defaults the export to the screening seed, then the first template', () => {
    expect(defaultExportTemplateId(T)).toBe(8);
    expect(defaultExportTemplateId([{ id: 5, seed: null }])).toBe(5);
    expect(defaultExportTemplateId([])).toBeNull();
  });

  it('says what the Ressourcetabel will hold', () => {
    expect(ressourcetabelHint(null)).toBe('Rapporten genereres uden ressourcetabel.');
    expect(ressourcetabelHint({ name: 'Hurtig genbrugsscreening', resolved_keys: ['a', 'b'] })).toBe(
      'Ressourcetabel med 2 felter fra "Hurtig genbrugsscreening".',
    );
    expect(ressourcetabelHint({ name: 'Tom', resolved_keys: [] })).toBe(
      'Skabelonen "Tom" har ingen felter — tabellen bliver tom.',
    );
  });
});
```

- [ ] **Step 2: Run to verify failure**

Run: `npm --prefix apps/rux/frontend test -- --run src/test/rapport.csvOptions.test.ts src/test/rapport.model.test.ts`
Expected: FAIL — module not found / not exported.

- [ ] **Step 3: Implement**

`apps/rux/frontend/src/rapport/csvOptions.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A template's `csv` JSON as typed CSV options (spec §6.3). The backend CSV
 * builder (`GET /resources/export.csv`) reads the same object, so
 * `CSV_FIELDS` and `CSV_DEFAULTS` must equal its field names and defaults
 * (R11). Writing merges into the stored object and keeps keys this module
 * does not own.
 */

import type { Template } from '../api/types';

export type CsvDelimiter = ',' | ';' | '\t';
export type CsvEncoding = 'utf-8' | 'utf-8-bom';
export type CsvHeader = 'label' | 'key';

export interface CsvOptions {
  delimiter: CsvDelimiter;
  encoding: CsvEncoding;
  header: CsvHeader;
}

/** The JSON field names in `templates.csv` (Phase 1's backend). */
export const CSV_FIELDS = { delimiter: 'delimiter', encoding: 'encoding', header: 'header' } as const;

export const CSV_DEFAULTS: CsvOptions = { delimiter: ',', encoding: 'utf-8', header: 'label' };

export const DELIMITER_OPTIONS: readonly { value: CsvDelimiter; label: string }[] = [
  { value: ';', label: 'Semikolon (;)' },
  { value: ',', label: 'Komma (,)' },
  { value: '\t', label: 'Tabulator' },
];

export const ENCODING_OPTIONS: readonly { value: CsvEncoding; label: string }[] = [
  { value: 'utf-8', label: 'UTF-8' },
  { value: 'utf-8-bom', label: 'UTF-8 med BOM (Excel)' },
];

export const HEADER_OPTIONS: readonly { value: CsvHeader; label: string }[] = [
  { value: 'label', label: 'Feltnavne (fx Betegnelse)' },
  { value: 'key', label: 'Nøgle-id (fx sys:name)' },
];

function asObject(csv: unknown): Record<string, unknown> {
  return csv !== null && typeof csv === 'object' && !Array.isArray(csv) ? (csv as Record<string, unknown>) : {};
}

function pick<T extends string>(value: unknown, allowed: readonly { value: T }[], fallback: T): T {
  return allowed.some((o) => o.value === value) ? (value as T) : fallback;
}

export function readCsvOptions(csv: unknown): CsvOptions {
  const o = asObject(csv);
  return {
    delimiter: pick(o[CSV_FIELDS.delimiter], DELIMITER_OPTIONS, CSV_DEFAULTS.delimiter),
    encoding: pick(o[CSV_FIELDS.encoding], ENCODING_OPTIONS, CSV_DEFAULTS.encoding),
    header: pick(o[CSV_FIELDS.header], HEADER_OPTIONS, CSV_DEFAULTS.header),
  };
}

export function writeCsvOptions(csv: unknown, patch: Partial<CsvOptions>): Record<string, unknown> {
  const next: Record<string, unknown> = { ...asObject(csv) };
  for (const [k, v] of Object.entries(patch) as [keyof CsvOptions, string][]) next[CSV_FIELDS[k]] = v;
  return next;
}

const FOLD: Record<string, string> = { æ: 'ae', ø: 'oe', å: 'aa' };

export function csvFileName(templateName: string): string {
  const slug = templateName
    .toLocaleLowerCase('da-DK')
    .replace(/[æøå]/g, (c) => FOLD[c])
    .normalize('NFKD')
    .replace(/[^a-z0-9]+/g, '-')
    .replace(/^-+|-+$/g, '');
  return slug ? `ressourcer-${slug}.csv` : 'ressourcer.csv';
}

export function downloadState(
  t: Pick<Template, 'resolved_keys'> | null,
  writing: boolean,
): { enabled: boolean; reason: string | null } {
  if (!t) return { enabled: false, reason: 'Vælg en skabelon.' };
  if (t.resolved_keys.length === 0)
    return { enabled: false, reason: 'Skabelonen har ingen felter — tilføj nogle under Skabeloner.' };
  if (writing) return { enabled: false, reason: 'Gemmer CSV-indstillingerne…' };
  return { enabled: true, reason: null };
}

```

Append to `apps/rux/frontend/src/rapport/model.ts` (add `Template` to the type import):

```ts
/** The select value for "Ingen" (no Ressourcetabel). */
export const NO_TEMPLATE = '';

export function parseTemplateChoice(value: string): number | null {
  return /^\d+$/.test(value) && Number(value) > 0 ? Number(value) : null;
}

/** The choice if its template still exists, else null (R10). */
export function validChoice(templates: readonly Pick<Template, 'id'>[], id: number | null): number | null {
  return id !== null && templates.some((t) => t.id === id) ? id : null;
}

/** Data-eksport's default: the screening seed, else the first template (R10). */
export function defaultExportTemplateId(templates: readonly Pick<Template, 'id' | 'seed'>[]): number | null {
  return (templates.find((t) => t.seed === 'screening') ?? templates[0])?.id ?? null;
}

export function ressourcetabelHint(t: Pick<Template, 'name' | 'resolved_keys'> | null): string {
  if (!t) return 'Rapporten genereres uden ressourcetabel.';
  const n = t.resolved_keys.length;
  if (n === 0) return `Skabelonen "${t.name}" har ingen felter — tabellen bliver tom.`;
  return `Ressourcetabel med ${n === 1 ? '1 felt' : `${n} felter`} fra "${t.name}".`;
}
```

If Phase 3 exported an equivalent of `defaultExportTemplateId` (Name map), make this a one-line delegation to it so both pickers share one rule, and keep the test.

- [ ] **Step 4: Run tests**

Run: `npm --prefix apps/rux/frontend test -- --run src/test/rapport.csvOptions.test.ts src/test/rapport.model.test.ts && npm --prefix apps/rux/frontend run typecheck`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/src/rapport/csvOptions.ts apps/rux/frontend/src/rapport/model.ts apps/rux/frontend/src/test/rapport.csvOptions.test.ts apps/rux/frontend/src/test/rapport.model.test.ts
git commit -m "feat(gui): CSV options and template choices for Rapport" -m "Typed read/merge of a template's csv JSON, the download gate, an ASCII file name, and the Ressourcetabel / Data-eksport default rules (spec §6.3)." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 5: Skabeloner route, list and editor

**Files:**
- Modify: `apps/rux/frontend/src/app/links.ts` (add `SKABELONER_PATH`)
- Modify: `apps/rux/frontend/src/app/navigation.ts` (`NAV_ENTRIES`, import)
- Modify: `apps/rux/frontend/src/app/App.tsx` (route)
- Modify: `apps/rux/frontend/src/test/navigation.test.ts`
- Create: `apps/rux/frontend/src/routes/SkabelonerPage.tsx`, `SkabelonerPage.module.css`
- Create: `apps/rux/frontend/src/components/skabeloner/TemplateList.tsx`, `TemplateList.module.css`
- Create: `apps/rux/frontend/src/components/skabeloner/TemplateEditor.tsx`, `TemplateEditor.module.css`

**Interfaces:**
- Consumes: Task 1 client calls; Task 2 `members.ts`; Task 3 `model.ts`; `useAsync`, `useMutationQueue` (scope `'app'`), `appWriteChain`, `useTextDraft`, `fieldKeyAction`/`formKeyDown` if needed, `useToast`, `Toast`, `ErrorBanner`, `EmptyState`, `Spinner`, `Pill`.
- Produces: `SKABELONER_PATH = '/skabeloner'`; `SkabelonerPage`; `TemplateList` props `{ templates: Template[]; selectedId: number | null; busy: boolean; missingSeeds: string[]; onSelect(id); onNew(); onRename(id); onDuplicate(id); onDelete(id); onRestoreSeeds() }`; `TemplateEditor` props `{ template: Template; keys: ResourceKey[]; nameRef: RefObject<HTMLInputElement | null>; onRename(name: string): void; onMembers(next: TemplateMember[]): void }`.

- [ ] **Step 1: Write the failing nav test**

In `navigation.test.ts`, the `'lists the case workflow…'` expectation becomes whatever Phase 2 left plus Skabeloner last. After Phase 2 it should read (spec §3):

```ts
  it('lists the case workflow in the spec order, Skabeloner last', () => {
    expect(entriesIn('sag').map((e) => e.label)).toEqual([
      'Overblik',
      'Kortlægning',
      'Viewport',
      'Miljø & prøver',
      'Rapport',
      'Indberetning',
      'Skabeloner',
    ]);
  });

  it('links Skabeloner live, not as a pending placeholder', () => {
    expect(NAV_ENTRIES.find((e) => e.label === 'Skabeloner')).toEqual({
      to: '/skabeloner',
      label: 'Skabeloner',
      group: 'sag',
    });
  });
```

Replace Phase 2's version of the first test (do not keep two); delete any Phase 2 test that asserts Skabeloner is pending.

- [ ] **Step 2: Run to verify failure**

Run: `npm --prefix apps/rux/frontend test -- --run src/test/navigation.test.ts`
Expected: FAIL — Skabeloner missing or carries `pending`.

- [ ] **Step 3: Wire the path, nav entry and route**

`links.ts`: add after `PROJEKTDATA_PATH` (skip if Phase 2 added it):

```ts
export const SKABELONER_PATH = '/skabeloner';
```

`navigation.ts`: import `SKABELONER_PATH`; make the last `sag` entry `{ to: SKABELONER_PATH, label: 'Skabeloner', group: 'sag' }` (replace a pending one). `App.tsx`: import `SkabelonerPage` and `SKABELONER_PATH`; `<Route path={SKABELONER_PATH} element={<SkabelonerPage />} />` next to `RAPPORT_PATH` (replace a Phase 2 placeholder element).

- [ ] **Step 4: Write `SkabelonerPage.tsx`**

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import { api } from '../api/client';
import type { ResourceKey, Template, TemplateMember } from '../api/types';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { useToast } from '../app/useToast';
import { appWriteChain } from '../app/writeChain';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { TemplateEditor } from '../components/skabeloner/TemplateEditor';
import { TemplateList } from '../components/skabeloner/TemplateList';
import {
  createLatestGate,
  deleteConfirmText,
  missingSeeds,
  nextTemplateName,
  replaceTemplate,
  selectAfterDelete,
  templateErrorMessage,
  withMembers,
} from '../skabeloner/model';
import styles from './SkabelonerPage.module.css';

/**
 * Skabeloner — the project's templates (spec §6.2). List on the left, editor
 * on the right (stacked on a phone). Every write joins the app write chain;
 * member edits save on change as full snapshots, and only the newest edit's
 * response is applied (R3). A failed write re-reads the list in the same
 * queued task, so optimistic state never outlives a failure.
 */
export function SkabelonerPage() {
  const loaded = useAsync(
    (s) => appWriteChain.idle().then(() => Promise.all([api.listTemplates(s), api.resourceKeys(s)])),
    [],
  );
  const [templates, setTemplates] = useState<Template[] | null>(null);
  const [keys, setKeys] = useState<ResourceKey[]>([]);
  const [selectedId, setSelectedId] = useState<number | null>(null);
  const nameRef = useRef<HTMLInputElement | null>(null);
  const [focusName, setFocusName] = useState(false);
  const [gate] = useState(createLatestGate);
  const toast = useToast(3200);
  const { busy, mutate } = useMutationQueue({ onError: (cause) => toast.show(templateErrorMessage(cause)) });

  useEffect(() => {
    if (!loaded.data) return;
    const [list, catalogue] = loaded.data;
    setTemplates(list);
    setKeys(catalogue);
    setSelectedId((id) => (id !== null && list.some((t) => t.id === id) ? id : (list[0]?.id ?? null)));
  }, [loaded.data]);

  // After "Ny skabelon" / "Omdøb", focus the name field once it is rendered.
  useEffect(() => {
    if (focusName && nameRef.current) {
      nameRef.current.focus();
      nameRef.current.select();
      setFocusName(false);
    }
  }, [focusName, selectedId, templates]);

  const relist = async () => {
    const list = await api.listTemplates();
    setTemplates(list);
    setSelectedId((id) => (id !== null && list.some((t) => t.id === id) ? id : (list[0]?.id ?? null)));
  };

  /** Run a write; on failure re-read the list before the error surfaces. */
  const write = (run: () => Promise<void>) =>
    mutate(async () => {
      try {
        await run();
      } catch (cause) {
        await relist().catch(() => undefined);
        throw cause;
      }
    });

  const onMembers = (id: number, next: TemplateMember[]) => {
    setTemplates((prev) => (prev ? withMembers(prev, id, next) : prev));
    const ticket = gate.next(id);
    void write(async () => {
      const saved = await api.patchTemplate(id, { members: next });
      if (gate.isLatest(id, ticket)) setTemplates((prev) => (prev ? replaceTemplate(prev, saved) : prev));
    });
  };

  const onRename = (id: number, name: string) =>
    void write(async () => {
      const saved = await api.patchTemplate(id, { name });
      setTemplates((prev) => (prev ? replaceTemplate(prev, saved) : prev));
    });

  const onNew = () =>
    void write(async () => {
      const created = await api.createTemplate({ name: nextTemplateName((templates ?? []).map((t) => t.name)), members: [] });
      setTemplates((prev) => [...(prev ?? []), created]);
      setSelectedId(created.id);
      setFocusName(true);
    });

  const onDuplicate = (id: number) =>
    void write(async () => {
      const copy = await api.duplicateTemplate(id);
      await relist();
      setSelectedId(copy.id);
    });

  const onDelete = (id: number) => {
    const t = templates?.find((x) => x.id === id);
    if (!t || !window.confirm(deleteConfirmText(t))) return;
    void write(async () => {
      await api.deleteTemplate(id);
      const ids = (templates ?? []).map((x) => x.id);
      setTemplates((prev) => (prev ? prev.filter((x) => x.id !== id) : prev));
      setSelectedId((sel) => (sel === id ? selectAfterDelete(ids, id) : sel));
    });
  };

  const onRestoreSeeds = () =>
    void write(async () => {
      await api.restoreSeedTemplates();
      await relist();
    });

  if (loaded.error && !templates) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={loaded.error} onRetry={loaded.reload} context="skabelonerne" />
      </div>
    );
  }
  if (!templates) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser skabeloner…" />
      </div>
    );
  }

  const selected = templates.find((t) => t.id === selectedId) ?? null;

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h2 className={styles.title}>Skabeloner</h2>
        <span className={styles.sub}>Feltudvalg til Kortlægning og Rapport</span>
      </header>
      <div className={styles.layout}>
        <TemplateList
          templates={templates}
          selectedId={selectedId}
          busy={busy}
          missingSeeds={missingSeeds(templates)}
          onSelect={setSelectedId}
          onNew={onNew}
          onRename={(id) => {
            setSelectedId(id);
            setFocusName(true);
          }}
          onDuplicate={onDuplicate}
          onDelete={onDelete}
          onRestoreSeeds={onRestoreSeeds}
        />
        {selected ? (
          <TemplateEditor
            key={selected.id}
            template={selected}
            keys={keys}
            nameRef={nameRef}
            onRename={(name) => onRename(selected.id, name)}
            onMembers={(next) => onMembers(selected.id, next)}
          />
        ) : (
          <EmptyState
            title="Ingen skabelon valgt"
            detail={templates.length === 0 ? 'Opret en ny, eller gendan standardskabelonerne.' : 'Vælg en skabelon i listen.'}
          />
        )}
      </div>
      <Toast message={toast.message} />
    </div>
  );
}
```

Notes for the implementer:
- `api.duplicateTemplate` is Phase 3's; if it returns the list or nothing instead of the new `Template`, select by name after `relist()` (the copy is the template whose name starts with `"<name> (kopi"` and has the highest id).
- A rename 409 must snap the field back: `useTextDraft` follows `current` while not focused, and `relist()` in `write` restores the server name, so nothing extra is needed.

- [ ] **Step 5: Write `TemplateList.tsx`**

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Template } from '../../api/types';
import { restoreSeedsTitle, seedLabel } from '../../skabeloner/model';
import { Pill } from '../Pill';
import styles from './TemplateList.module.css';

export interface TemplateListProps {
  templates: Template[];
  selectedId: number | null;
  busy: boolean;
  missingSeeds: string[];
  onSelect: (id: number) => void;
  onNew: () => void;
  onRename: (id: number) => void;
  onDuplicate: (id: number) => void;
  onDelete: (id: number) => void;
  onRestoreSeeds: () => void;
}

/** The template list and its actions (spec §6.2). Actions act on the selected row. */
export function TemplateList(p: TemplateListProps) {
  const sel = p.selectedId;
  return (
    <aside className={styles.panel} aria-label="Skabeloner">
      <div className={styles.toolbar}>
        <button type="button" className={styles.btnPrimary} onClick={p.onNew} disabled={p.busy}>
          Ny skabelon
        </button>
      </div>
      <ul className={styles.list}>
        {p.templates.map((t) => (
          <li key={t.id}>
            <button
              type="button"
              className={`${styles.row} ${t.id === sel ? styles.active : ''}`}
              aria-current={t.id === sel ? 'true' : undefined}
              onClick={() => p.onSelect(t.id)}
            >
              <span className={styles.name}>{t.name}</span>
              <span className={styles.meta}>
                {t.resolved_keys.length === 1 ? '1 felt' : `${t.resolved_keys.length} felter`}
                {seedLabel(t.seed) && <Pill tone="accent">{seedLabel(t.seed)}</Pill>}
              </span>
            </button>
          </li>
        ))}
      </ul>
      <div className={styles.actions}>
        <button type="button" className={styles.btnGhost} disabled={sel === null || p.busy} onClick={() => sel !== null && p.onRename(sel)}>
          Omdøb
        </button>
        <button type="button" className={styles.btnGhost} disabled={sel === null || p.busy} onClick={() => sel !== null && p.onDuplicate(sel)}>
          Dupliker
        </button>
        <button type="button" className={styles.btnDanger} disabled={sel === null || p.busy} onClick={() => sel !== null && p.onDelete(sel)}>
          Slet
        </button>
      </div>
      <button
        type="button"
        className={styles.textBtn}
        onClick={p.onRestoreSeeds}
        disabled={p.busy || p.missingSeeds.length === 0}
        title={restoreSeedsTitle(p.missingSeeds)}
      >
        Gendan standardskabeloner
      </button>
    </aside>
  );
}
```

`TemplateList.module.css` (tokens only):

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.panel {
  composes: panel from '../surfaces.module.css';
  display: flex;
  flex-direction: column;
  gap: var(--space-3);
  padding: var(--space-3);
}
.toolbar,
.actions {
  display: flex;
  flex-wrap: wrap;
  gap: var(--space-2);
}
.btnPrimary {
  composes: btnPrimary from '../controls.module.css';
}
.btnGhost {
  composes: btnGhost from '../controls.module.css';
}
.btnDanger {
  composes: btnDanger from '../controls.module.css';
}
.textBtn {
  composes: textBtn from '../controls.module.css';
  align-self: flex-start;
}
.list {
  display: flex;
  flex-direction: column;
  gap: var(--space-1);
  margin: 0;
  padding: 0;
  list-style: none;
}
.row {
  display: flex;
  width: 100%;
  flex-direction: column;
  gap: var(--space-1);
  padding: var(--space-2) var(--space-3);
  border: 1px solid transparent;
  border-radius: var(--radius-md);
  background: none;
  color: var(--color-text);
  font: inherit;
  text-align: left;
  cursor: pointer;
}
.row:hover {
  background: var(--color-surface-sunken);
}
.row:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: 0;
}
.active {
  border-color: var(--color-border-strong);
  background: var(--color-accent-muted);
}
.name {
  font-weight: var(--font-weight-bold);
  font-size: var(--font-size-sm);
}
.meta {
  display: flex;
  align-items: center;
  gap: var(--space-2);
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
}
```

- [ ] **Step 6: Write `TemplateEditor.tsx`**

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useRef, useState, type RefObject } from 'react';

import type { ResourceKey, Template, TemplateMember } from '../../api/types';
import { useTextDraft } from '../../app/useTextDraft';
import {
  addKey,
  categoryCounts,
  hasCategory,
  memberId,
  memberLabel,
  moveMember,
  removeMemberAt,
  resolveMembers,
  searchKeys,
  toggleCategory,
} from '../../skabeloner/members';
import { countLine } from '../../skabeloner/model';
import styles from './TemplateEditor.module.css';

export interface TemplateEditorProps {
  template: Template;
  keys: ResourceKey[];
  nameRef: RefObject<HTMLInputElement | null>;
  onRename: (name: string) => void;
  onMembers: (next: TemplateMember[]) => void;
}

/**
 * One template's editor (spec §6.2): name, Kategorier, Enkelte felter,
 * Rækkefølge and the resolved count. Every change calls `onMembers` with the
 * whole new list; the page saves it. Reorder works by drag and by the ↑/↓
 * buttons, and focus follows the moved row's button.
 */
export function TemplateEditor({ template, keys, nameRef, onRename, onMembers }: TemplateEditorProps) {
  const members = template.members;
  const name = useTextDraft(template.name, onRename, { required: true });
  const [query, setQuery] = useState('');
  const [dragFrom, setDragFrom] = useState<number | null>(null);
  const moveRefs = useRef(new Map<string, HTMLButtonElement | null>());

  const resolved = resolveMembers(members, keys);
  const missing = new Set(resolved.missing.map(memberId));
  const hits = searchKeys(keys, query, members);

  const move = (from: number, to: number, dir: 'up' | 'down') => {
    const next = moveMember(members, from, to);
    if (next === members) return;
    onMembers(next);
    const id = memberId(members[from]);
    requestAnimationFrame(() => moveRefs.current.get(`${id}:${dir}`)?.focus());
  };

  return (
    <section className={styles.panel} aria-label={`Skabelon ${template.name}`}>
      <label className={styles.field}>
        <span className={styles.fieldLabel}>Navn</span>
        <input
          ref={nameRef}
          className={styles.input}
          {...name.props}
          onKeyDown={(e) => {
            if (e.key === 'Enter') e.currentTarget.blur();
            if (e.key === 'Escape') name.revert(e.currentTarget);
          }}
        />
      </label>

      <p className={styles.count} aria-live="polite">
        {countLine(resolved.keys.length)}
      </p>

      <fieldset className={styles.group}>
        <legend className={styles.heading}>Kategorier</legend>
        <div className={styles.categories}>
          {categoryCounts(keys).map(({ category, count }) => (
            <label key={category} className={styles.check}>
              <input
                type="checkbox"
                className={styles.checkbox}
                checked={hasCategory(members, category)}
                onChange={() => onMembers(toggleCategory(members, category))}
              />
              <span>{category}</span>
              <span className={styles.muted}>{count}</span>
            </label>
          ))}
        </div>
      </fieldset>

      <div className={styles.group}>
        <h3 className={styles.heading}>Enkelte felter</h3>
        <input
          type="search"
          className={styles.input}
          placeholder="Søg i felter…"
          aria-label="Søg i felter"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
        />
        {hits.length > 0 && (
          <ul className={styles.hits}>
            {hits.map((k) => (
              <li key={k.id} className={styles.hit}>
                <span>
                  {k.label} <span className={styles.muted}>· {k.category}</span>
                </span>
                <button type="button" className={styles.textBtn} onClick={() => onMembers(addKey(members, k.id))}>
                  Tilføj
                </button>
              </li>
            ))}
          </ul>
        )}
        {query.trim() !== '' && hits.length === 0 && <p className={styles.muted}>Ingen felter matcher.</p>}
      </div>

      <div className={styles.group}>
        <h3 className={styles.heading}>Rækkefølge</h3>
        {members.length === 0 ? (
          <p className={styles.muted}>Skabelonen er tom — vælg kategorier eller tilføj felter.</p>
        ) : (
          <ol className={styles.order}>
            {members.map((m, i) => {
              const id = memberId(m);
              const l = memberLabel(m, keys);
              const gone = missing.has(id);
              return (
                <li
                  key={id}
                  className={`${styles.member} ${dragFrom === i ? styles.dragging : ''}`}
                  draggable
                  onDragStart={(e) => {
                    setDragFrom(i);
                    e.dataTransfer.effectAllowed = 'move';
                  }}
                  onDragOver={(e) => e.preventDefault()}
                  onDrop={(e) => {
                    e.preventDefault();
                    if (dragFrom !== null) onMembers(moveMember(members, dragFrom, i));
                    setDragFrom(null);
                  }}
                  onDragEnd={() => setDragFrom(null)}
                >
                  <span className={styles.grip} aria-hidden="true">⋮⋮</span>
                  <span className={`${styles.memberMain} ${gone ? styles.gone : ''}`}>
                    <span className={styles.memberName}>{l.label}</span>
                    <span className={styles.muted}>
                      {l.kind} · {l.detail}
                    </span>
                  </span>
                  <span className={styles.memberActions}>
                    <button
                      type="button"
                      className={styles.iconBtn}
                      aria-label={`Flyt ${l.label} op`}
                      disabled={i === 0}
                      ref={(el) => void moveRefs.current.set(`${id}:up`, el)}
                      onClick={() => move(i, i - 1, 'up')}
                    >
                      ↑
                    </button>
                    <button
                      type="button"
                      className={styles.iconBtn}
                      aria-label={`Flyt ${l.label} ned`}
                      disabled={i === members.length - 1}
                      ref={(el) => void moveRefs.current.set(`${id}:down`, el)}
                      onClick={() => move(i, i + 1, 'down')}
                    >
                      ↓
                    </button>
                    <button
                      type="button"
                      className={styles.iconBtn}
                      aria-label={`Fjern ${l.label}`}
                      onClick={() => onMembers(removeMemberAt(members, i))}
                    >
                      ×
                    </button>
                  </span>
                </li>
              );
            })}
          </ol>
        )}
      </div>
    </section>
  );
}
```

Check `useTextDraft`'s Esc handling first (`app/useTextDraft.ts`, `fieldKeyAction`): if `name.props` already handles Enter/Esc via its own `onKeyDown`, drop the inline `onKeyDown` and use the hook's. `members` keys are `memberId`s, which are unique because `addKey`/`toggleCategory` never add a duplicate; if Phase 1's migration can produce duplicate members, key by `${id}#${i}`.

`TemplateEditor.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.panel {
  composes: panel from '../surfaces.module.css';
  display: flex;
  flex-direction: column;
  gap: var(--space-4);
  padding: var(--space-4);
}
.field {
  composes: field from '../controls.module.css';
}
.fieldLabel {
  composes: fieldLabel from '../controls.module.css';
}
.input {
  composes: input from '../controls.module.css';
}
.checkbox {
  composes: checkbox from '../controls.module.css';
}
.textBtn {
  composes: textBtn from '../controls.module.css';
}
.count {
  margin: 0;
  font-size: var(--font-size-sm);
  font-weight: var(--font-weight-bold);
}
.group {
  display: flex;
  flex-direction: column;
  gap: var(--space-2);
  margin: 0;
  padding: 0;
  border: 0;
  min-width: 0;
}
.heading {
  composes: fieldLabel from '../controls.module.css';
  margin: 0;
  padding: 0;
}
.categories {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(14rem, 1fr));
  gap: var(--space-1) var(--space-3);
}
.check {
  display: flex;
  align-items: center;
  gap: var(--space-2);
  font-size: var(--font-size-sm);
}
.muted {
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
}
.hits,
.order {
  display: flex;
  flex-direction: column;
  gap: var(--space-1);
  margin: 0;
  padding: 0;
  list-style: none;
}
.hit {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: var(--space-2);
  padding: var(--space-1) var(--space-2);
  border-radius: var(--radius-sm);
  background: var(--color-surface-sunken);
  font-size: var(--font-size-sm);
}
.member {
  display: flex;
  align-items: center;
  gap: var(--space-2);
  padding: var(--space-2);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-md);
  background: var(--color-surface);
}
.dragging {
  opacity: 0.5;
}
.grip {
  color: var(--color-text-faint);
  cursor: grab;
}
.memberMain {
  display: flex;
  flex: 1;
  min-width: 0;
  flex-direction: column;
}
.memberName {
  font-size: var(--font-size-sm);
  overflow-wrap: anywhere;
}
.gone .memberName {
  text-decoration: line-through;
  color: var(--color-text-faint);
}
.memberActions {
  display: flex;
  gap: var(--space-1);
}
.iconBtn {
  composes: btnGhost from '../controls.module.css';
  min-width: 2rem;
  padding-inline: var(--space-1);
}
```

`SkabelonerPage.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.page {
  composes: page from './viewHead.module.css';
}
.head {
  composes: head from './viewHead.module.css';
}
.title {
  composes: title from './viewHead.module.css';
}
.sub {
  composes: sub from './viewHead.module.css';
}
.layout {
  display: grid;
  grid-template-columns: minmax(14rem, 18rem) minmax(0, 1fr);
  gap: var(--space-4);
  align-items: start;
}

/* The shell's phone breakpoint (navigation.ts DRAWER_QUERY). */
@media (max-width: 56.25rem) {
  .layout {
    grid-template-columns: minmax(0, 1fr);
  }
}
```

- [ ] **Step 7: Tests, typecheck, lint**

```bash
npm --prefix apps/rux/frontend test -- --run && npm --prefix apps/rux/frontend run typecheck
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/routes/SkabelonerPage.tsx apps/rux/frontend/src/routes/SkabelonerPage.module.css apps/rux/frontend/src/components/skabeloner/*.tsx apps/rux/frontend/src/components/skabeloner/*.module.css --tsx
```

Expected: all vitest pass (navigation test now green); no type errors; lint clean.

- [ ] **Step 8: Visual check (both themes, desktop + phone)**

```bash
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
cmake --build build --target rux   # only if build/apps/rux/rux is older than main's Phase 1–3 merge
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/corridor-clouds.rux" 8429 5182
for th in light dark; do
  bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5182/skabeloner" --out "$SP/shots/rt4-skabeloner" --theme $th --viewports desktop,mobile
done
```

Open every PNG with Read. Expected: both seeds listed with a `Standard` pill and their field counts; the selected seed's editor shows its name, the count line, ticked categories (`Materialepas (fuld)`) or 11 key members (`Hurtig genbrugsscreening`); at mobile width the list sits above the editor with no horizontal scroll; dark theme has no white panels or unreadable text. Then, by hand in the browser (http://localhost:5182/skabeloner): tick a category, drag a member, use ↑/↓ (focus stays on the moved row's arrow), add a key via search, rename to an existing name (toast "Der findes allerede…", name snaps back), "Ny skabelon", "Dupliker", "Slet" with confirm, then "Gendan standardskabeloner" after deleting a seed. Reload the page after each: the server state equals the screen. Stop: `bash .claude/skills/design-studio/scripts/dev_env.sh stop`.

- [ ] **Step 9: Commit**

```bash
git add apps/rux/frontend/src/app/links.ts apps/rux/frontend/src/app/navigation.ts apps/rux/frontend/src/app/App.tsx apps/rux/frontend/src/test/navigation.test.ts apps/rux/frontend/src/routes/SkabelonerPage.tsx apps/rux/frontend/src/routes/SkabelonerPage.module.css apps/rux/frontend/src/components/skabeloner
git commit -m "feat(gui): Skabeloner page" -m "List + editor for project templates (spec §6.2): new, rename, duplicate, delete with confirmation, restore seeds; Kategorier, Enkelte felter, Rækkefølge with drag and ↑/↓, struck-through missing members and a live field count. Edits save on change through the app write chain." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 6: Rapport — Ressourcetabel option and Data-eksport panel

**Files:**
- Create (unless Phase 3 has a reusable picker, R9): `apps/rux/frontend/src/components/rapport/TemplateSelect.tsx`
- Create: `apps/rux/frontend/src/components/rapport/DataExportPanel.tsx`, `DataExportPanel.module.css`
- Modify: `apps/rux/frontend/src/routes/RapportPage.tsx`, `RapportPage.module.css`

**Interfaces:**
- Consumes: `api.listTemplates`, `api.patchTemplate`, `api.resourcesExportCsvUrl`, `api.generateReport(templateId)` (Task 1); `rapport/csvOptions.ts`, `rapport/model.ts` additions (Task 4); `templateErrorMessage` (Task 3); `SKABELONER_PATH` (Task 5).
- Produces:
  - `TemplateSelect` props `{ id: string; label: string; templates: Template[]; value: number | null; onChange(id: number | null): void; allowNone?: boolean; disabled?: boolean }`
  - `DataExportPanel` props `{ templates: Template[]; selectedId: number | null; onSelect(id: number | null): void; onCsvChange(patch: Partial<CsvOptions>): void; writing: boolean; csvUrl: (id: number) => string }`

- [ ] **Step 1: Write `TemplateSelect.tsx`**

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Template } from '../../api/types';
import { NO_TEMPLATE, parseTemplateChoice } from '../../rapport/model';
import controls from '../controls.module.css';

export interface TemplateSelectProps {
  id: string;
  label: string;
  templates: Template[];
  value: number | null;
  onChange: (id: number | null) => void;
  /** Offer "Ingen" as the first option (the Ressourcetabel picker). */
  allowNone?: boolean;
  disabled?: boolean;
}

/** A labelled native select over the project's templates (R9). */
export function TemplateSelect({ id, label, templates, value, onChange, allowNone, disabled }: TemplateSelectProps) {
  return (
    <label className={controls.field} htmlFor={id}>
      <span className={controls.fieldLabel}>{label}</span>
      <select
        id={id}
        className={controls.input}
        value={value === null ? NO_TEMPLATE : String(value)}
        disabled={disabled}
        onChange={(e) => onChange(parseTemplateChoice(e.target.value))}
      >
        {allowNone && <option value={NO_TEMPLATE}>Ingen</option>}
        {!allowNone && value === null && <option value={NO_TEMPLATE}>Vælg skabelon…</option>}
        {templates.map((t) => (
          <option key={t.id} value={String(t.id)}>
            {t.name}
          </option>
        ))}
      </select>
    </label>
  );
}
```

(If the project lints direct imports of `controls.module.css` into a component — check how another component consumes shared classes; most compose them in their own module — create `TemplateSelect.module.css` composing `field`, `fieldLabel`, `input` instead.)

- [ ] **Step 2: Write `DataExportPanel.tsx` + css**

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link } from 'react-router-dom';

import type { Template } from '../../api/types';
import { SKABELONER_PATH } from '../../app/links';
import {
  csvFileName,
  DELIMITER_OPTIONS,
  downloadState,
  ENCODING_OPTIONS,
  HEADER_OPTIONS,
  readCsvOptions,
  type CsvOptions,
} from '../../rapport/csvOptions';
import { countLine } from '../../skabeloner/model';
import { TemplateSelect } from './TemplateSelect';
import styles from './DataExportPanel.module.css';

export interface DataExportPanelProps {
  templates: Template[];
  selectedId: number | null;
  onSelect: (id: number | null) => void;
  onCsvChange: (patch: Partial<CsvOptions>) => void;
  /** A CSV-option write is queued or in flight (R12). */
  writing: boolean;
  csvUrl: (id: number) => string;
}

/**
 * Data-eksport (spec §6.3): pick a template, set its CSV options (saved back
 * to the template), download the backend-built CSV.
 */
export function DataExportPanel({ templates, selectedId, onSelect, onCsvChange, writing, csvUrl }: DataExportPanelProps) {
  const t = templates.find((x) => x.id === selectedId) ?? null;
  const opts = readCsvOptions(t?.csv);
  const dl = downloadState(t, writing);

  return (
    <section className={styles.panel} aria-labelledby="rapport-dataeksport">
      <h3 id="rapport-dataeksport" className={styles.heading}>
        Data-eksport
      </h3>
      {templates.length === 0 ? (
        <p className={styles.muted}>
          Der er ingen skabeloner i projektet. Opret eller gendan dem under{' '}
          <Link className={styles.crossLink} to={SKABELONER_PATH}>
            Skabeloner
          </Link>
          .
        </p>
      ) : (
        <>
          <div className={styles.grid}>
            <TemplateSelect id="dataeksport-template" label="Skabelon" templates={templates} value={selectedId} onChange={onSelect} />
            <Choice label="Skilletegn" value={opts.delimiter} options={DELIMITER_OPTIONS} disabled={!t} onChange={(v) => onCsvChange({ delimiter: v })} />
            <Choice label="Tegnsæt" value={opts.encoding} options={ENCODING_OPTIONS} disabled={!t} onChange={(v) => onCsvChange({ encoding: v })} />
            <Choice label="Kolonneoverskrift" value={opts.header} options={HEADER_OPTIONS} disabled={!t} onChange={(v) => onCsvChange({ header: v })} />
          </div>
          <div className={styles.row}>
            {t && dl.enabled ? (
              <a className={styles.btnPrimary} href={csvUrl(t.id)} download={csvFileName(t.name)}>
                Download CSV
              </a>
            ) : (
              <button type="button" className={styles.btnPrimary} disabled>
                Download CSV
              </button>
            )}
            <span className={styles.muted} role="status">
              {dl.reason ?? `${countLine(t!.resolved_keys.length)} · én række pr. ressource`}
            </span>
            <Link className={styles.crossLink} to={SKABELONER_PATH}>
              Redigér skabeloner
            </Link>
          </div>
        </>
      )}
    </section>
  );
}

function Choice<T extends string>({
  label,
  value,
  options,
  disabled,
  onChange,
}: {
  label: string;
  value: T;
  options: readonly { value: T; label: string }[];
  disabled: boolean;
  onChange: (v: T) => void;
}) {
  return (
    <label className={styles.field}>
      <span className={styles.fieldLabel}>{label}</span>
      <select className={styles.input} value={value} disabled={disabled} onChange={(e) => onChange(e.target.value as T)}>
        {options.map((o) => (
          <option key={o.value} value={o.value}>
            {o.label}
          </option>
        ))}
      </select>
    </label>
  );
}
```

`DataExportPanel.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.panel {
  composes: panel from '../surfaces.module.css';
  display: flex;
  flex-direction: column;
  gap: var(--space-3);
  padding: var(--space-4);
}
.heading {
  composes: panelHeading from '../surfaces.module.css';
}
.grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(12rem, 1fr));
  gap: var(--space-3);
}
.field {
  composes: field from '../controls.module.css';
}
.fieldLabel {
  composes: fieldLabel from '../controls.module.css';
}
.input {
  composes: input from '../controls.module.css';
}
.btnPrimary {
  composes: btnPrimary from '../controls.module.css';
  text-decoration: none;
}
.crossLink {
  composes: crossLink from '../controls.module.css';
  margin-left: auto;
}
.row {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: var(--space-3);
}
.muted {
  margin: 0;
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
}
```

- [ ] **Step 3: Wire RapportPage**

In `RapportPage.tsx`:
- Load templates alongside the versions list: `const tpl = useAsync((s) => appWriteChain.idle().then(() => api.listTemplates(s)), []);` and mirror into state `const [templates, setTemplates] = useState<Template[]>([]);` via `useEffect` on `tpl.data`.
- State: `const [pdfTemplate, setPdfTemplate] = useState<number | null>(null);` (R10: starts at Ingen) and `const [exportTemplate, setExportTemplate] = useState<number | null>(null);`. When templates load: `setExportTemplate((id) => validChoice(list, id) ?? defaultExportTemplateId(list)); setPdfTemplate((id) => validChoice(list, id));`.
- A second queue for template writes on the app chain: `const csvQueue = useMutationQueue({ onError: (c) => toast.show(templateErrorMessage(c)) });` (default scope `'app'`). The existing generate queue stays `scope: 'page'`.
- `onCsvChange`:

```tsx
  const onCsvChange = (patch: Partial<CsvOptions>) => {
    const t = templates.find((x) => x.id === exportTemplate);
    if (!t) return;
    const csv = writeCsvOptions(t.csv, patch);
    setTemplates((prev) => replaceTemplate(prev, { ...t, csv }));
    void csvQueue.mutate(async () => {
      try {
        const saved = await api.patchTemplate(t.id, { csv });
        setTemplates((prev) => replaceTemplate(prev, saved));
      } catch (cause) {
        setTemplates(await api.listTemplates().catch(() => templates));
        throw cause;
      }
    });
  };
```

- `generate` passes the choice: `const created = await api.generateReport(validChoice(templates, pdfTemplate));`.
- In the head's `.actions`, before the button:

```tsx
          <div className={styles.ressourcetabel}>
            <TemplateSelect
              id="rapport-ressourcetabel"
              label="Ressourcetabel"
              templates={templates}
              value={validChoice(templates, pdfTemplate)}
              onChange={setPdfTemplate}
              allowNone
              disabled={busy}
            />
          </div>
```

  and under the hero, a one-line hint: `<p className={styles.heroScope}>{ressourcetabelHint(templates.find((t) => t.id === validChoice(templates, pdfTemplate)) ?? null)}</p>` — place it directly under the head (not inside the hero; the hero describes the survey), using a new `.hint` class.
- After the versions list (`VersionList`, unchanged — R13) and before the footnote:

```tsx
      {tpl.error && templates.length === 0 ? (
        <ErrorBanner error={tpl.error} onRetry={tpl.reload} context="skabelonerne" />
      ) : (
        <DataExportPanel
          templates={templates}
          selectedId={validChoice(templates, exportTemplate)}
          onSelect={setExportTemplate}
          onCsvChange={onCsvChange}
          writing={csvQueue.busy}
          csvUrl={(id) => api.resourcesExportCsvUrl(id)}
        />
      )}
```

- Imports: `Template` type; `TemplateSelect`, `DataExportPanel`; `writeCsvOptions`, `type CsvOptions`; `defaultExportTemplateId`, `ressourcetabelHint`, `validChoice` from `../rapport/model`; `replaceTemplate`, `templateErrorMessage` from `../skabeloner/model`.
- Update the component's doc comment: Rapport now also holds the Data-eksport panel and the Ressourcetabel choice (spec §6.3).

`RapportPage.module.css` additions:

```css
.ressourcetabel {
  min-width: 12rem;
}
.hint {
  margin: 0;
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
}
```

If `viewHead.module.css`'s `.actions` aligns items to the centre, add `align-items: flex-end;` on `.actions` here so the labelled select and the button share a baseline; check the desktop and mobile shots.

- [ ] **Step 4: Tests, typecheck, lint**

```bash
npm --prefix apps/rux/frontend test -- --run && npm --prefix apps/rux/frontend run typecheck
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/routes/RapportPage.tsx apps/rux/frontend/src/routes/RapportPage.module.css apps/rux/frontend/src/components/rapport/*.tsx apps/rux/frontend/src/components/rapport/*.module.css --tsx
```

Expected: PASS, lint clean.

- [ ] **Step 5: Visual + flow check (both themes, desktop + phone)**

```bash
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/corridor-clouds.rux" 8429 5182
for th in light dark; do
  bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5182/rapport" --out "$SP/shots/rt4-rapport" --theme $th --viewports desktop,mobile
done
```

Read every PNG. Expected: head shows the `Ressourcetabel` select (`Ingen`) beside `Generér ny version`; the hint line reads "Rapporten genereres uden ressourcetabel."; the version list is unchanged (incl. Inventarliste); the Data-eksport panel shows `Hurtig genbrugsscreening` preselected, three CSV selects, `Download CSV` and "11 felter · én række pr. ressource"; mobile stacks without horizontal scroll; dark theme readable.

By hand in the browser:
1. Choose `Semikolon (;)` and immediately click `Download CSV` — the button is disabled until the PATCH lands; the downloaded file uses `;` (open it: `head -3 ~/Downloads/ressourcer-hurtig-genbrugsscreening.csv`). Reload: the option is still `;` (saved to the template).
2. Choose `Kolonneoverskrift: Nøgle-id`, download: the header row is `sys:name;sys:quantity;…`.
3. Set Ressourcetabel to `Materialepas (fuld)`, `Generér ny version`, download the new PDF and confirm a Ressourcetabel section with stacked tables of ≤ 8 columns, Betegnelse repeated (Phase 1's Typst section). Generate again with `Ingen`: no section.
4. Delete the export-selected template on /skabeloner, come back to /rapport: the picker falls back to the default, nothing errors.

Stop the servers.

- [ ] **Step 6: Commit**

```bash
git add apps/rux/frontend/src/routes/RapportPage.tsx apps/rux/frontend/src/routes/RapportPage.module.css apps/rux/frontend/src/components/rapport
git commit -m "feat(gui): Rapport Data-eksport panel and Ressourcetabel option" -m "Rapport now exports the resources CSV for a chosen template, with delimiter, encoding and header saved back to the template, and can add a Ressourcetabel built from a template to a new PDF version (spec §6.3)." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 7: Delete ExportPage, redirect `/export`

**Files:**
- Delete: `apps/rux/frontend/src/routes/ExportPage.tsx`, `apps/rux/frontend/src/routes/ExportPage.module.css`
- Delete (if no other importer): `apps/rux/frontend/src/data/csvExport.ts`, `apps/rux/frontend/src/test/csvExport.test.ts`
- Modify: `apps/rux/frontend/src/app/App.tsx`, `apps/rux/frontend/src/api/client.ts` (`:1076-1107` export-templates block), `apps/rux/frontend/src/api/types.ts` (`ExportTemplate`, `:729-741`), `apps/rux/frontend/src/test/navigation.test.ts` (if it lists `/export`), Phase 2's redirect table + test if one exists

**Interfaces:**
- Consumes: `RAPPORT_PATH`.
- Produces: `/export` → `/rapport` (replace).

- [ ] **Step 1: Find every user**

```bash
grep -rn "ExportPage\|csvExport'\|csvExport\"\|ExportTemplate\|listExportTemplates\|createExportTemplate\|updateExportTemplate\|deleteExportTemplate\|'/export'" apps/rux/frontend/src
grep -rn "REDIRECT\|Navigate to" apps/rux/frontend/src/app
```

Expected: ExportPage, its css, `csvExport.ts` + its test, the client/types block, `App.tsx` route, maybe a nav test or a Phase 2 redirect table. `csvExportUrl` (`/exports/csv`) stays — R13.

- [ ] **Step 2: Pin the redirect in a test (only if Phase 2 made redirects data)**

If Phase 2 introduced a pure redirect table (e.g. `REDIRECTS` in `navigation.ts` with a test), add `/export → /rapport` to its test first and run it (FAIL), then add the row (PASS). If redirects are only JSX in `App.tsx`, there is nothing Node can test; the redirect is checked in Step 5.

- [ ] **Step 3: Delete and redirect**

```bash
git rm apps/rux/frontend/src/routes/ExportPage.tsx apps/rux/frontend/src/routes/ExportPage.module.css
git rm apps/rux/frontend/src/data/csvExport.ts apps/rux/frontend/src/test/csvExport.test.ts
```

In `App.tsx`: remove the `ExportPage` import; the `/export` route becomes `<Route path="/export" element={<Navigate to={RAPPORT_PATH} replace />} />` (or Phase 2's redirect table row; do not register both). Remove the four export-template methods from `client.ts`, the `ExportTemplate` import there and the interface in `types.ts`. Remove `/export` from any navigation test expectation that still lists it (Phase 2 should already have).

- [ ] **Step 4: Tests, typecheck, build**

Run: `npm --prefix apps/rux/frontend test -- --run && npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend run build`
Expected: PASS; the build emits no warning about a missing module.

- [ ] **Step 5: Redirect check**

With the dev servers up (Task 6 Step 5 command): `bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5182/export" --out "$SP/shots/rt4-export-redirect" --theme light --viewports desktop`. Read the PNG: it is the Rapport page and the Rapport nav entry is active. Stop the servers.

- [ ] **Step 6: Commit**

```bash
git add -A apps/rux/frontend/src
git commit -m "refactor(gui): retire the Eksport page" -m "Rapport's Data-eksport panel replaces it; /export redirects to /rapport. The old column-selection model and the export-template client calls go with it." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 8: Delete `rux gui`'s export-template routes

**Files:**
- Modify: `apps/rux/src/gui/Server.cpp` (`/api/v1/export-templates` routes, `:1231-1265` before Phase 1)
- Modify: `apps/rux/src/gui/api.cpp` (endpoint list `:716-723`; handlers `:2823-2870`)
- Modify: `apps/rux/include/gui/api.hpp` (declarations `:681-700`)
- Modify: `tests/unit/rux_gui/test_gui_api.cpp` (endpoint list `:173-177`)
- Modify: `docs/gui/openapi.yaml` (tag `export-templates` `:86`, paths `:2402-…`, schema `ExportTemplate`)

**Interfaces:**
- Consumes: nothing.
- Produces: no `rux gui` route under `/api/v1/export-templates`; a request there gets the SPA/404 behaviour of any unknown `/api` path.

- [ ] **Step 1: See what Phase 1 left**

```bash
grep -rn "export-templates\|export_template\|ExportTemplate" apps/rux apps/ruxd libs/reusex tests docs/gui --include=*.cpp --include=*.hpp --include=*.yaml
```

If no `apps/rux` hits remain, Phase 1 already removed them: skip to Step 6 (openapi) and only remove what is left. Note any `apps/ruxd` hits for the report (R14) — do not edit them.

- [ ] **Step 2: Update the endpoint-list test first**

In `tests/unit/rux_gui/test_gui_api.cpp`, delete the five `"… /api/v1/export-templates…"` lines (`:173-177`) from the expected endpoint list, and add a negative check in the same `TEST_CASE` (adapt the variable name to the test's):

```cpp
  for (const auto &e : endpoints)
    CHECK(e.path.find("/export-templates") == std::string::npos);
```

(Read the test first: if it compares the full list with `==`, deleting the lines is the failing change and the loop is extra protection.)

- [ ] **Step 3: Build and run to verify failure**

```bash
cmake --build build --target reusex_unit_tests rux_gui_tests 2>&1 | tail -5   # use the target that holds tests/unit/rux_gui (grep tests/CMakeLists.txt for rux_gui)
cd build && ctest --output-on-failure --parallel $(nproc) -R "endpoint" ; cd ..
```

Expected: FAIL — the server still lists the five endpoints.

- [ ] **Step 4: Delete the routes, handlers, declarations and listing**

- `Server.cpp`: delete both `app_.route_dynamic("/api/v1/export-templates…")` blocks; fix the section comment above `get("/api/v1/exports/csv")` to `// ---- CSV export (#459) ----`.
- `api.cpp`: delete the five `{"…", "/api/v1/export-templates…", …}` endpoint rows and the functions `list_export_templates_json`, `create_export_template_json`, `get_export_template_json`, `update_export_template_json`, `delete_export_template`, plus `template_record_json` if nothing else calls it (`grep -n template_record_json apps/rux/src/gui/api.cpp`).
- `api.hpp`: delete their declarations; retitle the section comment to `// --- CSV export (#459) ---`.
- `ProjectDB`: `grep -rn "list_export_templates\|add_export_template\|update_export_template\|delete_export_template\|export_template(" apps libs tests --include=*.cpp --include=*.hpp`. If only `ProjectDB.{hpp,cpp}` and their own tests reference them, leave them (Phase 1 owns the store; R14). Do not touch `apps/ruxd`.

- [ ] **Step 5: Build and test**

```bash
cmake --build build --target rux reusex_unit_tests 2>&1 | tail -5
cd build && ctest --output-on-failure --parallel $(nproc) -R "gui|endpoint|template" ; cd ..
```

Expected: build clean (no unused-function warnings), tests PASS.

- [ ] **Step 6: openapi**

In `docs/gui/openapi.yaml`, delete the `export-templates` tag entry (`:86`), the `/export-templates` and `/export-templates/{id}` path items, and `components.schemas.ExportTemplate` if nothing else `$ref`s it (`grep -n "ExportTemplate" docs/gui/openapi.yaml`). Validate:

```bash
python3 -c "import yaml,sys; yaml.safe_load(open('docs/gui/openapi.yaml'))" && echo ok
```

Expected: `ok`. (The lint workflow parses this file.)

- [ ] **Step 7: Commit**

```bash
git add apps/rux/src/gui/Server.cpp apps/rux/src/gui/api.cpp apps/rux/include/gui/api.hpp tests/unit/rux_gui/test_gui_api.cpp docs/gui/openapi.yaml
git commit -m "refactor(gui): drop the export-template routes" -m "Templates replaced export templates in schema v25 and the Eksport page is gone, so rux gui no longer serves /api/v1/export-templates. /exports/csv stays." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 9: Docs — CLAUDE.md, DIRECTION.md, phase spec

**Files:**
- Modify: `CLAUDE.md` (`gui` row, `:470`)
- Modify: `docs/DIRECTION.md` (changelog `:191`, plus any Active-workstreams line that names On-site, Materialedata or Eksport)
- Modify: `docs/design/gui-kortlaegning-redesign.md` (On-site rows/sections: `:25`, `:92`, `:332-360`, phases list `:391`)

- [ ] **Step 1: Find stale mentions**

```bash
grep -n -i "on-site\|onsite\|materialedata\|/export\b\|Eksport\b\|export-templates" CLAUDE.md docs/DIRECTION.md docs/design/gui-kortlaegning-redesign.md
```

Skip any item Phase 1–3 already updated.

- [ ] **Step 2: CLAUDE.md**

Replace the `gui` row's description with:

```markdown
| `gui` | — (serves the web frontend over the REST + WebSocket contract in `docs/gui/openapi.yaml`; `--bind`/`--allow-origin` to serve it beyond localhost, with no authentication) | `src/gui.cpp` |
```

- [ ] **Step 3: DIRECTION.md changelog**

Insert at the top of `## Direction changelog`:

```markdown
- **2026-10-02** — **GUI: resources, templates and navigation cleanup**
  ([spec](superpowers/specs/2026-10-02-resources-templates-ia-design.md)).
  Materials are now *resources* in the GUI: one per survey part, carrying any
  number of key/value pairs, with the material passport as one set of keys
  (schema v25). Project-stored **templates** select keys by category or one by
  one; Kortlægning edits through them, the new **Skabeloner** page maintains
  them (two seeds: Materialepas (fuld), Hurtig genbrugsscreening), and
  **Rapport** exports CSV and a PDF Ressourcetabel through them. Eksport and
  Materialedata are retired into Rapport and Kortlægning, Viewport moved into
  the case section, and **On-site left `rux gui` and the backend** — it moves
  to the future mobile app. xlsx export, a global template library and the
  Excel-supplied category selection are deferred.
```

If Active workstreams or Non-goals mention On-site in `rux gui`, reword to "On-site belongs to the mobile-app track".

- [ ] **Step 4: Phase spec On-site note**

In `docs/design/gui-kortlaegning-redesign.md`, directly under the `## Sager and On-site screens …` heading (`:332`), add:

```markdown
> **2026-10-02:** On-site was removed from `rux gui` and its backend
> (`samples.part_code` dropped in schema v25) and moved to the mobile-app
> track — see `docs/superpowers/specs/2026-10-02-resources-templates-ia-design.md` §8.
> The description below is kept as the design record for that app.
```

Append ` — later removed from rux gui; On-site moved to the mobile-app track (2026-10-02)` to phases list item 6 (`:391`), and mark the IA table's `/on-site` row (`:92`) `(removed 2026-10-02 → mobile app)`.

- [ ] **Step 5: Check and commit**

```bash
python3 -c "import yaml; yaml.safe_load(open('docs/gui/openapi.yaml'))" && echo ok   # untouched, sanity
git diff --stat
git add CLAUDE.md docs/DIRECTION.md docs/design/gui-kortlaegning-redesign.md
git commit -m "docs: resources/templates redesign and On-site's move to the mobile app" -m "CLAUDE.md's gui row no longer mentions phones, DIRECTION.md gets a dated changelog entry, and the phase spec notes that On-site left rux gui." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 10: Final verification sweep

**Files:** none changed unless a check fails (fix in the owning task's files, commit as `fix(gui): …`).

- [ ] **Step 1: Whole frontend**

```bash
npm --prefix apps/rux/frontend test -- --run && npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend run build
git diff --name-only main... -- 'apps/rux/frontend/src/**/*.css' 'apps/rux/frontend/src/**/*.tsx' | xargs python3 .claude/skills/design-studio/scripts/token_lint.py --tsx
```

Expected: all pass, lint clean.

- [ ] **Step 2: Backend tests touched by Task 8**

```bash
cmake --build build --target rux reusex_unit_tests
cd build && ctest --output-on-failure --parallel $(nproc) ; cd ..
```

Expected: PASS (the whole suite, since Task 8 changed a shared test file).

- [ ] **Step 3: Visual sweep (spec §9 Visual)**

```bash
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/corridor-clouds.rux" 8429 5182
for page in skabeloner rapport kortlaegning; do
  for th in light dark; do
    bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5182/$page" --out "$SP/shots/rt4-final" --theme $th --viewports desktop,mobile
  done
done
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

Read all 12 PNGs. Expected: sidebar `Sag` group ends with `Skabeloner`, no `Eksport`/`Materialedata`/`On-site` anywhere; Skabeloner and Rapport as described in Tasks 5–6; Kortlægning still renders its template picker (it reads the same templates). No horizontal page scroll at mobile width; no light surfaces in dark theme.

- [ ] **Step 4: Report**

List for the controller: the Name map as filled, the Phase 1 PDF field and CSV fields actually used, whether `apps/ruxd` still serves `/export-templates` (R14), and any deviation from this plan with its reason.
