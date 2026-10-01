<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# GUI Phase 3 — Kortlægning Frontend Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Load the project skill `design-studio` (`.claude/skills/design-studio/SKILL.md`) before touching any `.tsx`/`.css`.

**Goal:** Build the prototype-v2 **Kortlægning** workbench at `/kortlaegning` on the Phase 2 survey API: tabbed, filterable two-level table of survey types and their bygningsdele, evidence panel, detail panel, full-screen edit dialog, keyboard flow, toasts, and the sidebar review-queue badge.

**Architecture:** All behaviour that can be pure lives in `src/kortlaegning/` (vocabulary/formatting, workbench model, key map) and is unit-tested in Node. Components in `src/components/kortlaegning/` are presentational and read those modules; the route `src/routes/KortlaegningPage.tsx` owns state (survey data, selection, tab, filters, open groups, dialog) and calls `api.*` from `src/api/client.ts`. Visual correctness is verified by screenshots against `docs/gui/images/prototype-v2/kortlaegning.png` and `dialog.png`, using a seeded demo project.

**Tech Stack:** React 19, TypeScript, CSS Modules, vitest (Node, no DOM), the Phase 2 client (`api.survey`, `api.surveySummary`, `api.patchSurveyType`, `api.patchSurveyPart`, `api.samples`, `api.syncSurvey`, `api.renderUrl`, `api.instanceFrames`, `api.frameImageUrl`, `api.csvExportUrl`), sqlite3 CLI for the dev seed.

**Spec:** `docs/design/gui-kortlaegning-redesign.md` (§ Kortlægning screen)

## Global Constraints

- SPDX header on every new file (`// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `// SPDX-License-Identifier: GPL-3.0-or-later`; CSS uses the `/* … */` block form; shell/SQL use `#` / `--`).
- No literal colour, radius, spacing or font-size in `.module.css`, inline `style`, or TS constants — only `var(--…)`. Check with `python .claude/skills/design-studio/scripts/token_lint.py <changed css> --tsx`.
- Text on navy chrome uses `--color-on-chrome*`; pills/notices use `--tone-*-bg/-ink`; treatment colours use `--circ-*`; filled primary buttons use `--color-accent-deep` + `--color-on-accent`; table headers `--color-text-muted`.
- All UI copy Danish, taken from the prototype where it has it.
- No DOM test environment may be added; pure modules are tested, components are verified by screenshot (`bash .claude/skills/design-studio/scripts/shot.sh`, both themes).
- Frontend commands: `npm --prefix apps/rux/frontend test|run typecheck|run build`.
- The API is the Phase 2 contract (`docs/gui/openapi.yaml`): approving a type whose miljøstatus is `afventer` returns **422**; the client exposes it as `ApiRequestError.isUnprocessable`.
- The prototype's "360°" evidence tab is **"Foto"** here (the instance's best sensor frames): Phase 2 has no nearest-panorama endpoint.

## Review Focus

- **Approving a type whose sample is pending, from every entry point** (G in the table, the detail button, Ctrl/⌘+Enter in the dialog): the 422 must surface as the Danish toast naming the sample, selection must not advance, and nothing may look approved.
- **Keyboard shortcuts must not fire while typing** in the search box, a quantity field, a select or the note textarea — and Esc in a field must only blur it.
- **An empty project** (no survey types yet) must show an empty state with the "Opret kortlægning fra instanser" action, and its 422 ("run `rux create instances` first") must be shown as a readable notice, not an unhandled error.
- **Edits racing reloads:** after any PATCH the table must show the server's returned type, not a stale copy (use the response body, never re-derive).
- **Evidence images that fail** (503 no renderer / no GL, 422 missing data) must show an explanatory empty state in the panel instead of a broken image icon.

---

## File Structure

| File | Responsibility |
|---|---|
| `apps/rux/frontend/dev/seed-survey-demo.sh` (new) | copy a project and seed the prototype's demo survey via sqlite3 |
| `src/kortlaegning/vocab.ts` (new) | Danish labels, tones, number formatting/parsing |
| `src/kortlaegning/model.ts` (new) | tabs, filters, rows, selection, queue navigation, replace helpers |
| `src/kortlaegning/keys.ts` (new) | key → action map for table and dialog |
| `src/test/kortlaegning.vocab.test.ts`, `kortlaegning.model.test.ts`, `kortlaegning.keys.test.ts` (new) | their contracts |
| `src/components/Pill.tsx` + css, `ConfidenceBar.tsx` + css, `Kbd.tsx` + css, `Toast.tsx` + css (new) | shared atoms |
| `src/components/kortlaegning/SurveyTable.tsx` + css (new) | tabs, tools, key bar, two-level table |
| `src/components/kortlaegning/EvidencePanel.tsx` + css (new) | Plan / Foto / Punktsky / Rum-model |
| `src/components/kortlaegning/DetailPanel.tsx` + css (new) | fields and actions for the selection |
| `src/components/kortlaegning/EditDialog.tsx` + css (new) | full editing dialog |
| `src/routes/KortlaegningPage.tsx` + css (new) | state, data flow, keyboard |
| `src/app/App.tsx`, `src/app/navigation.ts`, `src/app/AppShell.tsx`, `src/test/navigation.test.ts` | route, un-pend entry, review-queue badge |

---

### Task 1: Demo seed for development screenshots

**Files:** Create `apps/rux/frontend/dev/seed-survey-demo.sh`; modify `apps/rux/frontend/README.md` (§ Development).

**Interfaces:** Produces `bash apps/rux/frontend/dev/seed-survey-demo.sh <source.rux> <dest.rux>` → a copy whose survey matches the prototype's Måløv Byvej 229 data (11 types, 18 parts, 3 samples).

- [ ] **Step 1: Write the script**

```bash
#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Copy a .rux project and seed the prototype-v2 demo survey (Måløv Byvej 229)
# into the copy, for screenshots and manual testing of Kortlægning. Parts are
# not linked to instances (the demo has none), so evidence renders show the
# whole cloud without a highlight. Never run this against a real project.
#
# Usage: seed-survey-demo.sh <source.rux> <dest.rux>
set -euo pipefail
src="${1:?source .rux}"
dst="${2:?destination .rux}"
[[ -f "$src" ]] || { echo "no such project: $src" >&2; exit 1; }
cp "$src" "$dst"
rm -f "$dst-wal" "$dst-shm"
# Opening the copy read-write once migrates it to the latest schema. `create
# survey` opens read-write; it may fail (no instances) after migrating, which is
# fine — the seed below replaces whatever it wrote.
rux -p "$dst" create survey > /dev/null 2>&1 || true
ver=$(sqlite3 "$dst" 'SELECT MAX(version) FROM schema_version;')
[[ "$ver" -ge 22 ]] || { echo "schema v$ver < 22 — build a newer rux" >&2; exit 1; }
sqlite3 "$dst" <<'SQL'
BEGIN;
DELETE FROM sample_links; DELETE FROM samples; DELETE FROM survey_parts; DELETE FROM survey_types;
INSERT INTO survey_types (id,name,eak_code,bim7aa_code,unit,treatment,review_status,confidence,mass_t,note,starred,semantic_class) VALUES
 (1,'Fundamenter & terrændæk, beton','17.01.01','131 Fundamenter','m³','bevaring','approved',0.94,640,'Bevares in situ — genanvendes i nyt byggeri på grunden.',0,-2),
 (2,'Betonsøjler, bærende','17.01.01','221 Bærende konstr.','stk','genbrug','queue',0.88,58,'Præfab søjler i god stand — direkte genbrug ved dokumenteret bæreevne.',0,-2),
 (3,'Betondæk, etagedæk','17.01.01','231 Etagedæk','m²','genanvendelse','queue',0.91,380,'Nedknuses til vejfyld/ny beton. Største enkeltfraktion.',0,-2),
 (4,'Facadeelementer, sandwich','17.01.01','211 Ydervægge','m²','genanvendelse','approved',0.90,190,'Elementsamlinger tillader hel nedtagning — afsætning undersøges.',0,-2),
 (5,'Stålspær, tagkonstruktion','17.04.05','271 Tagkonstruktion','stk','genbrug','queue',0.93,14,'Ensartede spær, boltede samlinger — høj genbrugsværdi.',1,-2),
 (6,'Vinduespartier, aluminium','17.04.02','312 Udv. vinduer','stk','genbrug','queue',0.82,3.1,'Ved ren fuge: salg som brugte partier.',0,-2),
 (7,'Trapezplader, tag','17.04.05','272 Tagdækning','m²','genanvendelse','approved',0.89,6.8,'Skrot/omsmeltning via metalgenvinding.',0,-2),
 (8,'Gulvbelægning, linoleum','17.09.04','421 Gulvbelægning','m²','nyttiggoerelse','queue',0.84,1.6,'Behandling afhænger af limprøve (asbest).',0,-2),
 (9,'Indvendige døre, træ','17.02.01','322 Indv. døre','stk','genbrug','queue',0.74,0.9,'Blandet stand — ★ fra on-site: 8–10 stk. skønnes direkte genbrugelige.',1,-2),
 (10,'Isolering, mineraluld','17.06.04','251 Isolering','m²','bortskaffelse','approved',0.87,2.4,'Deponi medmindre retur-ordning kan afsætte.',0,-2),
 (11,'Indvendige murvægge, malet','17.01.02','222 Indervægge','m²','bortskaffelse','queue',0.87,38,'Bly i maling påvist (P-02) — afrenses eller håndteres som forurenet.',0,-2);
INSERT INTO survey_parts (code,type_id,instance_guid,room_id,room_name,quantity) VALUES
 ('RX-010',1,NULL,1,'Production Hall',290),('RX-011',1,NULL,2,'Office Zone',30),
 ('RX-001',2,NULL,1,'Production Hall',18),('RX-002',2,NULL,4,'Entrance',6),
 ('RX-003',3,NULL,1,'Production Hall',980),('RX-004',3,NULL,2,'Office Zone',260),
 ('RX-005',4,NULL,6,'Facade',340),('RX-006',4,NULL,6,'Facade',280),
 ('RX-007',5,NULL,5,'Roof',22),
 ('RX-008',6,NULL,2,'Office Zone',26),('RX-009',6,NULL,1,'Production Hall',12),
 ('RX-012',7,NULL,5,'Roof',780),
 ('RX-013',8,NULL,2,'Office Zone',310),
 ('RX-014',9,NULL,2,'Office Zone',14),('RX-015',9,NULL,1,'Production Hall',10),
 ('RX-016',10,NULL,5,'Roof',480),
 ('RX-017',11,NULL,1,'Production Hall',170),('RX-018',11,NULL,3,'Technical Room',70);
INSERT INTO samples (id,code,title,what,stage,result) VALUES
 (1,'P-01','PCB i fugemasse','Fugemasse omkring vinduespartier','sendt',''),
 (2,'P-02','Bly i maling','Malede indervægge, Production Hall + Technical Room','svar','forurenet'),
 (3,'P-03','Asbest i linoleumslim','Gulvlim under linoleum, Office Zone','udtaget','');
INSERT INTO sample_links (sample_id,type_id) VALUES (1,6),(2,11),(3,8);
COMMIT;
SQL
echo "seeded demo survey into $dst"
```

`-2` is `core::kManualSemanticClass` (Phase 2 final fix): seeded types are hand-made, so `sync_survey` never files instances into them. If the Phase 2 column name for the instance link differs from `instance_guid`, read it from `libs/reusex/src/core/ProjectDB.cpp` `migrateToV22` and adapt the INSERT.

- [ ] **Step 2: Run it and check the API**

```bash
chmod +x apps/rux/frontend/dev/seed-survey-demo.sh
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/kort-demo.rux"
./build/apps/rux/rux -p "$SP/kort-demo.rux" gui --port 8440 --no-browser & S=$!
for i in $(seq 1 30); do curl -sf -o /dev/null localhost:8440/api/v1/health && break; sleep 1; done
curl -s localhost:8440/api/v1/survey | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d["counts"], len(d["types"]), sum(len(t["parts"]) for t in d["types"]))'
kill $S
```

Expected: `{'queue': 7, 'approved': 4, 'rejected': 0, 'all': 11} 11 18`. (`corridor-clouds.rux` is the scratch copy with a `cloud`; any project with a cloud works.)

- [ ] **Step 3: README** — under § Development add: "For Kortlægning work, seed the prototype's demo survey into a scratch copy: `bash dev/seed-survey-demo.sh <project.rux> /tmp/kort-demo.rux`, then `rux -p /tmp/kort-demo.rux gui --port 8420 --no-browser`. Never run it on a real project."

- [ ] **Step 4: Commit** — `git add apps/rux/frontend/dev/seed-survey-demo.sh apps/rux/frontend/README.md && git commit -m "chore(gui): demo survey seed for Kortlægning development"` (REUSE: the script carries its SPDX header).

---

### Task 2: Danish vocabulary and number formatting

**Files:** Create `src/kortlaegning/vocab.ts`, `src/test/kortlaegning.vocab.test.ts`.

**Interfaces:**

```ts
export type Tone = 'good' | 'warn' | 'wait' | 'crit' | 'accent';
export const TREATMENT_LABEL: Record<Treatment, string>;
export const ENV_LABEL: Record<EnvironmentStatus, string>;
export const ENV_TONE: Record<EnvironmentStatus, Tone>;
export const STAGE_LABEL: Record<SampleStage, string>;
export function circToken(t: Treatment): string;          // 'var(--circ-genbrug)'
export function formatNumber(n: number, maxFractionDigits?: number): string; // da-DK
export function formatQuantity(q: number, unit: string): string;
export function formatTonnes(t: number | null): string;   // '' when null
export function confidencePercent(c: number | null): number | null; // 0.82 → 82
export function parseDanishNumber(text: string): number | null;
```

- [ ] **Step 1: Failing test** — `src/test/kortlaegning.vocab.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { TREATMENTS } from '../api/types';
import {
  circToken,
  confidencePercent,
  ENV_LABEL,
  ENV_TONE,
  formatNumber,
  formatQuantity,
  formatTonnes,
  parseDanishNumber,
  STAGE_LABEL,
  TREATMENT_LABEL,
} from '../kortlaegning/vocab';

describe('kortlægning vocabulary', () => {
  it('labels every treatment in Danish and maps it to its circ token', () => {
    expect(TREATMENTS.map((t) => TREATMENT_LABEL[t])).toEqual([
      'Bevaring',
      'Genbrug',
      'Genanvendelse',
      'Nyttiggørelse',
      'Bortskaffelse',
    ]);
    expect(circToken('nyttiggoerelse')).toBe('var(--circ-nyttiggoerelse)');
  });

  it('labels and tones the environment statuses', () => {
    expect(ENV_LABEL.afventer).toBe('Afventer prøve');
    expect(ENV_LABEL.ren_proevesvar).toBe('Ren (prøvesvar)');
    expect(ENV_TONE).toEqual({
      ren_screening: 'good',
      afventer: 'wait',
      forurenet: 'crit',
      ren_proevesvar: 'good',
    });
    expect(STAGE_LABEL.sendt).toBe('Sendt til lab');
  });

  it('formats numbers the Danish way', () => {
    expect(formatNumber(1240)).toBe('1.240');
    expect(formatNumber(3.1)).toBe('3,1');
    expect(formatNumber(2.25, 2)).toBe('2,25');
    expect(formatQuantity(1240, 'm²')).toBe('1.240 m²');
    expect(formatTonnes(58)).toBe('58 t');
    expect(formatTonnes(null)).toBe('');
  });

  it('turns confidence into a whole percentage', () => {
    expect(confidencePercent(0.82)).toBe(82);
    expect(confidencePercent(null)).toBeNull();
  });

  it('parses what a Danish user types', () => {
    expect(parseDanishNumber('1.240')).toBe(1240);
    expect(parseDanishNumber('1.240,5')).toBe(1240.5);
    expect(parseDanishNumber('22,5')).toBe(22.5);
    expect(parseDanishNumber(' 18 ')).toBe(18);
    expect(parseDanishNumber('')).toBeNull();
    expect(parseDanishNumber('abc')).toBeNull();
    expect(parseDanishNumber('-3')).toBeNull(); // quantities are never negative
  });
});
```

- [ ] **Step 2: Run → FAIL** (`npm --prefix apps/rux/frontend test -- kortlaegning.vocab`).

- [ ] **Step 3: Implement `src/kortlaegning/vocab.ts`**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The Danish words and number formats Kortlægning shows. The wire vocabulary
 * (core/survey.hpp, docs/gui/openapi.yaml) stays ASCII; this is the only place
 * it becomes user-facing copy.
 */

import type { EnvironmentStatus, SampleStage, Treatment } from '../api/types';

export type Tone = 'good' | 'warn' | 'wait' | 'crit' | 'accent';

export const TREATMENT_LABEL: Record<Treatment, string> = {
  bevaring: 'Bevaring',
  genbrug: 'Genbrug',
  genanvendelse: 'Genanvendelse',
  nyttiggoerelse: 'Nyttiggørelse',
  bortskaffelse: 'Bortskaffelse',
};

export const ENV_LABEL: Record<EnvironmentStatus, string> = {
  ren_screening: 'Ren',
  afventer: 'Afventer prøve',
  forurenet: 'Forurenet',
  ren_proevesvar: 'Ren (prøvesvar)',
};

export const ENV_TONE: Record<EnvironmentStatus, Tone> = {
  ren_screening: 'good',
  afventer: 'wait',
  forurenet: 'crit',
  ren_proevesvar: 'good',
};

export const STAGE_LABEL: Record<SampleStage, string> = {
  planlagt: 'Planlagt',
  udtaget: 'Udtaget',
  sendt: 'Sendt til lab',
  svar: 'Svar modtaget',
};

/** The waste-hierarchy colour token for a treatment. */
export function circToken(t: Treatment): string {
  return `var(--circ-${t})`;
}

export function formatNumber(n: number, maxFractionDigits = 1): string {
  return n.toLocaleString('da-DK', { maximumFractionDigits: maxFractionDigits });
}

export function formatQuantity(q: number, unit: string): string {
  return `${formatNumber(q)} ${unit}`;
}

export function formatTonnes(t: number | null): string {
  return t === null ? '' : `${formatNumber(t)} t`;
}

export function confidencePercent(c: number | null): number | null {
  return c === null ? null : Math.round(c * 100);
}

/**
 * Parse a quantity as a Danish user types it: '.' groups thousands, ',' is the
 * decimal point. Returns null for empty, malformed or negative input.
 */
export function parseDanishNumber(text: string): number | null {
  const t = text.trim().replace(/\./g, '').replace(',', '.');
  if (t === '' || !/^\d+(\.\d+)?$/.test(t)) return null;
  return Number(t);
}
```

(If Node's ICU formats `1240` without the `.` separator, the test pins the behaviour the browser shows; fix `formatNumber` with an explicit `useGrouping: true` rather than editing the test.)

- [ ] **Step 4: Run → PASS; commit** — `git add apps/rux/frontend/src/kortlaegning/vocab.ts apps/rux/frontend/src/test/kortlaegning.vocab.test.ts && git commit -m "feat(gui): Danish vocabulary and number formatting for Kortlægning"`.

---

### Task 3: Workbench model

**Files:** Create `src/kortlaegning/model.ts`, `src/test/kortlaegning.model.test.ts`.

**Interfaces:**

```ts
export type Tab = 'queue' | 'approved' | 'all';
export type EnvFilter = 'ren' | 'afventer' | 'forurenet';
export interface Filters { search: string; roomId: number | null; env: EnvFilter | null }
export const NO_FILTERS: Filters;
export type Selection = { typeId: number; partCode: string | null } | null;
export type Row = { kind: 'type'; typeId: number } | { kind: 'part'; typeId: number; partCode: string };
export function envFilterOf(s: EnvironmentStatus): EnvFilter;
export function tabCounts(types: SurveyType[]): Record<Tab, number>;
export function visibleTypes(types: SurveyType[], tab: Tab, f: Filters): SurveyType[];
export function flattenRows(types: SurveyType[], open: ReadonlySet<number>): Row[];
export function sameSelection(row: Row, sel: Selection): boolean;
export function moveSelection(rows: Row[], sel: Selection, delta: number): Selection;
export function roomOptions(types: SurveyType[]): { id: number; name: string }[];
export function nextInQueue(types: SurveyType[], afterTypeId: number): Selection;
export function typeOf(types: SurveyType[], sel: Selection): SurveyType | null;
export function partOf(types: SurveyType[], sel: Selection): SurveyPart | null;
export function replaceType(types: SurveyType[], updated: SurveyType): SurveyType[];
export function replacePart(types: SurveyType[], updated: SurveyPart): SurveyType[];
```

Rules: rejected types never appear; tab `queue` = review_status queue, `approved` = approved, `all` = both; search is a case-insensitive substring on the type name; room filter keeps types with any part in that room; env filter `ren` covers `ren_screening` and `ren_proevesvar`. Rows: one `type` row per visible type, followed by its `part` rows (by code) when the type id is in `open`. `moveSelection` from null selects the first row; it clamps at the ends. `nextInQueue` picks the next queued, non-`afventer` type after `afterTypeId` in list order (wrapping), else any queued type, else null. `replacePart` updates the part in its type and recomputes that type's `quantity` as the parts' sum.

- [ ] **Step 1: Failing test** — `src/test/kortlaegning.model.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { SurveyPart, SurveyType } from '../api/types';
import {
  flattenRows,
  moveSelection,
  nextInQueue,
  NO_FILTERS,
  partOf,
  replacePart,
  replaceType,
  roomOptions,
  tabCounts,
  typeOf,
  visibleTypes,
} from '../kortlaegning/model';

function part(code: string, typeId: number, room: [number, string] | null, quantity: number): SurveyPart {
  return {
    code,
    type_id: typeId,
    cloud: null,
    instance_id: null,
    room_id: room ? room[0] : null,
    room_name: room ? room[1] : '',
    quantity,
    starred: false,
    note: '',
    material_guid: null,
    instance_guid: null,
    orphaned: false,
  };
}

function type(id: number, name: string, over: Partial<SurveyType> = {}): SurveyType {
  return {
    id,
    name,
    eak_code: '17.01.01',
    eak_name: 'Beton',
    bim7aa_code: '',
    unit: 'stk',
    treatment: 'genbrug',
    review_status: 'queue',
    confidence: 0.9,
    mass_t: 1,
    note: '',
    starred: false,
    semantic_class: -2,
    environment_status: 'ren_screening',
    sample_ids: [],
    quantity: 0,
    parts: [],
    created_at: '',
    updated_at: '',
    ...over,
  };
}

const TYPES: SurveyType[] = [
  type(1, 'Betonsøjler, bærende', { parts: [part('RX-001', 1, [1, 'Production Hall'], 18), part('RX-002', 1, [4, 'Entrance'], 6)], quantity: 24 }),
  type(2, 'Vinduespartier, aluminium', { environment_status: 'afventer', parts: [part('RX-008', 2, [2, 'Office Zone'], 26)], quantity: 26 }),
  type(3, 'Isolering, mineraluld', { review_status: 'approved', parts: [part('RX-016', 3, [5, 'Roof'], 480)], quantity: 480 }),
  type(4, 'Fejldetektion', { review_status: 'rejected' }),
  type(5, 'Indvendige murvægge', { environment_status: 'forurenet', parts: [part('RX-017', 5, [1, 'Production Hall'], 170)], quantity: 170 }),
];

describe('kortlægning model', () => {
  it('counts tabs without rejected types', () => {
    expect(tabCounts(TYPES)).toEqual({ queue: 3, approved: 1, all: 4 });
  });

  it('filters by tab, search, room and miljø', () => {
    expect(visibleTypes(TYPES, 'queue', NO_FILTERS).map((t) => t.id)).toEqual([1, 2, 5]);
    expect(visibleTypes(TYPES, 'all', NO_FILTERS).map((t) => t.id)).toEqual([1, 2, 3, 5]);
    expect(visibleTypes(TYPES, 'all', { ...NO_FILTERS, search: 'VINDUE' }).map((t) => t.id)).toEqual([2]);
    expect(visibleTypes(TYPES, 'all', { ...NO_FILTERS, roomId: 1 }).map((t) => t.id)).toEqual([1, 5]);
    expect(visibleTypes(TYPES, 'all', { ...NO_FILTERS, env: 'ren' }).map((t) => t.id)).toEqual([1, 3]);
    expect(visibleTypes(TYPES, 'all', { ...NO_FILTERS, env: 'afventer' }).map((t) => t.id)).toEqual([2]);
  });

  it('flattens open groups into type + part rows', () => {
    const rows = flattenRows(visibleTypes(TYPES, 'queue', NO_FILTERS), new Set([1]));
    expect(rows).toEqual([
      { kind: 'type', typeId: 1 },
      { kind: 'part', typeId: 1, partCode: 'RX-001' },
      { kind: 'part', typeId: 1, partCode: 'RX-002' },
      { kind: 'type', typeId: 2 },
      { kind: 'type', typeId: 5 },
    ]);
  });

  it('moves the selection through rows and clamps at the ends', () => {
    const rows = flattenRows(visibleTypes(TYPES, 'queue', NO_FILTERS), new Set([1]));
    expect(moveSelection(rows, null, 1)).toEqual({ typeId: 1, partCode: null });
    expect(moveSelection(rows, { typeId: 1, partCode: null }, 1)).toEqual({ typeId: 1, partCode: 'RX-001' });
    expect(moveSelection(rows, { typeId: 5, partCode: null }, 1)).toEqual({ typeId: 5, partCode: null });
    expect(moveSelection(rows, { typeId: 1, partCode: null }, -1)).toEqual({ typeId: 1, partCode: null });
    expect(moveSelection([], null, 1)).toBeNull();
  });

  it('lists rooms that have parts, by name', () => {
    expect(roomOptions(TYPES)).toEqual([
      { id: 4, name: 'Entrance' },
      { id: 2, name: 'Office Zone' },
      { id: 1, name: 'Production Hall' },
      { id: 5, name: 'Roof' },
    ]);
  });

  it('picks the next approvable queued type, then any queued, then none', () => {
    expect(nextInQueue(TYPES, 1)).toEqual({ typeId: 5, partCode: null }); // 2 is afventer
    expect(nextInQueue(TYPES, 5)).toEqual({ typeId: 1, partCode: null }); // wraps
    const onlyPending = [type(2, 'x', { environment_status: 'afventer' })];
    expect(nextInQueue(onlyPending, 2)).toEqual({ typeId: 2, partCode: null });
    expect(nextInQueue([type(3, 'y', { review_status: 'approved' })], 3)).toBeNull();
  });

  it('resolves the selected type and part', () => {
    expect(typeOf(TYPES, { typeId: 2, partCode: null })?.name).toBe('Vinduespartier, aluminium');
    expect(partOf(TYPES, { typeId: 1, partCode: 'RX-002' })?.quantity).toBe(6);
    expect(partOf(TYPES, { typeId: 1, partCode: null })).toBeNull();
  });

  it('replaces a type and a part from server responses', () => {
    const t = replaceType(TYPES, { ...TYPES[0], name: 'Søjler' });
    expect(t[0].name).toBe('Søjler');
    expect(t).not.toBe(TYPES);
    const p = replacePart(TYPES, { ...TYPES[0].parts[1], quantity: 10 });
    expect(p[0].parts[1].quantity).toBe(10);
    expect(p[0].quantity).toBe(28);
  });
});
```

(`orphaned` and `instance_guid` exist on `SurveyPart` after the Phase 2 final fix; if the TS interface field names differ, follow `src/api/types.ts`.)

- [ ] **Step 2: Run → FAIL.**

- [ ] **Step 3: Implement `src/kortlaegning/model.ts`**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The Kortlægning workbench as data: which types a tab and filters show, the
 * rows the table draws, how the selection moves, and how server responses are
 * folded back in. Pure, so the review flow is testable without a DOM.
 */

import type { EnvironmentStatus, SurveyPart, SurveyType } from '../api/types';

export type Tab = 'queue' | 'approved' | 'all';
export type EnvFilter = 'ren' | 'afventer' | 'forurenet';
export interface Filters {
  search: string;
  roomId: number | null;
  env: EnvFilter | null;
}
export const NO_FILTERS: Filters = { search: '', roomId: null, env: null };

export type Selection = { typeId: number; partCode: string | null } | null;
export type Row =
  | { kind: 'type'; typeId: number }
  | { kind: 'part'; typeId: number; partCode: string };

export function envFilterOf(s: EnvironmentStatus): EnvFilter {
  if (s === 'afventer') return 'afventer';
  if (s === 'forurenet') return 'forurenet';
  return 'ren';
}

export function tabCounts(types: SurveyType[]): Record<Tab, number> {
  const queue = types.filter((t) => t.review_status === 'queue').length;
  const approved = types.filter((t) => t.review_status === 'approved').length;
  return { queue, approved, all: queue + approved };
}

function inTab(t: SurveyType, tab: Tab): boolean {
  if (t.review_status === 'rejected') return false;
  if (tab === 'queue') return t.review_status === 'queue';
  if (tab === 'approved') return t.review_status === 'approved';
  return true;
}

export function visibleTypes(types: SurveyType[], tab: Tab, f: Filters): SurveyType[] {
  const q = f.search.trim().toLowerCase();
  return types.filter(
    (t) =>
      inTab(t, tab) &&
      (q === '' || t.name.toLowerCase().includes(q)) &&
      (f.roomId === null || t.parts.some((p) => p.room_id === f.roomId)) &&
      (f.env === null || envFilterOf(t.environment_status) === f.env),
  );
}

export function flattenRows(types: SurveyType[], open: ReadonlySet<number>): Row[] {
  const rows: Row[] = [];
  for (const t of types) {
    rows.push({ kind: 'type', typeId: t.id });
    if (open.has(t.id)) {
      for (const p of [...t.parts].sort((a, b) => a.code.localeCompare(b.code))) {
        rows.push({ kind: 'part', typeId: t.id, partCode: p.code });
      }
    }
  }
  return rows;
}

export function sameSelection(row: Row, sel: Selection): boolean {
  if (!sel || row.typeId !== sel.typeId) return false;
  return row.kind === 'type' ? sel.partCode === null : row.partCode === sel.partCode;
}

function toSelection(row: Row): Selection {
  return { typeId: row.typeId, partCode: row.kind === 'part' ? row.partCode : null };
}

export function moveSelection(rows: Row[], sel: Selection, delta: number): Selection {
  if (rows.length === 0) return null;
  const at = rows.findIndex((r) => sameSelection(r, sel));
  if (at < 0) return toSelection(rows[0]);
  const next = Math.min(rows.length - 1, Math.max(0, at + delta));
  return toSelection(rows[next]);
}

export function roomOptions(types: SurveyType[]): { id: number; name: string }[] {
  const byId = new Map<number, string>();
  for (const t of types)
    for (const p of t.parts)
      if (p.room_id !== null && !byId.has(p.room_id)) byId.set(p.room_id, p.room_name || `Rum ${p.room_id}`);
  return [...byId].map(([id, name]) => ({ id, name })).sort((a, b) => a.name.localeCompare(b.name, 'da'));
}

export function nextInQueue(types: SurveyType[], afterTypeId: number): Selection {
  const queued = types.filter((t) => t.review_status === 'queue');
  if (queued.length === 0) return null;
  const start = types.findIndex((t) => t.id === afterTypeId);
  const ordered = [...types.slice(start + 1), ...types.slice(0, start + 1)].filter(
    (t) => t.review_status === 'queue',
  );
  const pick = ordered.find((t) => t.environment_status !== 'afventer') ?? ordered[0];
  return { typeId: pick.id, partCode: null };
}

export function typeOf(types: SurveyType[], sel: Selection): SurveyType | null {
  return sel ? (types.find((t) => t.id === sel.typeId) ?? null) : null;
}

export function partOf(types: SurveyType[], sel: Selection): SurveyPart | null {
  if (!sel || sel.partCode === null) return null;
  return typeOf(types, sel)?.parts.find((p) => p.code === sel.partCode) ?? null;
}

export function replaceType(types: SurveyType[], updated: SurveyType): SurveyType[] {
  return types.map((t) => (t.id === updated.id ? updated : t));
}

export function replacePart(types: SurveyType[], updated: SurveyPart): SurveyType[] {
  return types.map((t) => {
    // A part re-filed to another type leaves this one and joins that one.
    const parts = t.parts.filter((p) => p.code !== updated.code);
    if (t.id === updated.type_id) parts.push(updated);
    if (parts.length === t.parts.length && t.id !== updated.type_id) return t;
    parts.sort((a, b) => a.code.localeCompare(b.code));
    return { ...t, parts, quantity: parts.reduce((s, p) => s + p.quantity, 0) };
  });
}
```

- [ ] **Step 4: Run → PASS; commit** — `git add apps/rux/frontend/src/kortlaegning/model.ts apps/rux/frontend/src/test/kortlaegning.model.test.ts && git commit -m "feat(gui): Kortlægning workbench model"`.

---

### Task 4: Key map

**Files:** Create `src/kortlaegning/keys.ts`, `src/test/kortlaegning.keys.test.ts`.

**Interfaces:**

```ts
export type EvidenceTab = 'plan' | 'foto' | 'punktsky' | 'rum';
export const EVIDENCE_TABS: readonly EvidenceTab[]; // order of the 1–4 keys
export type KortAction =
  | { type: 'move'; delta: number } | { type: 'expand' } | { type: 'collapse' } | { type: 'open' }
  | { type: 'approve' } | { type: 'approveNext' } | { type: 'reject' } | { type: 'star' }
  | { type: 'evidence'; tab: EvidenceTab } | { type: 'blur' } | { type: 'close' };
export interface KeyInput { key: string; metaKey?: boolean; ctrlKey?: boolean; altKey?: boolean; inField: boolean }
export function tableAction(k: KeyInput): KortAction | null;
export function dialogAction(k: KeyInput): KortAction | null;
```

Table: in a field only `Escape` → blur; modifiers → null; ↓/j +1, ↑/k −1, → expand, ← collapse, Enter open, g/G approve, a/A reject, v/V star, 1–4 evidence. Dialog: Escape → close (also in a field); Ctrl/⌘+Enter → approveNext (also in a field); PageDown +1, PageUp −1 (also in a field); otherwise in a field null; g approve, a reject, v star, 1–4 evidence.

- [ ] **Step 1: Failing test** — `src/test/kortlaegning.keys.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { dialogAction, EVIDENCE_TABS, tableAction } from '../kortlaegning/keys';

const k = (key: string, extra: Partial<{ metaKey: boolean; ctrlKey: boolean; altKey: boolean; inField: boolean }> = {}) => ({
  key,
  inField: false,
  ...extra,
});

describe('table keys', () => {
  it('navigates and acts', () => {
    expect(tableAction(k('ArrowDown'))).toEqual({ type: 'move', delta: 1 });
    expect(tableAction(k('k'))).toEqual({ type: 'move', delta: -1 });
    expect(tableAction(k('ArrowRight'))).toEqual({ type: 'expand' });
    expect(tableAction(k('ArrowLeft'))).toEqual({ type: 'collapse' });
    expect(tableAction(k('Enter'))).toEqual({ type: 'open' });
    expect(tableAction(k('G'))).toEqual({ type: 'approve' });
    expect(tableAction(k('a'))).toEqual({ type: 'reject' });
    expect(tableAction(k('v'))).toEqual({ type: 'star' });
    expect(tableAction(k('3'))).toEqual({ type: 'evidence', tab: 'punktsky' });
  });

  it('stays out of the way while typing, except Escape', () => {
    expect(tableAction(k('g', { inField: true }))).toBeNull();
    expect(tableAction(k('ArrowDown', { inField: true }))).toBeNull();
    expect(tableAction(k('Escape', { inField: true }))).toEqual({ type: 'blur' });
  });

  it('ignores modified keys so browser shortcuts keep working', () => {
    expect(tableAction(k('a', { ctrlKey: true }))).toBeNull();
    expect(tableAction(k('1', { metaKey: true }))).toBeNull();
  });
});

describe('dialog keys', () => {
  it('closes, approves-and-advances and pages even from a field', () => {
    expect(dialogAction(k('Escape', { inField: true }))).toEqual({ type: 'close' });
    expect(dialogAction(k('Enter', { ctrlKey: true, inField: true }))).toEqual({ type: 'approveNext' });
    expect(dialogAction(k('Enter', { metaKey: true }))).toEqual({ type: 'approveNext' });
    expect(dialogAction(k('PageDown', { inField: true }))).toEqual({ type: 'move', delta: 1 });
    expect(dialogAction(k('PageUp'))).toEqual({ type: 'move', delta: -1 });
  });

  it('uses letter shortcuts only outside fields', () => {
    expect(dialogAction(k('g'))).toEqual({ type: 'approve' });
    expect(dialogAction(k('g', { inField: true }))).toBeNull();
    expect(dialogAction(k('2'))).toEqual({ type: 'evidence', tab: 'foto' });
    expect(dialogAction(k('Enter'))).toBeNull();
  });

  it('orders the evidence tabs as the 1–4 keys', () => {
    expect([...EVIDENCE_TABS]).toEqual(['plan', 'foto', 'punktsky', 'rum']);
  });
});
```

- [ ] **Step 2: Run → FAIL.**

- [ ] **Step 3: Implement `src/kortlaegning/keys.ts`**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Kortlægning's keyboard flow as a pure key → action map, one for the table and
 * one for the edit dialog. Kept out of the components so "never fire while the
 * user is typing" is a tested rule, not an onKeyDown detail.
 */

export type EvidenceTab = 'plan' | 'foto' | 'punktsky' | 'rum';
export const EVIDENCE_TABS: readonly EvidenceTab[] = ['plan', 'foto', 'punktsky', 'rum'];

export type KortAction =
  | { type: 'move'; delta: number }
  | { type: 'expand' }
  | { type: 'collapse' }
  | { type: 'open' }
  | { type: 'approve' }
  | { type: 'approveNext' }
  | { type: 'reject' }
  | { type: 'star' }
  | { type: 'evidence'; tab: EvidenceTab }
  | { type: 'blur' }
  | { type: 'close' };

export interface KeyInput {
  key: string;
  metaKey?: boolean;
  ctrlKey?: boolean;
  altKey?: boolean;
  /** True when focus is in an input, select or textarea. */
  inField: boolean;
}

function letterAction(key: string): KortAction | null {
  switch (key.toLowerCase()) {
    case 'g':
      return { type: 'approve' };
    case 'a':
      return { type: 'reject' };
    case 'v':
      return { type: 'star' };
    default:
      break;
  }
  const n = Number(key);
  if (Number.isInteger(n) && n >= 1 && n <= EVIDENCE_TABS.length) {
    return { type: 'evidence', tab: EVIDENCE_TABS[n - 1] };
  }
  return null;
}

export function tableAction(k: KeyInput): KortAction | null {
  if (k.key === 'Escape') return k.inField ? { type: 'blur' } : null;
  if (k.inField || k.metaKey || k.ctrlKey || k.altKey) return null;
  switch (k.key) {
    case 'ArrowDown':
    case 'j':
      return { type: 'move', delta: 1 };
    case 'ArrowUp':
    case 'k':
      return { type: 'move', delta: -1 };
    case 'ArrowRight':
      return { type: 'expand' };
    case 'ArrowLeft':
      return { type: 'collapse' };
    case 'Enter':
      return { type: 'open' };
    default:
      return letterAction(k.key);
  }
}

export function dialogAction(k: KeyInput): KortAction | null {
  if (k.key === 'Escape') return { type: 'close' };
  if (k.key === 'Enter' && (k.metaKey || k.ctrlKey)) return { type: 'approveNext' };
  if (k.key === 'PageDown') return { type: 'move', delta: 1 };
  if (k.key === 'PageUp') return { type: 'move', delta: -1 };
  if (k.inField || k.metaKey || k.ctrlKey || k.altKey) return null;
  return letterAction(k.key);
}
```

- [ ] **Step 4: Run → PASS; commit** — `git add apps/rux/frontend/src/kortlaegning/keys.ts apps/rux/frontend/src/test/kortlaegning.keys.test.ts && git commit -m "feat(gui): Kortlægning keyboard map"`.

---

### Task 5: Shared atoms — Pill, ConfidenceBar, Kbd, Toast

**Files:** Create `src/components/Pill.tsx` + `Pill.module.css`, `ConfidenceBar.tsx` + `.module.css`, `Kbd.tsx` + `.module.css`, `Toast.tsx` + `.module.css`, and `src/app/useToast.ts`.

**Interfaces:**

```tsx
export function Pill(props: { tone?: Tone; treatment?: Treatment; children: ReactNode; title?: string }): JSX.Element;
export function ConfidenceBar(props: { percent: number | null }): JSX.Element;   // "—" when null
export function Kbd(props: { children: ReactNode }): JSX.Element;
export function Toast(props: { message: string | null }): JSX.Element;          // role=status, aria-live=polite
export function useToast(ms?: number): { message: string | null; show: (m: string) => void };
```

- [ ] **Step 1: Implement the atoms**

`Pill.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ReactNode } from 'react';

import type { Treatment } from '../api/types';
import type { Tone } from '../kortlaegning/vocab';
import styles from './Pill.module.css';

export interface PillProps {
  /** A semantic tone (good/warn/wait/crit/accent). */
  tone?: Tone;
  /** Or a waste-hierarchy step, drawn in its --circ-* colour. */
  treatment?: Treatment;
  title?: string;
  children: ReactNode;
}

/** A small status label: miljøstatus, behandling, BIM7AA, "Godkendt ✓". */
export function Pill({ tone = 'wait', treatment, title, children }: PillProps) {
  const cls = treatment ? `${styles.pill} ${styles[`circ_${treatment}`]}` : `${styles.pill} ${styles[tone]}`;
  return (
    <span className={cls} title={title}>
      {children}
    </span>
  );
}
```

`Pill.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.pill {
  display: inline-flex;
  align-items: center;
  gap: var(--space-1);
  padding: 0 var(--space-2);
  border-radius: var(--radius-sm);
  font-size: var(--font-size-xs);
  font-weight: var(--font-weight-bold);
  line-height: var(--line-height-normal);
  white-space: nowrap;
}

.good { background: var(--tone-good-bg); color: var(--tone-good-ink); }
.warn { background: var(--tone-warn-bg); color: var(--tone-warn-ink); }
.wait { background: var(--tone-wait-bg); color: var(--tone-wait-ink); }
.crit { background: var(--tone-crit-bg); color: var(--tone-crit-ink); }
.accent { background: var(--tone-accent-bg); color: var(--tone-accent-ink); }

/* Waste hierarchy: the step colour as ink on a light wash of itself. */
.circ_bevaring,
.circ_genbrug,
.circ_genanvendelse,
.circ_nyttiggoerelse,
.circ_bortskaffelse {
  background: color-mix(in srgb, var(--circ) 16%, var(--color-surface-raised));
  color: color-mix(in srgb, var(--circ) 80%, var(--color-text));
}
.circ_bevaring { --circ: var(--circ-bevaring); }
.circ_genbrug { --circ: var(--circ-genbrug); }
.circ_genanvendelse { --circ: var(--circ-genanvendelse); }
.circ_nyttiggoerelse { --circ: var(--circ-nyttiggoerelse); }
.circ_bortskaffelse { --circ: var(--circ-bortskaffelse); }
```

`ConfidenceBar.tsx` — renders `<span className={styles.conf}><span className={styles.bar}><i style={{ width: `${percent}%` }} /></span>{percent} %</span>`, or `<span className={styles.none}>—</span>` when null. (A percentage width is a data value, not a design value — the linter does not flag it.) CSS: `.conf { display:inline-flex; align-items:center; gap:var(--space-1); font-size:var(--font-size-xs); color:var(--color-text-muted); font-variant-numeric: tabular-nums; } .bar { width: calc(var(--space-6) + var(--space-1)); height: var(--space-1); border-radius: var(--radius-pill); background: var(--color-surface-sunken); overflow: hidden; } .bar i { display:block; height:100%; background: var(--color-accent); } .none { color: var(--color-text-faint); }`.

`Kbd.tsx` — `<kbd className={styles.kbd}>{children}</kbd>`; CSS: `.kbd { font: var(--font-weight-bold) var(--font-size-2xs) var(--font-sans); background: var(--color-surface-raised); border: 1px solid var(--color-border); border-bottom-width: 2px; border-radius: var(--radius-sm); padding: 0 var(--space-1); color: var(--color-text); }`.

`useToast.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useRef, useState } from 'react';

/** One transient message at a time; a new one replaces the old and restarts the timer. */
export function useToast(ms = 1800): { message: string | null; show: (m: string) => void } {
  const [message, setMessage] = useState<string | null>(null);
  const timer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);
  const show = useCallback(
    (m: string) => {
      setMessage(m);
      clearTimeout(timer.current);
      timer.current = setTimeout(() => setMessage(null), ms);
    },
    [ms],
  );
  useEffect(() => () => clearTimeout(timer.current), []);
  return { message, show };
}
```

`Toast.tsx` — `<div className={`${styles.toast} ${message ? styles.show : ''}`} role="status" aria-live="polite">{message}</div>`; CSS: fixed bottom-centre (`position: fixed; left: 50%; bottom: var(--space-5); transform: translateX(-50%);`), `background: var(--color-chrome); color: var(--color-on-chrome); padding: var(--space-2) var(--space-4); border-radius: var(--radius-lg); font-weight: var(--font-weight-bold); font-size: var(--font-size-sm); box-shadow: var(--shadow-md); opacity: 0; pointer-events: none; transition: opacity var(--duration-fast) var(--easing-standard); z-index: var(--z-toast); max-width: 90vw;` and `.show { opacity: 1; }`.

- [ ] **Step 2: Lint, typecheck, commit**

Run: `python .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/{Pill,ConfidenceBar,Kbd,Toast}.module.css --tsx` → OK (the `1px`/`2px` borders are hairlines the linter allows); `npm --prefix apps/rux/frontend run typecheck` → PASS.

```bash
git add apps/rux/frontend/src/components/{Pill,ConfidenceBar,Kbd,Toast}.tsx apps/rux/frontend/src/components/{Pill,ConfidenceBar,Kbd,Toast}.module.css apps/rux/frontend/src/app/useToast.ts
git commit -m "feat(gui): Pill, ConfidenceBar, Kbd and Toast atoms"
```

---

### Task 6: Survey table

**Files:** Create `src/components/kortlaegning/SurveyTable.tsx`, `SurveyTable.module.css`.

**Interfaces:**

```tsx
export interface SurveyTableProps {
  types: SurveyType[];               // already filtered by tab + filters
  counts: Record<Tab, number>;
  tab: Tab; onTab: (t: Tab) => void;
  filters: Filters; onFilters: (f: Filters) => void;
  rooms: { id: number; name: string }[];
  open: ReadonlySet<number>;
  selection: Selection;
  onSelect: (s: Selection) => void;   // click
  onToggle: (typeId: number) => void; // chevron / click on selected type
  onOpenDialog: (s: Selection) => void; // double-click
  onKeyDown: (e: React.KeyboardEvent) => void; // table-wrap keyboard
  tableRef: React.RefObject<HTMLDivElement | null>;
}
export function SurveyTable(props: SurveyTableProps): JSX.Element;
```

Structure (prototype `kortlaegning.png`): a raised panel with
1. tabs row: `Til gennemsyn (n)`, `Godkendt (n)`, `Alle (n)` (`role="tablist"`, buttons `role="tab"`, `aria-selected`);
2. tools row: `<input type="search" placeholder="Søg bygningsdel…" aria-label="Søg">`, room `<select aria-label="Rum">` with first option `Alle rum`, miljø `<select aria-label="Miljøstatus">` with `Al miljøstatus`/`Ren`/`Afventer prøve`/`Forurenet`;
3. key bar (`aria-label="Tastaturgenveje"`): `<Kbd>↑</Kbd><Kbd>↓</Kbd> naviger · <Kbd>→</Kbd><Kbd>←</Kbd> fold ud/ind · <Kbd>Enter</Kbd> åbn redigering · <Kbd>G</Kbd> godkend · <Kbd>A</Kbd> afvis · <Kbd>V</Kbd> vigtig · <Kbd>1</Kbd>–<Kbd>4</Kbd> evidens · <Kbd>Esc</Kbd> tilbage`;
4. a scroll wrap `<div ref={tableRef} tabIndex={0} aria-label="Kortlægningstabel — brug piletaster" onKeyDown={onKeyDown}>` containing `<table>` with header `Betegnelse · Mængde · EAK · BIM7AA · Behandling · Miljø · Status`;
   - type row: chevron `▸` (rotated when open, `aria-expanded`), `★` when starred (`--color-star`), name in bold, `n dele` faint; Mængde = **`formatQuantity(type.quantity, unit)`** + faint `formatTonnes(mass_t)`; EAK mono; BIM7AA; `<Pill treatment>` with `TREATMENT_LABEL`; `<Pill tone={ENV_TONE[env]}>{ENV_LABEL[env]}</Pill>`; status: approved → `<Pill tone="good">Godkendt ✓</Pill>`, else `<ConfidenceBar percent={confidencePercent(confidence)} />`;
   - part row (indented first cell): `RX-### · room_name` (+ `<Pill tone="warn" title="Instansen findes ikke længere — gennemgå eller flyt delen">forældet</Pill>` when `orphaned`), quantity + unit, EAK, empty, empty, empty, and in Status the faint text `—`;
   - selected row: `aria-selected="true"`, background `--color-accent-muted`, inset bar `box-shadow: inset 3px 0 0 var(--color-accent)`;
   - empty filtered list: one row spanning 7 columns, `Ingen rækker matcher filtrene.`
   - scroll the selected row into view (`useEffect` + `scrollIntoView({ block: 'nearest' })`).

Styling: copy the prototype's density via tokens — header cells `--font-size-2xs`, uppercase, `letter-spacing: var(--tracking-caps)`, `color: var(--color-text-muted)`, sticky (`position: sticky; top: 0; background: var(--color-surface-sunken); z-index: var(--z-panel)`); cells `padding: var(--space-2) var(--space-3)`, `border-bottom: 1px solid var(--color-border)`; the wrap scrolls (`overflow: auto; max-height: calc(100vh - var(--layout-titlebar-height) - var(--space-7) * 4)`); focus-visible outline on the wrap `outline-offset: calc(var(--space-0) - 2px)`; tabs underline active with `border-bottom: 2px solid var(--color-accent)` and `color: var(--color-accent-deep)`; inputs/selects `background: var(--color-surface-sunken); border: 1px solid var(--color-border); border-radius: var(--radius-md); padding: var(--space-1) var(--space-2); font-size: var(--font-size-sm)`; the search grows (`flex: 1 1 auto`). Chevron transition uses `--duration-fast`.

- [ ] **Step 1: Implement** the component and CSS per the structure above (complete JSX; every string as given).
- [ ] **Step 2: Lint + typecheck** (`token_lint.py … --tsx`, `run typecheck`).
- [ ] **Step 3: Commit** — `feat(gui): Kortlægning survey table`.

(Visual verification happens in Task 10 once the page renders it.)

---

### Task 7: Evidence panel

**Files:** Create `src/components/kortlaegning/EvidencePanel.tsx`, `EvidencePanel.module.css`.

**Interfaces:**

```tsx
export interface EvidencePanelProps {
  type: SurveyType | null;
  part: SurveyPart | null;
  tab: EvidenceTab; onTab: (t: EvidenceTab) => void;
  /** 'panel' (right column, 4 tabs) or 'stage' (dialog: thumbnails + large stage). */
  variant: 'panel' | 'stage';
}
export function evidenceSources(type: SurveyType | null, part: SurveyPart | null): EvidenceSource[]; // exported for the dialog thumbnails
export interface EvidenceSource { tab: EvidenceTab; label: string; caption: string; url: string | null; empty: string }
```

Sources (labels in this order, matching `EVIDENCE_TABS`):
- **Plan** — `api.renderUrl({ view: 'plan', layers: ['cloud'], width: 640, height: 480, highlight_instance, highlight_cloud })` where the highlight is the selected part's instance (`part.instance_id` + `part.cloud`), or for a type the first part with an instance; caption `Stueplan · snit i 1,2 m`.
- **Foto** — the best frame of the selected (or first linked) part: load `api.instanceFrames(cloud, instanceId)` and use `api.frameImageUrl(frames[0].frame_id, 'color', { maxSize: 960 })`; caption `Bedste foto · ramme <id>`; empty `Ingen foto — bygningsdelen er ikke koblet til en instans.`
- **Punktsky** — `api.renderUrl({ view: 'orbit', orbit_index: 1, layers: ['cloud'], width: 640, height: 480, … highlight })`; caption `Punktsky · bygningsdel markeret`.
- **Rum-model** — `api.renderUrl({ view: 'orbit', orbit_index: 1, layers: ['rooms'], width: 640, height: 480 })`; caption `Rumvis model · segmenterede rum`; empty when the render fails with the text below.
- When nothing is selected: the whole panel shows `<EmptyState title="Vælg en række for evidens." />`.

Every image uses `<img onError>` to switch to the source's empty text plus the generic line `Billedet kunne ikke tegnes — serveren mangler måske 3D-visning, eller projektet mangler data til denne visning.` (503/422 have no readable body for an `<img>`; the panel says what it can.) Images get `alt` = label.

`variant="panel"`: tabs (`Plan · Foto · Punktsky · Rum-model`, `role="tablist"`), then the image in a bordered well (`--radius-md`, `--color-border`), caption row below (`--font-size-xs`, faint): left the caption, right the selection's label (`RX-### · room` or the type name). `variant="stage"`: a 4-up thumbnail grid (each a button with the image and a navy caption strip `1 · Plan` …, active one with `--color-accent` border) above a large stage showing the active source with a tagline chip (`--color-chrome` 85 % mix background, `--color-on-chrome` text).

- [ ] **Step 1: Implement** (`useAsync` for the frame lookup keyed on `[part?.cloud, part?.instance_id]`).
- [ ] **Step 2: Lint + typecheck; commit** — `feat(gui): Kortlægning evidence panel`.

---

### Task 8: Detail panel

**Files:** Create `src/components/kortlaegning/DetailPanel.tsx`, `DetailPanel.module.css`.

**Interfaces:**

```tsx
export interface DetailPanelProps {
  type: SurveyType | null;
  part: SurveyPart | null;
  samples: Sample[];                       // all samples; the panel picks the type's
  busy: boolean;                           // a request is in flight
  onQuantity: (q: number) => void;         // type → redistributes; part → that part
  onTreatment: (t: Treatment) => void;
  onNote: (note: string) => void;          // committed on blur when changed
  onStar: () => void;
  onApprove: () => void;
  onReject: () => void;
  onReopen: () => void;
}
export function DetailPanel(props: DetailPanelProps): JSX.Element;
```

Layout (prototype right column, lower panel): head row with title (`RX-### · ` prefix for a part, then the type name), `<Pill tone="accent">{bim7aa_code}</Pill>` when set, miljø pill, `<Pill tone="warn">★ Vigtig</Pill>` when starred. Body: a 2-column field grid —
- **Mængde (denne del)** for a part / **Mængde (aggregeret)** for a type: text input showing `formatNumber(q)` + unit; Enter or blur parses with `parseDanishNumber` and calls `onQuantity` when the value changed and is valid (invalid → keep the old value);
- **EAK-kode**: `{eak_code} · {eak_name}` (mono code);
- **Behandling**: `<select>` over `TREATMENTS` with `TREATMENT_LABEL`;
- **Sikkerhed (AI)**: `<ConfidenceBar>`;
then the sample line: with linked samples, `Miljøstatus styres af ` + each linked sample as `{code} · {title}` + ` — {STAGE_LABEL[stage]}` (+ ` · {result}`), as plain text (Miljø & prøver is Phase 4); without, `Ingen prøve koblet — miljøstatus fra screening: ren.`; then **Proces / håndtering** textarea (note); then an actions row: `☆ Markér vigtig` / `★ Fjern vigtig` (ghost), spacer, and for a queued type `Afvis` + primary `Godkend mængde ✓` (disabled when `environment_status === 'afventer'` or `busy`), for an approved type `Genåbn`. Under the actions, when blocked: gate note in `--tone-warn-ink`: `Kan ikke godkendes endnu — afventer prøvesvar ({codes}).` With nothing selected: `<EmptyState title="Vælg en type eller bygningsdel i tabellen." />`.

Field labels: `--font-size-2xs`, uppercase, `--tracking-caps`, `--color-text-muted`. Inputs as in Task 6. Primary button: `background: var(--color-accent-deep); color: var(--color-on-accent); border-radius: var(--radius-md); padding: var(--space-2) var(--space-3); font-weight: var(--font-weight-bold);` disabled → `background: var(--color-text-faint); cursor: not-allowed`.

- [ ] **Step 1: Implement.** Keep the quantity and note drafts in local state reset when the selection changes (`key` on the panel by selection).
- [ ] **Step 2: Lint + typecheck; commit** — `feat(gui): Kortlægning detail panel`.

---

### Task 9: Edit dialog

**Files:** Create `src/components/kortlaegning/EditDialog.tsx`, `EditDialog.module.css`.

**Interfaces:**

```tsx
export interface EditDialogProps {
  type: SurveyType;
  part: SurveyPart | null;
  samples: Sample[];
  tab: EvidenceTab; onTab: (t: EvidenceTab) => void;
  busy: boolean;
  onSelectPart: (code: string | null) => void;   // part chips; null = "Alle"
  onPrev: () => void; onNext: () => void; onClose: () => void;
  onApproveNext: () => void; onReject: () => void;
  onQuantity: (q: number) => void; onTreatment: (t: Treatment) => void;
  onNote: (n: string) => void; onStar: () => void;
  onKeyDown: (e: React.KeyboardEvent) => void;
}
export function EditDialog(props: EditDialogProps): JSX.Element;
```

Structure (prototype `dialog.png`): a scrim (`position: fixed; inset: 0; background: color-mix(in srgb, var(--color-scrim) 62%, transparent); z-index: var(--z-toast)`), centred dialog `role="dialog" aria-modal="true" aria-labelledby` (`width: min(1180px, 96vw)` → express as `min(calc(var(--space-7) * 24), 96vw)`, `max-height: 92vh`, `background: var(--color-surface)`, `border-radius: var(--radius-xl)`, `box-shadow: var(--shadow-lg)`).
- Navy head: title (`--font-display`, uppercase) = `RX-### · ` prefix when a part is selected, then type name; BIM7AA pill, miljø pill, ★ pill; right side `‹` (title `Forrige række (PgUp)`), `›` (`Næste række (PgDn)`), `✕` (`Luk (Esc)`) chrome buttons.
- Body: 21rem left form column (use `--layout-bench-aside-width`) on `--color-surface-raised` with — **Bygningsdele (n)** part chips (`Alle · {total} {unit}` then `RX-### · room · {qty}`; active chip filled `--color-accent-deep`/`--color-on-accent`), **Mængde (denne del | aggregeret — fordeles på delene)** input + unit + tonnes, **EAK-kode**, **Behandling** select, **Sikkerhed (AI)** bar + `· n dele`, the sample line (as Task 8), **Proces / håndtering** textarea, **Fotos** row = up to 5 frame thumbnails from `api.instanceFrames` of the selected part (or first linked part) via `api.frameImageUrl(id,'color',{maxSize:160})`, `+n` when more, and `☆ Markér vigtig`; right: `<EvidencePanel variant="stage">`.
- Foot: hints `<Kbd>⌘/Ctrl</Kbd>+<Kbd>Enter</Kbd> godkend & næste · <Kbd>PgUp</Kbd><Kbd>PgDn</Kbd> skift række/del · <Kbd>1</Kbd>–<Kbd>4</Kbd> skift visning · <Kbd>Esc</Kbd> luk`, spacer, gate note (as Task 8), `Afvis`, primary `Godkend & næste ✓` (for an approved type: `Godkendt ✓ — næste`; disabled while blocked or busy).
- Focus: focus the quantity input on open and after each move; trap Tab inside the dialog (first/last focusable wrap); restore focus to the table wrap on close (the page does this).
- `onKeyDown` on the dialog root.

- [ ] **Step 1: Implement.**
- [ ] **Step 2: Lint + typecheck; commit** — `feat(gui): Kortlægning edit dialog`.

---

### Task 10: Kortlægning page, route, navigation and badge

**Files:** Create `src/routes/KortlaegningPage.tsx`, `KortlaegningPage.module.css`; modify `src/app/App.tsx`, `src/app/navigation.ts`, `src/app/AppShell.tsx`, `src/test/navigation.test.ts`.

**Interfaces:** consumes everything above; produces the `/kortlaegning` route; `navigation.ts` Kortlægning entry loses `pending`; `AppShell` passes `badges={{ reviewQueue: summary.counts.queue }}` to `Sidebar`, from `api.surveySummary()` reloaded when the page changes the survey (expose a tiny `SurveyCountsContext` with `refresh()` from AppShell, consumed by the page after each mutation).

- [ ] **Step 1: Navigation test first** — in `navigation.test.ts`, change the pending-entries expectation so Kortlægning is live: add `it('makes Kortlægning a live destination', () => { expect(NAV_ENTRIES.find((e) => e.to === '/kortlaegning')?.pending).toBeUndefined(); });` → run → FAIL; remove `pending` from that entry in `navigation.ts` → PASS.

- [ ] **Step 2: The page** — `KortlaegningPage.tsx` holds:
  - `const { data, error, loading, reload } = useAsync((s) => Promise.all([api.survey(s), api.samples(s), api.surveySummary(s)]), [])`; local `types` state initialised from `data`, replaced by `replaceType`/`replacePart` with every PATCH response body;
  - `tab` (default `'queue'`), `filters` (`NO_FILTERS`), `open` (Set; the selected type is opened on select), `selection`, `evidenceTab` (`'plan'`), `dialogOpen`, `busy`, `useToast()`;
  - derived: `visible = visibleTypes(types, tab, filters)`, `rows = flattenRows(visible, open)`, `counts = tabCounts(types)`, `rooms = roomOptions(types)`, `selType = typeOf(types, selection)`, `selPart = partOf(types, selection)`;
  - initial selection: first row of `rows` once data arrives; focus the table wrap on mount (`tableRef.current?.focus({ preventScroll: true })`).
  - **Actions** (each sets `busy`, awaits, clears `busy`, and calls the shell's counts `refresh()`):
    - approve: `api.patchSurveyType(selType.id, { review_status: 'approved' })` → `replaceType`; toast `✓ {name} godkendt · {n} tilbage i køen`; select `nextInQueue(newTypes, id)`. On `ApiRequestError` with `isUnprocessable`: toast `Kan ikke godkendes — afventer prøvesvar ({codes})` (codes from the type's `sample_ids` → `samples`), selection unchanged. Other errors → toast `Kunne ikke gemme: {message}`.
    - reject: `review_status: 'rejected'` → toast `Afvist som fejldetektion — fjernet fra listen`; select `nextInQueue`.
    - reopen: `review_status: 'queue'`.
    - star: type → `{ starred: !starred }`; part → `api.patchSurveyPart(code, { starred: !part.starred })` → `replacePart`.
    - quantity: part → `patchSurveyPart(code, { quantity })`; type → `patchSurveyType(id, { quantity })`.
    - treatment → `patchSurveyType(id, { treatment })`; note → type `{ note }` / part `patchSurveyPart(code, { note })`.
  - **Keyboard**: table wrap `onKeyDown` → `tableAction({ key, metaKey, ctrlKey, altKey, inField: isField(e.target) })` where `isField` checks `INPUT/SELECT/TEXTAREA`; handle `move` (moveSelection over `rows`), `expand` (add to `open`), `collapse` (remove; if a part was selected select its type), `open` (dialog), `approve`/`reject`/`star`, `evidence` (set tab), `blur` (`(e.target as HTMLElement).blur(); tableRef.current?.focus()`). Call `e.preventDefault()` for handled actions. Dialog root `onKeyDown` → `dialogAction`: `close`, `approveNext` (approve if queued, else just move to next), `move` (moveSelection over `flattenRows(visible, new Set([...open, selection.typeId]))` so PgUp/PgDn walk parts too), the letters, `evidence`.
  - **Layout** (prototype `kortlaegning.png`): view head — `<h2>Kortlægning</h2>` + sub `Ressourcekortlægning · bygningsdele pr. rum, grupperet pr. type` + right `<a className={styles.btnGhost} href={api.csvExportUrl()} download>Eksport (XLS)</a>`; coverage notice (warn tone) when `summary.unlabeled_points` or `summary.rooms_without_parts.length`: `<b>Dækning:</b> {formatNumber(unlabeled)} punkter uklassificeret · {n} rum uden registrerede bygningsdele ({names})` (omit the parts that are absent); bench grid `grid-template-columns: minmax(0, 1fr) var(--layout-bench-aside-width); gap: var(--space-4); align-items: start` stacking below 1080px; left `SurveyTable`, right column `EvidencePanel variant="panel"` above `DetailPanel`; `EditDialog` when open; `Toast`.
  - **Empty project** (`types.length === 0` after load): `<EmptyState title="Ingen kortlægning endnu" detail="Opret typer og bygningsdele ud fra projektets instanser. Kræver at rux create instances er kørt." action={<button className={styles.btnPrimary} onClick={sync}>Opret kortlægning fra instanser</button>} />`; `sync` calls `api.syncSurvey()` then `reload()`; a 422 shows its message in an `ErrorBanner`-style notice (`context="Kortlægning"`); a sync report with `parts_orphaned > 0` toasts `{n} del(e) peger på instanser der ikke findes længere`.
  - **Errors** on load: `<ErrorBanner error={error} onRetry={reload} context="Kortlægning" />`; loading: `<Spinner label="Indlæser kortlægning…" />`.
- [ ] **Step 3: Route** — `App.tsx`: import and add `<Route path="/kortlaegning" element={<KortlaegningPage />} />`.
- [ ] **Step 4: Badge** — `AppShell.tsx`: fetch `api.surveySummary()` with `useAsync`, keep a `refresh` (the hook's `reload`) in a `SurveyCountsContext` (new file `src/app/SurveyCountsContext.tsx` exporting the provider value `{ refresh: () => void }` and `useSurveyCounts()`), and pass `badges={{ reviewQueue: summary?.counts.queue }}` to `Sidebar`. A 404/older server must not break the shell: on error, pass no badge.
- [ ] **Step 5: Verify against the prototype**

```bash
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/kort-demo.rux"
RUX_BIN="$PWD/build/apps/rux/rux" bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/kort-demo.rux"
bash .claude/skills/design-studio/scripts/shot.sh http://localhost:5173/kortlaegning --out shots/p3 --theme light --viewports desktop --wait 2500
bash .claude/skills/design-studio/scripts/shot.sh http://localhost:5173/kortlaegning --out shots/p3 --theme dark --viewports desktop --wait 2500
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

Open both PNGs with Read and compare with `docs/gui/images/prototype-v2/kortlaegning.png`: tabs `Til gennemsyn (7) · Godkendt (4) · Alle (11)`, the sidebar badge `7`, the first queued type selected and expanded, treatment pills in the circ colours, `Afventer prøve` on Vinduespartier and Gulvbelægning, `Forurenet` on murvægge, the evidence panel showing a Plan render, the detail panel with Godkend disabled for an `afventer` type. For the dialog, write a one-off Playwright script in the scratchpad (run with the nix python `shot.sh` uses) that opens `/kortlaegning`, presses `Enter`, and screenshots; compare with `dialog.png`. Also press `G` on an `afventer` type and screenshot the toast. Fix what differs, re-shoot.

- [ ] **Step 6: Gates** — `npm --prefix apps/rux/frontend test && npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend run build`; `token_lint.py` on every new `.module.css` with `--tsx`; `reuse lint`.
- [ ] **Step 7: Commit** — `feat(gui): Kortlægning workbench at /kortlaegning`.

---

### Task 11: Docs

**Files:** `apps/rux/frontend/README.md` (Layout: add `kortlaegning/`), `docs/design/gui-kortlaegning-redesign.md` (§ Kortlægning screen: note the 360° → Foto substitution and that sample links show as text until Phase 4), `.claude/skills/design-studio/references/reusex-frontend.md` (directory map: `src/kortlaegning/` pure modules, `components/kortlaegning/`, the demo seed).

- [ ] Edit, `reuse lint`, commit `docs(gui): Kortlægning structure and evidence substitution`.

## Phase exit criteria

- Frontend `test`, `typecheck`, `build` pass; token lint clean on new CSS; `reuse lint` compliant.
- Screenshots of `/kortlaegning` (light + dark) and the dialog match the prototype's structure on the seeded demo; the 422 toast appears on approving an `afventer` type.
- Follow-up issues: nearest-panorama endpoint for a true 360° tab; Miljø & prøver links (Phase 4).
