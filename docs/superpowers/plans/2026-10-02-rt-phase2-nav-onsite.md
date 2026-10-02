<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Resources & Templates Phase 2 — Navigation + On-site Frontend Removal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Load the project skill `design-studio` (`.claude/skills/design-studio/SKILL.md`) before touching any `.tsx`/`.css`.

**Goal:** Regroup the sidebar so each job appears once (spec §3), remove every piece of On-site from the web frontend (spec §8, frontend bullets), and express the old-path redirects as data that Phases 3 and 4 extend.

**Architecture:**
- `src/app/navigation.ts` stays the single, React-free nav contract. It gains `REDIRECTS`, a list of `{from, to}`. `App.tsx` renders each one as `<Navigate replace>`.
- The redirect sources added here are `/on-site` (the real path) and `/onsite` (the spelling the spec uses).
- The On-site route, its components, its pure model, its link helpers and its test are deleted.
- So is everything that only existed to show what On-site wrote:
  - Miljø's "Udtaget ved RX-…" line (`takenAt`);
  - `Sample.part_code`;
  - `SampleCreate.part_code` / `stage`;
  - the Sager "På pladsen med telefonen" panel.
- Materialedata and Eksport leave the nav, but their routes stay registered, so they are still reachable by URL until Phase 3 and Phase 4 delete them.

**Tech Stack:** React 19, react-router-dom 7, TypeScript, CSS Modules, vitest (Node, no DOM). Playwright via the design-studio scripts for the visual check.

**Spec:** `docs/superpowers/specs/2026-10-02-resources-templates-ia-design.md`. This plan covers §3 (Navigation) and §8 (frontend bullets). The phase split comes from the controller's context `rt-plan-context.md`, which overrides §11 where they differ.

## Rulings (spec silent, or contradicted by the code)

- **R1 — Skabeloner appears in the nav now, as a `pending` entry, with no route.**
  - **The mechanism already exists.** `NavEntry.pending` renders an inert `<span>` whose title gives the reason (`components/Sidebar.tsx:49-51`). A "not built yet" destination is supposed to be visible (`navigation.ts:8-13`), so the Sag group gets its final shape in this phase.
  - **Why no placeholder route:** a stub route would be throwaway UI that has to be designed, linted and shot.
  - **Until Phase 4,** `/skabeloner` falls through to the `*` catch-all and lands on Overblik.
  - **What Phase 4 changes:**
    - delete the `pending` field from the Skabeloner entry;
    - register `<Route path={SKABELONER_PATH} element={<SkabelonerPage />} />`;
    - flip the navigation test `keeps Skabeloner pending until its page exists` to assert `pending` is undefined.
  - `SKABELONER_PATH = '/skabeloner'` is exported from `app/links.ts` now.
- **R2 — Two redirects for On-site, both to `/kortlaegning`.**
  - The route was actually `/on-site` (`links.ts:170`), while spec §3 writes `/onsite`. Both redirect.
  - The query string is dropped, because `?del=` means nothing to Kortlægning. `<Navigate to="/kortlaegning">` does that by itself.
- **R3 — Redirects are data in `navigation.ts`.** The shape is `REDIRECTS: readonly Redirect[]`, where `Redirect` is `{ from: string; to: string }`.
  - `App.tsx` maps the list to routes, placed before the `*` catch-all.
  - Phase 3 appends `{ from: '/materials', to: KORTLAEGNING_PATH }` and deletes the `/materials` `<Route>` and its import in the same change.
  - Phase 4 appends `{ from: '/export', to: RAPPORT_PATH }` and deletes the `/export` `<Route>`.
  - The navigation tests check that every redirect source is absent from `NAV_ENTRIES`, that every target is a live nav entry, and that no redirect chains.
  - Node has no DOM, so nothing can test that a redirect source isn't *also* still a `<Route>` in `App.tsx`. Whoever adds a redirect deletes that route in the same commit.
- **R4 — Materialedata (`/materials`) and Eksport (`/export`) leave the nav only.**
  - Their `<Route>`s, pages and imports in `App.tsx` stay untouched.
  - `components/InstanceList.tsx:72` (`navigate('/materials')`) keeps working because the route still exists. Phase 3's redirect keeps it working afterwards; Phase 3 decides whether to retarget it.
- **R5 — Viewport moves to Sag, between Kortlægning and Miljø & prøver** (spec §3 order). Its path stays `/viewport`, so the keep-alive check in `App.tsx:63` is unaffected.
- **R6 — Miljø's "Udtaget ved RX-008 · Office Zone" line is deleted.** It shows `samples.part_code`, which Phase 1 drops. The spec counts it as "a `samples` field used only by On-site".
  - `takenAt` and `TakenAt` go, along with their test and the `SampleCard` block.
  - `partLabel` is still used by the other Kortlægning components, so its definition stays. Only `miljoe/model.ts`'s import of it goes.
- **R7 — `SampleCreate` loses `part_code` and `stage`.** `Sample` loses `part_code`. Miljø's `createBody` never sent either field, so no request changes.
  - **This phase works before or after Phase 1.** An extra `part_code` in a response is just ignored.
- **R8 — The Sager panel "På pladsen med telefonen" is deleted,** together with `phoneCommand` and `NO_AUTH_WARNING`. It existed to reach On-site from a phone (Phase 6 R10). §8 keeps `--bind`/`--allow-origin` only as generic options, with no phone wording.
  - The "Åbn en anden sag" panel and `OPEN_ANOTHER_COMMAND` stay.
- **R9 — The ★ filter in Kortlægning stays.** `starred` is a part field (the `sys:starred` key in spec §4.3), not On-site's. Only comments that credit On-site are reworded.
- **R10 — Docs touched here are frontend-local only:**
  - `apps/rux/frontend/README.md`;
  - the design-studio frontend map `.claude/skills/design-studio/references/reusex-frontend.md`.

  CLAUDE.md, DIRECTION.md and the phase-spec note are Phase 4's. `apps/rux/frontend/dev/seed-survey-demo.sh` (the On-site samples, `part_code`) is the "dev and demo fixtures" backend bullet, so it is Phase 1's. If Phase 1 left it, it is out of scope here; report it, don't fix it.

## Global Constraints

- **SPDX header on every new file.** This plan creates none, only edits and deletions.
- **Tokens only in CSS.** Never edit `src/tokens.css`. Lint every changed CSS/TSX file with `python3 .claude/skills/design-studio/scripts/token_lint.py <files> --tsx`.
- **Paths stay extensionless** (`App.tsx:42-45`). Redirect sources and targets included.
- **Copy:**
  - UI copy is Danish;
  - nav labels are exactly as in spec §3: `Overblik · Kortlægning · Viewport · Miljø & prøver · Rapport · Indberetning · Skabeloner` and `Projektdata · Posegraf · Pipeline · Kørselslog · Billeder · Geometri · Instanser · Labels`.
- **Every phase leaves the app working.** `/materials` and `/export` must still render their pages at the end of this phase.
- **Do not touch the backend, `docs/gui/openapi.yaml`, CLAUDE.md or DIRECTION.md.** Those are Phase 1 and Phase 4.
- **Commands:**
  - `npm --prefix apps/rux/frontend run typecheck`;
  - `npm --prefix apps/rux/frontend test -- --run`;
  - `npm --prefix apps/rux/frontend run build`.
- **Commits:**
  - never `--no-verify`;
  - each message ends with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` and `Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf`.

## Review Focus

- **Old bookmarks with a query.** `/on-site?del=RX-008` must land on `/kortlaegning`, without a query and without a history entry (`replace`).
  - Covered by R2/R3 and the App.tsx step in Task 1.
  - Checked by hand in Task 4's visual check (the URL bar after load).
- **No redirect lands on another redirect or on nothing.** A target must be a live, non-pending nav path.
  - Pinned by the test `redirects land on a live entry, never on another redirect` in Task 1.
  - Phases 3/4 append to the same list, so this test guards them too.
- **A removed entry must not linger.** No nav entry may point at `/on-site`, `/onsite`, `/materials` or `/export`.
  - Pinned by `drops the pages the redesign retires from the nav` in Task 1.
- **`/materials` and `/export` still render after this phase** (R4). They are off the nav but not dead.
  - Checked in Task 4's visual check, which shoots both URLs.
- **Leftover On-site references.** Any import, CSS class or comment naming On-site would be dead weight or a broken build.
  - Pinned by the grep gate in Task 2 (no `on-?site` in `src`);
  - and by `links no longer export the On-site helpers` and `sager no longer exports the phone command` in Task 2.

---

## File map

| File | Change | Task |
|---|---|---|
| `apps/rux/frontend/src/app/links.ts` | add `SKABELONER_PATH`; later remove On-site helpers (lines 170-187) | 1, 2 |
| `apps/rux/frontend/src/app/navigation.ts` | regroup `NAV_ENTRIES`, add `Redirect`/`REDIRECTS`, drop `ONSITE_PATH` import | 1 |
| `apps/rux/frontend/src/app/App.tsx` | drop On-site route/import, render `REDIRECTS` | 1 |
| `apps/rux/frontend/src/test/navigation.test.ts` | new order, removed entries, Skabeloner pending, redirects | 1 |
| `routes/OnsitePage.{tsx,module.css}`, `components/onsite/*`, `onsite/model.ts`, `test/onsite.model.test.ts` | delete | 2 |
| `apps/rux/frontend/src/test/links.test.ts` | drop On-site block, assert the helpers are gone | 2 |
| `apps/rux/frontend/src/routes/SagerPage.{tsx,module.css}`, `sager/model.ts`, `test/sager.model.test.ts` | drop the phone panel | 2 |
| `app/onceGuard.ts`, `app/editorKeys.ts`, `components/controls.module.css`, `kortlaegning/{model,photo,vocab}.ts`, `miljoe/model.ts` | reword comments that credit On-site | 2 |
| `apps/rux/frontend/src/api/types.ts:930-931, 993-997` | drop `Sample.part_code`, `SampleCreate.part_code`/`stage` | 3 |
| `apps/rux/frontend/src/miljoe/model.ts:199-213`, `components/miljoe/SampleCard.tsx:185, 224-235` | drop `takenAt` | 3 |
| `test/miljoe.model.test.ts:242-256`, `test/survey.client.test.ts:86-103`, `test/surveyFixtures.ts:52`, `test/kortlaegning.page.test.ts:52`, `test/kortlaegning.detailPanel.test.ts:77` | follow the type change | 3 |
| `apps/rux/frontend/README.md`, `.claude/skills/design-studio/references/reusex-frontend.md` | frontend map without On-site | 4 |

---

### Task 1: Nav regroup, Skabeloner pending entry, redirects as data

**Files:**
- Modify: `apps/rux/frontend/src/app/links.ts:128-133` (add one constant)
- Modify: `apps/rux/frontend/src/app/navigation.ts:17-94`
- Modify: `apps/rux/frontend/src/app/App.tsx:10-19, 31, 87, 97`
- Test: `apps/rux/frontend/src/test/navigation.test.ts`

**Interfaces:**
- Produces:
  - `SKABELONER_PATH: '/skabeloner'` (links.ts);
  - `interface Redirect { from: string; to: string }` and `REDIRECTS: readonly Redirect[]` (navigation.ts).
- Phases 3/4 append to `REDIRECTS`. Phase 4 removes `pending` from the Skabeloner entry.

- [ ] **Step 1: Write the failing tests.** In `test/navigation.test.ts`, extend the import and replace the three tests at lines 21-54 (`lists the case workflow…`, `lists On-site last…`, `keeps every existing technical route…`) with the block below. Leave every other test as is.

```ts
import {
  ALL_CASES_PATH,
  DRAWER_QUERY,
  NAV_ENTRIES,
  REDIRECTS,
  badgeText,
  drawerClickCloses,
  drawerKeyAction,
  displayProjectName,
  entriesIn,
} from '../app/navigation';
```

```ts
  it('lists the case workflow in the spec §3 order', () => {
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

  it('lists the tools in the spec §3 order', () => {
    expect(entriesIn('tools').map((e) => e.label)).toEqual([
      'Projektdata',
      'Posegraf',
      'Pipeline',
      'Kørselslog',
      'Billeder',
      'Geometri',
      'Instanser',
      'Labels',
    ]);
  });

  it('shows each job exactly once', () => {
    const labels = NAV_ENTRIES.map((e) => e.label);
    expect(new Set(labels).size).toBe(labels.length);
  });

  it('drops the pages the redesign retires from the nav', () => {
    const paths = NAV_ENTRIES.map((e) => e.to);
    for (const gone of ['/on-site', '/onsite', '/materials', '/export']) {
      expect(paths, gone).not.toContain(gone);
    }
  });

  it('keeps Viewport live, now in the case group', () => {
    expect(NAV_ENTRIES.find((e) => e.to === '/viewport')).toEqual({
      to: '/viewport',
      label: 'Viewport',
      group: 'sag',
    });
  });

  it('keeps Skabeloner pending until its page exists', () => {
    const entry = NAV_ENTRIES.find((e) => e.to === '/skabeloner');
    expect(entry?.group).toBe('sag');
    expect(entry?.pending).toBeDefined();
  });

  it('keeps every remaining technical route reachable under Værktøjer', () => {
    const tools = entriesIn('tools').map((e) => e.to);
    for (const path of [
      '/projektdata',
      '/graph-view',
      '/pipeline',
      '/pipeline/log',
      '/frames',
      '/geometry',
      '/instances',
      '/labels',
    ]) {
      expect(tools).toContain(path);
    }
  });
```

  Then add a new `describe` block after `describe('navigation model', …)`:

```ts
describe('redirects for retired paths (spec §3)', () => {
  it('sends both On-site spellings to Kortlægning', () => {
    expect(REDIRECTS).toContainEqual({ from: '/on-site', to: '/kortlaegning' });
    expect(REDIRECTS).toContainEqual({ from: '/onsite', to: '/kortlaegning' });
  });

  it('never redirects from a path the nav still lists', () => {
    const navPaths = new Set(NAV_ENTRIES.map((e) => e.to));
    for (const r of REDIRECTS) expect(navPaths.has(r.from), r.from).toBe(false);
  });

  it('redirects land on a live entry, never on another redirect', () => {
    const live = new Set(NAV_ENTRIES.filter((e) => e.pending === undefined).map((e) => e.to));
    const sources = new Set(REDIRECTS.map((r) => r.from));
    for (const r of REDIRECTS) {
      expect(live.has(r.to), `${r.from} → ${r.to}`).toBe(true);
      expect(sources.has(r.to), `${r.from} → ${r.to}`).toBe(false);
    }
  });

  it('uses unique, extensionless sources', () => {
    const from = REDIRECTS.map((r) => r.from);
    expect(new Set(from).size).toBe(from.length);
    for (const p of from) expect(p, p).not.toMatch(/\./);
  });
});
```

- [ ] **Step 2: Run the tests and watch them fail.**

  Run: `npm --prefix apps/rux/frontend test -- --run src/test/navigation.test.ts`

  Expected: FAIL. `REDIRECTS` is undefined (an import error or `toContainEqual` on undefined), and the order assertions fail on `On-site`.

- [ ] **Step 3: Add the path constant.** In `app/links.ts`, after `export const PROJEKTDATA_PATH = '/projektdata';` (line 133):

```ts
/** Skabeloner — the template editor (resources/templates spec §6.2). Its page arrives in Phase 4. */
export const SKABELONER_PATH = '/skabeloner';
```

- [ ] **Step 4: Regroup the nav and add the redirects.** In `app/navigation.ts`:

  1. In the import from `./links` (lines 20-28), replace `ONSITE_PATH,` with `SKABELONER_PATH,`. Keep the list alphabetical: it goes after `RAPPORT_PATH`.
  2. Replace `NAV_ENTRIES` (lines 75-94) with:

```ts
export const NAV_ENTRIES: readonly NavEntry[] = [
  { to: OVERBLIK_PATH, label: 'Overblik', group: 'sag', end: true },
  { to: KORTLAEGNING_PATH, label: 'Kortlægning', group: 'sag', badge: 'reviewQueue' },
  { to: '/viewport', label: 'Viewport', group: 'sag' },
  { to: MILJOE_PATH, label: 'Miljø & prøver', group: 'sag', badge: 'pendingSamples' },
  { to: RAPPORT_PATH, label: 'Rapport', group: 'sag' },
  { to: INDBERETNING_PATH, label: 'Indberetning', group: 'sag' },
  {
    to: SKABELONER_PATH,
    label: 'Skabeloner',
    group: 'sag',
    pending: 'Skabeloner er på vej — her samler du felter, du bruger igen og igen.',
  },

  { to: PROJEKTDATA_PATH, label: 'Projektdata', group: 'tools' },
  { to: '/graph-view', label: 'Posegraf', group: 'tools' },
  { to: '/pipeline', label: 'Pipeline', group: 'tools', end: true },
  { to: '/pipeline/log', label: 'Kørselslog', group: 'tools' },
  { to: '/frames', label: 'Billeder', group: 'tools' },
  { to: '/geometry', label: 'Geometri', group: 'tools' },
  { to: '/instances', label: 'Instanser', group: 'tools' },
  { to: '/labels', label: 'Labels', group: 'tools' },
];

/** An old path that still resolves, so bookmarks and cross-links keep working. */
export interface Redirect {
  from: string;
  to: string;
}

/**
 * Retired paths (resources/templates spec §3), rendered by App.tsx as
 * `<Navigate replace>` ahead of the catch-all, so they never stack history.
 * The query is dropped. A source must not also be a `<Route>` in App.tsx:
 * whoever adds one here deletes that route in the same change. Phase 3 adds
 * `/materials`, Phase 4 `/export`.
 */
export const REDIRECTS: readonly Redirect[] = [
  { from: '/on-site', to: KORTLAEGNING_PATH },
  { from: '/onsite', to: KORTLAEGNING_PATH },
];
```

- [ ] **Step 5: Route the redirects.** In `app/App.tsx`:

  1. Remove `ONSITE_PATH,` from the `./links` import (line 14).
  2. Change line 19 to `import { ALL_CASES_PATH, REDIRECTS } from './navigation';`.
  3. Delete line 31 (`import { OnsitePage } …`) and line 87 (`<Route path={ONSITE_PATH} … />`).
  4. Insert this directly before the catch-all `<Route path="*" …>` (line 97):

```tsx
          {REDIRECTS.map((r) => (
            <Route key={r.from} path={r.from} element={<Navigate to={r.to} replace />} />
          ))}
```

  `/materials` and `/export` keep their `<Route>`s (R4). `OnsitePage.tsx` is now unreferenced. It still compiles, because `ONSITE_PATH` and the other helpers are still exported from `links.ts` until Task 2.

- [ ] **Step 6: Run the tests and the typecheck.**

  Run: `npm --prefix apps/rux/frontend test -- --run src/test/navigation.test.ts && npm --prefix apps/rux/frontend run typecheck`

  Expected: PASS, no type errors.

- [ ] **Step 7: Lint and commit.**

```bash
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/app/App.tsx --tsx
git add apps/rux/frontend/src/app/links.ts apps/rux/frontend/src/app/navigation.ts \
        apps/rux/frontend/src/app/App.tsx apps/rux/frontend/src/test/navigation.test.ts
git commit -m "feat(gui): regroup the nav and redirect the retired On-site path

Viewport joins the case group, Skabeloner is listed as pending, and
Materialedata, Eksport and On-site leave the sidebar. Retired paths are
data (REDIRECTS) rendered as <Navigate replace>.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 2: Delete the On-site frontend and the Sager phone panel

**Files:**
- Delete:
  - `apps/rux/frontend/src/routes/OnsitePage.tsx` and `OnsitePage.module.css`;
  - `apps/rux/frontend/src/components/onsite/{CaptureSheet,CaptureStage,PartPicker}.{tsx,module.css}`;
  - `apps/rux/frontend/src/onsite/model.ts`;
  - `apps/rux/frontend/src/test/onsite.model.test.ts`.
- Modify: `apps/rux/frontend/src/app/links.ts:170-187`, `test/links.test.ts`
- Modify: `apps/rux/frontend/src/routes/SagerPage.tsx:5, 8, 14-23, 26-31, 101-111`, `routes/SagerPage.module.css:52-65, 78-80`
- Modify: `apps/rux/frontend/src/sager/model.ts:5-9, 89-101`, `test/sager.model.test.ts:7-15, 89-100`
- Modify (comments only):
  - `app/onceGuard.ts:6-7`;
  - `app/editorKeys.ts:62-63`;
  - `components/controls.module.css:141-143`;
  - `kortlaegning/model.ts:19`;
  - `kortlaegning/photo.ts:7-9`;
  - `kortlaegning/vocab.ts:131-134`;
  - `miljoe/model.ts:159`.

**Interfaces:**
- Consumes: Task 1, where `App.tsx` no longer imports `OnsitePage`.
- Produces: `links.ts` without `ONSITE_PATH`, `onsiteHref`, `parseOnsiteQuery` and `rawOnsiteDel`; `sager/model.ts` without `phoneCommand` and `NO_AUTH_WARNING`.
- Nothing outside the deleted files imports the On-site modules. Every helper they imported is still used elsewhere:
  - `createBody`, `gateChanges`, `statusPill` (Miljø);
  - `resolvePhotoState`, `FrameLookup`, `hasInstanceLink`, `instanceKey`, `PHOTO_EMPTY_TEXT` (EvidencePanel / EditDialog);
  - `replacePart` (KortlaegningPage);
  - `confidencePercent`, `STAGE_LABEL`.

  So none of them is removed.

- [ ] **Step 1: Write the failing tests.**

  In `test/links.test.ts`:
  - shrink the import to `newSampleHref, parseMiljoeQuery, parseTypeQuery, sampleHref, surveyTypeHref` and add `import * as links from '../app/links';`;
  - delete `describe('on-site links', …)` (lines 49-77);
  - add:

```ts
describe('retired On-site links', () => {
  it('links no longer export the On-site helpers', () => {
    for (const name of ['ONSITE_PATH', 'onsiteHref', 'parseOnsiteQuery', 'rawOnsiteDel']) {
      expect(name in links, name).toBe(false);
    }
  });
});
```

  In `test/sager.model.test.ts`:
  - remove `NO_AUTH_WARNING` and `phoneCommand` from the import and add `import * as sager from '../sager/model';`;
  - replace `describe('commands (R1, R10)', …)` (lines 89-100) with:

```ts
describe('commands (R1)', () => {
  it('says how to open another case', () => {
    expect(OPEN_ANOTHER_COMMAND).toBe('rux -p <fil>.rux gui');
  });

  it('sager no longer exports the phone command (On-site moved to the mobile app)', () => {
    expect('phoneCommand' in sager).toBe(false);
    expect('NO_AUTH_WARNING' in sager).toBe(false);
  });
});
```

- [ ] **Step 2: Run the tests and watch them fail.**

  Run: `npm --prefix apps/rux/frontend test -- --run src/test/links.test.ts src/test/sager.model.test.ts`

  Expected: FAIL, because `ONSITE_PATH` and `phoneCommand` are still exported.

- [ ] **Step 3: Delete the On-site files.**

```bash
git rm apps/rux/frontend/src/routes/OnsitePage.tsx apps/rux/frontend/src/routes/OnsitePage.module.css \
       apps/rux/frontend/src/components/onsite/CaptureSheet.tsx apps/rux/frontend/src/components/onsite/CaptureSheet.module.css \
       apps/rux/frontend/src/components/onsite/CaptureStage.tsx apps/rux/frontend/src/components/onsite/CaptureStage.module.css \
       apps/rux/frontend/src/components/onsite/PartPicker.tsx apps/rux/frontend/src/components/onsite/PartPicker.module.css \
       apps/rux/frontend/src/onsite/model.ts apps/rux/frontend/src/test/onsite.model.test.ts
```

- [ ] **Step 4: Remove the link helpers.** In `app/links.ts`, delete everything from `export const ONSITE_PATH = '/on-site';` (line 170) to the end of `rawOnsiteDel` (line 187), including the three doc comments. The file then ends with `parseTypeQuery`.

- [ ] **Step 5: Remove the Sager phone panel.**

  In `sager/model.ts`:
  - delete `phoneCommand` with its doc comment, and `NO_AUTH_WARNING` (lines 89-101);
  - reword the header comment (lines 5-9) to:

```ts
/**
 * Sager as data: the one card `rux gui` can show (R1) and the command that
 * opens another case. Every figure is read off a server response; this module
 * only words it.
 */
```

  In `routes/SagerPage.tsx`:
  - delete `import { Link } from 'react-router-dom';` (line 5); it is only used by the removed link;
  - change line 8 to `import { OVERBLIK_PATH } from '../app/links';`;
  - drop `NO_AUTH_WARNING,` and `phoneCommand,` from the `../sager/model` import;
  - delete the block from `<h2 className={styles.panelHeading}>På pladsen med telefonen</h2>` through `<p className={styles.warn}>{NO_AUTH_WARNING}</p>` (lines 101-111);
  - reword the component's doc comment (lines 26-31) to:

```tsx
/**
 * Sager — the case list (R1). `rux gui` serves one project, so the list is
 * that project's card, plus how to open another. The grid is the
 * prototype's, so a longer list from a multi-case server drops in without a
 * layout change.
 */
```

  In `routes/SagerPage.module.css`:
  - change the grouped selector `.text,\n.muted,\n.warn {` to `.text,\n.muted {`;
  - delete the `.warn { composes: notice … }` rule;
  - delete the `.crossLink { composes: crossLink … }` rule.

  `.command` stays, because `OPEN_ANOTHER_COMMAND` uses it.

- [ ] **Step 6: Reword the comments that credit On-site.** These are comment-only edits, each replacing the exact text shown:

  - `app/onceGuard.ts:6-7`: `(Rapport's\n * generate, Miljø's create, On-site's sample and ★)` → `(Rapport's\n * generate, Miljø's create)`.
  - `app/editorKeys.ts:62-63`: `(Miljø's\n * \`+ Ny prøve\`, On-site's sample form)` → `(Miljø's\n * \`+ Ny prøve\`)`.
  - `components/controls.module.css:141-143`: replace the whole comment with
    `/* A link to another case screen (Kortlægning ↔ Miljø & prøver). One style\n * everywhere, so "this opens the other screen" reads the same on every screen\n * (Phase 6 R12). */`.
  - `kortlaegning/model.ts:19`: `/** Only types that are ★, or have a ★ part — what On-site marks (Phase 6 R13). */` → `/** Only types that are ★, or have a ★ part (Phase 6 R13). */`.
  - `kortlaegning/photo.ts:7-9`: `Pure, so Kortlægning's evidence panel and On-site share one rule:\n * another key's photo` → `Pure, so Kortlægning's evidence panel and edit dialog share one\n * rule: another key's photo`.
  - `kortlaegning/vocab.ts:131-134`: `one copy for\n * Kortlægning's evidence panel and On-site's capture stage ("et foto", so\n * "Intet foto").` → `one copy for\n * Kortlægning's evidence panel ("et foto", so "Intet foto").`.
  - `miljoe/model.ts:159`: `shared by Miljø's toast and On-site's sample toast.` → `worded for Miljø's toast.`.

- [ ] **Step 7: Run the grep gate.**

  Run: `grep -rniE "on-?site" apps/rux/frontend/src`

  Expected: no output. If anything prints, remove it, or reword it if it is a comment.

- [ ] **Step 8: Run the tests, the typecheck and the build.**

  Run: `npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend test -- --run && npm --prefix apps/rux/frontend run build`

  Expected: all PASS. The suite no longer contains `onsite.model.test.ts`.

- [ ] **Step 9: Lint and commit.**

```bash
python3 .claude/skills/design-studio/scripts/token_lint.py \
  apps/rux/frontend/src/routes/SagerPage.tsx apps/rux/frontend/src/routes/SagerPage.module.css \
  apps/rux/frontend/src/components/controls.module.css --tsx
git add -A apps/rux/frontend/src
git commit -m "refactor(gui): remove the On-site frontend

On-site moves to the mobile-app track (resources/templates spec §8):
delete its route, components, model and link helpers, and the Sager
phone panel that pointed at it.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 3: Drop the On-site-only sample fields from the frontend

**Files:**
- Modify: `apps/rux/frontend/src/api/types.ts:930-931` (`Sample.part_code`), `:993-997` (`SampleCreate.part_code`, `stage`)
- Modify: `apps/rux/frontend/src/miljoe/model.ts:20, 199-213`
- Modify: `apps/rux/frontend/src/components/miljoe/SampleCard.tsx:21, 185, 224-235`
- Test:
  - `test/miljoe.model.test.ts:26, 242-256`;
  - `test/survey.client.test.ts:86-103`;
  - `test/surveyFixtures.ts:52`;
  - `test/kortlaegning.page.test.ts:52`;
  - `test/kortlaegning.detailPanel.test.ts:77`.

**Interfaces:**
- Consumes: Task 2, where `onsite/model.ts` (the other `part_code`/`stage` sender) is gone.
- Produces:
  - `Sample` = `{id, code, title, what, stage, result, type_ids, created_at, updated_at}`;
  - `SampleCreate` = `{title, what?, type_ids?}`.

  This matches Phase 1's backend contract for `POST /samples`.

- [ ] **Step 1: Write the failing tests.**

  In `test/miljoe.model.test.ts`:
  - delete `takenAt,` from the import (line 26) and add `import * as miljoe from '../miljoe/model';`;
  - replace `describe('where a sample was taken (Phase 6 R8)', …)` (lines 242-256) with:

```ts
describe('where a sample was taken', () => {
  it('is gone with On-site: samples no longer carry a part code', () => {
    expect('takenAt' in miljoe).toBe(false);
  });
});
```

  In `test/survey.client.test.ts`, replace `it('registers a sample at a part, already taken', …)` (lines 86-103) with:

```ts
  it('creates a sample with only Miljø’s fields', async () => {
    const { calls, api } = client({ id: 4, code: 'P-04' }, 201);
    await api.createSample({ title: 'Asbest i fugemasse', what: 'Fuge mod nord', type_ids: [6] });
    expect(calls[0]).toMatchObject({ url: '/api/v1/samples', method: 'POST' });
    expect(JSON.parse(calls[0].body!)).toEqual({
      title: 'Asbest i fugemasse',
      what: 'Fuge mod nord',
      type_ids: [6],
    });
  });
```

- [ ] **Step 2: Run the tests and watch them fail.**

  Run: `npm --prefix apps/rux/frontend test -- --run src/test/miljoe.model.test.ts`

  Expected: FAIL in `is gone with On-site`, because `takenAt` is still exported.

- [ ] **Step 3: Drop the fields from the API types.** In `api/types.ts`:
  - delete from `Sample` the two lines `/** The bygningsdel the sample was taken at (On-site, schema v24); null otherwise. */` and `part_code: string | null;`;
  - in `SampleCreate`, delete the `part_code?` member and the `stage?` member, with their doc comments (lines 993-996). The interface becomes:

```ts
/** Body of `POST /samples`. */
export interface SampleCreate {
  title: string;
  what?: string;
  type_ids?: number[];
}
```

- [ ] **Step 4: Remove `takenAt`.**

  In `miljoe/model.ts`:
  - delete `export interface TakenAt { … }` and `export function takenAt(…) { … }` (lines 199-213), with their doc comments;
  - delete `import { partLabel } from '../kortlaegning/model';` (line 20), its only user.

  In `components/miljoe/SampleCard.tsx`:
  - drop `takenAt,` from the `../../miljoe/model` import (line 21);
  - delete `const taken = takenAt(sample, types);` (line 185);
  - delete the whole `{taken && ( <p className={styles.what}> … </p> )}` block (lines 224-235).

  `Link`, `surveyTypeHref` and `styles.typeLink` stay, because `LinkedLine` (line 60) still uses them.

- [ ] **Step 5: Fix the fixtures.** Delete the line `part_code: null,` from:
  - `test/surveyFixtures.ts:52`;
  - `test/kortlaegning.page.test.ts:52`;
  - `test/kortlaegning.detailPanel.test.ts:77`.

  Then run `grep -rn "part_code" apps/rux/frontend/src`. Expected: no output.

- [ ] **Step 6: Run the tests, the typecheck and the build.**

  Run: `npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend test -- --run && npm --prefix apps/rux/frontend run build`

  Expected: all PASS.

- [ ] **Step 7: Lint and commit.**

```bash
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/miljoe/SampleCard.tsx --tsx
git add apps/rux/frontend/src
git commit -m "refactor(gui): drop the On-site-only sample fields

samples.part_code and the part_code/stage create parameters leave the
backend in v25; remove Sample.part_code, SampleCreate.part_code/stage
and Miljø's \"Udtaget ved\" line that displayed it.

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 4: Frontend docs and the visual check

**Files:**
- Modify: `apps/rux/frontend/README.md:141, 158, 177-181, 186-190, 199-204`
- Modify: `.claude/skills/design-studio/references/reusex-frontend.md:57-58, 76-80, 84-87`

**Interfaces:** none (docs and verification).

- [ ] **Step 1: Update `apps/rux/frontend/README.md`.**
  - Line 141: `Kortlægning ↔ Miljø & prøver ↔ Overblik ↔ On-site deep links` → `Kortlægning ↔ Miljø & prøver ↔ Overblik deep links`. Also add to the `links.ts` description that it holds the route constants, and to the `navigation.ts` description (if it is listed) that it holds the retired-path `REDIRECTS`.
  - Lines 157-158: drop `and\n│ onsite/ (CaptureStage, CaptureSheet, PartPicker)` from the components list. Join the list with `and sager/ (CaseCard) — plus the`.
  - Lines 177-178: the `sager/` row becomes `model.ts (case status, card stats and text, the open-another command)`.
  - Lines 180-182: delete the `onsite/` row (three lines).
  - Line 189: drop `, OnsitePage` from the `routes/` row.
  - Lines 201-204: replace the paragraph with:

```markdown
`/sager` lists the one open case. Below 900px the sidebar is a drawer.
Retired paths (`/on-site`) redirect to Kortlægning; see `REDIRECTS` in
`src/app/navigation.ts`. Materialedata (`/materials`) and Eksport (`/export`)
are off the sidebar and still reachable by URL until Kortlægning and Rapport
take them over.
```

- [ ] **Step 2: Update the design-studio frontend map** (`.claude/skills/design-studio/references/reusex-frontend.md`):
  - delete the `onsite/ CaptureStage, CaptureSheet, PartPicker — On-site's\n phone sheet` lines (57-58);
  - the `sager/` row reads `text, the open-another command)`;
  - delete the `onsite/` pure-module row (78-80);
  - drop `, OnsitePage` from the `routes/` row (87).

- [ ] **Step 3: Run the gates once more.**

  Run:
  - `grep -rniE "on-?site|onsite" apps/rux/frontend/src apps/rux/frontend/README.md .claude/skills/design-studio/references/reusex-frontend.md`;
  - `npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend test -- --run && npm --prefix apps/rux/frontend run build`.

  Expected:
  - the grep prints only the new README sentence about the `/on-site` redirect;
  - typecheck, tests and build all PASS.

- [ ] **Step 4: Visual check (both themes, desktop and phone).**

  Start the dev environment against a *copy* of a scratch project. `dev_env.sh` serves a copy, so never point it at a tracked fixture. Use any v24/v25 `.rux` the worktree has; `$SP/corridor-clouds.rux` is the Phases 3–6 one.

```bash
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
RUX_BIN="$PWD/build/apps/rux/rux" bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/corridor-clouds.rux" 8426 5179
for theme in light dark; do
  bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5179/" --out "$SP/shots/rt2-nav" --theme $theme --viewports desktop,390x844
  bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5179/sager" --out "$SP/shots/rt2-sager" --theme $theme --viewports desktop,390x844
  bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5179/miljoe" --out "$SP/shots/rt2-miljoe" --theme $theme --viewports desktop
done
bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5179/on-site?del=RX-008" --out "$SP/shots/rt2-redirect" --theme light --viewports desktop
bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5179/materials" --out "$SP/shots/rt2-materials" --theme light --viewports desktop
bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5179/export" --out "$SP/shots/rt2-export" --theme light --viewports desktop
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

  Look at every shot and confirm:
  - **Nav (Overblik):** Sag reads `Overblik · Kortlægning · Viewport · Miljø & prøver · Rapport · Indberetning · Skabeloner`, with Skabeloner muted and inert. Værktøjer reads `Projektdata · Posegraf · Pipeline · Kørselslog · Billeder · Geometri · Instanser · Labels`. In the phone shot, the drawer lists the same entries.
  - **Sager:** only the "Åbn en anden sag" panel, with no leftover gap or heading.
  - **Miljø:** sample cards without an "Udtaget ved" line.
  - **Redirect:** the `/on-site?del=RX-008` shot shows Kortlægning.
  - **Old pages:** the `/materials` and `/export` shots show Materialedata and Eksport, not Overblik (R4).

  If `RUX_BIN` isn't built in this worktree, build only the `rux` target first (`cmake --build build --target rux`, timeout 600000 ms, re-run on timeout).

- [ ] **Step 5: Commit.**

```bash
git add apps/rux/frontend/README.md .claude/skills/design-studio/references/reusex-frontend.md
git commit -m "docs(gui): frontend map without On-site, note the redirects

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

## Handover to Phases 3 and 4

- **Redirects:** append to `REDIRECTS` in `apps/rux/frontend/src/app/navigation.ts`, and in the same commit delete the old `<Route>` and its page import from `App.tsx`. The navigation tests then check that the target is a live entry and that no redirect chains.
  - Phase 3: `{ from: '/materials', to: KORTLAEGNING_PATH }`. Also look at `components/InstanceList.tsx:72` (`navigate('/materials')`). It will hit the redirect; retarget it if Kortlægning is the better destination.
  - Phase 4: `{ from: '/export', to: RAPPORT_PATH }`.
- **Skabeloner:** the nav entry already exists with `pending` and `to: SKABELONER_PATH` (`'/skabeloner'`, exported from `app/links.ts`). No route is registered, so the URL falls through to `*` (Overblik) until then. Phase 4:
  1. deletes the `pending` field;
  2. adds `<Route path={SKABELONER_PATH} element={<SkabelonerPage />} />` before the redirects;
  3. changes the test `keeps Skabeloner pending until its page exists` to `expect(entry?.pending).toBeUndefined()`.
- **Fields:** `Sample.part_code` and `SampleCreate.part_code`/`stage` are gone from `api/types.ts`. Phase 3's new resource types start from that file.
