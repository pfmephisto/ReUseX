<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX rux GUI — frontend reference

Deep specifics for the `design-studio` skill. Read once before your first build
in `apps/rux/frontend/`; skim later for the token names.

## Table of contents
1. Stack and directory layout
2. The token system (full inventory)
3. Theme model (light-default, three-way toggle)
4. Dev environment and the proxy (why it's mandatory)
5. The API/WS contract — the frontend is a pure client
6. Testing
7. Qt — the second surface

---

## 1. Stack and directory layout

React 19 + Vite + TypeScript + three.js, CSS Modules. Node comes from
`nix develop` (nodejs 22; there's a `gui-dev` helper in `shell.nix`).

```
apps/rux/frontend/src/
├── api/          Contract layer: types.ts (hand-mirrors openapi.yaml),
│                 client.ts (typed fetch), events.ts (WS + reconnect)
├── app/          App shell, routing, cross-cutting state (JobsContext,
│                 LabelQueueContext, SurveyCountsContext), useAsync,
│                 useTheme, serialQueue + useMutationQueue (a page's writes
│                 on one chain; `busy` gates buttons, never field commits),
│                 keyTargets (isField/isControl), saveError (failed-save
│                 copy), errorCopy (failed-*load* copy, `ErrorBanner`'s),
│                 links (Kortlægning ↔ Miljø & prøver ↔ Overblik deep links
│                 and route constants), editorKeys (in-place editor keys),
│                 textDraft (commit-on-blur rule, pure) + useTextDraft (its
│                 hook, shared by Miljø & prøver and Overblik)
├── components/   Presentational, contract-agnostic blocks — reuse these:
│                 DataTable, StatCard, Sidebar, EmptyState, ErrorBanner,
│                 ParameterForm, SelectDropdown, MultiSelectDropdown,
│                 JobToaster, LayerPanel, PeekPanel, LabelLegend, Pill,
│                 ConfidenceBar, Kbd, Toast, CircularityBar, …
│                 kortlaegning/  SurveyTable, EvidencePanel, DetailPanel,
│                 EditDialog, SampleLine — the Kortlægning workbench's presentational
│                 layer, built on the shared Pill/ConfidenceBar/Kbd/Toast above
│                 miljoe/  StageChain, LinkPicker, SampleCard, NewSampleForm —
│                 Miljø & prøver's presentational layer, same shared blocks
│                 overblik/  CaseHero, KpiRow, QuickLinks, ProjectMetaForm —
│                 Overblik's case hero, KPI row and quick-link cards
│                 rapport/  VersionList — Rapport's version list
│                 indberetning/  FractionTable — Indberetning's fraction table
│                 sager/  CaseCard — Sager's case card
│                 controls.module.css (buttons and fields) and
│                 surfaces.module.css (panels and notices) are the shared
│                 CSS every case screen's own components `composes` from
├── kortlaegning/ Pure modules behind the workbench: vocab.ts (Danish labels,
│                 number formatting), model.ts (tabs, filters, selection, row
│                 flattening, initialViewFor), samples.ts (a type's linked
│                 samples, the sample line), keys.ts (tableAction/dialogAction
│                 keyboard maps)
├── miljoe/       Pure modules behind Miljø & prøver: model.ts (stage chain,
│                 patches, link toggling, gate feedback)
├── overblik/     Pure module behind Overblik: model.ts (circularity percents,
│                 KPI row, quick links, hero subline, metadata-editor commits)
├── rapport/      Pure module behind Rapport: model.ts (version date/size
│                 formatting, Komplet/Udkast from the stored blocking count,
│                 draft notice, generation toasts)
├── indberetning/ Pure module behind Indberetning: model.ts (fraction table,
│                 blocking-list notice, send gate)
├── sager/        Pure module for Sager: model.ts (case status, card stats and
│                 text, the open-another command)
├── pipeline/     Pure stage-runner logic (stageModel, params, history)
├── routes/       Page compositions: OverblikPage, Dashboard (now at
│                 `/projektdata`), ViewportPage, PipelinePage,
│                 PipelineLogPage, GraphViewPage, FramesPage, DataPage,
│                 GeometryPage, InstancesPage, MaterialsPage, ExportPage,
│                 KortlaegningPage, MiljoePage, RapportPage, IndberetningPage,
│                 SagerPage
│                 (KortlaegningPage and MiljoePage use app/SurveyCountsContext
│                 for the two sidebar badges: review queue and pending
│                 samples), plus viewHead.module.css, the shared header CSS
│                 every case screen composes from
├── viewport/     three.js: PointCloudScene, MeshScene, PanoramaScene,
│                 PoseGraphScene, SplatScene, clipping box, camera views,
│                 label colours, paged cloud stream (useCloudStream)
├── theme.ts      Pure theme model (resolveTheme, storage) — DOM-free core
├── tokens.css    Design tokens — OWNED BY the design project, do not edit
└── base.css      Global reset built on the tokens
```

Every component is `Foo.tsx` + `Foo.module.css`. New `.cpp`-style file placement
rule: presentational → `components/`, page → `routes/`, 3D → `viewport/`, pure
logic → `pipeline/` or a sibling `*.ts`. SPDX header on every file.

Page writes go through `app/useMutationQueue` — never a second ad-hoc chain.

A case screen's own CSS (Kortlægning, Miljø & prøver, Overblik, Rapport,
Indberetning) `composes` from `components/controls.module.css`,
`components/surfaces.module.css` and `routes/viewHead.module.css` —
never copy a button or panel rule into a new module.

Esc in a text field reverts the draft without committing (R10); Esc
elsewhere on a case screen closes the open panel/dialog.

`ErrorBanner`'s `context` prop is a Danish definite noun phrase (e.g.
"projektoversigten"), never an English fragment or an indefinite noun — both
of `explainLoadError`'s sentences read it inline.

Page writes join the app-wide `appWriteChain` (`useMutationQueue`); a case
screen's first load starts with `appWriteChain.idle()`. Only a page whose
writes change no survey state and run long (Rapport) passes `scope: 'page'`.

A link to another case screen composes `crossLink` from `controls.module.css`
— never a local link colour.

Below 900px the shell's sidebar is a drawer (`AppShell`, `Sidebar`,
`TitleBar`); a new case screen must not overflow `<main>` at 390px — wrap it
or scroll it inside its own panel.

## 2. The token system (full inventory)

`src/tokens.css` defines these on `:root` (light, the default theme) with a
`[data-theme='dark']` override re-pointing the themed roles. **Reference them
by name; never write the value.**

- **Surfaces:** `--color-canvas` (3D viewport bg, near-black in both themes),
  `--color-surface`, `--color-surface-raised`, `--color-surface-overlay`,
  `--color-surface-sunken`, `--color-scrim` (modal backdrop)
- **Chrome (navy title bar + sidebar):** `--color-chrome`,
  `--color-chrome-raised`, `--color-chrome-border`, `--color-on-chrome`,
  `--color-on-chrome-muted`
- **Border:** `--color-border`, `--color-border-strong`, `--color-border-focus`
- **Text:** `--color-text`, `--color-text-muted`, `--color-text-faint`,
  `--color-text-inverse`
- **Accent:** `--color-accent`, `--color-accent-hover`, `--color-accent-muted`,
  `--color-accent-deep` (accent text, filled primary buttons), `--color-on-accent`,
  `--color-star` ("vigtig" ★)
- **Tone (bg/ink pairs for pills and notices):**
  `--tone-good|warn|wait|crit|accent-bg|ink`
- **Categorical: affaldshierarki (waste hierarchy, ranked best→worst):**
  `--circ-bevaring`, `--circ-genbrug`, `--circ-genanvendelse`,
  `--circ-nyttiggoerelse`, `--circ-bortskaffelse`
- **Categorical: chips (select/multiselect tag colours, ≥4.5:1 in both
  themes):** `--chip-blue|red|green|purple|yellow|gray-bg|ink`
- **Status (job/stage lifecycle, distinct in luminance too):**
  `--color-status-queued|running|succeeded|failed|cancelled`
- **Categorical labels (Okabe-Ito, colourblind-safe — a correctness constraint):**
  `--label-0`…`--label-7`, `--label-count`, `--label-unlabeled`
- **Viewport geometry:** `--mesh-surface` (mid-gray albedo for untextured mesh so
  it reads against the near-black canvas)
- **Type:** `--font-display` (Oswald, headings/eyebrows), `--font-sans`,
  `--font-mono` (tables are mono by design — figures align),
  `--font-size-2xs|xs|sm|md|lg|xl|2xl|3xl`,
  `--font-weight-regular|medium|bold`, `--line-height-tight|normal`,
  `--tracking-caps` (uppercase labels), `--tracking-wide` (sidebar eyebrow)
- **Space:** `--space-0`…`--space-7` (0, .25, .5, .75, 1, 1.5, 2, 3 rem)
- **Radius:** `--radius-sm|md|lg|xl|pill`
- **Shadow:** `--shadow-sm|md|lg|panel` (lighter alpha in the light theme)
- **Layout:** `--layout-titlebar-height`, `--layout-nav-width`,
  `--layout-panel-width`, `--layout-bench-aside-width` (Kortlægning's
  evidence/detail column)
- **Motion:** `--duration-fast|normal`, `--easing-standard` (both durations zero
  out under `prefers-reduced-motion`)
- **Z-index:** `--z-panel|titlebar|toast`

If a design needs something outside this set, that's a **new token/role for the
design project**, not a literal. Flag it in hand-off.

## 3. Theme model

Light is the default theme. `<html data-theme>` carries the *resolved* value
(`light`|`dark`); the user *preference* is `light`|`dark`|`system` (`light` is
`DEFAULT_THEME_PREFERENCE`), stored in `localStorage` under `reusex-theme`
(`dark` and `system` are honoured when explicitly stored). `index.html` has a
FOUC-guard inline script that applies the resolved theme before first
paint — it's a hand-mirror of `theme.ts`, keep the key/rule in sync; both fall
back to `light` when nothing valid is stored or `matchMedia` is unavailable.
`useTheme()` owns the live sync; mount exactly one instance.

For screenshots, force the app's preference with `screenshot.py --theme
light|dark|system` (it seeds `localStorage['reusex-theme']` before load). Always
review **both** themes — the dark block only re-points the themed roles, so a
component that hardcodes a colour will look right in one and wrong in the
other.

## 4. Dev environment and the proxy

```bash
nix develop
npm --prefix apps/rux/frontend install
rux -p tests/fixtures/scans/office_corridor.rux gui --port 8420 --no-browser  # backend
npm --prefix apps/rux/frontend run dev                                         # http://localhost:5173
```

`scripts/dev_env.sh start [project.rux]` does both and prints the URLs; `stop` tears them down from any shell. It serves a fresh **copy** of the project (`.superpowers/dev-env/project/`), never the file you name — `rux gui` migrates and leaves -wal/-shm beside whatever it opens. Do not run the bare `rux -p tests/fixtures/... gui` line above against the tracked fixture; copy it first.

The Vite dev server proxies `/api` (REST **and** the `/api/v1/events` WebSocket)
to `http://localhost:8420` (override with `RUX_GUI_URL`). **The proxy is
mandatory:** `rux gui` (Crow 1.3) answers `OPTIONS` before parsing headers, so it
can't do a CORS preflight — a bare cross-origin call fails. Proxying keeps the
browser same-origin. Screenshot `http://localhost:5173/...`, never `:8420`
directly.

Production: `npm run build` → `dist/`, served by `rux gui --assets ./dist` (Nix
build wires this into `$out/share/reusex/gui` as a separate derivation).

For Kortlægning screenshots/manual testing, seed the prototype's demo survey
(Måløv Byvej 229 — 11 types, 18 parts, 3 samples; 7 in the review queue,
4 approved) into a scratch copy with `dev/seed-survey-demo.sh <in.rux>
<out.rux>`, then point `rux gui` at the copy. Never run it on a real project.
`--varied` adds two samples for Miljø & prøver (multi-link answered, unlinked
planned). `--varied` also seeds three report versions (v1 and v2 drafts, v3
complete) for Rapport, a ★ part with a note (RX-014) and P-06 taken at
RX-013.

## 5. The API/WS contract

The frontend is a **pure client of the shared contract**, not of a particular
server: `docs/gui/openapi.yaml` + `docs/gui/websocket-events.md`
(+ `docs/gui/binary-points.md` for the point stream). `rux gui` implements it
today; `ruxd` will implement the same paths later (issue #265, Phase 6) and the
frontend must not be able to tell which. So when a new view needs data:
- If the endpoint exists, mirror its shape into `src/api/types.ts` exactly.
- If it doesn't, design against the *intended* shape, stub it, and flag the
  missing endpoint in hand-off — do not invent fields silently.

## 6. Testing

`npm --prefix apps/rux/frontend test` (vitest) — **not** part of `ctest` (keeping
Node off the C++ test critical path is deliberate). CI runs it as a separate
`frontend` job. Tests cover pure-logic modules against recorded fixtures in
`src/test/fixtures.ts` (captured from a real `rux gui` on
`office_corridor.rux` — re-record, never hand-edit). No DOM env is configured, so
presentational components aren't unit-tested — that's what §5 screenshots are
for. Always `npm run typecheck` before hand-off.

## 7. Qt — the second surface

Issue #265 chose the web GUI, with Qt 6 as the native fallback/companion. It's
not built yet. The design intent: **one design system, two renderers.** The token
*names and roles* in `tokens.css` are the cross-surface contract; a future Qt
theme is meant to be **generated from them** (into a `QPalette` + QSS), not
re-picked by eye.

Practical consequence for design work **today**:
- Any visual decision that lives only as a literal in a `.module.css` is
  invisible to that generator and will make the Qt client drift from the web
  look. Keep decisions as tokens.
- When you need a concept the current tokens can't express, add a *named*
  token/role (flag it for the design project) instead of a one-off value.
- Layout tokens (`--layout-*`) and the semantic status/label scales are exactly
  the kind of thing both surfaces must agree on — treat them as shared API.
