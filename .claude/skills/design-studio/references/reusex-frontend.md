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
3. Theme model (dark-first, three-way toggle)
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
│                 LabelQueueContext), useAsync, useTheme
├── components/   Presentational, contract-agnostic blocks — reuse these:
│                 DataTable, StatCard, NavRail, EmptyState, ErrorBanner,
│                 ParameterForm, SelectDropdown, MultiSelectDropdown,
│                 JobToaster, LayerPanel, PeekPanel, LabelLegend, …
├── pipeline/     Pure stage-runner logic (stageModel, params, history)
├── routes/       Page compositions: Dashboard, ViewportPage, PipelinePage,
│                 PipelineLogPage, GraphViewPage, FramesPage, DataPage,
│                 GeometryPage, InstancesPage, MaterialsPage, ExportPage
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

## 2. The token system (full inventory)

`src/tokens.css` defines these on `:root` (dark) with a `[data-theme='light']`
override for chrome only. **Reference them by name; never write the value.**

- **Surfaces:** `--color-canvas` (3D viewport bg, near-black in both themes),
  `--color-surface`, `--color-surface-raised`, `--color-surface-overlay`,
  `--color-surface-sunken`
- **Border:** `--color-border`, `--color-border-strong`, `--color-border-focus`
- **Text:** `--color-text`, `--color-text-muted`, `--color-text-faint`,
  `--color-text-inverse`
- **Accent:** `--color-accent`, `--color-accent-hover`, `--color-accent-muted`,
  `--color-on-accent`
- **Status (job/stage lifecycle, distinct in luminance too):**
  `--color-status-queued|running|succeeded|failed|cancelled`
- **Categorical labels (Okabe-Ito, colourblind-safe — a correctness constraint):**
  `--label-0`…`--label-7`, `--label-count`, `--label-unlabeled`
- **Viewport geometry:** `--mesh-surface` (mid-gray albedo for untextured mesh so
  it reads against the near-black canvas)
- **Type:** `--font-sans`, `--font-mono` (tables are mono by design — figures
  align), `--font-size-xs|sm|md|lg|xl|2xl`, `--font-weight-regular|medium|bold`,
  `--line-height-tight|normal`
- **Space:** `--space-0`…`--space-7` (0, .25, .5, .75, 1, 1.5, 2, 3 rem)
- **Radius:** `--radius-sm|md|lg|pill`
- **Shadow:** `--shadow-sm|md|lg` (lighter alpha in the light theme)
- **Layout:** `--layout-titlebar-height`, `--layout-nav-width`,
  `--layout-panel-width`
- **Motion:** `--duration-fast|normal`, `--easing-standard` (both durations zero
  out under `prefers-reduced-motion`)
- **Z-index:** `--z-panel|titlebar|toast`

If a design needs something outside this set, that's a **new token/role for the
design project**, not a literal. Flag it in hand-off.

## 3. Theme model

Dark-first. `<html data-theme>` carries the *resolved* value (`light`|`dark`);
the user *preference* is `system`|`light`|`dark`, stored in `localStorage` under
`reusex-theme`. `index.html` has a FOUC-guard inline script that applies the
resolved theme before first paint — it's a hand-mirror of `theme.ts`, keep the
key/rule in sync. `useTheme()` owns the live sync; mount exactly one instance.

For screenshots, force the app's preference with `screenshot.py --theme
light|dark|system` (it seeds `localStorage['reusex-theme']` before load). Always
review **both** themes — the light block only re-points chrome, so a component
that hardcodes a colour will look right in one and wrong in the other.

## 4. Dev environment and the proxy

```bash
nix develop
npm --prefix apps/rux/frontend install
rux -p tests/fixtures/scans/office_corridor.rux gui --port 8420 --no-browser  # backend
npm --prefix apps/rux/frontend run dev                                         # http://localhost:5173
```

`scripts/dev_env.sh start` does both and prints the URLs; `stop` tears them down.

The Vite dev server proxies `/api` (REST **and** the `/api/v1/events` WebSocket)
to `http://localhost:8420` (override with `RUX_GUI_URL`). **The proxy is
mandatory:** `rux gui` (Crow 1.3) answers `OPTIONS` before parsing headers, so it
can't do a CORS preflight — a bare cross-origin call fails. Proxying keeps the
browser same-origin. Screenshot `http://localhost:5173/...`, never `:8420`
directly.

Production: `npm run build` → `dist/`, served by `rux gui --assets ./dist` (Nix
build wires this into `$out/share/reusex/gui` as a separate derivation).

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
