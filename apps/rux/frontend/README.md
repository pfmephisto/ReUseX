<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX GUI frontend

The web frontend served by `ruxd --local` (formerly `rux gui`,
[issue #265](https://github.com/pfmephisto/ReUseX/issues/265)) — a Vite +
React + TypeScript single-page app that renders a `.rux` project: its summary,
pipeline log, stages, jobs, and a three.js point-cloud viewport. Phase 2 built
the shell, dashboard and viewport; Phase 3 ([#305]) added the pipeline runner —
stage cards with server-described parameter forms, run/cancel with live
progress, and the durable history timeline.

[#305]: https://github.com/pfmephisto/ReUseX/issues/305

It is a **pure client of the shared API contract** in
[`docs/gui/openapi.yaml`](../../../docs/gui/openapi.yaml) and
[`docs/gui/websocket-events.md`](../../../docs/gui/websocket-events.md), never
of a particular server. `ruxd` implements that contract: today `ruxd --local`
serves one project; multi-case routes (`/api/v1/cases/{cid}/…`) and users
follow (spec `docs/superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md`).
The `rux gui` subcommand that used to serve it is gone.

## Development

```bash
nix develop                 # provides nodejs_22
npm install
npm run dev                 # http://localhost:5173  (or: gui-dev)
```

In another terminal, start the server the dev app talks to:

```bash
ruxd --local scan.rux        # http://127.0.0.1:8420; --port to change
```

Or let `.claude/skills/design-studio/scripts/dev_env.sh start [project.rux]`
start both halves against a throwaway copy of the project.

The Vite dev server proxies `/api` — REST **and** the `/api/v1/events`
WebSocket — to `http://localhost:8420`. Override the target with `RUX_GUI_URL`:

```bash
RUX_GUI_URL=http://localhost:9000 npm run dev
```

**The proxy is mandatory, not a convenience.** `ruxd` cannot answer a CORS
preflight — Crow 1.3 replies to `OPTIONS` from `Router::handle_initial()`,
before the request headers are parsed, so the server never sees the `Origin`
and cannot emit a correct preflight response — which means any JSON-bodied
cross-origin call from a bare `vite dev` would fail. Proxying makes the browser
talk only to the Vite origin, so CORS never enters into it. See
[`docs/gui/README.md`](../../../docs/gui/README.md) § "The frontend must be
same-origin".

For Kortlægning work, seed the prototype's demo survey into a scratch copy:
`bash dev/seed-survey-demo.sh <project.rux> /tmp/kort-demo.rux`, then
`ruxd --local /tmp/kort-demo.rux`. Never run it on a
real project.

For Miljø & prøver work, add `--varied`
(`bash dev/seed-survey-demo.sh --varied <project.rux> /tmp/miljoe-demo.rux`).
This seeds two extra samples, one answered and linked to two types and one
planned and unlinked, on top of the prototype's three. Screenshots against
`miljoe.png` use the plain seed. It also seeds three Ressourcekortlægning
versions (v1 and v2 drafts, v3 complete) with stub PDFs, for Rapport. Shots
against `overblik.png`, `rapport.png` and `indberetning.png` use the plain
seed.

## Production

```bash
npm run build               # tsc --noEmit && vite build  ->  dist/
ruxd --local scan.rux --assets ./dist
```

`ruxd --local` resolves its asset directory as `--assets`, then
`$RUX_GUI_ASSETS`, then `<install prefix>/share/reusex/gui`
(`apps/ruxd/src/api/assets.cpp`). The Nix build takes the third path:
`pkgs/reusex-gui-frontend/package.nix` builds this bundle as its own derivation
and `default.nix` copies it into `$out/share/reusex/gui`, so
`nix shell .#default -c ruxd --local scan.rux` serves a UI with no flags. Keeping it a separate derivation is deliberate — a lockfile
change must not invalidate the multi-hour C++ build, and a `.cpp` edit must not
re-run npm.

## Testing

```bash
npm test                    # vitest run
npm run test:watch
```

Frontend tests run under **npm only — they are not part of `ctest`**, and that
is intentional: wiring vitest into CTest would put a Node toolchain on the
critical path of the C++ test suite, which must stay buildable and runnable
from a plain `nix develop` + `cmake` + `ctest` with no npm anywhere in it. CI
runs them as a separate `frontend` job in `.github/workflows/ci.yml`.

Tests cover the pure-logic modules (API client with an injected `fetch`, the
event reducer, the chunk/pagination state machine, the stage-card view model,
the parameter form and the history timeline) against the recorded payloads in
`src/test/fixtures.ts`. Those are captured verbatim from a real server (then
`rux gui`, now `ruxd --local`) serving `tests/fixtures/scans/office_corridor.rux` — re-record them when the
contract changes, never hand-edit. No DOM environment is configured, so no
jsdom dependency is carried, and nothing under `pipeline/` may reach for one.

## Design tokens

`src/tokens.css` is **owned by the Claude Design project "ReUseX GUI"**
(project id `19f0cf0f-1f7c-43a7-8311-1fd67e34bbb7`). Its values are the
prototype-v2 identity (`docs/design/gui-kortlaegning-redesign.md`), adopted
before the first `/design-sync`; push them to the design project with
`/design-sync` rather than hand-tuning. This repo owns the token *names* and
their roles; the design project owns every value on the right-hand side.

The corollary is a hard rule for everything else under `src/`: no colour,
radius, spacing or type size is ever written literally in a component, only
`var(--...)`. That is what makes the first real sync a value swap rather than a
refactor.

Fonts (Oswald, Archivo) are bundled from `@fontsource/*` in `src/fonts.ts`;
the app never fetches fonts at runtime.

## Layout

```
src/
├── api/          Contract-facing layer: types.ts (generated-by-hand mirrors of
│                 openapi.yaml), client.ts (typed fetch wrapper), events.ts
│                 (WebSocket channel + reconnect)
├── app/          App shell, routing, cross-cutting state (JobsContext,
│                 SurveyCountsContext — drives both sidebar badges, the
│                 review queue and pending samples), useAsync, useToast, serialQueue.ts (a promise
│                 chain whose tasks settle in commit order; idle() waits for them),
│                 writeChain.ts (the one app-wide chain every page's writes
│                 join and the case screens' first loads wait for),
│                 useMutationQueue.ts (a page's writes on that chain; `busy`
│                 gates buttons only, field commits are never dropped),
│                 keyTargets.ts (classifies a key event's target so page
│                 shortcuts never fire while typing), links.ts (the
│                 Kortlægning ↔ Miljø & prøver ↔ Overblik deep links,
│                 segmentHref for a Segmentering frame + seed pixel, and
│                 route constants), useSam3.ts (the managed SAM3 model's
│                 status chip and runWithProvisioning for one view),
│                 saveError.ts (Danish copy for a failed
│                 save: 409/503 get their own message, else the server's),
│                 errorCopy.ts (Danish copy for a failed *load*, keyed by a
│                 definite noun phrase naming what didn't load),
│                 editorKeys.ts (Esc/Enter inside an in-place editor),
│                 textDraft.ts (what a commit-on-blur field sends; pure)
│                 and useTextDraft.ts (the hook over it, used by Miljø &
│                 prøver and Overblik)
├── components/   Presentational, contract-agnostic building blocks
│                 (DataTable, StatCard, Sidebar, JobToaster, Pill,
│                 ConfidenceBar, Kbd, Toast, CircularityBar, ...), plus
│                 kortlaegning/ (SurveyTable, EvidencePanel, DetailPanel,
│                 EditDialog, SampleLine), miljoe/ (StageChain, LinkPicker,
│                 SampleCard, NewSampleForm), overblik/ (CaseHero, KpiRow,
│                 QuickLinks, ProjectMetaForm), skabeloner/ (TemplateList,
│                 TemplateEditor, ColumnList), rapport/ (DataExportPanel,
│                 TemplateSelect, VersionList), indberetning/ (FractionTable),
│                 sager/ (CaseCard) and segmentering/ (Filmstrip,
│                 SegmentStage, ResourceDialog), Sam3StatusChip (the SAM3
│                 model's provisioning chip) — plus the shared controls.module.css
│                 (buttons, fields and the crossLink every cross-screen link
│                 composes) and surfaces.module.css (panels and notices)
│                 every case screen composes from
├── data/         Pure data modules for the technical views, among them
│                 segmentView.ts (the Segmentering view: filmstrip, prompts
│                 in image pixels, the request, result list, mask overlay,
│                 resource-dialog choices), sam3Provisioning.ts (managed
│                 SAM3 status copy and the 503 → poll → retry flow),
│                 labelPng.ts (decodes the 16-bit segmentation PNG),
│                 labelQueue.ts (the batch label queue) and
│                 segmentPrompts.ts (the point marker size)
├── kortlaegning/ Pure modules for the Kortlægning workbench: vocab.ts
│                 (Danish labels, number formatting), model.ts (tabs,
│                 filters, selection, row flattening, initialViewFor for a
│                 `?type=` deep link), samples.ts (a type's linked samples
│                 and its sample line), keys.ts (keyboard maps),
│                 segmentLink.ts (the S key: a part's best frame and pixel
│                 for Segmentering)
├── miljoe/       Pure modules for Miljø & prøver: model.ts (stage chain,
│                 patches, link toggling, gate feedback)
├── overblik/     Pure module for Overblik: model.ts (circularity percents,
│                 the KPI row, quick links, the hero's subline and its
│                 metadata-editor commits)
├── skabeloner/   Pure modules for Skabeloner: model.ts (template CRUD copy,
│                 the duplicate/rename/delete flow, the resolved-count line),
│                 members.ts (category/key editing, the local port of the
│                 server's resolve_template, catalogue search), columns.ts
│                 (Egne felter: rename, option-list and delete copy for user
│                 columns)
├── rapport/      Pure modules for Rapport: model.ts (version date/size
│                 formatting, the Komplet/Udkast pill from the stored
│                 blocking count, the draft notice and generation toasts, the
│                 Ressourcetabel choice), csvOptions.ts (the Data-eksport CSV
│                 options and download-filename parsing)
├── indberetning/ Pure module for Indberetning: model.ts (the fraction table,
│                 the blocking-list notice, the send gate)
├── sager/        Pure module for Sager: model.ts (case status, card stats and
│                 text, the open-another command)
├── pipeline/     Pure stage-runner logic: the card view model (stageModel),
│                 parameter-form parsing and the omit-defaults submit rule
│                 (params), and the pipeline_log timeline (history)
├── routes/       Page-level compositions (OverblikPage, PipelinePage, ViewportPage,
│                 KortlaegningPage, SegmenteringPage, MiljoePage,
│                 SkabelonerPage, RapportPage, IndberetningPage, SagerPage),
│                 plus the shared
│                 viewHead.module.css a case screen's header composes from
├── viewport/     three.js point-cloud rendering: PointCloudScene, the paged
│                 cloud stream (useCloudStream, pagination), point decoding
│                 (decode.ts) and label colour mapping (labelColors.ts)
├── test/         Fixtures shared by the vitest suites
├── tokens.css    Design tokens — see above, do not edit by hand
└── base.css      Global reset / element defaults, built on the tokens
```

`/` is Overblik, the case landing page; the technical inventory (the old
Projektdata screen) is its closed "Projektdata" section at the foot, and
`/projektdata` redirects to `/`. The sidebar's Værktøjer group is collapsed by
default and opens itself on a tools route.
`/sager` lists the one open case. `/segmentering` (Segmentering, in the case
group after Viewport) runs SAM3 on one frame with text, box and point prompts
and files a mask as a new resource; Billeder's old segment fold-out is now a
link there. Below 900px the sidebar is a drawer.
Retired paths (`/on-site`, `/onsite`, `/materials`, `/export`, `/projektdata`)
redirect to Kortlægning (the first three), Rapport (`/export`) or Overblik
(`/projektdata`); see `REDIRECTS` in
`src/app/navigation.ts`. Materialedata (`MaterialsPage`, `MaterialTable`) is
gone — its data lives under Alle egenskaber on Kortlægning now. Eksport
(`ExportPage`) is gone too — Rapport's Data-eksport panel (`DataExportPanel`)
replaced it, backed by Skabeloner (`/skabeloner`, `SkabelonerPage`), which
manages the resource templates both screens read.
