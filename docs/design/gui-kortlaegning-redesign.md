<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# GUI redesign: prototype v2 + Kortlægning workbench

Status: **proposed** (2026-09-30). Anchor: #265 (GUI application).

## Source

A colleague's clickable prototype, *ReUseX Overblik — prototype v2*
(claude.ai artifact `At7n5is3zp7cYX54faGJkn`). It defines one committed visual
identity and six screens:

| # | Screen | What it is |
|---|---|---|
| 01 | **Sager** | Case list: one card per building survey |
| 02a | **Overblik** | Case dashboard: hero, KPIs, circularity bar, quick links, BBR line |
| 02b | **Kortlægning** | The review workbench — the focus of this redesign |
| 02c | **Miljø & prøver** | Environmental samples that gate approval |
| 02d | **Rapport** | Ressourcekortlægning report versions |
| 02e | **Indberetning** | Waste fractions per EAK code for bygningsaffald.dk |
| 03 | **On-site** | A phone "interruption" sheet during capture |

Screenshots of the prototype, rendered headlessly, are kept in
`docs/gui/images/prototype-v2/` for reference.

## Decisions (agreed with the maintainer, 2026-09-30)

1. **Scope: everything in the prototype**, delivered in phases (below).
2. **Themes: light is the default**, the prototype's palette. A navy-derived
   **dark theme is kept** and the Light / Dark / System toggle keeps working.
   The 3D viewport canvas stays near-black in both themes (the prototype does
   the same — its point-cloud stage is `#171C24`).
3. **Language: Danish**, using the prototype's copy, on every screen this
   redesign touches. Technical tool screens not redesigned here (Pipeline,
   Frames, Graph View, …) stay English until they are.
4. **Data: backend fields too.** Kortlægning, Miljø & prøver and Indberetning
   get typed storage in `ProjectDB`, library entry points in `libs/reusex`
   (Library-first, DIRECTION.md), REST endpoints in `rux gui`, and entries in
   `docs/gui/openapi.yaml`. Nothing is smuggled into free-form passport
   properties.

## Visual identity

The token system stays the mechanism: the prototype's values go into
`src/tokens.css`, components keep using `var(--…)` only.

- **Palette (light, default)** — navy chrome `#1D2D3D` / `#24374B` /
  `#35495E`, workbench `#E9E9E7`, surfaces `#FFFFFF` / `#F2F2F0`, ink
  `#1C2530` / `#5A6470` / `#8B939D`, line `#D8D8D4`, accent `#5980A6` /
  `#3E5F80` / `#E4EBF2`.
- **New token roles** the old system could not express (names owned by the
  repo, values by the design project):
  - `--color-chrome`, `--color-chrome-raised`, `--color-chrome-border`,
    `--color-on-chrome`, `--color-on-chrome-muted` — the navy sidebar/title bar.
  - `--color-accent-deep` — accent text on light accent fills.
  - Tone pairs for pills/notices: `--tone-good-bg|ink`, `--tone-warn-bg|ink`,
    `--tone-wait-bg|ink`, `--tone-crit-bg|ink`, `--tone-accent-bg|ink`.
  - `--color-star` — the "vigtig" marker.
  - Waste hierarchy (affaldshierarki), a categorical scale:
    `--circ-bevaring`, `--circ-genbrug`, `--circ-genanvendelse`,
    `--circ-nyttiggoerelse`, `--circ-bortskaffelse`.
  - `--font-display` — condensed display face for headings and big figures.
- **Type** — display: Oswald 500/600/700, uppercase headings; body: Archivo
  400–700 at 13.5px base. **Self-hosted** via `@fontsource/*` (OFL-1.1), never
  from Google Fonts: `rux gui` is local-first and must render offline.
- **Chrome** — a 44px navy title bar and a 13.5rem navy sidebar: `PROJEKT`
  label, project name, the case navigation with count badges, a "Værktøjer"
  group for the existing technical screens, and "← Alle sager" at the bottom.
- The light theme is the `:root` block; `[data-theme='dark']` overrides it.
  With no stored preference the app now starts **light**.
- Correctness constraints kept: `--color-canvas` near-black in both themes;
  `--label-0..7` stays the Okabe-Ito set.

## Information architecture

| Route | Screen | Replaces |
|---|---|---|
| `/sager` | Sager | — (new) |
| `/` | Overblik | Dashboard (Overview) |
| `/kortlaegning` | Kortlægning | Materials as the primary materials view |
| `/miljoe` | Miljø & prøver | — (new) |
| `/rapport` | Rapport | report part of Export |
| `/indberetning` | Indberetning | — (new) |
| `/on-site` | On-site | — (new, phone layout) |
| `/viewport`, `/graph-view`, `/pipeline`, `/pipeline/log`, `/frames`, `/geometry`, `/instances`, `/materials`, `/labels`, `/export` | Værktøjer group | unchanged, restyled by the tokens only |

`/materials` (the free-form passport spreadsheet) stays as a tool: it edits
arbitrary MaterialEPAS properties Kortlægning does not model.

**Sager and one project per server.** `rux gui` serves one `.rux`. The Sager
screen lists the open project as its card and says how to open another
(`rux -p <fil>.rux gui`). Listing and switching between many cases is the
remote/queued server's job (`ruxd`, #265 Phase 6) and gets its own issue; the
screen is built so a longer list drops in without layout change.

## Kortlægning domain model

The prototype's two-level table:

- **Type** (group row) = a new **survey type** entity. It groups many parts —
  "Vinduespartier, aluminium" with RX-008 and RX-009. It cannot be a material
  passport: `rux create materials` writes one passport *per instance*, so a
  passport-as-type would always have exactly one part.
- **Bygningsdel** (child row, `RX-###`) = one **instance** (`instances`), with
  a per-part quantity and room. Its material passport, when the instance has
  one (`instance_materials`), is shown alongside.

New storage (schema v22):

- `survey_types` — name, EAK code, BIM7AA code, unit, treatment (bevaring |
  genbrug | genanvendelse | nyttiggoerelse | bortskaffelse), review status
  (queue | approved | rejected), AI confidence (nullable), mass in tonnes
  (nullable), process note, starred, and the semantic class it was seeded from.
- `survey_parts` — code `RX-###` (primary key), type, instance (by its stable
  GUID — survives `rux create instances` re-runs; nullable so a manually
  added part needs no instance), room id + name, quantity, starred, note.
- `samples` — code `P-##`, title, what was sampled, stage (planlagt |
  udtaget | sendt | svar), result (null | ren | forurenet).
- `sample_links` — many-to-many sample ↔ passport.

Derived, never stored (pure library functions, unit-tested):

- **Miljøstatus** of a type: no linked samples → *ren (screening)*; any linked
  sample with result *forurenet* → *forurenet*; else any sample before stage
  *svar* → *afventer prøve*; else *ren (prøvesvar)*.
- **Approval gate**: a type whose miljøstatus is *afventer prøve* cannot be
  approved — the library throws `SamplePendingError`, the API answers
  `422 Unprocessable Entity`. (Not 409: in this API 409 already means "a
  pipeline job holds the writer lock, retry later", and this is not retryable.)
- **Quantity redistribution**: setting a type's quantity scales its parts
  proportionally (equal split when they are all zero), two decimals, with the
  rounding remainder on the last part.
- **Circularity breakdown**: tonnes per treatment over non-rejected types.
- **Fractions** (Indberetning): approved tonnes aggregated per EAK code;
  blocking rows = types still in the queue or awaiting a sample.

Rejecting a type ("Afvis — fejldetektion") sets status *rejected*; it is hidden
from Til gennemsyn / Godkendt / Alle but not deleted, so it can be restored.

**Populating the survey.** A new idempotent library entry point
`sync_survey(ProjectDB&)` creates a `survey_parts` row for every instance that
lacks one, filed under a survey type per semantic class (created on first use,
named from the semantic cloud's label definitions), with the next `RX-###`
code and the instance's room (from the `rooms` label cloud, majority vote over
the instance's points). It never overwrites user edits and never moves a part
a user re-filed. It is exposed as `rux create survey` and
`POST /api/v1/survey/sync`.

**Evidence renders.** Plan / Punktsky / Rum-model images come from the existing
headless `render_view()` with a new instance-highlight option. `rux_gui_lib`
must not link VTK (same rule as for SAM3), so the rux app injects an
`IViewRenderer`, exactly like `IFrameSegmenter`; without one the endpoint
answers 503.

## Kortlægning screen (the prototype, component by component)

- **Header** — `KORTLÆGNING` + sub "Ressourcekortlægning · bygningsdele pr. rum,
  grupperet pr. type" + `Eksport (XLS)` (the existing CSV export endpoint,
  labelled for spreadsheet use).
- **Coverage notice** (warn tone) — unclassified points from the instance
  label cloud and rooms without scan coverage, from `GET /survey/summary`.
- **Workbench** — table panel left, 21.5rem right column (evidence + detail);
  stacks below 1080px.
- **Tabs** — Til gennemsyn (n) · Godkendt (n) · Alle (n).
- **Tools** — search (Søg bygningsdel…), room filter (Alle rum), miljø filter
  (Al miljøstatus / Ren / Afventer prøve / Forurenet).
- **Keyboard bar** — ↑↓ naviger · →← fold ud/ind · Enter åbn redigering ·
  G godkend · A afvis · V vigtig · 1–4 evidens · Esc tilbage.
- **Table** — columns Betegnelse · Mængde · EAK · BIM7AA · Behandling · Miljø ·
  Status. Group rows: chevron, ★, name, "n dele", quantity + unit + tonnes.
  Child rows: `RX-### · Rum`, quantity, EAK, photo count. Treatment as a
  circularity-coloured pill, miljø as a tone pill, status as a confidence bar
  or "Godkendt ✓". Sticky header, keyboard-focusable, selected row with an
  accent inset bar.
- **Evidence panel** — tabs Plan · Foto · Punktsky · Rum-model:
  - *Plan*: server-rendered floor plan (`render_view`, `plan` preset) with the
    selected instance highlighted.
  - *Foto*: no nearest-panorama endpoint exists yet, so this tab substitutes a
    regular sensor-frame photo ("Bedste foto") of the selected part's instance
    — or of the first linked part's instance, for a type row — not a true 360°
    view. Swap in a real 360° tab once the endpoint lands (see follow-ups).
  - *Punktsky*: server-rendered orbit view of the cloud, instance highlighted,
    plus "Åbn i viewport".
  - *Rum-model*: server-rendered view of the `rooms` layer.
- **Detail panel** — title, BIM7AA pill, miljø pill, ★ pill; Mængde (editable;
  on a type it is redistributed proportionally over the parts), EAK, Behandling
  (select), Sikkerhed (AI); sample line (plain text today — "Miljøstatus
  styres af P-01 · PCB i fugemasse" — not yet a link to Miljø & prøver, since
  that screen doesn't exist until Phase 4); Proces / håndtering note;
  ☆ Markér vigtig, Afvis, Godkend mængde ✓ (disabled with a gate note while a
  sample is pending), Genåbn.
- **Edit dialog** (Enter / double-click) — navy header with prev/next/close;
  left form (part chips, quantity, EAK, behandling, sikkerhed, sample line,
  note, photos = the instance's best frames, ☆); right evidence with four
  numbered thumbnails and a large stage; footer shortcuts, gate note, Afvis,
  Godkend & næste ✓. ⌘/Ctrl+Enter approve-and-next, PgUp/PgDn move, 1–4
  switch view, Esc close.
- **Toast** after approve/reject: "✓ <type> godkendt · n tilbage i køen".

## Phases

Each phase is a separate PR that leaves the app working.

1. **Identity & shell** — tokens, fonts, theme default, navy title bar and
   sidebar, routes, Danish labels; existing screens restyled by tokens.
2. **Survey backend** — schema v22 (survey types, parts, samples, links),
   library model + derivations + approval gate, `sync_survey`,
   `rux create survey`, REST endpoints, render endpoint with instance
   highlight, openapi.yaml, and the frontend contract layer (types + client).
3. **Kortlægning frontend** — the workbench, detail panel, edit dialog,
   keyboard flow, evidence panel.
4. **Miljø & prøver** — the samples screen (stage chain, links, result entry)
   on the Phase 2 API.
5. **Overblik, Rapport, Indberetning** — KPIs, circularity, report versions on
   the existing report endpoints, fraction table and send gate.
6. **Sager & On-site** — case list (single-project), phone capture sheet
   writing ★ / note / sample against a bygningsdel.

## Out of scope / follow-up issues

- Multi-project case list and switching (`ruxd`, #265 Phase 6).
- bygningsaffald.dk API submission (v1 produces the numbers in the portal's
  structure; the send button is gated but posts nowhere).
- BBR lookup (the prototype marks it "Demodata"; shown only when project
  metadata carries a BFE number).
- Lab integration (e.g. Milva) for automatic sample results.
- Pushing the new token values to the Claude Design project ("ReUseX GUI") with
  `/design-sync` after Phase 1 merges — a maintainer-initiated step.
- A nearest-panorama endpoint, so the evidence panel's Foto tab can become a
  true 360° tab instead of substituting a sensor-frame photo.
- Linking the detail panel's sample line to the Miljø & prøver screen (Phase 4)
  instead of rendering it as plain text.
- A survey-specific export — "Eksport (XLS)" currently downloads the existing
  material-passport CSV (`/exports/csv`), not a Kortlægning-shaped spreadsheet.
- Two pieces of the structure above that v1 does not draw yet: the child
  rows' photo count and the Punktsky tab's "Åbn i viewport" link.
- A `--border-width` token: the tab underline offset in `SurveyTable.module.css`
  and `EvidencePanel.module.css` currently computes it as `calc(-1 * 1px)`
  because no border-width token exists yet.
