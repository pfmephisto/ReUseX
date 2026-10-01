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
`docs/gui/images/prototype-v2/` for reference. `rapport.png` and
`indberetning.png` were added in Phase 5, rendered the same way from the
artifact.

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
| `/projektdata` | Projektdata | the old Dashboard/inventory content, moved here (Phase 5) |
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
- `sample_links` — many-to-many sample ↔ survey type. (Originally written as
  "passport"; miljøstatus and the approval gate are properties of a type, so
  the link is to the type — as implemented in Phase 2.)

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
- **Fractions** (Indberetning): approved tonnes per (EAK code, behandling,
  contaminated). *Bevaring* never counts: it stays in the building, so it is
  not waste. A type awaiting a sample is withheld even when approved.
  Contaminated tonnes get their own row. The **blocking list** holds every
  non-rejected type still in the queue (`review`), awaiting a sample
  (`sample`), or approved without tonnes (`mass`), and the report is ready
  when that list is empty and there is at least one fraction row (an empty
  survey has nothing to report).

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
  (select), Sikkerhed (AI); sample line, each sample a link to its card in
  Miljø & prøver (or `Registrér prøve` when none is linked); Proces /
  håndtering note;
  ☆ Markér vigtig, Afvis, Godkend mængde ✓ (disabled with a gate note while a
  sample is pending), Genåbn.
- **Edit dialog** (Enter / double-click) — navy header with prev/next/close;
  left form (part chips, quantity, EAK, behandling, sikkerhed, sample line,
  note, photos = the instance's best frames, ☆); right evidence with four
  numbered thumbnails and a large stage; footer shortcuts, gate note, Afvis,
  Godkend & næste ✓. ⌘/Ctrl+Enter approve-and-next, PgUp/PgDn move, 1–4
  switch view, Esc close.
- **Toast** after approve/reject: "✓ <type> godkendt · n tilbage i køen".

## Miljø & prøver screen (the prototype, component by component)

- **Chrome** — the same navy title bar and sidebar; Miljø & prøver carries a
  neutral count badge (not the "hot" accent Kortlægning's review queue uses).
- **View head** — `MILJØ & PRØVER` + sub "Prøver styrer miljøstatus på de
  koblede bygningsdele", and `+ Ny prøve`.
- **Sample cards** — a vertical stack, one per sample: a title row (code +
  title, a stage/result pill, "Koblet: <types>"), then what was sampled on
  its own line, a
  stage chain (Planlagt — Udtaget — Sendt til lab — Svar modtaget) and an
  action row that depends on stage: *sendt* offers `Registrér svar: Ren` /
  `Registrér svar: Forurenet`; *udtaget* offers `Næste trin →`; *svar* with a
  result shows only a note that miljøstatus has been updated.
- **Footnote** — factual, not a sketch note: a prøvesvar updates miljøstatus
  on every linked type at once; lab integration (e.g. Milva) is a follow-up.

v1 adds, beyond the prototype:

- an inline `Rediger` editor (title, what, links) opened from the `Koblet:`
  line, since the prototype has no list/detail split or link editor;
- `Fortryd svar`, the one backward step the screen offers: it returns a
  sample from *svar* to *sendt* with no result — not to *svar* with no
  result, which would otherwise count as clean and silently un-gate its
  types;
- recording a result as one combined patch, `{stage: 'svar', result}`,
  checked by the backend against the merged state;
- a toast after every sample change reporting the approval-gate effect (how
  many types were un-gated or re-gated by the change);
- deep links both ways: `?sample=<id>` scrolls to and highlights a card,
  `?ny=<typeId>` opens the create form pre-linked to that type, and a linked
  type's name on a card opens `/kortlaegning?type=<id>`;
- the badge is `pending_samples` (from `GET /survey/summary`) — samples not
  yet at stage *svar* — never a client-side count.

## Overblik, Rapport and Indberetning screens (the prototype, component by component)

**Overblik** — the case hero (name, address, year, registration date,
organisation — only the fields the record has, R5), its in-place editor
(click the hero to open `ProjectMetaForm`; name/address/notes commit on
blur, the year field validates and reverts on an invalid draft, Esc reverts
without saving); a five-tile KPI row (Komponenter, Klassificeret, Bevaring /
genbrug, Til gennemsyn, Prøver afventer); the circularity bar; and quick
links to Kortlægning, Miljø & prøver, Rapport and Indberetning, each with a
live sub-line. v1 changes from the prototype:

- the KPI **Klassificeret** (share of the instance cloud's points that carry
  an instance label) stands in for the prototype's **Scanningsdækning**,
  which nothing in the project measures (R4; a real scan-coverage measure is
  a follow-up, `.github/issue-drafts/35-scan-coverage-kpi.md`);
- Indberetning's quick-link sub-line says "n typer blokerer" from
  `GET /survey/fractions` whenever anything blocks — approved types can still
  block (a pending sample, no tonnes) — and "n af m typer godkendt" otherwise;
- no aerial photo, sync chip, case number, MRK or demolition deadline — none
  of that is stored yet (R5, a follow-up); correspondingly no BBR line (R6,
  shown only once a BFE number can be stored);
- the hero's in-place editor is new: the prototype's hero is static;
- the old Dashboard/inventory screen moved from `/` to `/projektdata`, a
  `Værktøjer` entry, so Overblik could take the landing route.

**Rapport** — Ressourcekortlægning report versions, newest first: each one
numbered `v<n>` with its generation date, file size and a `Komplet`/`Udkast`
pill; `Generér ny version` (disabled while another version is generating;
while a pipeline job holds the writer lock it stays enabled, and a click gets
the server's 409 and a toast saying to try again); a draft notice naming how
many types still block completeness; and an Inventarliste download. v1
changes:

- versions are numbered `v<n>` and marked `Komplet`/`Udkast` from the
  **stored** blocking-type count at generation time (schema v23,
  `ReportPdfVersion.blocking_types`), not inferred client-side — a version
  generated before that column existed shows "Ukendt status" instead of
  guessing;
- the PDF itself opens with the approved survey as its first section (the
  kortlægning data `Create survey` populates), ahead of the material-passport
  columns Export already produced;
- Inventarliste is **not** a stored version: it is the live CSV export
  (`/exports/csv`), downloaded fresh every time, since a bygningsdel list
  changes between report generations;
- the hero's figures are the whole survey (every non-rejected type), while
  the PDF reports approved, reportable types only; the hero is captioned
  "Hele kortlægningen (inkl. ikke-godkendte)" so the two are not confused;
- no MRK signature or approval workflow on a version — nothing models
  approval yet (R7, a follow-up, `.github/issue-drafts/34-report-approval-xls-version.md`).

Known edge, not a defect: generating a version from a survey with no
reportable rows (no types at all, or none approved yet with nothing blocking)
records `blocking_types: 0`, so the version is marked `Komplet` though its
survey section is empty. Nothing blocks, so it is not a draft either; the
Indberetning send gate, which needs at least one fraction, stays shut.

**Indberetning** — a fraction table (EAK code, behandling, mass in tonnes,
contaminated flag), a blocking-list notice while types are still in the
queue or awaiting a sample, a CSV download of the ready fractions, and
`Send til bygningsaffald.dk`. v1 changes:

- the send button is gated like the prototype's (disabled while any type
  blocks the report, and also while there is no fraction to report) but,
  clicked, **posts nothing** — it shows a notice saying so, since no
  bygningsaffald.dk integration exists (R9, out of scope below;
  `.github/issue-drafts/33-bygningsaffald-submission.md`);
- the CSV download is new: the prototype has no export affordance on this
  screen.

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
   the existing report endpoints, fraction table and send gate. (done in
   Phase 5)
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
- ~~Linking the detail panel's sample line to the Miljø & prøver screen~~ —
  done in Phase 4: the sample line is now a link to each linked sample's card
  (or `Registrér prøve` when none is linked).
- A survey-specific export — "Eksport (XLS)" currently downloads the existing
  material-passport CSV (`/exports/csv`), not a Kortlægning-shaped spreadsheet.
- Two pieces of the structure above that v1 does not draw yet: the child
  rows' photo count and the Punktsky tab's "Åbn i viewport" link.
- A `--border-width` token: the tab underline offset in `SurveyTable.module.css`
  and `EvidencePanel.module.css` currently computes it as `calc(-1 * 1px)`
  because no border-width token exists yet.
- Uploading the lab's miljørapport (PDF) to a sample — needs blob storage and
  an endpoint; the prototype's `Upload miljørapport (PDF)` button is not drawn
  until then.
- Rewinding a sample's stage beyond `Fortryd svar` (back to *sendt*).
- The `AppShell` overflows horizontally at 390px, when the topbar and the
  open sidebar are both shown. This predates Phase 4 and affects every route.
  Phase 6 (On-site) designs the phone layout and owns the collapsible
  sidebar.
- Cross-screen staleness: each page has its own mutation queue, so a queued
  Miljø & prøver write can land after Kortlægning's mount `GET /survey`, and
  Kortlægning then shows a stale gate. Either an app-level `SerialQueue` that
  initial loads await, or a Kortlægning re-read when the survey counts change.
- Kortlægning's selection is not in the URL, so Back after a cross-screen link
  returns to row 0. Keep `?type=<selected>` updated with `replace`.
- The cross-links are styled differently: Kortlægning's `SampleLine` uses
  accent-deep with an underline, Miljø & prøver's `.typeLink` muted text with a
  border-strong underline. Pick one.
- Case identity in project metadata: BFE number, case number
  (`RX-2026-0047`), MRK and the demolition deadline — schema, `PATCH
  /projects`, `rux set` — then Overblik's hero line and the BBR line with a
  verified BBR link. Until then BBR is not drawn.
- Esc in the Kortlægning edit dialog still closes *and saves* (the closing
  blur commits); every other editor drops the draft (Phase 5 R10).
- Report approval (`Godkendt` / MRK signature on a version) and a stored XLS
  inventory version.
- A scan-coverage measure for the building, so Overblik can show the
  prototype's `Scanningsdækning` instead of `Klassificeret`.
