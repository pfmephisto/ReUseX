<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# GUI Phase 5 — Overblik, Rapport, Indberetning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Load the project skill `design-studio` (`.claude/skills/design-studio/SKILL.md`) before touching any `.tsx`/`.css`.

**Goal:** Build the last three case screens of prototype v2 and fix the two known problems first.
- **Overblik** at `/` replaces the old Dashboard. It shows the case hero with an editor for the case details, five KPIs, the circularity bar and four quick links.
- **Rapport** at `/rapport` lists the Ressourcekortlægning report versions on the existing report endpoints. Each version is marked complete or draft. The PDF gains a survey section built from the approved types.
- **Indberetning** at `/indberetning` shows the EAK fraction table with its blocking rows and the gated send button, which posts nowhere.
- **Two fixes:** `rux gui` no longer answers concurrent reads with 503, and `dev_env.sh` no longer modifies the tracked fixture or loses its servers.

**Architecture:** As in Phases 3–4, logic that can be pure is pure and unit-tested in Node:
- `src/overblik/model.ts`: percents, KPIs, quick links, the hero line, year and metadata commits;
- `src/rapport/model.ts`: dates, sizes, version titles and status, notices, toasts;
- `src/indberetning/model.ts`: fraction rows, the footer status, the CSV and the send notice;
- `src/app/errorCopy.ts`: the Danish load-error copy.

Components are presentational. Each route owns its state and writes through `src/app/useMutationQueue.ts`. Every number on screen comes from a response body; the client only formats it.

The backend changes are small and library-first:
- `fractions_by_eak` gains the prototype's rules and a blocking list;
- the survey summary gains two KPIs;
- `report_pdfs` gains a per-version blocking count (schema v23);
- the PDF gains a survey section;
- every `ProjectDB` connection waits out short locks, and `rux gui` holds one read connection open for its lifetime.

**Tech Stack:** C++20 (`reusex_core`, `rux_gui_lib`, `ruxd`), sqlite3, Typst, Catch2 v3. React 19, react-router-dom 7, TypeScript, CSS Modules and vitest (Node, no DOM). Playwright via the design-studio scripts.

**Spec:** `docs/design/gui-kortlaegning-redesign.md`. The relevant sections are § Information architecture, § Kortlægning domain model (circularity, fractions), § Phases item 5 and § Out of scope (bygningsaffald.dk, BBR, ErrorBanner, 503, AppShell 390px, Esc).

**Prototype:**
- `docs/gui/images/prototype-v2/overblik.png`;
- `docs/gui/images/prototype-v2/rapport.png` and `indberetning.png`. These two are new: they were rendered on 2026-10-01 from the colleague's artifact `At7n5is3zp7cYX54faGJkn` (headless Chromium, 1440×1000, the same way as the existing shots) and are committed with this plan.

## The prototype, component by component

### `overblik.png` (1440×1000, light)

- **Chrome:** the Phase 1 navy title bar and sidebar. **Overblik** is active. Kortlægning carries the hot accent badge **7**, Miljø & prøver the neutral **2**. Rapport and Indberetning have no badge.
- **Hero:** a full-width rounded panel with a dark aerial photo behind a navy wash.
  - The name `MÅLØV BYVEJ 229` is set in the display face, uppercase, in on-chrome white.
  - Under it a muted line reads `Måløv Byvej 229, 2760 Måløv · RX-2026-0047 · MRK: Pernille Foss (MRK-P) · Frist for nedrivningstilladelse: 12. sep 2026`.
  - A small good-tone chip `Synkroniseret` sits top right.
- **KPI row:** five white raised tiles of equal width. Each has a big display-face figure over an uppercase, letter-spaced caption:
  - `11` KOMPONENTER;
  - `72 %` SCANNINGSDÆKNING (the `%` is set small);
  - `54 %` BEVARING / GENBRUG;
  - `7` TIL GENNEMSYN, with the figure in warn ink (brown);
  - `2` PRØVER AFVENTER, with the figure in crit ink (red).
- **Circularity panel:**
  - the heading `CIRKULARITETSOVERSIGT` in the display face;
  - a 14px stacked bar of the five affaldshierarki colours, with a 2px white gap between segments;
  - a legend of swatches with label and bold percent: `Bevaring 48 % · Genbrug 6 % · Genanvendelse 43 % · Nyttiggørelse 0 % · Bortskaffelse 3 %`.
- **Quick links:** four white cards in a row, each with a bold title and a muted sub line:
  - `Kortlægning` / `7 til gennemsyn · 11 typer`;
  - `Miljø & prøver` / `2 prøver afventer svar`;
  - `Rapport` / `RX-2026-0047 · 3 versioner`;
  - `Indberetning` / `4 af 11 typer godkendt`.
- **BBR line:** plain text below the cards: `BBR BFE 7241923 · Bygning 1 · opført 1978 · erhverv/produktion`, a wait-tone `Demodata` pill, and an accent link `Åbn i BBR →`.

### `rapport.png` (1440×1000, light)

- **View head:**
  - `RAPPORT` in the display face;
  - the muted sub `Ressourcekortlægningsrapport · RX-2026-0047`;
  - right-aligned, the filled primary button `Generér ny version`.
- **Hero panel:** white and raised.
  - `RESSOURCEKORTLÆGNING — MÅLØV BYVEJ 229` in the display face;
  - the muted line `RX-2026-0047 · 11 komponenter · 54 % bevaring/genbrug · 1 forurenet · 2 prøver afventer`;
  - the same circularity bar as Overblik, without a legend.
- **Version list:** one white panel. Each row has, from left to right:
  - a small bordered format tag (`PDF` / `XLS`);
  - a bold name over a muted `date · size`;
  - right-aligned, a status pill and a ghost `↓` download button.

  The rows are:
  - `Ressourcekortlægning — endelig` / `09.08.2026 · 2,4 MB`, good pill `Godkendt`;
  - `Inventarliste` / `21.05.2026 · 84 KB`, accent pill `Eksport`;
  - `Ressourcekortlægning — v2` / `21.05.2026 · 2,1 MB`, wait pill `Udkast`.
- **Footnote:** `Kun godkendte mængder indgår i rapporten. Versioner er uforanderlige — en ny generering giver en ny version med tidsstempel og MRK-signatur.`

### `indberetning.png` (1440×1000, light)

- **View head:**
  - `INDBERETNING`;
  - the sub `Affaldsfraktioner til bygningsaffald.dk`;
  - right-aligned, the primary `Send til bygningsaffald.dk`. It is disabled and drawn grey here, because types still block.
- **Note:** a muted paragraph capped at about 44rem: `Fraktionerne herunder er aggregeret pr. EAK-kode — **kun godkendte mængder** tælles med. Rækker der afventer gennemsyn eller miljøsvar er vist nederst og blokerer afsendelse. (API-integration: senere — v1 genererer tallene i portalens struktur.)`
- **Fraction table:** in a white panel that scrolls horizontally on a narrow screen.
  - The columns are `EAK-KODE · FRAKTION · BEHANDLING · MÆNGDE (right-aligned) · STATUS`. The header is in faint uppercase caps on the sunken surface.
  - **Ready rows:** `17.01.01 Beton Genanvendelse 190 t`, `17.04.05 Jern og stål Genanvendelse 6,8 t` and `17.06.04 Isoleringsmateriale Bortskaffelse 2,4 t`. Each has a good pill `Klar ✓` and the amount in bold. The approved *Bevaring* foundations (640 t) are **not** listed: the prototype's code says "bevaring forlader ikke bygningen".
  - **Blocking rows:** below the ready rows, in faint text, one per unapproved type, in type order. Each shows the EAK code, the type name, the treatment, the tonnes in parentheses, e.g. `(58 t)`, and a pill. The pill is wait `Afventer prøvesvar` for Vinduespartier and Gulvbelægning, whose samples are open. It is warn `Afventer godkendelse` for the other five.
  - **Footer row:** `I alt (godkendt)`, `199,2 t` and a warn pill `7 typer blokerer`.
- In the prototype's code, a fraction row also gets a crit `Forurenet` pill when a contributing type is contaminated. The send button is enabled exactly when nothing blocks.

## Rulings (spec silent, or contradicted by the code or the prototype)

- **R1 — The 503 is fixed in the server, not the client.**
  - **Cause:**
    - Every `rux gui` request opens its own read-only `ProjectDB` (`Server.cpp` `with_db`), and no connection sets a busy timeout. sqlite's default is to fail at once.
    - The project is in WAL mode, which the startup read-write open sets.
    - Readers do not block each other in WAL mode, but two short windows still take a lock:
      - when the last open connection closes, sqlite takes the database file's exclusive lock to try a checkpoint;
      - the next opener then has to rebuild the WAL index.
    - A full page load (shell + page: six GETs at once) opens and closes connections constantly, and a reader that lands in either window gets `SQLITE_BUSY`. `with_db` maps that to `503 "project database is busy"`.
    - A busy open can also silently read `schema_version` as `-1`, because `tableExists` treats a busy step as "no table".
  - **Fix, two parts, both server-side:**
    - every `ProjectDB` connection gets `sqlite3_busy_timeout(5000)`;
    - `rux gui` holds one read-only `ProjectDB` open for its whole lifetime (the *read anchor*), so no request is ever the last connection and the WAL index is never torn down between requests.
  - A 503 from `with_db` now only means that a lock was held for more than 5 s, i.e. a real writer. No client retry is added.
- **R2 — `dev_env.sh` serves a throwaway copy and keeps its state in the repo.**
  - `start` copies the project into `<repo>/.superpowers/dev-env/project/` and serves the copy. The tracked fixture is never opened, and every `start` gets a fresh copy.
  - Pidfiles and logs move from `$TMPDIR` to `<repo>/.superpowers/dev-env/`. `$TMPDIR` changes with every `nix develop`; the new directory is gitignored and is the same path from every shell of a given checkout or worktree.
  - Servers run under `setsid`, so `stop` kills npm *and* the vite process it spawned.
  - `start` refuses to start a second pair.
- **R3 — Fraction rules (the prototype's, made explicit).** In `core::fractions_by_eak`:
  - Rejected types are ignored, as before.
  - **`bevaring` never counts.** Material kept in situ is not waste. An unapproved bevaring type still blocks, because it must be reviewed for the report.
  - **A type awaiting a sample is withheld and blocks, even if approved,** with reason `sample`. Its answer can make it contaminated, which changes how it is reported. This reverses the Phase 2 test `FractionsByEak_ApprovedButAfventer_CountsAsBlocking` ("still counted: it is approved"). Task 4 rewrites that test.
  - Any other unapproved type blocks with reason `review`.
  - Rows group by `(EAK code, treatment, contaminated)`. Contaminated tonnes get their own row with a `Forurenet` flag and are never merged into a clean fraction. The prototype only flagged the whole EAK row, which hides how much of it is contaminated.
  - The response gains the `blocking` list, which holds each blocking type's id, name, EAK code, treatment, tonnes and reason. That way the table's blocking rows come from the server and are never re-derived from `GET /survey`.
- **R4 — Overblik KPIs.**
  - `Komponenter` = `counts.all`.
  - `Scanningsdækning 72 %` is hard-coded in the prototype and nothing in the project measures scan coverage of the building. The tile becomes **`Klassificeret`**: the share of the instance cloud's points that carry an instance label. It is computed server-side as the new `SurveySummary.classified_share`, and shows `—` with the hint `Kræver instansskyen` when the project has no instance cloud.
  - `Bevaring / genbrug` = `reuse_share`.
  - `Til gennemsyn` = `counts.queue`, in warn ink when above 0.
  - `Prøver afventer` = `pending_samples`, in crit ink when above 0.
  - The legend's percents use largest-remainder rounding, so they always sum to 100. On the demo seed this reproduces the prototype's 48/6/43/0/3.
- **R5 — The Overblik hero is drawn from what the project stores.**
  - **Dropped:**
    - the aerial photo, because no image source exists;
    - the `Synkroniseret` chip, because sync is a `ruxd` concept;
    - the case number `RX-2026-0047`, the MRK name and the demolition deadline. `ProjectInfo` has no fields for them; they become a follow-up together with R6.
  - The name is the metadata record's name, else the `.rux` stem. The sub line joins address · `opført <year>` · `registreret <date>` · `udarbejdet af <org>`.
  - `Rediger sagsoplysninger` opens an inline editor. Each field commits on blur (`PATCH /projects/{id}`, sparse upsert), following the Phase 4 rules.
  - The old Dashboard (cloud, mesh and component inventory, pipeline log) moves unchanged to the tool route **`/projektdata`** (`Projektdata` under Værktøjer). Its English metadata editor stays with it: spec decision 3 leaves technical screens as they are until they are redesigned.
- **R6 — No BBR line in v1.**
  - The spec shows BBR "only when project metadata carries a BFE number". `ProjectInfo` has no BFE field, so the condition can never hold, and building the line now would be dead code.
  - Follow-up: one item for the BFE number + case number + MRK + deadline in project metadata (schema, `PATCH /projects`, `rux set`), the BBR line, and a verified BBR link format.
- **R7 — Report versions (schema v23).**
  - `report_pdfs` gains `blocking_types INTEGER`, the number of types that blocked the report when the PDF was generated. It is `NULL` for versions made before v23.
  - The list adds a server-computed `version` ordinal, 1 for the first PDF. Versions are never deleted, so it is stable.
  - The UI shows `<label> — v<n>` and a pill:
    - `Komplet` (good) at 0 blocking types;
    - `Udkast` (wait) above 0;
    - none when the count is `NULL`.
  - The prototype's `Godkendt` and "MRK-signatur" imply an approval workflow that does not exist, so neither is drawn. The footnote drops "og MRK-signatur".
  - The prototype's `XLS Inventarliste` row becomes a last, fixed row `CSV Inventarliste` linking to the live `/exports/csv` download, with an accent pill `Eksport`. It is not a stored version.
- **R8 — The PDF gains a Kortlægning section.**
  - The prototype's "Kun godkendte mængder indgår i rapporten" is false today, because the Typst report lists only material passports.
  - `assemble_report_data` adds `survey`: one row per **approved** type (name, BIM7AA, EAK, summed part quantity + unit, tonnes, treatment, miljøstatus, in Danish), the approved-only circularity totals, and the blocking count. The template renders it before the passport table.
  - The two template copies (`kTypstTemplate` and `apps/rux/resources/report.typ`) change together.
- **R9 — The send gate posts nothing.**
  - `Send til bygningsaffald.dk` is enabled exactly when `fractions.ready`, as in the prototype.
  - Clicking it sends no request. It shows the notice `Ikke sendt. Direkte indberetning til bygningsaffald.dk er ikke koblet på endnu — hent tallene som CSV og indtast dem i portalen.`
  - `Hent fraktioner (CSV)` is always available. It holds the ready rows, `;`-separated with decimal commas (Danish Excel), formatted on the client from the server's rows. The bygningsaffald.dk API stays out of scope.
- **R10 — Esc convention: Esc in a text field drops that field's draft without committing and leaves the field; Esc outside a text field closes or backs out.**
  - This is Miljø's convention (`editorKeyAction`, `useTextDraft.revert`) and the platform's. Esc means *cancel* everywhere else, and an Esc that saves is a trap.
  - The new hero editor follows it.
  - **Kortlægning's DetailPanel is brought in line in Phase 5 (Task 16).** There, Esc reverts the quantity or note draft and returns focus to the table, so "Esc tilbage" still holds. Enter in the quantity field still commits.
  - **The EditDialog is left as it is.** Esc closes the modal and the closing blur commits, as Phase 3 built it. Its blur/commit ordering is the most review-scarred code in the frontend, and changing it needs its own pass. That is a follow-up.
- **R11 — `ErrorBanner` becomes Danish everywhere, including on the English tool screens.** A banner on a tool screen would otherwise read half Danish and half English, which is worse than a Danish one.
  - The heading reads `Kunne ikke hente <subject>` and the button `Prøv igen`.
  - The 503 copy now says that a running step was writing to the project, which after R1 is the only thing a 503 can mean on a read.
  - All 20 `context` strings become Danish definite noun phrases (Task 9 lists them).
  - `WriteBanner` (Dashboard/Export only) is not touched.
- **R12 — The AppShell's 390px overflow is out of scope.**
  - The fix is a collapsible sidebar for every route, which needs a design pass. Phase 6 (On-site) designs the phone layout, which is where it belongs; Phase 5 files it there.
  - Phase 5's own content must reflow inside the content column. Task 17 checks that `<main>` has no horizontal overflow at 768px wide. A 390px shot is taken for the record only.
- **R13 — `ExportPage` keeps the CSV export and the HTML preview.** PDF generation and the version table move to Rapport. Export gets a one-line pointer `PDF reports are generated and stored on Rapport →`. It stays English, as a tool screen.
- **R14 — Shared CSS.**
  - `components/miljoe/controls.module.css` moves to `components/controls.module.css`, because three screens now compose from it.
  - Two new files:
    - `routes/viewHead.module.css`: the case screens' page frame and view head;
    - `components/surfaces.module.css`: the raised panel, its heading and the warn notice.
  - Miljø's head composes from `viewHead`.
- **R15 — The sidebar's project name follows a hero edit.** `SurveyCountsContext` gains `refreshProject()` (the shell's `GET /project` reload). Overblik's queue calls it in `onSettled`.

## Global Constraints

- SPDX header on every new file:
  - TS/TSX: `// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `// SPDX-License-Identifier: GPL-3.0-or-later`;
  - C++: the same `//` form;
  - CSS: the `/* … */` block form;
  - shell and Python: `#`;
  - Markdown: the `<!-- … -->` block;
  - PNG: a `<name>.png.license` sidecar.
- **Tokens only.** No literal colour, radius, spacing or font size in `.module.css`, inline `style`, or TS strings/constants; only `var(--…)`. 1px/2px hairline borders and focus outlines are the allowed exception, as in Phases 3–4. A unitless `flexGrow` computed from data is not a design value.
  - **Never edit `src/tokens.css`.**
  - Check every changed CSS/TSX file with `python3 .claude/skills/design-studio/scripts/token_lint.py <files> --tsx`.
- **Token roles:**
  - pills use `<Pill tone>` (`--tone-*-bg/-ink`) or `<Pill treatment>`;
  - filled primary buttons use `--color-accent-deep` + `--color-on-accent` via `controls.module.css` `btnPrimary`;
  - field labels use `--font-size-2xs`, uppercase, `--tracking-caps`, `--color-text-muted`;
  - headings and big figures use `--font-display`;
  - the circularity colours are `--circ-*`;
  - the hero uses `--color-chrome` / `--color-on-chrome(-muted)`.
- **Shared CSS goes through `composes`**, never copy-paste: `controls.module.css` (buttons, fields), `surfaces.module.css` (panel, notice), `viewHead.module.css` (page, head).
- All UI copy on the case screens is Danish. Use the prototype's words where it has them; otherwise use the strings given in this plan verbatim. The tool screens stay English, except the shared `ErrorBanner` (R11).
- **Mutations go through one serialised chain per page, `src/app/useMutationQueue.ts`.** Never write a second ad-hoc chain.
- **Field commits are never dropped and never gated on `busy`.** These are the hero editor's blurs. **Only buttons are gated on `busy`:** `Generér ny version`.
- **An untouched field blur never commits.**
  - For text fields, a draft equal to the current value after trimming sends nothing (`textCommit`).
  - For the year field, `yearCommit` returns `send: false`.
  - Esc in a text field reverts the draft and blurs *without* committing (R10).
- **Keyboard handlers classify their target** with `src/app/keyTargets.ts` (`kindOf` / `isField` / `isControl`). Enter/Space on buttons, links and checkboxes keep their native activation. Phase 5 adds no global (document-level) shortcuts.
- Server state is never re-derived on the client. KPIs, percents' inputs, blocking rows, readiness, version numbers and draft status come from response bodies. The client formats them: it rounds percents, formats numbers and builds the CSV text.
- No DOM test environment may be added: vitest runs in Node (`vite.config.ts` `environment: 'node'`). Testable logic is a pure exported function. Components are verified by screenshot in both themes and by the Playwright flow.
- **Browser checks use no fixed sleeps.**
  - They wait with Playwright `expect(...)` / `expect_response` / `expect_download` and assert on **captured requests** (`page.on("request")` / `page.on("response")`).
  - "Nothing was sent" is asserted by ordering. A later, real commit's response is awaited first, and then the captured list must hold only that later request.
  - Static shots come from the same Playwright script, not from `shot.sh --wait`.
- **Backend builds run inside `nix develop`** with exactly this configure line:

  ```bash
  cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DCMAKE_CUDA_COMPILER=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9/bin/nvcc -DCUDAToolkit_ROOT=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9 -DCUDA_TOOLKIT_ROOT_DIR=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9
  ```

  - A build can run past 10 minutes. Run it with `run_in_background` or re-run the same `cmake --build …`, which resumes where it stopped.
  - **Never touch `/home/mephisto/repos/ReUseX/build`**: the worktree has its own `build/`.
- Frontend commands run from the worktree root: `npm --prefix apps/rux/frontend test|run typecheck|run build`.
- Work happens in `/home/mephisto/repos/ReUseX/.worktrees/gui-phase5` (branch `gui-phase5-overblik`).
- Scratch projects live in `SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad`. `$SP/corridor-clouds.rux` is the cloud-bearing source project that Phases 3–4 used. Never seed or serve a tracked fixture in place.
- Every commit carries the session trailers, written out in each command below. Never `--no-verify`.

## Review Focus

- **The busy fix (R1):**
  - `ProjectDBReadOnly_WaitsOutABrieflyHeldLock` must fail before the fix and pass after it;
  - the socket test issues concurrent GETs from 8 keep-alive clients and requires every one to answer 200;
  - the read anchor is never used for a query;
  - no frontend retry was added.
- **Fractions (R3):**
  - bevaring is absent from `fractions`;
  - an approved type awaiting a sample is absent from `fractions` and present in `blocking` with reason `sample`;
  - contaminated tonnes have their own row;
  - `blocking_types == blocking.size()`;
  - the demo seed gives exactly the prototype's three rows, `199,2 t` and 7 blocking types.
- **Schema v23 (R7):** a pre-v23 version reads `blocking_types: null`, also through a read-only open of an un-migrated project (`columnExists` gate). `version` counts in generation order. `rux gui` and `ruxd` serialise the same two fields.
- **The PDF (R8):** only approved types appear in the survey section. Both template copies are identical in the changed region. The Typst smoke test compiles where typst is on PATH.
- **The hero editor:**
  - focusing and leaving a field sends nothing;
  - Esc reverts without sending;
  - an emptied name snaps back;
  - a non-year snaps back with the toast;
  - the sidebar name follows;
  - commits made during a request all arrive, in order.
- **Send gate (R9):**
  - the button is disabled iff not ready;
  - clicking it fires no request;
  - the CSV holds only the ready rows.
- **Esc in Kortlægning (R10):** typing a quantity and pressing Esc sends no `PATCH` and leaves focus on the table. Enter still commits. The EditDialog is unchanged.
- **ErrorBanner (R11):** every caller passes a Danish definite noun phrase, and no English remains in the banner.
- **No 503** appears in any captured response during the Task 17 flow.

---

## File Structure

| File | Responsibility |
|---|---|
| `libs/reusex/src/core/ProjectDB.cpp` (modify) | busy timeout on every connection; schema v23; `report_pdfs.blocking_types` + `version` |
| `libs/reusex/include/core/ProjectDB.hpp` (modify) | `ReportPdfRecord::version`, `::blocking_types`; `add_report_pdf(…, blocking_types)` |
| `apps/rux/src/gui/Server.cpp` (modify) | the read anchor |
| `libs/reusex/include/core/survey.hpp`, `src/core/survey.cpp` (modify) | `TypeTotals` id/name, `Fraction::contaminated`, `BlockingReason`, `BlockingType`, `FractionReport::blocking`, new `fractions_by_eak` rules, Danish labels |
| `libs/reusex/include/core/survey_service.hpp`, `src/core/survey_service.cpp` (modify) | `type_totals` fills id/name; `ReportSurveyRow`, `report_survey_rows` |
| `libs/reusex/include/core/report_generator.hpp`, `src/core/report_generator.cpp` (modify) | survey section in the data + template; `report_blocking_types` |
| `apps/rux/resources/report.typ` (modify) | canonical template copy, same change |
| `apps/rux/src/gui/survey.cpp` (modify) | summary `classified_share`, `contaminated_types`; fractions `contaminated`, `blocking` |
| `apps/rux/include/gui/api.hpp`, `src/gui/api.cpp`, `src/gui/edits.cpp` (modify) | `report_version_json`; generation records the blocking count |
| `apps/ruxd/src/handlers/reports.cpp` (modify) | same two fields; blocking count on POST |
| `docs/gui/openapi.yaml` (modify) | `SurveySummary`, `SurveyFractions`, `ReportPdfVersion`, report POST description |
| `tests/unit/core/test_project_db_concurrent_reads.cpp` (new) | busy-timeout regression + concurrent opens |
| `tests/unit/rux_gui/test_gui_server_socket.cpp` (modify) | concurrent GETs on a fresh server |
| `tests/unit/core/test_survey.cpp`, `tests/unit/rux_gui/test_gui_survey.cpp` (modify) | fraction and summary rules |
| `tests/unit/core/test_project_db_reports.cpp`, `test_project_db_survey.cpp` (modify) | v23 round trip and migration; latest-version assertion |
| `tests/unit/core/test_report_survey.cpp`, `tests/unit/rux_gui/test_gui_reports.cpp` (new) | report survey rows, labels, blocking count, Typst smoke; version JSON |
| `.claude/skills/design-studio/scripts/dev_env.sh` (modify), `SKILL.md`, `references/reusex-frontend.md` (modify) | R2 |
| `apps/rux/frontend/dev/seed-survey-demo.sh` (modify) | `--varied` adds three report versions |
| `src/api/types.ts` (modify) | new contract fields |
| `src/test/surveyFixtures.ts` (modify), `src/test/kortlaegning.page.test.ts` (modify) | builders for summary / fractions / versions |
| `src/app/errorCopy.ts` (new), `src/components/ErrorBanner.tsx` (modify), 14 callers (modify), `src/test/errorCopy.test.ts` (new) | R11 |
| `src/components/controls.module.css` (moved), `src/components/surfaces.module.css` (new), `src/routes/viewHead.module.css` (new) | R14 |
| `src/app/links.ts` (modify) | `OVERBLIK_PATH`, `RAPPORT_PATH`, `INDBERETNING_PATH`, `PROJEKTDATA_PATH` |
| `src/overblik/model.ts` (new), `src/test/overblik.model.test.ts` (new) | Overblik logic |
| `src/components/CircularityBar.tsx` + css (new) | the shared stacked bar + legend |
| `src/components/StatCard.tsx` + css (modify) | `kpi`, `unit`, `ink` |
| `src/components/overblik/CaseHero.tsx`, `KpiRow.tsx`, `QuickLinks.tsx`, `ProjectMetaForm.tsx` + css (new) | Overblik pieces |
| `src/miljoe/useTextDraft.ts` (modify) | `onChange` accepts a textarea event too |
| `src/app/SurveyCountsContext.tsx`, `src/app/AppShell.tsx` (modify) | `refreshProject` (R15) |
| `src/routes/OverblikPage.tsx` + css (new), `src/routes/Dashboard.tsx` (doc comment) | `/` and `/projektdata` |
| `src/rapport/model.ts` (new), `src/test/rapport.model.test.ts` (new), `src/components/rapport/VersionList.tsx` + css (new), `src/routes/RapportPage.tsx` + css (new) | Rapport |
| `src/routes/ExportPage.tsx` + css (modify) | R13 |
| `src/indberetning/model.ts` (new), `src/test/indberetning.model.test.ts` (new), `src/components/indberetning/FractionTable.tsx` + css (new), `src/routes/IndberetningPage.tsx` + css (new) | Indberetning |
| `src/app/App.tsx`, `src/app/navigation.ts`, `src/test/navigation.test.ts` (modify) | routes and live entries |
| `src/components/kortlaegning/useQuantityNoteDrafts.ts`, `DetailPanel.tsx`, `src/test/kortlaegning.detailPanel.test.ts` (modify) | R10 in Kortlægning |
| `docs/gui/images/prototype-v2/rapport.png`, `indberetning.png` + `.license` (new, committed with this plan) | reference shots |
| docs: `docs/design/gui-kortlaegning-redesign.md`, `apps/rux/frontend/README.md`, `.claude/skills/design-studio/references/reusex-frontend.md` | screens, rules, follow-ups |

Unless a path is absolute or starts with a top-level directory, `src/…` is under `apps/rux/frontend/`.

---

### Task 1: Worktree build and a baseline of the 503

**Files:** None in the repo. This task records the 503 as it is before Task 2.

**Interfaces:**
- Produces a worktree `build/apps/rux/rux`, `build/apps/ruxd/ruxd` and `build/tests/reusex_unit_tests`.
- Produces a recorded count of 503s under concurrent GETs. Task 2 compares against it.

- [ ] **Step 1: Build** (inside `nix develop`, the CUDA-workaround configure line; may exceed 10 minutes, so re-run the same command to resume)

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
nix develop --command bash -c 'cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DCMAKE_CUDA_COMPILER=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9/bin/nvcc -DCUDAToolkit_ROOT=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9 -DCUDA_TOOLKIT_ROOT_DIR=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9 && cmake --build build --target rux ruxd reusex_unit_tests -j"$(nproc)"'
npm --prefix apps/rux/frontend ci
nix develop --command typst --version
```

Expected:
- `build/apps/rux/rux` and `build/apps/ruxd/ruxd` exist;
- `npm ci` completes;
- `typst --version` prints a version, because the devshell provides it (#456).

- [ ] **Step 2: Baseline the 503** on a throwaway seeded copy

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/p5-busy.rux"
./build/apps/rux/rux -p "$SP/p5-busy.rux" gui --port 8451 --no-browser > "$SP/p5-busy.log" 2>&1 & S=$!
for i in $(seq 1 30); do curl -sf -o /dev/null localhost:8451/api/v1/health && break; sleep 1; done
for round in $(seq 1 10); do
  printf '%s\n' health project survey/summary survey/fractions survey reports/ressourcekortlaegning samples clouds \
    | xargs -P 8 -I{} curl -s -o /dev/null -w '%{http_code}\n' "localhost:8451/api/v1/{}"
done | sort | uniq -c | tee "$SP/p5-busy-before.txt"
kill $S
```

Expected: a `200` line, and usually some `503` lines (the race is timing-dependent). Keep `$SP/p5-busy-before.txt`, because Task 2 Step 6 runs the same loop. A run with no 503 is not a reason to skip Task 2. Its regression proof is the deterministic Catch2 test, not this loop.

No commit (this task changes nothing in the repo).

---

### Task 2: Reads never fail while nothing writes (R1)

**Files:**
- Modify `libs/reusex/src/core/ProjectDB.cpp` (`Impl` constructor).
- Modify `apps/rux/src/gui/Server.cpp` (`Server::Impl` constructor and members).
- Create `tests/unit/core/test_project_db_concurrent_reads.cpp`.
- Modify `tests/unit/rux_gui/test_gui_server_socket.cpp`.

**Interfaces:**
- No public API change.
- Every `ProjectDB` connection, read-only or read-write, waits up to 5 s for a lock before `SQLITE_BUSY`.
- `rux gui` holds one read-only `ProjectDB` for its lifetime.

- [ ] **Step 1: Failing tests.** Create `tests/unit/core/test_project_db_concurrent_reads.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Concurrent read-only ProjectDB connections (GUI Phase 5, R1). A reader must
// wait out a briefly held lock instead of failing at once, and many readers
// opening and closing together — what `rux gui` does on a full page load —
// must never see SQLITE_BUSY.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using reusex::ProjectDB;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB()
      : reusex::test_support::TempPath("test_projectdb_concurrent", ".rux") {}
};
} // namespace

TEST_CASE("ProjectDbReadOnly_WaitsOutABrieflyHeldLock",
          "[projectdb][concurrency]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path); // creates, migrates and switches to WAL
  }

  // A second connection takes the file's exclusive lock and holds it for
  // 300 ms. EXCLUSIVE locking mode on a WAL database bypasses the shared WAL
  // index and keeps that lock until close, so no other connection can read
  // in the meantime. Assertions stay on the main thread (Catch2 is not
  // thread-safe); the holder only records return codes.
  int open_rc = -1;
  int lock_rc = -1;
  std::promise<void> locked;
  std::thread holder([&] {
    sqlite3 *raw = nullptr;
    open_rc = sqlite3_open(tmp.path.string().c_str(), &raw);
    lock_rc = sqlite3_exec(raw,
                           "PRAGMA locking_mode=EXCLUSIVE;"
                           "BEGIN EXCLUSIVE;"
                           "CREATE TABLE IF NOT EXISTS lock_probe (x INTEGER);"
                           "COMMIT;",
                           nullptr, nullptr, nullptr);
    locked.set_value();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    sqlite3_close(raw);
  });
  locked.get_future().wait();

  const auto start = std::chrono::steady_clock::now();
  int version = -1;
  std::string error;
  try {
    ProjectDB db(tmp.path, /*readOnly=*/true);
    version = db.schema_version();
    (void)db.survey_types();
  } catch (const std::exception &e) {
    error = e.what();
  }
  const auto waited = std::chrono::steady_clock::now() - start;
  holder.join();

  REQUIRE(open_rc == SQLITE_OK);
  REQUIRE(lock_rc == SQLITE_OK);
  INFO("read failed with: " << error);
  CHECK(error.empty());
  // Without a busy timeout the open's schema probe sees SQLITE_BUSY and
  // silently reads the version as -1.
  CHECK(version == ProjectDB::latest_schema_version());
  CHECK(waited >= std::chrono::milliseconds(200));
}

TEST_CASE("ProjectDbReadOnly_ManyConcurrentOpens_NeverBusy",
          "[projectdb][concurrency]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
  }

  constexpr int kThreads = 8;
  constexpr int kRounds = 40;
  std::atomic<int> failures{0};
  std::mutex first_mutex;
  std::string first_error;
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t)
    threads.emplace_back([&] {
      for (int i = 0; i < kRounds; ++i) {
        try {
          ProjectDB db(tmp.path, /*readOnly=*/true);
          if (db.schema_version() != ProjectDB::latest_schema_version())
            throw std::runtime_error("schema_version read as " +
                                     std::to_string(db.schema_version()));
          (void)db.survey_types();
          (void)db.list_report_pdfs();
        } catch (const std::exception &e) {
          if (failures.fetch_add(1) == 0) {
            std::lock_guard lock(first_mutex);
            first_error = e.what();
          }
        }
      }
    });
  for (auto &thread : threads)
    thread.join();

  INFO("first failure: " << first_error);
  CHECK(failures.load() == 0);
}
```

In `tests/unit/rux_gui/test_gui_server_socket.cpp`, add `#include <mutex>` to the standard includes. Then append:

```cpp
TEST_CASE("RunningServer_ConcurrentReadsOnAFreshServer_NeverBusy",
          "[gui][server][socket][concurrency]") {
  // A full page load fires the shell's and the page's GETs at once, and each
  // opens its own read-only connection. None may answer 503 while nothing
  // writes (GUI Phase 5, R1).
  ::unsetenv("RUX_GUI_ASSETS");
  TempPath project("test_gui_server_socket", ".rux");

  ServerOptions options;
  options.project = project.path;
  options.port = free_port();
  options.open_browser = false;
  options.threads = 8;
  RunningServer server(std::move(options));

  const std::vector<std::string> paths{
      "/api/v1/project",        "/api/v1/survey/summary",
      "/api/v1/survey/fractions", "/api/v1/survey",
      "/api/v1/samples",        "/api/v1/reports/ressourcekortlaegning"};
  constexpr int kClients = 8;
  constexpr int kRounds = 15;
  std::mutex mutex;
  std::vector<std::string> failures;
  std::vector<std::thread> clients;
  for (int c = 0; c < kClients; ++c)
    clients.emplace_back([&, c] {
      try {
        KeepAliveConnection connection(server.port());
        for (int i = 0; i < kRounds; ++i) {
          const std::string &path =
              paths[static_cast<std::size_t>(c + i) % paths.size()];
          const Response response = connection.get(path);
          if (response.status != 200) {
            std::lock_guard lock(mutex);
            failures.push_back(path + " -> " +
                               std::to_string(response.status) + " " +
                               response.body);
          }
        }
      } catch (const std::exception &e) {
        std::lock_guard lock(mutex);
        failures.push_back(std::string("client error: ") + e.what());
      }
    });
  for (auto &client : clients)
    client.join();

  INFO((failures.empty() ? std::string() : failures.front()));
  CHECK(failures.empty());
}
```

- [ ] **Step 2: Run → the deterministic test FAILS**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
nix develop --command bash -c 'cmake --build build --target reusex_unit_tests -j"$(nproc)" && ctest --test-dir build -R "ProjectDbReadOnly_WaitsOut|ProjectDbReadOnly_ManyConcurrent|ConcurrentReadsOnAFreshServer" --output-on-failure'
```

Expected: `ProjectDbReadOnly_WaitsOutABrieflyHeldLock` fails. Either `error` is non-empty ("database is locked") or `version == -1`. The other two may pass or fail depending on timing.

- [ ] **Step 3: Busy timeout.** In `libs/reusex/src/core/ProjectDB.cpp`, inside `class ProjectDB::Impl`, directly under the `static constexpr int LATEST_SCHEMA_VERSION = 22;` line (Task 6 later bumps it to 23), add:

```cpp
  // How long one statement waits for a lock another connection holds before
  // failing with SQLITE_BUSY. sqlite's default is not to wait at all.
  static constexpr int BUSY_TIMEOUT_MS = 5000;
```

In the `Impl(std::filesystem::path path, bool ro)` constructor, directly after the `if (sqlite3_open_v2(...) != SQLITE_OK) { ... }` block and before `if (!readOnly) {`, insert:

```cpp
    // Every connection waits out a briefly held lock instead of failing at
    // once. With WAL, readers never block each other, but a reader whose open
    // races another connection's close (which takes the file lock to try a
    // checkpoint) or its WAL-index rebuild got SQLITE_BUSY immediately — the
    // 503s on a full `rux gui` page load. Writers never block WAL readers, so
    // the wait only ever covers those millisecond windows; a lock held past
    // the timeout is still reported.
    sqlite3_busy_timeout(db, BUSY_TIMEOUT_MS);
```

- [ ] **Step 4: Read anchor.** In `apps/rux/src/gui/Server.cpp`, in `Server::Impl`'s constructor, directly after the closing `}` of the startup block that opens the project read-write and logs `"Project '{}' opened (schema v{})"`, insert:

```cpp
    // Hold one read-only connection for the server's lifetime. Every request
    // opens and closes its own connection (ProjectDB is not thread-safe), and
    // without an anchor the last of them to close tries a checkpoint under the
    // file's exclusive lock and drops the WAL index, which the next request
    // then has to rebuild. Readers landing in those windows saw SQLITE_BUSY.
    // The anchor holds no transaction and is never queried, so it blocks no
    // writer and no checkpoint.
    read_anchor_ =
        std::make_unique<reusex::ProjectDB>(options_.project, /*readOnly=*/true);
```

At the end of the member list, directly after `IViewRenderer *view_renderer_ = nullptr;`, add:

```cpp

  /// Keeps the project's WAL index alive between requests (see the
  /// constructor). Never used for a query.
  std::unique_ptr<reusex::ProjectDB> read_anchor_;
```

In the doc comment of `with_db`, replace the sentence that starts "Writers are the job worker and the editor endpoints" with the paragraph below. Leave the rest of the comment as it is.

```cpp
  /// Writers are the job worker and the editor endpoints (with_write below);
  /// both hold the runner's writer lock, so at most one of them is writing at
  /// any moment. Each connection waits up to ProjectDB's busy timeout (5 s),
  /// and the server-lifetime read anchor keeps the WAL index alive, so a 503
  /// here means a lock really was held that long — by a writer.
```

- [ ] **Step 5: Run → PASS**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
nix develop --command bash -c 'cmake --build build --target rux reusex_unit_tests -j"$(nproc)" && ctest --test-dir build -R "ProjectDbReadOnly_|ConcurrentReadsOnAFreshServer|RunningServer_|ProjectDb" --output-on-failure --parallel "$(nproc)"'
```

Expected: all pass. That includes the existing `ProjectDbReadOnly_OldSchema_FileUnchanged`: the busy timeout writes nothing.

- [ ] **Step 6: Repeat the Task 1 loop**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
./build/apps/rux/rux -p "$SP/p5-busy.rux" gui --port 8451 --no-browser > "$SP/p5-busy.log" 2>&1 & S=$!
for i in $(seq 1 30); do curl -sf -o /dev/null localhost:8451/api/v1/health && break; sleep 1; done
for round in $(seq 1 10); do
  printf '%s\n' health project survey/summary survey/fractions survey reports/ressourcekortlaegning samples clouds \
    | xargs -P 8 -I{} curl -s -o /dev/null -w '%{http_code}\n' "localhost:8451/api/v1/{}"
done | sort | uniq -c
kill $S
```

Expected: one line, `80 200`.

- [ ] **Step 7: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
git add libs/reusex/src/core/ProjectDB.cpp apps/rux/src/gui/Server.cpp tests/unit/core/test_project_db_concurrent_reads.cpp tests/unit/rux_gui/test_gui_server_socket.cpp
git commit -m "fix(gui): concurrent reads never answer 503 while nothing writes" -m "Every ProjectDB connection now sets a 5 s busy timeout, and rux gui holds one read-only connection for its lifetime so the WAL index is never torn down between requests. Before, a reader racing another connection's close or WAL-index rebuild got SQLITE_BUSY at once." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 3: `dev_env.sh` serves a copy and finds its servers again (R2)

**Files:**
- Modify `.claude/skills/design-studio/scripts/dev_env.sh` (full rewrite below).
- Modify `.claude/skills/design-studio/SKILL.md` and `.claude/skills/design-studio/references/reusex-frontend.md`.

**Interfaces:**
- The usage is unchanged: `dev_env.sh start [project.rux] [gui_port] [vite_port] | stop | status`.
- `start` serves `<repo>/.superpowers/dev-env/project/<name>.rux`, a fresh copy, and prints that path.
- State lives in `<repo>/.superpowers/dev-env/`.

- [ ] **Step 1: Replace the script** with:

```bash
#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Bring the rux GUI up for a screenshot pass, or tear it down.
#
# The frontend is a pure client of `rux gui`, and the Vite dev server MUST proxy
# to it (rux gui / Crow 1.3 cannot answer a CORS preflight, so a bare
# cross-origin call fails — see apps/rux/frontend/README.md). This script starts
# both halves against a project and prints the URL to screenshot.
#
# The project is never served in place. `start` copies it into the run
# directory and serves the copy, because `rux gui` migrates the schema and
# leaves -wal/-shm files beside whatever it opens — a git-tracked fixture
# included. Every `start` serves a fresh copy, so a flow that mutates the
# project can simply be re-run.
#
# Run state (pidfiles, logs, the served copy) lives in
# <repo>/.superpowers/dev-env/: gitignored, and the same path from every shell.
# It used to live under $TMPDIR, which `nix develop` points somewhere new each
# time, so `stop` from another shell missed the servers.
#
# Usage:
#   dev_env.sh start [project.rux] [gui_port] [vite_port]
#   dev_env.sh stop
#   dev_env.sh status
#
# Defaults: project=tests/fixtures/scans/office_corridor.rux, gui_port=8420,
# vite_port=5173. Run from inside `nix develop` (provides node + the rux binary).
# Override the rux binary with RUX_BIN=... (defaults to `rux` on PATH, then
# ./build/apps/rux/rux).
set -euo pipefail

# Repo root = two dirs above this script's skill dir (.claude/skills/design-studio).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
FRONTEND="$REPO_ROOT/apps/rux/frontend"
RUN_DIR="$REPO_ROOT/.superpowers/dev-env"
mkdir -p "$RUN_DIR"

cmd="${1:-start}"

resolve_rux() {
  if [[ -n "${RUX_BIN:-}" ]]; then echo "$RUX_BIN"; return; fi
  if command -v rux >/dev/null 2>&1; then command -v rux; return; fi
  if [[ -x "$REPO_ROOT/build/apps/rux/rux" ]]; then echo "$REPO_ROOT/build/apps/rux/rux"; return; fi
  echo ""; return
}

alive() {
  [[ -f "$1" ]] && kill -0 "$(cat "$1" 2>/dev/null)" 2>/dev/null
}

# Each server runs in its own session (setsid), so its pid is also its process
# group id: signalling the group stops npm *and* the vite node process it
# spawned, which a plain `kill <npm pid>` left running on the port.
kill_pidfile() {
  local f="$1"
  [[ -f "$f" ]] || return 0
  local pid; pid="$(cat "$f" 2>/dev/null || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.25
    done
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$f"
}

case "$cmd" in
  start)
    project="${2:-$REPO_ROOT/tests/fixtures/scans/office_corridor.rux}"
    gui_port="${3:-8420}"
    vite_port="${4:-5173}"

    if alive "$RUN_DIR/gui.pid" || alive "$RUN_DIR/vite.pid"; then
      echo "error: already running (bash $SCRIPT_DIR/dev_env.sh status); stop it first" >&2
      exit 1
    fi
    rux_bin="$(resolve_rux)"
    if [[ -z "$rux_bin" ]]; then
      echo "error: no rux binary found. Build it (cmake --build build) or set RUX_BIN=." >&2
      exit 1
    fi
    if [[ ! -f "$project" ]]; then
      echo "error: project not found: $project" >&2
      exit 1
    fi
    if [[ ! -d "$FRONTEND/node_modules" ]]; then
      echo "note: installing frontend deps (first run)…" >&2
      npm --prefix "$FRONTEND" install
    fi

    # Serve a throwaway copy, never the original (see the header).
    rm -rf "$RUN_DIR/project"
    mkdir -p "$RUN_DIR/project"
    served="$RUN_DIR/project/$(basename "$project")"
    cp "$project" "$served"
    if [[ -f "$project-wal" ]]; then cp "$project-wal" "$served-wal"; fi

    echo "Starting rux gui  ($rux_bin) on :$gui_port against a copy of $(basename "$project")…"
    RUX_GUI_LOG="$RUN_DIR/gui.log"
    setsid nohup "$rux_bin" -p "$served" gui --port "$gui_port" --no-browser \
      >"$RUX_GUI_LOG" 2>&1 &
    echo $! > "$RUN_DIR/gui.pid"

    echo "Starting vite dev on :$vite_port (proxying /api -> :$gui_port)…"
    VITE_LOG="$RUN_DIR/vite.log"
    RUX_GUI_URL="http://localhost:$gui_port" \
      setsid nohup npm --prefix "$FRONTEND" run dev -- --port "$vite_port" --strictPort \
      >"$VITE_LOG" 2>&1 &
    echo $! > "$RUN_DIR/vite.pid"

    # Wait for Vite to answer before handing back to the caller.
    url="http://localhost:$vite_port"
    for _ in $(seq 1 60); do
      if curl -sf -o /dev/null "$url" 2>/dev/null; then
        echo
        echo "  UP:      $url        (screenshot this, never :$gui_port directly)"
        echo "  api:     proxied to  http://localhost:$gui_port"
        echo "  project: $served   (a copy; refreshed by every start)"
        echo "  logs:    $RUX_GUI_LOG , $VITE_LOG"
        echo "  stop:    bash $SCRIPT_DIR/dev_env.sh stop"
        exit 0
      fi
      sleep 1
    done
    echo "error: vite did not come up on $url in 60s. Check $VITE_LOG" >&2
    exit 1
    ;;

  stop)
    kill_pidfile "$RUN_DIR/vite.pid"
    kill_pidfile "$RUN_DIR/gui.pid"
    rm -rf "$RUN_DIR/project"
    echo "Stopped."
    ;;

  status)
    for name in gui vite; do
      f="$RUN_DIR/$name.pid"
      if alive "$f"; then
        echo "$name: running (pid $(cat "$f"))"
      else
        echo "$name: not running"
      fi
    done
    if [[ -d "$RUN_DIR/project" ]]; then
      echo "project: $(ls "$RUN_DIR/project"/*.rux 2>/dev/null | head -1)"
    fi
    ;;

  *)
    echo "usage: dev_env.sh {start [project.rux] [gui_port] [vite_port] | stop | status}" >&2
    exit 2
    ;;
esac
```

- [ ] **Step 2: One-time cleanup of servers the old script lost.** Find the strays with `pgrep -af 'rux .*gui --port|vite.*--strictPort'`. Kill only processes that belong to a dev env, either by their pid or with `pkill -f 'vite.*--strictPort'`. In the main checkout, check `git -C /home/mephisto/repos/ReUseX status --porcelain tests/fixtures`. If it lists `office_corridor.rux` as modified, or shows `office_corridor.rux-wal` / `-shm` (the 2026-10-01 incident), first make sure no `rux gui` process is still serving it. Then:
  - restore the file with `git -C /home/mephisto/repos/ReUseX checkout -- tests/fixtures/scans/office_corridor.rux`;
  - delete the two sidecars.

  `.gitignore` re-includes `tests/fixtures/**`, which is why the sidecars show up as untracked.

- [ ] **Step 3: Verify both defects are gone**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start
git status --porcelain tests/fixtures                        # (prints nothing)
ls tests/fixtures/scans/                                       # office_corridor.rux, .license, README.md only
ls .superpowers/dev-env/project/                               # office_corridor.rux (+ -wal/-shm of the copy)
nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh status   # gui: running / vite: running — from a second nix develop
nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start    # error: already running …
nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh stop
curl -s -o /dev/null -w '%{http_code}\n' localhost:5173       # 000
curl -s -o /dev/null -w '%{http_code}\n' localhost:8420/api/v1/health   # 000
pgrep -af 'vite.*--strictPort' || echo "no vite left"         # no vite left
git status --porcelain tests/fixtures                        # (prints nothing)
```

- [ ] **Step 4: Docs.**
  - In `.claude/skills/design-studio/SKILL.md` § 5, change the comment line above `dev_env.sh start` to: `# one command: starts \`rux gui\` on a COPY of the fixture (never the tracked file) + the Vite dev server, prints URLs; state in .superpowers/dev-env/`.
  - In `.claude/skills/design-studio/references/reusex-frontend.md` § 4, replace "`scripts/dev_env.sh start` does both and prints the URLs; `stop` tears them down." with: "`scripts/dev_env.sh start [project.rux]` does both and prints the URLs; `stop` tears them down from any shell. It serves a fresh **copy** of the project (`.superpowers/dev-env/project/`), never the file you name — `rux gui` migrates and leaves -wal/-shm beside whatever it opens. Do not run the bare `rux -p tests/fixtures/... gui` line above against the tracked fixture; copy it first."

- [ ] **Step 5: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
git add .claude/skills/design-studio/scripts/dev_env.sh .claude/skills/design-studio/SKILL.md .claude/skills/design-studio/references/reusex-frontend.md
git commit -m "fix(design-studio): dev_env serves a project copy; state survives nix develop" -m "start copied nothing: rux gui migrated the tracked fixture in place and left -wal/-shm in the checkout. Pidfiles lived under \$TMPDIR, which nix develop changes, so stop missed the servers; npm's vite child also outlived a plain kill. Now: a fresh copy per start, state in .superpowers/dev-env/, setsid process groups, and start refuses to double-start." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 4: Fraction rules and the blocking list (R3)

**Files:**
- Modify `libs/reusex/include/core/survey.hpp`, `libs/reusex/src/core/survey.cpp`, `libs/reusex/src/core/survey_service.cpp` (`type_totals`).
- Modify `apps/rux/src/gui/survey.cpp` (`survey_fractions_json`).
- Modify `docs/gui/openapi.yaml` (`/survey/fractions`, `SurveyFractions`).
- Test: `tests/unit/core/test_survey.cpp`, `tests/unit/rux_gui/test_gui_survey.cpp`.

**Interfaces** (in `reusex::core`, replacing the current `TypeTotals` / `Fraction` / `FractionReport`):

```cpp
struct TypeTotals {
  Treatment treatment = Treatment::genanvendelse;
  ReviewStatus status = ReviewStatus::queue;
  std::optional<double> mass_t;
  std::string eak_code;
  EnvironmentStatus environment = EnvironmentStatus::ren_screening;
  /// Identify the type in a blocking list; not used by the arithmetic.
  int64_t type_id = 0;
  std::string name;
};
struct Fraction {
  std::string eak_code;
  std::string name;
  Treatment treatment;
  double mass_t = 0.0;
  bool contaminated = false;
};
enum class BlockingReason { review, sample };
std::string_view to_string(BlockingReason); // "review" | "sample"
struct BlockingType {
  int64_t type_id = 0;
  std::string name;
  std::string eak_code;
  Treatment treatment = Treatment::genanvendelse;
  std::optional<double> mass_t;
  BlockingReason reason = BlockingReason::review;
};
struct FractionReport {
  std::vector<Fraction> fractions;
  double total_t = 0.0;
  std::size_t blocking_types = 0; // == blocking.size()
  std::vector<BlockingType> blocking;
};
```

- Wire: `SurveyFractions.fractions[].contaminated: boolean`, and `SurveyFractions.blocking: {type_id, name, eak_code, treatment, mass_t|null, reason: 'review'|'sample'}[]`, in type-id order.

- [ ] **Step 1: Failing tests.** In `tests/unit/core/test_survey.cpp`, **replace** the two test cases `FractionsByEak_GroupsApprovedByCodeAndTreatment` and `FractionsByEak_ApprovedButAfventer_CountsAsBlocking` with:

```cpp
TEST_CASE("FractionsByEak_GroupsApprovedByCodeAndTreatment_LeavesBevaringOut",
          "[survey]") {
  std::vector<TypeTotals> types{
      {Treatment::genanvendelse, ReviewStatus::approved, 380.0, "17.01.01",
       EnvironmentStatus::ren_screening},
      {Treatment::genanvendelse, ReviewStatus::approved, 190.0, "17.01.01",
       EnvironmentStatus::ren_screening},
      {Treatment::bevaring, ReviewStatus::approved, 640.0, "17.01.01",
       EnvironmentStatus::ren_screening},
      {Treatment::genanvendelse, ReviewStatus::approved, 6.8, "17.04.05",
       EnvironmentStatus::ren_screening},
      {Treatment::genbrug, ReviewStatus::queue, 58.0, "17.01.01",
       EnvironmentStatus::ren_screening, 2, "Betonsøjler, bærende"},
      {Treatment::genbrug, ReviewStatus::rejected, 5.0, "17.02.01",
       EnvironmentStatus::ren_screening},
  };
  const auto r = fractions_by_eak(types);
  REQUIRE(r.fractions.size() == 2);
  CHECK(r.fractions[0].eak_code == "17.01.01");
  CHECK(r.fractions[0].treatment == Treatment::genanvendelse);
  CHECK(r.fractions[0].mass_t == Approx(570.0));
  CHECK(r.fractions[0].name == "Beton");
  CHECK_FALSE(r.fractions[0].contaminated);
  CHECK(r.fractions[1].eak_code == "17.04.05");
  CHECK(r.total_t == Approx(576.8)); // bevaring stays in the building
  REQUIRE(r.blocking.size() == 1);   // the queued type; rejected never blocks
  CHECK(r.blocking_types == 1);
  CHECK(r.blocking[0].type_id == 2);
  CHECK(r.blocking[0].name == "Betonsøjler, bærende");
  CHECK(r.blocking[0].reason == BlockingReason::review);
  REQUIRE(r.blocking[0].mass_t.has_value());
  CHECK(*r.blocking[0].mass_t == Approx(58.0));
}

TEST_CASE("FractionsByEak_ApprovedButAfventer_IsWithheldAndBlocks",
          "[survey]") {
  // The answer can make it contaminated, which changes how it is reported.
  std::vector<TypeTotals> types{{Treatment::genbrug, ReviewStatus::approved,
                                 3.1, "17.04.02", EnvironmentStatus::afventer,
                                 6, "Vinduespartier, aluminium"}};
  const auto r = fractions_by_eak(types);
  CHECK(r.fractions.empty());
  CHECK(r.total_t == Approx(0.0));
  REQUIRE(r.blocking.size() == 1);
  CHECK(r.blocking[0].reason == BlockingReason::sample);
  CHECK(r.blocking[0].type_id == 6);
}

TEST_CASE("FractionsByEak_QueuedBevaring_BlocksButNeverCounts", "[survey]") {
  std::vector<TypeTotals> types{{Treatment::bevaring, ReviewStatus::queue,
                                 640.0, "17.01.01",
                                 EnvironmentStatus::ren_screening, 1,
                                 "Fundamenter & terrændæk, beton"}};
  const auto r = fractions_by_eak(types);
  CHECK(r.fractions.empty());
  REQUIRE(r.blocking.size() == 1);
  CHECK(r.blocking[0].reason == BlockingReason::review);
}

TEST_CASE("FractionsByEak_ContaminatedTonnes_GetTheirOwnRow", "[survey]") {
  std::vector<TypeTotals> types{
      {Treatment::bortskaffelse, ReviewStatus::approved, 38.0, "17.01.02",
       EnvironmentStatus::forurenet},
      {Treatment::bortskaffelse, ReviewStatus::approved, 4.0, "17.01.02",
       EnvironmentStatus::ren_proevesvar},
  };
  const auto r = fractions_by_eak(types);
  REQUIRE(r.fractions.size() == 2);
  CHECK_FALSE(r.fractions[0].contaminated); // clean first
  CHECK(r.fractions[0].mass_t == Approx(4.0));
  CHECK(r.fractions[1].contaminated);
  CHECK(r.fractions[1].mass_t == Approx(38.0));
  CHECK(r.fractions[1].name == "Mursten");
  CHECK(r.total_t == Approx(42.0));
  CHECK(r.blocking.empty());
}

TEST_CASE("BlockingReason_WireStrings", "[survey]") {
  CHECK(to_string(BlockingReason::review) == "review");
  CHECK(to_string(BlockingReason::sample) == "sample");
}
```

In `tests/unit/rux_gui/test_gui_survey.cpp`, append these checks at the end of `SurveyFractionsJson_ApprovedOnly_ReadyFlag` (type `b` is queued genbrug 58 t):

```cpp
  CHECK(j.at("fractions").at(0).at("contaminated") == false);
  REQUIRE(j.at("blocking").size() == 1);
  CHECK(j.at("blocking").at(0).at("name") == "b");
  CHECK(j.at("blocking").at(0).at("reason") == "review");
  CHECK(j.at("blocking").at(0).at("treatment") == "genbrug");
  CHECK(j.at("blocking").at(0).at("mass_t").get<double>() == Approx(58));
  CHECK(j.at("blocking").at(0).at("type_id").get<int64_t>() > 0);
```

and add:

```cpp
TEST_CASE("SurveyFractionsJson_BlockingMassNullWhenUnset", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  ProjectDB::SurveyTypeRecord t;
  t.name = "Uden tonnage";
  t.eak_code = "17.02.01";
  db.add_survey_type(t); // mass_t stays nullopt
  const auto j = survey_fractions_json(db);
  REQUIRE(j.at("blocking").size() == 1);
  CHECK(j.at("blocking").at(0).at("mass_t").is_null());
  CHECK(j.at("ready") == false);
}
```

- [ ] **Step 2: Run → FAIL** (it does not compile: `BlockingReason`, `blocking` and `contaminated` are unknown)

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
nix develop --command bash -c 'cmake --build build --target reusex_unit_tests -j"$(nproc)"'
```

- [ ] **Step 3: Implement.** In `libs/reusex/include/core/survey.hpp`, replace `struct TypeTotals {…};`, `struct Fraction {…};`, `struct FractionReport {…};` and the `fractions_by_eak` declaration with the **Interfaces** block above, followed by:

```cpp
/// Approved tonnes per (EAK code, treatment, contaminated), for the
/// bygningsaffald.dk report (GUI Phase 5, R3). Rejected types are ignored.
/// `bevaring` never counts: it stays in the building, so it is not waste — but
/// an unapproved bevaring type still blocks. A type awaiting a sample blocks
/// (reason `sample`) and is withheld even when approved, because its answer
/// can make it contaminated. Any other unapproved type blocks (`review`).
/// Contaminated tonnes are never merged into a clean fraction. Rows are in
/// code, then waste-hierarchy, then clean-before-contaminated order; the
/// blocking list is in input order.
FractionReport fractions_by_eak(const std::vector<TypeTotals> &);
```

In `libs/reusex/src/core/survey.cpp`, add `#include <tuple>` to the includes, and replace `fractions_by_eak` with:

```cpp
std::string_view to_string(BlockingReason r) {
  return r == BlockingReason::sample ? "sample" : "review";
}

FractionReport fractions_by_eak(const std::vector<TypeTotals> &types) {
  FractionReport report;
  std::map<std::tuple<std::string, int, bool>, double> grouped;
  for (const auto &t : types) {
    if (t.status == ReviewStatus::rejected)
      continue;
    const bool awaiting = t.environment == EnvironmentStatus::afventer;
    if (awaiting || t.status != ReviewStatus::approved) {
      report.blocking.push_back(
          BlockingType{t.type_id, t.name, t.eak_code, t.treatment, t.mass_t,
                       awaiting ? BlockingReason::sample
                                : BlockingReason::review});
      continue;
    }
    if (t.treatment == Treatment::bevaring)
      continue;
    grouped[{t.eak_code, static_cast<int>(t.treatment),
             t.environment == EnvironmentStatus::forurenet}] +=
        t.mass_t.value_or(0.0);
  }
  for (const auto &[key, mass] : grouped) {
    const auto &[code, treatment, contaminated] = key;
    report.fractions.push_back(Fraction{code,
                                        std::string(eak_fraction_name(code)),
                                        static_cast<Treatment>(treatment),
                                        mass, contaminated});
    report.total_t += mass;
  }
  report.blocking_types = report.blocking.size();
  return report;
}
```

In `libs/reusex/src/core/survey_service.cpp` `type_totals`, change the `push_back` to:

```cpp
    out.push_back({t.treatment, t.review_status, t.mass_t, t.eak_code,
                   env.at(t.id), t.id, t.name});
```

In `apps/rux/src/gui/survey.cpp`, replace `survey_fractions_json` with:

```cpp
json survey_fractions_json(const reusex::ProjectDB &db) {
  const auto report = core::fractions_by_eak(core::type_totals(db));
  json list = json::array();
  for (const auto &f : report.fractions)
    list.push_back({{"eak_code", f.eak_code},
                    {"name", f.name},
                    {"treatment", std::string(core::to_string(f.treatment))},
                    {"mass_t", f.mass_t},
                    {"contaminated", f.contaminated}});
  json blocking = json::array();
  for (const auto &b : report.blocking)
    blocking.push_back(
        {{"type_id", b.type_id},
         {"name", b.name},
         {"eak_code", b.eak_code},
         {"treatment", std::string(core::to_string(b.treatment))},
         {"mass_t", opt(b.mass_t)},
         {"reason", std::string(core::to_string(b.reason))}});
  return {{"fractions", std::move(list)},
          {"total_t", report.total_t},
          {"blocking_types", report.blocking_types},
          {"blocking", std::move(blocking)},
          {"ready", report.blocking_types == 0}};
}
```

Update the doc comment of `survey_fractions_json` in `apps/rux/include/gui/survey.hpp` to: `/// \`GET /survey/fractions\`: approved tonnes per EAK code, treatment and contamination (core::fractions_by_eak), with the types that still block a waste report.`

- [ ] **Step 4: openapi.**
  - In `docs/gui/openapi.yaml` `/survey/fractions` `get.description`, replace the text with: "Approved tonnes per EAK code, treatment and contamination for the bygningsaffald.dk report. `bevaring` never counts (it stays in the building). A type awaiting a sample is withheld even when approved. `blocking` lists every non-rejected type that keeps the report from being sent; the report is `ready` when it is empty."
  - Replace the `SurveyFractions` schema with:

```yaml
    SurveyFractions:
      type: object
      required: [fractions, total_t, blocking_types, blocking, ready]
      properties:
        fractions:
          type: array
          items:
            type: object
            required: [eak_code, name, treatment, mass_t, contaminated]
            properties:
              eak_code: { type: string }
              name: { type: string, description: EAK fraction name; empty for an unknown code }
              treatment: { $ref: "#/components/schemas/Treatment" }
              mass_t: { type: number }
              contaminated:
                type: boolean
                description: The tonnes come from types whose miljøstatus is forurenet; never merged with clean tonnes.
        total_t: { type: number }
        blocking_types: { type: integer, description: Length of `blocking`. }
        blocking:
          type: array
          description: Non-rejected types that keep the report from being sent, in type-id order.
          items:
            type: object
            required: [type_id, name, eak_code, treatment, mass_t, reason]
            properties:
              type_id: { type: integer }
              name: { type: string }
              eak_code: { type: string }
              treatment: { $ref: "#/components/schemas/Treatment" }
              mass_t: { type: number, nullable: true }
              reason:
                type: string
                enum: [review, sample]
                description: "`sample`: miljøstatus is afventer (blocks even when approved). `review`: not approved yet."
        ready: { type: boolean }
```

- [ ] **Step 5: Run → PASS**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
nix develop --command bash -c 'cmake --build build --target reusex_unit_tests -j"$(nproc)" && ctest --test-dir build -R "FractionsByEak|BlockingReason|SurveyFractionsJson|SurveyJson|SurveySummaryJson|gui_api_contract_parses" --output-on-failure'
python3 scripts/check-openapi.py
```

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/core/survey.hpp libs/reusex/src/core/survey.cpp libs/reusex/src/core/survey_service.cpp apps/rux/include/gui/survey.hpp apps/rux/src/gui/survey.cpp docs/gui/openapi.yaml tests/unit/core/test_survey.cpp tests/unit/rux_gui/test_gui_survey.cpp
git commit -m "feat(survey): fractions leave out bevaring, withhold pending types, list blockers" -m "Indberetning needs the prototype's rules and its blocking rows from the server: bevaring stays in the building and is not waste; a type awaiting a sample is withheld even when approved; contaminated tonnes get their own row; GET /survey/fractions now lists every blocking type with its reason." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 5: Two summary KPIs: `classified_share` and `contaminated_types` (R4)

**Files:**
- Modify `apps/rux/src/gui/survey.cpp` (`survey_summary_json`) and `apps/rux/include/gui/survey.hpp` (its doc comment).
- Modify `docs/gui/openapi.yaml` (`SurveySummary`).
- Test: `tests/unit/rux_gui/test_gui_survey.cpp`.

**Interfaces:**
- `SurveySummary.classified_share: number | null` is the share (0..1) of the instance cloud's points with a non-zero label. It is `null` without an instance cloud, or when that cloud is empty.
- `SurveySummary.contaminated_types: integer` counts non-rejected types whose miljøstatus is `forurenet`.

- [ ] **Step 1: Failing test.** Append to `tests/unit/rux_gui/test_gui_survey.cpp`:

```cpp
TEST_CASE("SurveySummaryJson_ClassifiedShare_ContaminatedTypes",
          "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  reusex::CloudL labels;
  for (std::uint32_t l : {0u, 1u, 1u, 2u}) {
    pcl::Label p;
    p.label = l;
    labels.push_back(p);
  }
  db.save_point_cloud("instances", labels, "test", "{}");

  const auto walls = add_type(db, "Murvægge", core::Treatment::bortskaffelse, 38);
  const auto gone = add_type(db, "Afvist", core::Treatment::bortskaffelse, 1,
                             core::ReviewStatus::rejected);
  const auto lead = db.add_sample("Bly i maling", "");
  ProjectDB::SamplePatch answered;
  answered.stage = core::SampleStage::svar;
  answered.result = core::SampleResult::forurenet;
  db.update_sample(lead.id, answered);
  db.set_sample_links(lead.id, {walls, gone});

  const auto j = survey_summary_json(db);
  CHECK(j.at("classified_share").get<double>() == Approx(0.75));
  CHECK(j.at("unlabeled_points") == 1);
  CHECK(j.at("contaminated_types") == 1); // the rejected one never counts
}

TEST_CASE("SurveySummaryJson_NoInstanceCloud_ClassifiedShareNull",
          "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto j = survey_summary_json(db);
  CHECK(j.at("classified_share").is_null());
  CHECK(j.at("contaminated_types") == 0);
}
```

- [ ] **Step 2: Run → FAIL** (`classified_share` is missing, so `at` throws)

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
nix develop --command bash -c 'cmake --build build --target reusex_unit_tests -j"$(nproc)" && ctest --test-dir build -R "SurveySummaryJson" --output-on-failure'
```

- [ ] **Step 3: Implement.** In `survey_summary_json`, replace the `json unlabeled = nullptr; if (db.has_point_cloud(instances_cloud)) {…}` block with:

```cpp
  json unlabeled = nullptr;
  json classified = nullptr;
  if (db.has_point_cloud(instances_cloud)) {
    if (const auto cloud = db.point_cloud_label(instances_cloud)) {
      std::size_t n = 0;
      for (const auto &p : *cloud)
        if (p.label == 0)
          ++n;
      unlabeled = n;
      if (!cloud->empty())
        classified = 1.0 - static_cast<double>(n) /
                               static_cast<double>(cloud->size());
    }
  }

  int contaminated = 0;
  for (const auto &t : totals)
    if (t.status != core::ReviewStatus::rejected &&
        t.environment == core::EnvironmentStatus::forurenet)
      ++contaminated;
```

In the returned object, add these two members after `{"pending_samples", pending},`:

```cpp
          {"classified_share", std::move(classified)},
          {"contaminated_types", contaminated},
```

Update the doc comment in `apps/rux/include/gui/survey.hpp` to: `/// \`GET /survey/summary\`: counts, circularity (tonnes per affaldshierarki step), reuse share, pending-sample and contaminated-type counts, and the coverage signals (unlabeled points, classified share, rooms without a part) that depend on clouds the project may not have yet.`

- [ ] **Step 4: openapi.** In `SurveySummary`:
  - change `required:` to `[counts, circularity, total_mass_t, reuse_share, pending_samples, contaminated_types, unlabeled_points, classified_share, rooms_without_parts]`;
  - add these properties:

```yaml
        contaminated_types: { type: integer, description: Non-rejected types whose miljøstatus is forurenet. }
        classified_share:
          type: number
          nullable: true
          description: Share (0..1) of the instance cloud's points that carry an instance label; null without an instance cloud. Overblik's "Klassificeret" KPI.
```

- [ ] **Step 5: Run → PASS**, with the same `ctest` line plus `python3 scripts/check-openapi.py`.

- [ ] **Step 6: Commit**

```bash
git add apps/rux/src/gui/survey.cpp apps/rux/include/gui/survey.hpp docs/gui/openapi.yaml tests/unit/rux_gui/test_gui_survey.cpp
git commit -m "feat(gui): survey summary reports classified share and contaminated types" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 6: Report versions record their completeness; the PDF lists the approved survey (R7, R8)

**Files:**
- Modify `libs/reusex/include/core/ProjectDB.hpp` and `libs/reusex/src/core/ProjectDB.cpp` (v23, `ReportPdfRecord`, `add_report_pdf`, `list_report_pdfs`).
- Modify `libs/reusex/include/core/survey.hpp` and `libs/reusex/src/core/survey.cpp` (Danish labels).
- Modify `libs/reusex/include/core/survey_service.hpp` and `libs/reusex/src/core/survey_service.cpp` (`report_survey_rows`).
- Modify `libs/reusex/include/core/report_generator.hpp`, `libs/reusex/src/core/report_generator.cpp` and `apps/rux/resources/report.typ`.
- Modify `apps/rux/include/gui/api.hpp`, `apps/rux/src/gui/api.cpp` and `apps/rux/src/gui/edits.cpp`.
- Modify `apps/ruxd/src/handlers/reports.cpp`.
- Modify `docs/gui/openapi.yaml`.
- Test: `tests/unit/core/test_project_db_reports.cpp`, `tests/unit/core/test_project_db_survey.cpp`, new `tests/unit/core/test_report_survey.cpp`, new `tests/unit/rux_gui/test_gui_reports.cpp`.

**Interfaces:**

```cpp
// ProjectDB (schema v23)
struct ReportPdfRecord {
  int64_t id = 0;
  std::string created_at;
  std::string label;
  std::size_t size_bytes = 0;
  /// 1-based position in generation order (v1 is the first PDF ever made).
  int version = 0;
  /// Survey types that still blocked the report when it was generated;
  /// nullopt for versions made before schema v23.
  std::optional<int> blocking_types;
};
ReportPdfRecord add_report_pdf(const std::vector<std::uint8_t> &pdf,
                               const std::string &label = "",
                               std::optional<int> blocking_types = std::nullopt);
// core/survey.hpp
std::string_view treatment_label_da(Treatment);          // "Nyttiggørelse"
std::string_view environment_label_da(EnvironmentStatus); // "Ren (prøvesvar)"
// core/survey_service.hpp
struct ReportSurveyRow {
  std::string name, bim7aa_code, eak_code, unit;
  double quantity = 0.0; // sum of the type's parts
  std::optional<double> mass_t;
  Treatment treatment = Treatment::genanvendelse;
  EnvironmentStatus environment = EnvironmentStatus::ren_screening;
};
std::vector<ReportSurveyRow> report_survey_rows(const ProjectDB &db); // approved, id order
// core/report_generator.hpp (namespace reusex)
int report_blocking_types(const ProjectDB &db); // fractions_by_eak(...).blocking_types
// apps/rux/include/gui/api.hpp (namespace rux::gui)
nlohmann::json report_version_json(const reusex::ProjectDB::ReportPdfRecord &r);
```

- Wire: `ReportPdfVersion` gains `version: integer` and `blocking_types: integer | null`. These come from both `rux gui` and `ruxd`.

- [ ] **Step 1: Failing storage tests.** Append to `tests/unit/core/test_project_db_reports.cpp`:

```cpp
TEST_CASE("ReportPdfs_VersionOrdinalAndBlockingTypes", "[projectdb][reports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const std::vector<std::uint8_t> pdf{'%', 'P', 'D', 'F'};
  const auto a = db.add_report_pdf(pdf, "Ressourcekortlægning", 7);
  const auto b = db.add_report_pdf(pdf, "Ressourcekortlægning", 0);
  const auto c = db.add_report_pdf(pdf, "Ressourcekortlægning");
  CHECK(a.version == 1);
  CHECK(b.version == 2);
  CHECK(c.version == 3);
  CHECK(a.blocking_types == std::optional<int>(7));
  CHECK_FALSE(c.blocking_types.has_value());

  const auto list = db.list_report_pdfs(); // newest first
  REQUIRE(list.size() == 3);
  CHECK(list[0].id == c.id);
  CHECK(list[0].version == 3);
  CHECK_FALSE(list[0].blocking_types.has_value());
  CHECK(list[1].blocking_types == std::optional<int>(0));
  CHECK(list[2].version == 1);
  CHECK(list[2].blocking_types == std::optional<int>(7));
}

TEST_CASE("ReportPdfs_MigratesFromV22_OldVersionsHaveNoBlockingCount",
          "[projectdb][reports][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    db.add_report_pdf({'%', 'P', 'D', 'F'}, "Ressourcekortlægning", 3);
  }
  {
    // Roll back to v22: drop the v23 column and its version row.
    sqlite3 *raw = nullptr;
    REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
    const char *sql = "ALTER TABLE report_pdfs DROP COLUMN blocking_types;"
                      "DELETE FROM schema_version WHERE version = 23;";
    REQUIRE(sqlite3_exec(raw, sql, nullptr, nullptr, nullptr) == SQLITE_OK);
    sqlite3_close(raw);
  }
  {
    // Read-only opens never migrate; the list must still work on v22.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    const auto list = ro.list_report_pdfs();
    REQUIRE(list.size() == 1);
    CHECK(list[0].version == 1);
    CHECK_FALSE(list[0].blocking_types.has_value());
  }
  ProjectDB db(tmp.path, /*readOnly=*/false);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  const auto list = db.list_report_pdfs();
  REQUIRE(list.size() == 1);
  CHECK_FALSE(list[0].blocking_types.has_value());
}
```

In `tests/unit/core/test_project_db_survey.cpp` `SurveySchema_MigratesFromV21`, change `CHECK(db.schema_version() == 22);` to `CHECK(db.schema_version() == ProjectDB::latest_schema_version());`. Rolling back to v21 now re-runs both v22 and v23.

- [ ] **Step 2: Failing survey/report tests.** Create `tests/unit/core/test_report_survey.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The PDF report's survey section (GUI Phase 5, R8): which types it lists,
// their summed quantities, the Danish labels, the blocking count stored with a
// version and — where typst is installed — that the template compiles.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/report_generator.hpp>
#include <core/survey_service.hpp>

#include "../../support/temp_path.hpp"

#include <cstdlib>
#include <string>

using reusex::ProjectDB;
namespace core = reusex::core;
using Catch::Approx;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_report_survey") {}
};

int64_t add_type(ProjectDB &db, const char *name, core::Treatment tr,
                 double mass, core::ReviewStatus st) {
  ProjectDB::SurveyTypeRecord t;
  t.name = name;
  t.treatment = tr;
  t.mass_t = mass;
  t.review_status = st;
  t.eak_code = "17.01.01";
  t.bim7aa_code = "211 Ydervægge";
  t.unit = "m²";
  return db.add_survey_type(t).id;
}

void add_part(ProjectDB &db, const char *code, int64_t type, double qty) {
  db.add_survey_part({code, type, std::nullopt, std::nullopt, std::nullopt,
                      "Facade", qty, false, "", {}, {}});
}
} // namespace

TEST_CASE("ReportSurveyRows_ApprovedOnly_QuantitySummed", "[report][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto facade =
      add_type(db, "Facadeelementer, sandwich", core::Treatment::genanvendelse,
               190, core::ReviewStatus::approved);
  add_type(db, "Betonsøjler, bærende", core::Treatment::genbrug, 58,
           core::ReviewStatus::queue);
  add_type(db, "Fejldetektion", core::Treatment::genbrug, 1,
           core::ReviewStatus::rejected);
  add_part(db, "RX-005", facade, 340);
  add_part(db, "RX-006", facade, 280);

  const auto rows = core::report_survey_rows(db);
  REQUIRE(rows.size() == 1);
  CHECK(rows[0].name == "Facadeelementer, sandwich");
  CHECK(rows[0].bim7aa_code == "211 Ydervægge");
  CHECK(rows[0].quantity == Approx(620));
  CHECK(rows[0].unit == "m²");
  REQUIRE(rows[0].mass_t.has_value());
  CHECK(*rows[0].mass_t == Approx(190));
  CHECK(rows[0].treatment == core::Treatment::genanvendelse);
  CHECK(rows[0].environment == core::EnvironmentStatus::ren_screening);
}

TEST_CASE("ReportBlockingTypes_CountsUnapprovedNonRejected",
          "[report][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_type(db, "a", core::Treatment::genanvendelse, 1,
           core::ReviewStatus::approved);
  add_type(db, "b", core::Treatment::genbrug, 1, core::ReviewStatus::queue);
  add_type(db, "c", core::Treatment::genbrug, 1, core::ReviewStatus::rejected);
  CHECK(reusex::report_blocking_types(db) == 1);
}

TEST_CASE("DanishLabels_MatchTheFrontendVocabulary", "[report][survey]") {
  // Mirrors apps/rux/frontend/src/kortlaegning/vocab.ts.
  CHECK(core::treatment_label_da(core::Treatment::bevaring) == "Bevaring");
  CHECK(core::treatment_label_da(core::Treatment::nyttiggoerelse) ==
        "Nyttiggørelse");
  CHECK(core::environment_label_da(core::EnvironmentStatus::afventer) ==
        "Afventer prøve");
  CHECK(core::environment_label_da(core::EnvironmentStatus::ren_proevesvar) ==
        "Ren (prøvesvar)");
}

TEST_CASE("ReportPdf_WithSurveySection_Compiles", "[report][typst]") {
  if (std::system("command -v typst > /dev/null 2>&1") != 0)
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto facade =
      add_type(db, "Facadeelementer, sandwich", core::Treatment::genanvendelse,
               190, core::ReviewStatus::approved);
  add_part(db, "RX-005", facade, 340);
  add_type(db, "Betonsøjler, bærende", core::Treatment::genbrug, 58,
           core::ReviewStatus::queue);

  const auto pdf = reusex::generate_ressourcekortlaegning_pdf(db);
  REQUIRE(pdf.size() > 4);
  CHECK(std::string(pdf.begin(), pdf.begin() + 4) == "%PDF");
}
```

Create `tests/unit/rux_gui/test_gui_reports.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ReportPdfVersion JSON (GUI Phase 5, R7): version ordinal and the blocking
// count, null for a version without one.

#include <catch2/catch_test_macros.hpp>

#include <gui/api.hpp>

#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>

#include <nlohmann/json.hpp>

using namespace rux::gui;
using reusex::ProjectDB;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_gui_reports") {}
};
} // namespace

TEST_CASE("ReportVersionsJson_VersionAndBlockingTypes", "[gui][reports]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.add_report_pdf({'%', 'P', 'D', 'F'}, "Ressourcekortlægning", 2);
  db.add_report_pdf({'%', 'P', 'D', 'F'}, "Ressourcekortlægning");
  const auto j = list_report_pdfs_json(db);
  const auto &v = j.at("versions");
  REQUIRE(v.size() == 2);
  CHECK(v.at(0).at("version") == 2);
  CHECK(v.at(0).at("blocking_types").is_null());
  CHECK(v.at(1).at("version") == 1);
  CHECK(v.at(1).at("blocking_types") == 2);
  CHECK(v.at(1).at("label") == "Ressourcekortlægning");
  CHECK(v.at(1).at("size_bytes") == 4);
}
```

- [ ] **Step 3: Run → FAIL** (it does not compile: `version`, `blocking_types`, `report_survey_rows`, `report_blocking_types` and the labels are unknown)

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
nix develop --command bash -c 'cmake --build build --target reusex_unit_tests -j"$(nproc)"'
```

- [ ] **Step 4: Schema v23 and storage.** In `libs/reusex/src/core/ProjectDB.cpp`:
  1. Set `static constexpr int LATEST_SCHEMA_VERSION = 23;`.
  2. In `runMigrations`, after the `if (current < 22) { migrateToV22(); }` block, add `if (current < 23) { migrateToV23(); }`.
  3. Directly after `migrateToV22()`'s closing brace, add:

```cpp
  void migrateToV23() {
    reusex::info("Migrating database to schema version 23");

    // GUI Phase 5 (Rapport): how many survey types still blocked the report
    // when a PDF version was generated, so the GUI can tell a complete
    // version from a draft. NULL for versions generated before v23.
    if (!columnExists("report_pdfs", "blocking_types"))
      execOrThrow("ALTER TABLE report_pdfs ADD COLUMN blocking_types INTEGER;");

    insertSchemaVersion(
        23, "Record blocking survey types per report PDF version (Phase 5)");
    reusex::info("Migration to schema version 23 complete");
  }

  /// 1-based position of a report version in generation order.
  int reportVersionOf(int64_t id) const {
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db,
                           "SELECT COUNT(*) FROM report_pdfs WHERE id <= ?;",
                           -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("report version: prepare failed: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int64(stmt, 1, id);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("report version: query failed: " +
                               std::string(sqlite3_errmsg(db)));
    return sqlite3_column_int(stmt, 0);
  }
```

  4. Replace `ProjectDB::add_report_pdf` with:

```cpp
ProjectDB::ReportPdfRecord
ProjectDB::add_report_pdf(const std::vector<std::uint8_t> &pdf,
                          const std::string &label,
                          std::optional<int> blocking_types) {
  impl_->checkWritable();

  ReportPdfRecord rec;
  {
    const char *sql = R"(
      INSERT INTO report_pdfs (label, pdf_blob, blocking_types)
      VALUES (?, ?, ?)
      RETURNING id, created_at, length(pdf_blob);
    )";
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("add_report_pdf: prepare failed: " +
                               std::string(sqlite3_errmsg(impl_->db)));
    StmtGuard guard(stmt);

    sqlite3_bind_text(stmt, 1, label.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_blob(stmt, 2, pdf.data(), static_cast<int>(pdf.size()),
                      SQLITE_TRANSIENT);
    if (blocking_types)
      sqlite3_bind_int(stmt, 3, *blocking_types);
    else
      sqlite3_bind_null(stmt, 3);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("add_report_pdf: insert failed: " +
                               std::string(sqlite3_errmsg(impl_->db)));

    rec.id = sqlite3_column_int64(stmt, 0);
    if (const auto *ts =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)))
      rec.created_at = ts;
    rec.size_bytes = static_cast<std::size_t>(sqlite3_column_int64(stmt, 2));
  }
  rec.label = label;
  rec.blocking_types = blocking_types;
  rec.version = impl_->reportVersionOf(rec.id);
  return rec;
}
```

  5. Replace `ProjectDB::list_report_pdfs` with:

```cpp
std::vector<ProjectDB::ReportPdfRecord> ProjectDB::list_report_pdfs() const {
  // A read-only open of a pre-v23 project has no blocking_types column.
  const bool has_blocking =
      impl_->columnExists("report_pdfs", "blocking_types");
  const std::string sql =
      std::string("SELECT id, label, created_at, length(pdf_blob), ") +
      (has_blocking ? "blocking_types" : "NULL") +
      ", (SELECT COUNT(*) FROM report_pdfs r2 WHERE r2.id <= r.id) "
      "FROM report_pdfs r ORDER BY id DESC;";

  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, sql.c_str(), -1, &stmt, nullptr) !=
      SQLITE_OK)
    throw std::runtime_error("list_report_pdfs: prepare failed: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  StmtGuard guard(stmt);

  std::vector<ReportPdfRecord> out;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    ReportPdfRecord rec;
    rec.id = sqlite3_column_int64(stmt, 0);
    if (const auto *s =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)))
      rec.label = s;
    if (const auto *s =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2)))
      rec.created_at = s;
    rec.size_bytes = static_cast<std::size_t>(sqlite3_column_int64(stmt, 3));
    if (sqlite3_column_type(stmt, 4) != SQLITE_NULL)
      rec.blocking_types = sqlite3_column_int(stmt, 4);
    rec.version = sqlite3_column_int(stmt, 5);
    out.push_back(std::move(rec));
  }
  return out;
}
```

  In `libs/reusex/include/core/ProjectDB.hpp`, replace `struct ReportPdfRecord` and the `add_report_pdf` declaration with the **Interfaces** block's versions. Keep the existing doc comments, and add `@param blocking_types Survey types that still blocked the report (schema v23)`.

- [ ] **Step 5: Labels, rows, blocking count.** In `libs/reusex/include/core/survey.hpp`, after the `to_string(EnvironmentStatus)` declaration, add:

```cpp
/// Danish user-facing labels (the PDF report). Mirror
/// apps/rux/frontend/src/kortlaegning/vocab.ts TREATMENT_LABEL / ENV_LABEL.
std::string_view treatment_label_da(Treatment);
std::string_view environment_label_da(EnvironmentStatus);
```

In `libs/reusex/src/core/survey.cpp`, add:

```cpp
std::string_view treatment_label_da(Treatment t) {
  switch (t) {
  case Treatment::bevaring:
    return "Bevaring";
  case Treatment::genbrug:
    return "Genbrug";
  case Treatment::genanvendelse:
    return "Genanvendelse";
  case Treatment::nyttiggoerelse:
    return "Nyttiggørelse";
  case Treatment::bortskaffelse:
    return "Bortskaffelse";
  }
  return {};
}

std::string_view environment_label_da(EnvironmentStatus e) {
  switch (e) {
  case EnvironmentStatus::ren_screening:
    return "Ren";
  case EnvironmentStatus::afventer:
    return "Afventer prøve";
  case EnvironmentStatus::forurenet:
    return "Forurenet";
  case EnvironmentStatus::ren_proevesvar:
    return "Ren (prøvesvar)";
  }
  return {};
}
```

In `libs/reusex/include/core/survey_service.hpp`, after `update_sample_checked`, add the `ReportSurveyRow` struct from **Interfaces** and:

```cpp
/// One row per **approved** survey type, in id order, for the PDF report's
/// survey section ("kun godkendte mængder indgår"). `quantity` is the sum of
/// the type's parts.
std::vector<ReportSurveyRow> report_survey_rows(const ProjectDB &db);
```

In `libs/reusex/src/core/survey_service.cpp`, after `type_totals`, add:

```cpp
std::vector<ReportSurveyRow> report_survey_rows(const ProjectDB &db) {
  const auto env = environment_statuses(db);
  std::map<int64_t, double> quantity;
  for (const auto &p : db.survey_parts())
    quantity[p.type_id] += p.quantity;
  std::vector<ReportSurveyRow> rows;
  for (const auto &t : db.survey_types()) {
    if (t.review_status != ReviewStatus::approved)
      continue;
    rows.push_back({t.name, t.bim7aa_code, t.eak_code, t.unit,
                    quantity[t.id], t.mass_t, t.treatment, env.at(t.id)});
  }
  return rows;
}
```

`<map>` is already included through the header.

In `libs/reusex/include/core/report_generator.hpp`, after `generate_ressourcekortlaegning_pdf`, add:

```cpp
/// How many survey types keep the report from being complete right now: the
/// length of core::fractions_by_eak's blocking list. Stored with each PDF
/// version (ProjectDB::add_report_pdf) so the GUI can tell a complete version
/// from a draft.
int report_blocking_types(const ProjectDB &db);
```

- [ ] **Step 6: The survey section in the PDF.** In `libs/reusex/src/core/report_generator.cpp`:
  1. Add the includes `#include <reusex/core/survey_service.hpp>`, `#include <fmt/format.h>` and `#include <algorithm>`.
  2. In the anonymous namespace, add:

```cpp
/// 1234.5 -> "1234,5"; whole numbers lose the decimal ("640"). The report is
/// Danish, so the decimal mark is a comma.
std::string da_number(double v) {
  std::string s = fmt::format("{:.1f}", v);
  if (s.size() > 2 && s.compare(s.size() - 2, 2, ".0") == 0)
    s.resize(s.size() - 2);
  std::replace(s.begin(), s.end(), '.', ',');
  return s;
}
```

  3. In `assemble_report_data`, directly before `return data;`, add:

```cpp
  // Survey (Kortlægning): approved types only — "kun godkendte mængder
  // indgår i rapporten" (GUI Phase 5, R8).
  nlohmann::json rows = nlohmann::json::array();
  std::vector<core::TypeTotals> approved;
  for (const auto &r : core::report_survey_rows(db)) {
    rows.push_back(
        {{"name", r.name},
         {"bim7aa", r.bim7aa_code},
         {"eak", r.eak_code},
         {"quantity", da_number(r.quantity) + " " + r.unit},
         {"mass", r.mass_t ? da_number(*r.mass_t) + " t" : std::string("—")},
         {"treatment", std::string(core::treatment_label_da(r.treatment))},
         {"environment",
          std::string(core::environment_label_da(r.environment))}});
    approved.push_back({r.treatment, core::ReviewStatus::approved, r.mass_t,
                        r.eak_code, r.environment});
  }
  const auto breakdown = core::circularity_breakdown(approved);
  nlohmann::json circ = nlohmann::json::array();
  for (std::size_t i = 0; i < core::kTreatmentCount; ++i)
    if (breakdown[i] > 0.0)
      circ.push_back(
          {{"label", std::string(core::treatment_label_da(
                         static_cast<core::Treatment>(i)))},
           {"tonnes", da_number(breakdown[i]) + " t"}});
  data["survey"] = {{"rows", std::move(rows)},
                    {"circularity", std::move(circ)},
                    {"blocking", report_blocking_types(db)}};
```

  4. After the anonymous namespace and before `generate_ressourcekortlaegning_pdf`, add:

```cpp
int report_blocking_types(const ProjectDB &db) {
  return static_cast<int>(
      core::fractions_by_eak(core::type_totals(db)).blocking_types);
}
```

  `assemble_report_data` (in the anonymous namespace above) can call it because the declaration in `report_generator.hpp`, the file's first include, is already visible there. No forward declaration is needed.

  5. In `kTypstTemplate` **and** in `apps/rux/resources/report.typ`, replace

```typst
#v(0.6cm)
#line(length: 100%, stroke: 0.4pt + luma(180))
#v(0.5cm)
```

  with

```typst
#v(0.6cm)
#line(length: 100%, stroke: 0.4pt + luma(180))
#v(0.5cm)

// ── Kortlægning: approved survey types ───────────────────────────────────────

#let survey = data.survey

#text(size: 13pt, weight: "bold")[Kortlægning]
#v(0.2cm)
#if survey.blocking > 0 [
  #text(size: 9pt, style: "italic")[Udkast — #survey.blocking type(r) afventer gennemsyn eller prøvesvar og indgår ikke i mængderne.]
  #v(0.2cm)
]
#if survey.rows.len() == 0 [
  _Ingen godkendte typer endnu._
] else {
  table(
    columns: (2fr, 1.5fr, 0.9fr, 1fr, 0.8fr, 1.1fr, 1.1fr),
    stroke: 0.3pt + luma(190),
    inset: (x: 5pt, y: 5pt),
    fill: (col, row) => if row == 0 { luma(215) } else { white },
    table.header([*Type*], [*BIM7AA*], [*EAK*], [*Mængde*], [*Tons*], [*Behandling*], [*Miljø*]),
    ..survey.rows.map(r => (r.name, r.bim7aa, r.eak, r.quantity, r.mass, r.treatment, r.environment)).flatten(),
  )
}
#if survey.circularity.len() > 0 [
  #v(0.2cm)
  #text(size: 9pt)[Cirkularitet (godkendte typer): #survey.circularity.map(c => c.label + " " + c.tonnes).join(" · ")]
]
#v(0.6cm)
#text(size: 13pt, weight: "bold")[Materialepas]
#v(0.2cm)
```

  In `apps/rux/resources/report.typ`, also extend the `data.json` structure comment at the top with:

```typst
//     "survey": {
//       "rows": [{"name", "bim7aa", "eak", "quantity", "mass", "treatment", "environment"}],
//       "circularity": [{"label": "Genanvendelse", "tonnes": "196,8 t"}],
//       "blocking": 7
//     },
```

  Check that the two copies match in the changed region:

```bash
diff <(sed -n '/Kortlægning: approved survey types/,/Materialepas\]/p' apps/rux/resources/report.typ) \
     <(sed -n '/Kortlægning: approved survey types/,/Materialepas\]/p' libs/reusex/src/core/report_generator.cpp) && echo "templates in sync"
```

  The command must print `templates in sync`.

- [ ] **Step 7: Serialise and record the count.**
  1. In `apps/rux/include/gui/api.hpp`, before `list_report_pdfs_json`, add:

```cpp
/// One ReportPdfVersion: id, created_at, label, size_bytes, version and
/// blocking_types (null before schema v23). Shared by the list and the POST.
nlohmann::json report_version_json(const reusex::ProjectDB::ReportPdfRecord &r);
```

  2. In `apps/rux/src/gui/api.cpp`, replace `list_report_pdfs_json` with:

```cpp
json report_version_json(const reusex::ProjectDB::ReportPdfRecord &r) {
  return {{"id", r.id},
          {"created_at", r.created_at},
          {"label", r.label},
          {"size_bytes", r.size_bytes},
          {"version", r.version},
          {"blocking_types",
           r.blocking_types ? json(*r.blocking_types) : json(nullptr)}};
}

json list_report_pdfs_json(const reusex::ProjectDB &db) {
  json arr = json::array();
  for (const auto &r : db.list_report_pdfs())
    arr.push_back(report_version_json(r));
  return json{{"versions", std::move(arr)}};
}
```

  3. In `apps/rux/src/gui/edits.cpp`, replace `generate_report_pdf_json` with the version below. Storing stays outside the `try`, so a locked database still maps to 503 in `with_write`, not 500.

```cpp
json generate_report_pdf_json(reusex::ProjectDB &db) {
  const int blocking = reusex::report_blocking_types(db);
  std::vector<std::uint8_t> pdf;
  try {
    pdf = reusex::generate_ressourcekortlaegning_pdf(db);
  } catch (const std::exception &e) {
    throw HttpError(500, std::string("PDF generation failed: ") + e.what());
  }
  return report_version_json(
      db.add_report_pdf(pdf, "Ressourcekortlægning", blocking));
}
```

  If `edits.cpp` does not yet include `<gui/api.hpp>`, add `#include "gui/api.hpp"` next to its existing gui includes.

  4. In `apps/ruxd/src/handlers/reports.cpp`, make two changes:
     - extend `record_json` with `{"version", r.version}` and `{"blocking_types", r.blocking_types ? nlohmann::json(*r.blocking_types) : nlohmann::json(nullptr)}`;
     - in the POST handler, change `db.add_report_pdf(pdf, "Ressourcekortlægning")` to `db.add_report_pdf(pdf, "Ressourcekortlægning", reusex::report_blocking_types(db))`.

- [ ] **Step 8: openapi.**
  - In `ReportPdfVersion`, set `required: [id, created_at, label, size_bytes, version, blocking_types]` and add:

```yaml
        version:
          type: integer
          description: 1-based position in generation order — the "v3" a user sees. Versions are never deleted, so it is stable.
        blocking_types:
          type: integer
          nullable: true
          description: Survey types that still blocked the report (in the queue or awaiting a sample) when this PDF was generated; 0 means complete. Null for versions generated before schema v23.
```

  - In `POST /reports/ressourcekortlaegning`, after the description's first paragraph, add the paragraph below. Change the `created_at` description to "Generation time, UTC, as stored by sqlite (`YYYY-MM-DD HH:MM:SS`) — clients must read it as UTC."

```yaml
        The PDF opens with a Kortlægning section listing the **approved** survey
        types (name, BIM7AA, EAK, quantity, tonnes, behandling, miljøstatus)
        and their circularity totals, then the material passports. The new
        version records `blocking_types` — how many types were not yet
        approved or still awaited a sample — so a draft can be told apart.
```

- [ ] **Step 9: Run → PASS**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
nix develop --command bash -c 'cmake --build build --target rux ruxd reusex_unit_tests -j"$(nproc)" && ctest --test-dir build -R "ReportPdfs_|ReportSurveyRows|ReportBlockingTypes|DanishLabels|ReportPdf_With|ReportVersionsJson|SurveySchema_MigratesFromV21|ProjectDb|gui_api_contract_parses" --output-on-failure --parallel "$(nproc)"'
python3 scripts/check-openapi.py
```

Expected: all pass. `ReportPdf_WithSurveySection_Compiles` **runs**, because typst is on the devshell PATH; it is not skipped. If `ruxd`'s own report route test lives in the heavy binary, also run `nix develop --command bash -c 'cmake --build build --target reusex_unit_tests_vision -j"$(nproc)" && ctest --test-dir build -R "Report" --output-on-failure'`.

- [ ] **Step 10: Look at a real PDF.** Generate one from the demo seed through the server, then open page 1 with Read:

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/p5-pdf.rux"
nix develop --command bash -c "./build/apps/rux/rux -p '$SP/p5-pdf.rux' gui --port 8452 --no-browser > '$SP/p5-pdf.log' 2>&1 & S=\$!; for i in \$(seq 1 30); do curl -sf -o /dev/null localhost:8452/api/v1/health && break; sleep 1; done; curl -s -X POST -H 'Content-Type: application/json' -d '{}' localhost:8452/api/v1/reports/ressourcekortlaegning; echo; curl -s -o '$SP/p5-report.pdf' localhost:8452/api/v1/reports/ressourcekortlaegning/1; kill \$S"
```

Expected: the POST answers `{"blocking_types":7,…,"version":1,…}`. Read `$SP/p5-report.pdf` (pages "1"). It shows:
- the "Kortlægning" heading;
- the italic draft line "Udkast — 7 type(r) …";
- a 7-column table with the four approved types: Fundamenter (bevaring, 640 t), Facadeelementer, Trapezplader and Isolering;
- the circularity line;
- then "Materialepas".

- [ ] **Step 11: Commit**

```bash
git add libs/reusex/include/core/ProjectDB.hpp libs/reusex/src/core/ProjectDB.cpp libs/reusex/include/core/survey.hpp libs/reusex/src/core/survey.cpp libs/reusex/include/core/survey_service.hpp libs/reusex/src/core/survey_service.cpp libs/reusex/include/core/report_generator.hpp libs/reusex/src/core/report_generator.cpp apps/rux/resources/report.typ apps/rux/include/gui/api.hpp apps/rux/src/gui/api.cpp apps/rux/src/gui/edits.cpp apps/ruxd/src/handlers/reports.cpp docs/gui/openapi.yaml tests/unit/core/test_project_db_reports.cpp tests/unit/core/test_project_db_survey.cpp tests/unit/core/test_report_survey.cpp tests/unit/rux_gui/test_gui_reports.cpp
git commit -m "feat(report): versions record completeness (schema v23); PDF lists the approved survey" -m "report_pdfs gains blocking_types, and versions carry a stable 1-based ordinal, so Rapport can mark a version complete or draft. The Typst report opens with a Kortlægning section built from the approved types only. rux gui and ruxd serialise both new fields." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 7: The demo seed gets report versions

**Files:**
- Modify `apps/rux/frontend/dev/seed-survey-demo.sh` and `apps/rux/frontend/README.md` (§ Development).

**Interfaces:** `--varied` also seeds three report versions:
- v1, 2026-05-21 09:12, 9 blocking;
- v2, 2026-05-21 14:30, 7 blocking;
- v3, 2026-08-09 10:05, 0 blocking (complete).

Each carries a 15-byte stub PDF, which is enough for list and download. The plain seed seeds no versions. With `--varied`, the project must be on schema ≥ 23.

- [ ] **Step 1: Edit the seed.**
  1. Extend the `--varied` comment line with: `, plus three report versions (two drafts, one complete) for Rapport.`
  2. Directly after the existing `[[ "$ver" -ge 22 ]] || …` line, add:

```bash
[[ "$varied" -eq 0 || "$ver" -ge 23 ]] || { echo "schema v$ver < 23 — --varied needs a rux with report versions" >&2; exit 1; }
```

  3. Inside the `if [[ "$varied" -eq 1 ]]` heredoc, before `COMMIT;`, add:

```sql
INSERT INTO report_pdfs (label,created_at,pdf_blob,blocking_types) VALUES
 ('Ressourcekortlægning','2026-05-21 09:12:00',X'255044462D312E340A2525454F460A',9),
 ('Ressourcekortlægning','2026-05-21 14:30:00',X'255044462D312E340A2525454F460A',7),
 ('Ressourcekortlægning','2026-08-09 10:05:00',X'255044462D312E340A2525454F460A',0);
```

- [ ] **Step 2: Check**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh --varied "$SP/corridor-clouds.rux" "$SP/p5-varied.rux"
sqlite3 "$SP/p5-varied.rux" "SELECT id, blocking_types, length(pdf_blob) FROM report_pdfs ORDER BY id;"
```

Expected: `1|9|15`, `2|7|15`, `3|0|15`.

- [ ] **Step 3: README.** In `apps/rux/frontend/README.md` § Development, extend the `--varied` sentence with: "It also seeds three Ressourcekortlægning versions (v1 and v2 drafts, v3 complete) with stub PDFs, for Rapport. Shots against `overblik.png`, `rapport.png` and `indberetning.png` use the plain seed."

- [ ] **Step 4: Commit**

```bash
git add apps/rux/frontend/dev/seed-survey-demo.sh apps/rux/frontend/README.md
git commit -m "chore(gui): --varied seed adds three report versions for Rapport" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 8: Frontend contract layer and test builders

**Files:**
- Modify `src/api/types.ts`, `src/test/surveyFixtures.ts` and `src/test/kortlaegning.page.test.ts`.

**Interfaces** (in `src/api/types.ts`):

```ts
export interface SurveySummary {
  counts: SurveyCounts;
  circularity: Record<Treatment, number>;
  total_mass_t: number;
  reuse_share: number | null;
  pending_samples: number;
  /** Non-rejected types whose miljøstatus is forurenet. */
  contaminated_types: number;
  unlabeled_points: number | null;
  /** Share (0..1) of the instance cloud's points with an instance label; null without one. */
  classified_share: number | null;
  rooms_without_parts: string[];
}
export interface SurveyFraction {
  eak_code: string;
  /** EAK fraction name; empty for an unknown code. */
  name: string;
  treatment: Treatment;
  mass_t: number;
  /** From types whose miljøstatus is forurenet; never merged with clean tonnes. */
  contaminated: boolean;
}
export type BlockingReason = 'review' | 'sample';
export interface SurveyBlockingType {
  type_id: number;
  name: string;
  eak_code: string;
  treatment: Treatment;
  mass_t: number | null;
  /** `sample`: awaiting a sample (blocks even when approved); `review`: not approved yet. */
  reason: BlockingReason;
}
export interface SurveyFractions {
  fractions: SurveyFraction[];
  total_t: number;
  blocking_types: number;
  /** In type-id order. */
  blocking: SurveyBlockingType[];
  ready: boolean;
}
export interface ReportPdfVersion {
  id: number;
  /** Generation time, UTC, as sqlite stores it ("2026-08-09 10:05:00"). Parse with `parseServerTime`. */
  created_at: string;
  label: string;
  size_bytes: number;
  /** 1-based, generation order: the "v3" a user sees. */
  version: number;
  /** Types that blocked the report at generation; 0 = complete; null before schema v23. */
  blocking_types: number | null;
}
```

- [ ] **Step 1: Types.** Replace the existing `SurveySummary`, `SurveyFraction`, `SurveyFractions` and `ReportPdfVersion` in `src/api/types.ts` with the block above. Keep each existing JSDoc line as it is, and add the new ones.

- [ ] **Step 2: Builders.** Append to `src/test/surveyFixtures.ts`. In its import, change `import type { Sample, SurveyType } from '../api/types';` to `import type { ReportPdfVersion, Sample, SurveyBlockingType, SurveyFraction, SurveyFractions, SurveySummary, SurveyType } from '../api/types';`.

```ts
/** The demo seed's summary (prototype v2 numbers). */
export function surveySummary(over: Partial<SurveySummary> = {}): SurveySummary {
  return {
    counts: { queue: 7, approved: 4, rejected: 0, all: 11 },
    circularity: { bevaring: 640, genbrug: 76, genanvendelse: 576.8, nyttiggoerelse: 1.6, bortskaffelse: 40.4 },
    total_mass_t: 1334.8,
    reuse_share: 716 / 1334.8,
    pending_samples: 2,
    contaminated_types: 1,
    unlabeled_points: null,
    classified_share: null,
    rooms_without_parts: [],
    ...over,
  };
}

export function fraction(over: Partial<SurveyFraction> = {}): SurveyFraction {
  return { eak_code: '17.01.01', name: 'Beton', treatment: 'genanvendelse', mass_t: 190, contaminated: false, ...over };
}

export function blockingType(over: Partial<SurveyBlockingType> = {}): SurveyBlockingType {
  return {
    type_id: 2,
    name: 'Betonsøjler, bærende',
    eak_code: '17.01.01',
    treatment: 'genbrug',
    mass_t: 58,
    reason: 'review',
    ...over,
  };
}

/** The demo seed's fractions: three ready rows, 199,2 t, seven blocking types. */
export function surveyFractions(over: Partial<SurveyFractions> = {}): SurveyFractions {
  const blocking = [
    blockingType(),
    blockingType({ type_id: 3, name: 'Betondæk, etagedæk', treatment: 'genanvendelse', mass_t: 380 }),
    blockingType({ type_id: 5, name: 'Stålspær, tagkonstruktion', eak_code: '17.04.05', mass_t: 14 }),
    blockingType({ type_id: 6, name: 'Vinduespartier, aluminium', eak_code: '17.04.02', mass_t: 3.1, reason: 'sample' }),
    blockingType({ type_id: 8, name: 'Gulvbelægning, linoleum', eak_code: '17.09.04', treatment: 'nyttiggoerelse', mass_t: 1.6, reason: 'sample' }),
    blockingType({ type_id: 9, name: 'Indvendige døre, træ', eak_code: '17.02.01', mass_t: 0.9 }),
    blockingType({ type_id: 11, name: 'Indvendige murvægge, malet', eak_code: '17.01.02', treatment: 'bortskaffelse', mass_t: 38 }),
  ];
  return {
    fractions: [
      fraction(),
      fraction({ eak_code: '17.04.05', name: 'Jern og stål', mass_t: 6.8 }),
      fraction({ eak_code: '17.06.04', name: 'Isoleringsmateriale', treatment: 'bortskaffelse', mass_t: 2.4 }),
    ],
    total_t: 199.2,
    blocking_types: blocking.length,
    blocking,
    ready: false,
    ...over,
  };
}

export function reportVersion(over: Partial<ReportPdfVersion> = {}): ReportPdfVersion {
  return {
    id: 1,
    created_at: '2026-08-09 10:05:00',
    label: 'Ressourcekortlægning',
    size_bytes: 2516582,
    version: 1,
    blocking_types: 0,
    ...over,
  };
}
```

  Update the file's top comment to: `/** Builders for survey types, samples, summaries, fractions and report versions in the Phase 4–5 tests. */`

- [ ] **Step 3: Fix the one old builder.** In `src/test/kortlaegning.page.test.ts` `summary()`, add `contaminated_types: 0,` after `pending_samples: 0,` and `classified_share: null,` after `unlabeled_points: null,`.

- [ ] **Step 4: Check**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend test
```

Expected: both pass. If `typecheck` flags another literal that builds one of the four types, add the missing fields to it in the same way. `ExportPage.tsx` only reads `ReportPdfVersion`, so it is unaffected.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/src/api/types.ts apps/rux/frontend/src/test/surveyFixtures.ts apps/rux/frontend/src/test/kortlaegning.page.test.ts
git commit -m "feat(gui): contract types for blocking fractions, KPIs and report versions" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 9: A Danish `ErrorBanner` (R11)

**Files:**
- Create `src/app/errorCopy.ts` and `src/test/errorCopy.test.ts`.
- Modify `src/components/ErrorBanner.tsx` and the 14 files with a `context=` (listed in Step 4).

**Interfaces:**

```ts
export interface LoadErrorCopy {
  heading: string;
  message: string;
  /** Whether retrying is the thing to do about it. */
  retryIsTheAnswer: boolean;
}
/** `subject` is a Danish definite noun phrase: "projektoversigten", "prøverne". */
export function explainLoadError(error: Error, subject?: string): LoadErrorCopy;
export const RETRY_LABEL = 'Prøv igen';
```

- [ ] **Step 1: Failing test** — `src/test/errorCopy.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import { explainLoadError, RETRY_LABEL } from '../app/errorCopy';

describe('load-error copy', () => {
  it('says a 503 is a busy project and retry is the answer', () => {
    const c = explainLoadError(new ApiRequestError(503, 'busy', '/survey'), 'kortlægningen');
    expect(c.heading).toBe('Kunne ikke hente kortlægningen');
    expect(c.message).toBe(
      'Projektdatabasen var optaget — et kørende trin skrev til projektet, da kortlægningen blev hentet. Intet er galt; prøv igen om et øjeblik.',
    );
    expect(c.retryIsTheAnswer).toBe(true);
  });

  it('does not offer retry as the answer for 501 and 404', () => {
    expect(explainLoadError(new ApiRequestError(501, 'x', '/x'), 'rapportversionerne')).toEqual({
      heading: 'Kunne ikke hente rapportversionerne',
      message: 'Denne serverversion understøtter ikke rapportversionerne endnu.',
      retryIsTheAnswer: false,
    });
    expect(explainLoadError(new ApiRequestError(404, 'x', '/x'), 'punktskylisten').message).toBe(
      'Findes ikke i projektet: punktskylisten er ikke i den åbne .rux-fil.',
    );
  });

  it('shows a network failure as its own message, with a generic subject by default', () => {
    const c = explainLoadError(new TypeError('Failed to fetch'));
    expect(c).toEqual({ heading: 'Kunne ikke hente dataene', message: 'Failed to fetch', retryIsTheAnswer: true });
  });

  it('labels the retry button in Danish', () => {
    expect(RETRY_LABEL).toBe('Prøv igen');
  });
});
```

- [ ] **Step 2: Run → FAIL** with `npm --prefix apps/rux/frontend test -- errorCopy`.

- [ ] **Step 3: Implement** — `src/app/errorCopy.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * What a failed load means, in Danish — the copy behind `ErrorBanner`.
 *
 * The three statuses the contract singles out want different reactions, so
 * they are not collapsed into one "request failed": 503 is transient (a
 * running step held the project) and wants the button, 501 is a missing
 * feature and 404 says the data is not in this project — retrying helps with
 * neither. `subject` is a definite noun phrase so both sentences read:
 * "Kunne ikke hente projektoversigten".
 */

import { ApiRequestError } from '../api/client';

export interface LoadErrorCopy {
  heading: string;
  message: string;
  /** Whether retrying is the thing to do about it. */
  retryIsTheAnswer: boolean;
}

export const RETRY_LABEL = 'Prøv igen';

export function explainLoadError(error: Error, subject = 'dataene'): LoadErrorCopy {
  const heading = `Kunne ikke hente ${subject}`;
  if (error instanceof ApiRequestError) {
    if (error.isRetryable) {
      return {
        heading,
        message: `Projektdatabasen var optaget — et kørende trin skrev til projektet, da ${subject} blev hentet. Intet er galt; prøv igen om et øjeblik.`,
        retryIsTheAnswer: true,
      };
    }
    if (error.isNotImplemented) {
      return { heading, message: `Denne serverversion understøtter ikke ${subject} endnu.`, retryIsTheAnswer: false };
    }
    if (error.isNotFound) {
      return { heading, message: `Findes ikke i projektet: ${subject} er ikke i den åbne .rux-fil.`, retryIsTheAnswer: false };
    }
  }
  // A network failure falls through too, with the fetch rejection's text.
  // Never the stack: it tells the user nothing.
  return { heading, message: error.message, retryIsTheAnswer: true };
}
```

Replace `src/components/ErrorBanner.tsx` with:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { explainLoadError, RETRY_LABEL } from '../app/errorCopy';
import styles from './ErrorBanner.module.css';

export interface ErrorBannerProps {
  error: Error;
  onRetry?: () => void;
  /**
   * What was being loaded, as a Danish definite noun phrase — "projektoversigten",
   * "prøverne". Used in both sentences; defaults to "dataene".
   */
  context?: string;
}

/** A failed load, said so the user can act on it (copy: `app/errorCopy.ts`). */
export function ErrorBanner({ error, onRetry, context }: ErrorBannerProps) {
  const { heading, message, retryIsTheAnswer } = explainLoadError(error, context);

  return (
    <div className={styles.banner} role="alert">
      <div className={styles.body}>
        <span className={styles.heading}>{heading}</span>
        <p className={styles.message}>{message}</p>
      </div>
      {onRetry && (
        <button
          type="button"
          className={`${styles.retry} ${retryIsTheAnswer ? styles.primary : ''}`}
          onClick={onRetry}
        >
          {RETRY_LABEL}
        </button>
      )}
    </div>
  );
}
```

- [ ] **Step 4: Translate every `context`.** Change these exact strings (`grep -rn 'context=' apps/rux/frontend/src --include=*.tsx | grep -v Provider` lists them):

| File | Old `context` | New `context` |
|---|---|---|
| `components/ComponentsPane.tsx` | `"building components"` | `"bygningskomponenterne"` |
| `components/ComponentsPane.tsx` | `` {`component ${name}`} `` | `` {`komponenten ${name}`} `` |
| `components/LabelsPane.tsx` | `"the cloud list"` | `"punktskylisten"` |
| `components/LabelsPane.tsx` | `` {`the '${cloud}' legend`} `` | `` {`forklaringen til '${cloud}'`} `` |
| `components/FrameDetail.tsx` | `` {`sensor frame ${id}`} `` | `` {`sensorbillede ${id}`} `` |
| `components/InstanceList.tsx` (2×) | `"the instance list"`, `"the cloud list"` | `"instanslisten"`, `"punktskylisten"` |
| `routes/FramesPage.tsx` | `"the frame inventory"`, `"the panorama list"` | `"billedoversigten"`, `"panoramalisten"` |
| `routes/PipelineLogPage.tsx` | `"the pipeline log"` | `"kørselsloggen"` |
| `routes/PipelinePage.tsx` | `"the stage catalogue"` | `"trinkataloget"` |
| `routes/Dashboard.tsx` | `"the project summary"`, `"the pipeline log"` | `"projektoversigten"`, `"kørselsloggen"` |
| `routes/MiljoePage.tsx` | `"Miljø & prøver"` | `"prøverne"` |
| `routes/ViewportPage.tsx` | `"cloud inventory"` | `"punktskyoversigten"` |
| `routes/KortlaegningPage.tsx` | `"Kortlægning"` | `"kortlægningen"` |
| `routes/ExportPage.tsx` | `"project summary"`, `"material columns"`, `"material passports"`, `"report versions"` | `"projektoversigten"`, `"materialekolonnerne"`, `"materialepassene"`, `"rapportversionerne"` |

  Then confirm nothing English is left:

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
grep -rn 'context="[^"]*\b\(the\|list\|log\|summary\|frame\|cloud\|component\|material\|report\)\b' apps/rux/frontend/src --include=*.tsx || echo "all banner subjects are Danish"
```

- [ ] **Step 5: Check and commit**

```bash
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
git add apps/rux/frontend/src/app/errorCopy.ts apps/rux/frontend/src/test/errorCopy.test.ts apps/rux/frontend/src/components apps/rux/frontend/src/routes
git commit -m "feat(gui): ErrorBanner speaks Danish on every screen" -m "The shared banner was English on the redesigned Danish screens. Its copy moves to a tested pure module, and every caller now passes a Danish definite noun phrase, so a tool screen does not get a half-English sentence." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 10: Shared CSS for the case screens (R14)

**Files:**
- Move `src/components/miljoe/controls.module.css` → `src/components/controls.module.css`.
- Modify `src/components/miljoe/LinkPicker.module.css`, `NewSampleForm.module.css`, `SampleCard.module.css` and `src/routes/MiljoePage.module.css`.
- Create `src/components/surfaces.module.css` and `src/routes/viewHead.module.css`.

**Interfaces:**
- `controls.module.css` keeps its classes: `checkbox`, `btnGhost`, `btnPrimary`, `btnDanger`, `textBtn`, `fields`, `field`, `fieldLabel`, `input`.
- `surfaces.module.css`: `panel`, `panelHeading`, `notice`.
- `viewHead.module.css`: `page`, `head`, `title`, `sub`, `actions`, `footnote`.

- [ ] **Step 1: Move the controls**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5/apps/rux/frontend/src
git mv components/miljoe/controls.module.css components/controls.module.css
sed -i "s#from './controls.module.css'#from '../controls.module.css'#" components/miljoe/LinkPicker.module.css components/miljoe/NewSampleForm.module.css components/miljoe/SampleCard.module.css
sed -i "s#from '../components/miljoe/controls.module.css'#from '../components/controls.module.css'#" routes/MiljoePage.module.css
grep -rn "controls.module.css" . | grep -v "^./components/controls.module.css"
```

Every line printed must point at `components/controls.module.css`, either as `../controls.module.css` or as `../components/controls.module.css`. In the moved file, change the header comment's first sentence to: "Controls shared by the case screens (Miljø & prøver, Overblik, Rapport, Indberetning)."

- [ ] **Step 2: Surfaces** — `src/components/surfaces.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * Surfaces shared by the case screens: the raised white panel the prototype
 * draws content in, its display-face heading, and the warn notice. Pull one in
 * with `composes: panel from '../components/surfaces.module.css';`.
 */

.panel {
  min-width: 0;
  background: var(--color-surface-raised);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-lg);
  box-shadow: var(--shadow-sm);
}

.panelHeading {
  margin: 0;
  font-size: var(--font-size-lg);
  text-transform: uppercase;
}

.notice {
  margin: 0;
  padding: var(--space-2) var(--space-3);
  border-radius: var(--radius-md);
  background: var(--tone-warn-bg);
  color: var(--tone-warn-ink);
  font-size: var(--font-size-sm);
}
```

- [ ] **Step 3: View head** — `src/routes/viewHead.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * The case screens' page frame and view head (prototype `.view-head`): a
 * display-face title, a muted sub line, and the page's actions on the right.
 * Compose from here — `composes: head from './viewHead.module.css';`.
 */

.page {
  display: flex;
  flex-direction: column;
  gap: var(--space-4);
  padding: var(--space-5);
  min-width: 0;
}

.head {
  display: flex;
  flex-wrap: wrap;
  align-items: baseline;
  gap: var(--space-3);
}

.title {
  margin: 0;
  font-size: var(--font-size-3xl);
  text-transform: uppercase;
}

.sub {
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
}

.actions {
  display: flex;
  flex-wrap: wrap;
  gap: var(--space-2);
  align-self: center;
  margin-left: auto;
}

.footnote {
  margin: 0;
  font-size: var(--font-size-xs);
  color: var(--color-text-faint);
}
```

- [ ] **Step 4: Miljø composes its head.** In `src/routes/MiljoePage.module.css`, replace the bodies of `.page`, `.head`, `.title`, `.sub` and `.footnote` with a single `composes` line each, and leave every other rule unchanged:

```css
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

.footnote {
  composes: footnote from './viewHead.module.css';
}
```

`viewHead`'s `.title` adds `margin: 0` and `.page` adds `min-width: 0`. Both are no-ops for Miljø's `h2`, which `base.css` already resets, and for its flex child.

- [ ] **Step 5: Check.** Run the commands below. Then write `$SP/phase5_shots.py` exactly as given in Task 17 Step 1 (it is reused there), start `dev_env.sh` on a plain seed as in Task 17 Step 1, and run the script with `miljoe` as the third argument. Open `miljoe-light-desktop.png`: the head and cards must look as they did in Phase 4.

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/controls.module.css apps/rux/frontend/src/components/surfaces.module.css apps/rux/frontend/src/routes/viewHead.module.css apps/rux/frontend/src/routes/MiljoePage.module.css apps/rux/frontend/src/components/miljoe --tsx
```

- [ ] **Step 6: Commit**

```bash
git add -A apps/rux/frontend/src/components apps/rux/frontend/src/routes/viewHead.module.css apps/rux/frontend/src/routes/MiljoePage.module.css
git commit -m "refactor(gui): case-screen controls, surfaces and view head are shared CSS" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 11: Overblik model and the circularity bar (R4, R5)

**Files:**
- Create `src/overblik/model.ts`, `src/test/overblik.model.test.ts`, `src/components/CircularityBar.tsx` and `src/components/CircularityBar.module.css`.
- Modify `src/app/links.ts`.

**Interfaces:**

```ts
// src/app/links.ts (added)
export const OVERBLIK_PATH = '/';
export const RAPPORT_PATH = '/rapport';
export const INDBERETNING_PATH = '/indberetning';
export const PROJEKTDATA_PATH = '/projektdata';
// src/overblik/model.ts
export interface CircSegment { treatment: Treatment; label: string; tonnes: number; percent: number }
export function wholePercents(values: readonly number[]): number[];
export function circularitySegments(circularity: Record<Treatment, number>): CircSegment[];
export interface Kpi { key: string; value: string; unit?: string; label: string; ink?: 'warn' | 'crit'; hint?: string }
export function percentText(share: number | null): string;
export function kpis(s: SurveySummary): Kpi[];
export interface QuickLink { to: string; title: string; sub: string }
export function versionsText(v: readonly ReportPdfVersion[] | null | undefined): string;
export function quickLinks(s: SurveySummary, versions: readonly ReportPdfVersion[] | null | undefined): QuickLink[];
export function caseName(summary: ProjectSummary, project?: ProjectInfo): string;
export function heroSubline(p: ProjectInfo | undefined): string;
export type MetaField = 'name' | 'building_address' | 'survey_date' | 'survey_organisation' | 'notes';
export interface MetaFieldSpec { key: MetaField; label: string; required?: boolean; multiline?: boolean }
export const META_FIELDS: readonly MetaFieldSpec[];
export type ProjectPatch = Parameters<RuxApiClient['patchProject']>[1];
export function metaPatch(key: MetaField, value: string): ProjectPatch;
export type YearCommit = { send: true; value: number | null } | { send: false; invalid: boolean };
export function yearCommit(draft: string, current: number | undefined): YearCommit;
export const INVALID_YEAR_TOAST: string;
```

- [ ] **Step 1: Failing tests** — `src/test/overblik.model.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { ProjectInfo, ProjectSummary } from '../api/types';
import {
  caseName,
  circularitySegments,
  heroSubline,
  INVALID_YEAR_TOAST,
  kpis,
  metaPatch,
  percentText,
  quickLinks,
  versionsText,
  wholePercents,
  yearCommit,
} from '../overblik/model';
import { reportVersion, surveySummary } from './surveyFixtures';

function projectSummary(projects: ProjectInfo[], path = 'maaloev.rux'): ProjectSummary {
  return {
    path,
    schema_version: 23,
    projects,
    clouds: [],
    meshes: [],
    sensor_frames: { total_count: 0, segmented_count: 0 },
    panoramic_images: { total_count: 0, matched_count: 0 },
    components: { total_count: 0, count_by_type: {} },
    materials: [],
  };
}

describe('whole percents', () => {
  it('reproduces the prototype legend on the demo tonnes', () => {
    expect(wholePercents([640, 76, 576.8, 1.6, 40.4])).toEqual([48, 6, 43, 0, 3]);
  });

  it('always sums to 100 and is all zero for no tonnes', () => {
    expect(wholePercents([1, 1, 1]).reduce((a, b) => a + b, 0)).toBe(100);
    expect(wholePercents([2, 1])).toEqual([67, 33]);
    expect(wholePercents([0, 0])).toEqual([0, 0]);
  });
});

describe('circularity segments', () => {
  it('keeps waste-hierarchy order and only steps with tonnes', () => {
    const segs = circularitySegments({ bevaring: 0, genbrug: 10, genanvendelse: 30, nyttiggoerelse: 0, bortskaffelse: 0 });
    expect(segs.map((s) => [s.treatment, s.label, s.tonnes, s.percent])).toEqual([
      ['genbrug', 'Genbrug', 10, 25],
      ['genanvendelse', 'Genanvendelse', 30, 75],
    ]);
  });

  it('is empty when nothing has tonnes', () => {
    expect(circularitySegments({ bevaring: 0, genbrug: 0, genanvendelse: 0, nyttiggoerelse: 0, bortskaffelse: 0 })).toEqual([]);
  });
});

describe('KPIs', () => {
  it('reads the prototype row from the demo summary', () => {
    expect(kpis(surveySummary())).toEqual([
      { key: 'types', value: '11', label: 'Komponenter' },
      { key: 'classified', value: '—', label: 'Klassificeret', hint: 'Kræver instansskyen' },
      { key: 'reuse', value: '54', unit: '%', label: 'Bevaring / genbrug' },
      { key: 'queue', value: '7', label: 'Til gennemsyn', ink: 'warn' },
      { key: 'samples', value: '2', label: 'Prøver afventer', ink: 'crit' },
    ]);
  });

  it('drops the action ink at zero and shows the classified share', () => {
    const k = kpis(surveySummary({ counts: { queue: 0, approved: 11, rejected: 0, all: 11 }, pending_samples: 0, classified_share: 0.724 }));
    expect(k.find((x) => x.key === 'queue')?.ink).toBeUndefined();
    expect(k.find((x) => x.key === 'samples')?.ink).toBeUndefined();
    expect(k.find((x) => x.key === 'classified')).toEqual({ key: 'classified', value: '72', unit: '%', label: 'Klassificeret' });
  });

  it('formats a missing share as a dash', () => {
    expect(percentText(null)).toBe('—');
    expect(percentText(0.536)).toBe('54');
  });
});

describe('quick links', () => {
  it('says what waits on each screen', () => {
    expect(quickLinks(surveySummary(), [reportVersion(), reportVersion({ id: 2, version: 2 }), reportVersion({ id: 3, version: 3 })])).toEqual([
      { to: '/kortlaegning', title: 'Kortlægning', sub: '7 til gennemsyn · 11 typer' },
      { to: '/miljoe', title: 'Miljø & prøver', sub: '2 prøver afventer svar' },
      { to: '/rapport', title: 'Rapport', sub: '3 versioner' },
      { to: '/indberetning', title: 'Indberetning', sub: '4 af 11 typer godkendt' },
    ]);
  });

  it('counts versions in Danish, and says when they are loading or failed', () => {
    expect(versionsText([])).toBe('Ingen versioner endnu');
    expect(versionsText([reportVersion()])).toBe('1 version');
    expect(versionsText(undefined)).toBe('Henter versioner…');
    expect(versionsText(null)).toBe('Versioner kunne ikke hentes');
    expect(quickLinks(surveySummary({ pending_samples: 1 }), [])[1].sub).toBe('1 prøve afventer svar');
  });
});

describe('case hero', () => {
  const record: ProjectInfo = {
    id: 'p1',
    name: 'Måløv Byvej 229',
    building_address: 'Måløv Byvej 229, 2760 Måløv',
    year_of_construction: 1978,
    survey_date: '2026-08-09',
    survey_organisation: 'Link Arkitektur',
  };

  it('names the case from the record, else the file', () => {
    expect(caseName(projectSummary([record]))).toBe('Måløv Byvej 229');
    expect(caseName(projectSummary([]), { ...record, name: '  ' })).toBe('maaloev');
    expect(caseName(projectSummary([record]), { ...record, name: 'Ny' })).toBe('Ny');
  });

  it('joins only the fields that are set', () => {
    expect(heroSubline(record)).toBe(
      'Måløv Byvej 229, 2760 Måløv · opført 1978 · registreret 2026-08-09 · udarbejdet af Link Arkitektur',
    );
    expect(heroSubline({ id: 'p', name: 'x', year_of_construction: 0 })).toBe('');
    expect(heroSubline(undefined)).toBe('');
  });
});

describe('metadata commits', () => {
  it('clears an emptied optional field with null, never the name', () => {
    expect(metaPatch('building_address', '')).toEqual({ building_address: null });
    expect(metaPatch('notes', 'Tag udskiftet 2004')).toEqual({ notes: 'Tag udskiftet 2004' });
    expect(metaPatch('name', 'Måløv')).toEqual({ name: 'Måløv' });
  });

  it('sends a year only when it changed and is a year', () => {
    expect(yearCommit('1978', 1978)).toEqual({ send: false, invalid: false });
    expect(yearCommit(' 1979 ', 1978)).toEqual({ send: true, value: 1979 });
    expect(yearCommit('', 1978)).toEqual({ send: true, value: null });
    expect(yearCommit('', 0)).toEqual({ send: false, invalid: false });
    expect(yearCommit('', undefined)).toEqual({ send: false, invalid: false });
    expect(yearCommit('0', 1978)).toEqual({ send: true, value: null });
    expect(yearCommit('nittenhalvfjerds', 1978)).toEqual({ send: false, invalid: true });
    expect(yearCommit('19780', undefined)).toEqual({ send: false, invalid: true });
    expect(INVALID_YEAR_TOAST).toBe('Byggeår skal være et årstal, fx 1978.');
  });
});
```

- [ ] **Step 2: Run → FAIL** with `npm --prefix apps/rux/frontend test -- overblik`.

- [ ] **Step 3: Links.** In `src/app/links.ts`, directly after `export const KORTLAEGNING_PATH = '/kortlaegning';`, add the four constants from **Interfaces**. Extend the file comment's first sentence to: "The links between the case screens, as data:".

- [ ] **Step 4: Model** — `src/overblik/model.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Overblik as data: the circularity percents, the KPI row, the quick links,
 * the case hero and the metadata editor's commits. Every figure is read off a
 * server response; this module only rounds and words it.
 */

import type { RuxApiClient } from '../api/client';
import type { ProjectInfo, ProjectSummary, ReportPdfVersion, SurveySummary, Treatment } from '../api/types';
import { TREATMENTS } from '../api/types';
import { INDBERETNING_PATH, KORTLAEGNING_PATH, MILJOE_PATH, RAPPORT_PATH } from '../app/links';
import { TREATMENT_LABEL } from '../kortlaegning/vocab';

export interface CircSegment {
  treatment: Treatment;
  label: string;
  tonnes: number;
  /** Whole percent; a bar's percents always sum to 100. */
  percent: number;
}

/**
 * Whole percents that sum to exactly 100 (largest remainder). Rounding each
 * share on its own can print 99 % or 101 % in total; ties go to the earlier
 * (higher waste-hierarchy) step.
 */
export function wholePercents(values: readonly number[]): number[] {
  const total = values.reduce((s, v) => s + v, 0);
  if (total <= 0) return values.map(() => 0);
  const exact = values.map((v) => (100 * v) / total);
  const out = exact.map((e) => Math.floor(e));
  let left = 100 - out.reduce((s, v) => s + v, 0);
  const byRemainder = exact.map((e, i) => ({ i, r: e - out[i] })).sort((a, b) => b.r - a.r || a.i - b.i);
  for (const { i } of byRemainder) {
    if (left <= 0) break;
    out[i] += 1;
    left -= 1;
  }
  return out;
}

/** The bar's segments in waste-hierarchy order — only steps that have tonnes. */
export function circularitySegments(circularity: Record<Treatment, number>): CircSegment[] {
  const tonnes = TREATMENTS.map((t) => Math.max(0, circularity[t] ?? 0));
  const percents = wholePercents(tonnes);
  return TREATMENTS.map((t, i) => ({
    treatment: t,
    label: TREATMENT_LABEL[t],
    tonnes: tonnes[i],
    percent: percents[i],
  })).filter((s) => s.tonnes > 0);
}

export interface Kpi {
  key: string;
  value: string;
  unit?: string;
  label: string;
  /** Ink for a figure that asks for action (prototype: queue warn, samples crit). */
  ink?: 'warn' | 'crit';
  hint?: string;
}

export function percentText(share: number | null): string {
  return share === null ? '—' : String(Math.round(share * 100));
}

/**
 * The five KPI tiles (R4). "Scanningsdækning" in the prototype is not
 * measured by anything in the project; "Klassificeret" is: the share of the
 * instance cloud's points that carry an instance label.
 */
export function kpis(s: SurveySummary): Kpi[] {
  const classified: Kpi =
    s.classified_share === null
      ? { key: 'classified', value: '—', label: 'Klassificeret', hint: 'Kræver instansskyen' }
      : { key: 'classified', value: percentText(s.classified_share), unit: '%', label: 'Klassificeret' };
  const reuse: Kpi =
    s.reuse_share === null
      ? { key: 'reuse', value: '—', label: 'Bevaring / genbrug' }
      : { key: 'reuse', value: percentText(s.reuse_share), unit: '%', label: 'Bevaring / genbrug' };
  const queue: Kpi = { key: 'queue', value: String(s.counts.queue), label: 'Til gennemsyn' };
  if (s.counts.queue > 0) queue.ink = 'warn';
  const samples: Kpi = { key: 'samples', value: String(s.pending_samples), label: 'Prøver afventer' };
  if (s.pending_samples > 0) samples.ink = 'crit';
  return [{ key: 'types', value: String(s.counts.all), label: 'Komponenter' }, classified, reuse, queue, samples];
}

export interface QuickLink {
  to: string;
  title: string;
  sub: string;
}

/** `undefined`: still loading; `null`: the list failed to load. */
export function versionsText(v: readonly ReportPdfVersion[] | null | undefined): string {
  if (v === undefined) return 'Henter versioner…';
  if (v === null) return 'Versioner kunne ikke hentes';
  if (v.length === 0) return 'Ingen versioner endnu';
  return v.length === 1 ? '1 version' : `${v.length} versioner`;
}

export function quickLinks(s: SurveySummary, versions: readonly ReportPdfVersion[] | null | undefined): QuickLink[] {
  return [
    { to: KORTLAEGNING_PATH, title: 'Kortlægning', sub: `${s.counts.queue} til gennemsyn · ${s.counts.all} typer` },
    {
      to: MILJOE_PATH,
      title: 'Miljø & prøver',
      sub: s.pending_samples === 1 ? '1 prøve afventer svar' : `${s.pending_samples} prøver afventer svar`,
    },
    { to: RAPPORT_PATH, title: 'Rapport', sub: versionsText(versions) },
    { to: INDBERETNING_PATH, title: 'Indberetning', sub: `${s.counts.approved} af ${s.counts.all} typer godkendt` },
  ];
}

/** The record's name (the edited one when given), else the `.rux` file stem. */
export function caseName(summary: ProjectSummary, project?: ProjectInfo): string {
  const name = (project ?? summary.projects[0])?.name?.trim();
  return name ? name : summary.path.replace(/\.rux$/i, '');
}

/** The hero's sub line: only the metadata the record has (R5). */
export function heroSubline(p: ProjectInfo | undefined): string {
  if (!p) return '';
  const parts: string[] = [];
  const address = p.building_address?.trim();
  if (address) parts.push(address);
  if (p.year_of_construction && p.year_of_construction > 0) parts.push(`opført ${p.year_of_construction}`);
  const date = p.survey_date?.trim();
  if (date) parts.push(`registreret ${date}`);
  const org = p.survey_organisation?.trim();
  if (org) parts.push(`udarbejdet af ${org}`);
  return parts.join(' · ');
}

export type MetaField = 'name' | 'building_address' | 'survey_date' | 'survey_organisation' | 'notes';

export interface MetaFieldSpec {
  key: MetaField;
  label: string;
  required?: boolean;
  multiline?: boolean;
}

/** The editor's text fields, in order; the year sits between address and date. */
export const META_FIELDS: readonly MetaFieldSpec[] = [
  { key: 'name', label: 'Navn', required: true },
  { key: 'building_address', label: 'Adresse' },
  { key: 'survey_date', label: 'Registreringsdato' },
  { key: 'survey_organisation', label: 'Udarbejdet af' },
  { key: 'notes', label: 'Noter', multiline: true },
];

export type ProjectPatch = Parameters<RuxApiClient['patchProject']>[1];

/** One field's sparse PATCH. An emptied optional field is cleared with null. */
export function metaPatch(key: MetaField, value: string): ProjectPatch {
  if (key === 'name') return { name: value };
  return { [key]: value === '' ? null : value };
}

export type YearCommit = { send: true; value: number | null } | { send: false; invalid: boolean };

export const INVALID_YEAR_TOAST = 'Byggeår skal være et årstal, fx 1978.';

/**
 * What the year field sends on blur. Unchanged → nothing (an untouched blur
 * never commits); empty or 0 → clear (null); four digits → that year;
 * anything else → nothing, `invalid` so the caller reverts and says why.
 */
export function yearCommit(draft: string, current: number | undefined): YearCommit {
  const cur = current && current > 0 ? current : null;
  const t = draft.trim();
  if (t === '' || t === '0') return cur === null ? { send: false, invalid: false } : { send: true, value: null };
  if (!/^\d{1,4}$/.test(t)) return { send: false, invalid: true };
  const n = Number(t);
  return n === cur ? { send: false, invalid: false } : { send: true, value: n };
}
```

- [ ] **Step 5: The bar** — `src/components/CircularityBar.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { formatTonnes } from '../kortlaegning/vocab';
import type { CircSegment } from '../overblik/model';
import styles from './CircularityBar.module.css';

export interface CircularityBarProps {
  segments: CircSegment[];
  /** Overblik shows the legend; Rapport's hero shows the bar alone. */
  legend?: boolean;
}

/**
 * The affaldshierarki as one stacked bar: each step's share of the tonnes in
 * its --circ-* colour. Widths are flex-grow by tonnes, so no percentage is a
 * style literal; the legend prints the whole percents (largest remainder).
 */
export function CircularityBar({ segments, legend = true }: CircularityBarProps) {
  const described = segments.map((s) => `${s.label} ${s.percent} %`).join(', ');
  return (
    <div className={styles.wrap}>
      <div className={styles.bar} role="img" aria-label={`Fordeling af materialemængde på affaldshierarkiet: ${described}`}>
        {segments.map((s) => (
          <span
            key={s.treatment}
            className={`${styles.segment} ${styles[`circ_${s.treatment}`]}`}
            style={{ flexGrow: s.tonnes }}
            title={`${s.label}: ${formatTonnes(s.tonnes)}`}
          />
        ))}
      </div>
      {legend && (
        <ul className={styles.legend}>
          {segments.map((s) => (
            <li key={s.treatment} className={styles.item}>
              <i className={`${styles.swatch} ${styles[`circ_${s.treatment}`]}`} aria-hidden="true" />
              {s.label} <b className="mono">{s.percent} %</b>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
```

`src/components/CircularityBar.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.wrap {
  display: flex;
  flex-direction: column;
  gap: var(--space-2);
}

.bar {
  display: flex;
  height: var(--space-3);
  overflow: hidden;
  border-radius: var(--radius-sm);
  background: var(--color-surface-sunken);
}

.segment {
  flex-basis: 0;
  min-width: 1px;
}

/* The prototype's white seam between steps. */
.segment + .segment {
  border-left: 2px solid var(--color-surface-raised);
}

.legend {
  display: flex;
  flex-wrap: wrap;
  gap: var(--space-2) var(--space-4);
  margin: 0;
  padding: 0;
  list-style: none;
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
}

.item {
  display: inline-flex;
  align-items: center;
  gap: var(--space-1);
}

.item b {
  color: var(--color-text);
}

.swatch {
  display: inline-block;
  width: var(--space-3);
  height: var(--space-3);
  border-radius: var(--radius-sm);
}

.circ_bevaring {
  background: var(--circ-bevaring);
}
.circ_genbrug {
  background: var(--circ-genbrug);
}
.circ_genanvendelse {
  background: var(--circ-genanvendelse);
}
.circ_nyttiggoerelse {
  background: var(--circ-nyttiggoerelse);
}
.circ_bortskaffelse {
  background: var(--circ-bortskaffelse);
}
```

- [ ] **Step 6: Run → PASS, lint, commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
npm --prefix apps/rux/frontend test -- overblik links
npm --prefix apps/rux/frontend run typecheck
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/CircularityBar.module.css apps/rux/frontend/src/components/CircularityBar.tsx --tsx
git add apps/rux/frontend/src/overblik apps/rux/frontend/src/test/overblik.model.test.ts apps/rux/frontend/src/components/CircularityBar.tsx apps/rux/frontend/src/components/CircularityBar.module.css apps/rux/frontend/src/app/links.ts
git commit -m "feat(gui): Overblik model and the shared circularity bar" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 12: KPI tiles, hero, quick links, metadata editor and the shell refresh (R5, R10, R15)

**Files:**
- Modify `src/components/StatCard.tsx` and `StatCard.module.css`.
- Create these components, each with a `.module.css`, in `src/components/overblik/`: `CaseHero.tsx`, `KpiRow.tsx`, `QuickLinks.tsx` and `ProjectMetaForm.tsx`.
- Modify `src/miljoe/useTextDraft.ts`, `src/app/SurveyCountsContext.tsx` and `src/app/AppShell.tsx`.

**Interfaces:**

```ts
// StatCard (added props)
kpi?: boolean;            // big display-face figure
unit?: string;            // set small after the figure, e.g. "%"
ink?: 'warn' | 'crit';    // figure colour for "asks for action"
// CaseHero
interface CaseHeroProps { name: string; subline: string; editing: boolean; onToggle: () => void; toggleRef: Ref<HTMLButtonElement> }
// KpiRow
interface KpiRowProps { kpis: Kpi[] }
// QuickLinks
interface QuickLinksProps { links: QuickLink[] }
// ProjectMetaForm
interface ProjectMetaFormProps {
  project: ProjectInfo | undefined;
  onCommit: (patch: ProjectPatch) => void;  // never gated on busy
  onInvalidYear: () => void;
  onClose: () => void;
}
// SurveyCountsContext
interface SurveyCounts { refresh: () => void; refreshProject: () => void }
```

- [ ] **Step 1: StatCard.** Replace `src/components/StatCard.tsx`'s props and component with:

```tsx
export interface StatCardProps {
  label: string;
  value: string | number;
  /** Secondary figure — a breakdown of `value`, not a second headline. */
  hint?: string;
  /** `muted` for a tile that is present but carries nothing yet. */
  tone?: 'default' | 'muted';
  /** Overblik's KPI look: a big display-face figure. */
  kpi?: boolean;
  /** A unit set small after the figure, e.g. "%". */
  unit?: string;
  /** Figure colour for a number that asks for action. */
  ink?: 'warn' | 'crit';
}

export function StatCard({ label, value, hint, tone = 'default', kpi = false, unit, ink }: StatCardProps) {
  const cls = [styles.card, tone === 'muted' ? styles.muted : '', kpi ? styles.kpi : '', ink ? styles[ink] : '']
    .filter(Boolean)
    .join(' ');
  return (
    <div className={cls}>
      <span className={`${styles.value} ${kpi ? '' : 'mono'}`}>
        {value}
        {unit && <small className={styles.unit}> {unit}</small>}
      </span>
      <span className={styles.label}>{label}</span>
      {hint && <span className={styles.hint}>{hint}</span>}
    </div>
  );
}
```

Append to `StatCard.module.css`:

```css
/* Overblik KPI tile (prototype `.kpi`): display face, a small unit, ink for action. */
.kpi {
  box-shadow: var(--shadow-sm);
}

.kpi .value {
  font-family: var(--font-display);
  font-size: var(--font-size-3xl);
  font-weight: var(--font-weight-medium);
}

.kpi .label {
  letter-spacing: var(--tracking-caps);
  font-weight: var(--font-weight-bold);
}

.unit {
  font-size: var(--font-size-md);
}

.warn .value {
  color: var(--tone-warn-ink);
}

.crit .value {
  color: var(--tone-crit-ink);
}
```

- [ ] **Step 2: `useTextDraft` accepts a textarea.** In `src/miljoe/useTextDraft.ts`, change the `props.onChange` type to `(e: ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => void;`. Nothing else changes, and Miljø's call sites still type-check, because a handler that takes the wider event is assignable to an input's `onChange`.

- [ ] **Step 3: Shell refresh (R15).** Replace `src/app/SurveyCountsContext.tsx`'s interface and default with:

```ts
export interface SurveyCounts {
  /** Re-fetch the survey summary behind the sidebar badges. */
  refresh: () => void;
  /** Re-fetch the project summary behind the sidebar's project name (Overblik's hero editor). */
  refreshProject: () => void;
}

const SurveyCountsContext = createContext<SurveyCounts>({ refresh: () => {}, refreshProject: () => {} });
```

and extend its file comment: "…without the two sharing survey state. `refreshProject` does the same for the project name after a metadata edit."

In `src/app/AppShell.tsx`, replace `const { data: summary } = useAsync<ProjectSummary>((signal) => api.projectSummary(signal), []);` with:

```ts
  const project = useAsync<ProjectSummary>((signal) => api.projectSummary(signal), []);
  const summary = project.data;
```

Then replace the `surveyCounts` memo with:

```ts
  const surveyCounts = useMemo(
    () => ({ refresh: survey.reload, refreshProject: project.reload }),
    [survey.reload, project.reload],
  );
```

- [ ] **Step 4: Hero** — `src/components/overblik/CaseHero.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Ref } from 'react';

import styles from './CaseHero.module.css';

export interface CaseHeroProps {
  name: string;
  /** Joined metadata, or '' when the record has none yet. */
  subline: string;
  editing: boolean;
  onToggle: () => void;
  /** Focus returns here when the editor closes. */
  toggleRef: Ref<HTMLButtonElement>;
}

/** The case's navy hero (prototype `.hero`): name, metadata line, edit toggle. */
export function CaseHero({ name, subline, editing, onToggle, toggleRef }: CaseHeroProps) {
  return (
    <section className={styles.hero} aria-labelledby="case-name">
      <h2 id="case-name" className={styles.name}>
        {name}
      </h2>
      <p className={subline ? styles.sub : `${styles.sub} ${styles.empty}`}>
        {subline || 'Ingen sagsoplysninger endnu — adresse, byggeår og registrering tilføjes her.'}
      </p>
      <button
        ref={toggleRef}
        type="button"
        className={styles.edit}
        aria-expanded={editing}
        aria-controls="case-meta-form"
        onClick={onToggle}
      >
        {editing ? 'Luk redigering' : 'Rediger sagsoplysninger'}
      </button>
    </section>
  );
}
```

`CaseHero.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* Navy in both themes, like the chrome; no aerial photo (R5). */
.hero {
  display: flex;
  flex-direction: column;
  gap: var(--space-2);
  padding: var(--space-5);
  border: 1px solid var(--color-chrome-border);
  border-radius: var(--radius-lg);
  background: var(--color-chrome);
  box-shadow: var(--shadow-sm);
}

.name {
  margin: 0;
  font-size: var(--font-size-3xl);
  text-transform: uppercase;
  color: var(--color-on-chrome);
}

.sub {
  margin: 0;
  font-size: var(--font-size-sm);
  color: var(--color-on-chrome-muted);
}

.empty {
  font-style: italic;
}

.edit {
  composes: textBtn from '../controls.module.css';
  align-self: flex-start;
  font-size: var(--font-size-sm);
  color: var(--color-on-chrome);
}

.edit:focus-visible {
  outline-color: var(--color-on-chrome);
}
```

- [ ] **Step 5: KPI row and quick links** — `src/components/overblik/KpiRow.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Kpi } from '../../overblik/model';
import { StatCard } from '../StatCard';
import styles from './KpiRow.module.css';

export interface KpiRowProps {
  kpis: Kpi[];
}

/** The five KPI tiles; they wrap rather than shrink below a readable width. */
export function KpiRow({ kpis }: KpiRowProps) {
  return (
    <ul className={styles.row} aria-label="Nøgletal">
      {kpis.map((k) => (
        <li key={k.key}>
          <StatCard kpi label={k.label} value={k.value} unit={k.unit} ink={k.ink} hint={k.hint} />
        </li>
      ))}
    </ul>
  );
}
```

`KpiRow.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.row {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(calc(var(--space-7) * 3), 1fr));
  gap: var(--space-3);
  margin: 0;
  padding: 0;
  list-style: none;
}
```

`src/components/overblik/QuickLinks.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link } from 'react-router-dom';

import type { QuickLink } from '../../overblik/model';
import styles from './QuickLinks.module.css';

export interface QuickLinksProps {
  links: QuickLink[];
}

/** The four case screens, each with what is waiting there. */
export function QuickLinks({ links }: QuickLinksProps) {
  return (
    <nav aria-label="Sagens skærme">
      <ul className={styles.grid}>
        {links.map((l) => (
          <li key={l.to}>
            <Link to={l.to} className={styles.link}>
              <span className={styles.title}>{l.title}</span>
              <span className={styles.sub}>{l.sub}</span>
            </Link>
          </li>
        ))}
      </ul>
    </nav>
  );
}
```

`QuickLinks.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(calc(var(--space-7) * 4), 1fr));
  gap: var(--space-3);
  margin: 0;
  padding: 0;
  list-style: none;
}

.link {
  composes: panel from '../surfaces.module.css';
  display: flex;
  flex-direction: column;
  gap: var(--space-1);
  height: 100%;
  padding: var(--space-3) var(--space-4);
  color: var(--color-text);
  text-decoration: none;
}

.link:hover {
  border-color: var(--color-accent);
}

.link:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: 2px;
}

.title {
  font-size: var(--font-size-md);
  font-weight: var(--font-weight-bold);
}

.sub {
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
}
```

- [ ] **Step 6: The editor** — `src/components/overblik/ProjectMetaForm.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Fragment, useEffect, useRef, useState, type KeyboardEvent } from 'react';

import type { ProjectInfo } from '../../api/types';
import { kindOf } from '../../app/keyTargets';
import { editorKeyAction } from '../../miljoe/model';
import { useTextDraft } from '../../miljoe/useTextDraft';
import { META_FIELDS, metaPatch, yearCommit, type MetaFieldSpec, type ProjectPatch } from '../../overblik/model';
import styles from './ProjectMetaForm.module.css';

export interface ProjectMetaFormProps {
  project: ProjectInfo | undefined;
  /** One field's sparse patch. Never gated on busy: the page queues it. */
  onCommit: (patch: ProjectPatch) => void;
  onInvalidYear: () => void;
  onClose: () => void;
}

/**
 * The case details, edited in place (R5). Each field commits on blur. An
 * untouched blur sends nothing. Esc in a field drops its draft without
 * sending (R10). Esc elsewhere in the form closes it. Enter in a single-line
 * field commits by leaving the field.
 */
export function ProjectMetaForm({ project, onCommit, onInvalidYear, onClose }: ProjectMetaFormProps) {
  return (
    <section
      id="case-meta-form"
      className={styles.form}
      aria-label="Sagsoplysninger"
      onKeyDown={(e) => {
        const action = editorKeyAction({ key: e.key, kind: kindOf(e.target), ctrlKey: e.ctrlKey, metaKey: e.metaKey, altKey: e.altKey });
        if (action === 'close') {
          e.preventDefault();
          onClose();
        }
      }}
    >
      <div className={styles.grid}>
        {META_FIELDS.map((spec) => (
          <Fragment key={spec.key}>
            <MetaTextField spec={spec} current={project?.[spec.key] ?? ''} onCommit={onCommit} />
            {spec.key === 'building_address' && (
              <MetaYearField current={project?.year_of_construction} onCommit={onCommit} onInvalid={onInvalidYear} />
            )}
          </Fragment>
        ))}
      </div>
      <p className={styles.hint}>Ændringer gemmes, når du forlader feltet. Esc fortryder feltet; Esc igen lukker.</p>
    </section>
  );
}

function MetaTextField({
  spec,
  current,
  onCommit,
}: {
  spec: MetaFieldSpec;
  current: string;
  onCommit: (patch: ProjectPatch) => void;
}) {
  const draft = useTextDraft(current, (value) => onCommit(metaPatch(spec.key, value)), spec.required);
  const id = `meta-${spec.key}`;
  const onKeyDown = (e: KeyboardEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    const action = editorKeyAction({ key: e.key, kind: kindOf(e.target), ctrlKey: e.ctrlKey, metaKey: e.metaKey, altKey: e.altKey });
    if (action === 'revert') {
      e.preventDefault();
      e.stopPropagation(); // the form's Esc-closes must not also fire
      draft.revert(e.currentTarget);
    } else if (action === 'commit' && !spec.multiline) {
      e.preventDefault();
      e.currentTarget.blur();
    }
  };
  return (
    <div className={spec.multiline ? `${styles.field} ${styles.wide}` : styles.field}>
      <label className={styles.label} htmlFor={id}>
        {spec.label}
      </label>
      {spec.multiline ? (
        <textarea id={id} className={styles.textarea} rows={3} {...draft.props} onKeyDown={onKeyDown} />
      ) : (
        <input id={id} type="text" className={styles.input} {...draft.props} onKeyDown={onKeyDown} />
      )}
    </div>
  );
}

function MetaYearField({
  current,
  onCommit,
  onInvalid,
}: {
  current: number | undefined;
  onCommit: (patch: ProjectPatch) => void;
  onInvalid: () => void;
}) {
  const shown = current && current > 0 ? String(current) : '';
  const [draft, setDraft] = useState(shown);
  const focused = useRef(false);
  const skipNextCommit = useRef(false);
  useEffect(() => {
    if (!focused.current) setDraft(shown);
  }, [shown]);

  return (
    <div className={styles.field}>
      <label className={styles.label} htmlFor="meta-year">
        Opført (år)
      </label>
      <input
        id="meta-year"
        type="text"
        inputMode="numeric"
        className={`${styles.input} mono`}
        value={draft}
        onChange={(e) => setDraft(e.target.value)}
        onFocus={() => {
          focused.current = true;
        }}
        onBlur={() => {
          focused.current = false;
          if (skipNextCommit.current) {
            skipNextCommit.current = false;
            return;
          }
          const c = yearCommit(draft, current);
          if (c.send) {
            onCommit({ year_of_construction: c.value });
            return;
          }
          if (c.invalid) onInvalid();
          setDraft(shown);
        }}
        onKeyDown={(e) => {
          const action = editorKeyAction({ key: e.key, kind: kindOf(e.target), ctrlKey: e.ctrlKey, metaKey: e.metaKey, altKey: e.altKey });
          if (action === 'revert') {
            e.preventDefault();
            e.stopPropagation();
            skipNextCommit.current = true;
            setDraft(shown);
            e.currentTarget.blur();
          } else if (action === 'commit') {
            e.preventDefault();
            e.currentTarget.blur();
          }
        }}
      />
    </div>
  );
}
```

`ProjectMetaForm.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.form {
  composes: panel from '../surfaces.module.css';
  display: flex;
  flex-direction: column;
  gap: var(--space-3);
  padding: var(--space-4);
}

.grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(calc(var(--space-7) * 5), 1fr));
  gap: var(--space-3);
}

.field {
  composes: field from '../controls.module.css';
}

.wide {
  grid-column: 1 / -1;
}

.label {
  composes: fieldLabel from '../controls.module.css';
}

.input {
  composes: input from '../controls.module.css';
}

.textarea {
  composes: input from '../controls.module.css';
  resize: vertical;
}

.hint {
  margin: 0;
  font-size: var(--font-size-xs);
  color: var(--color-text-faint);
}
```

`project?.[spec.key]` covers `survey_date`, `survey_organisation` and `notes`, which are all `string | undefined` on `ProjectInfo`, and the `?? ''` makes them a string.

- [ ] **Step 7: Check and commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend test
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/StatCard.module.css apps/rux/frontend/src/components/overblik apps/rux/frontend/src/components/StatCard.tsx --tsx
git add apps/rux/frontend/src/components/StatCard.tsx apps/rux/frontend/src/components/StatCard.module.css apps/rux/frontend/src/components/overblik apps/rux/frontend/src/miljoe/useTextDraft.ts apps/rux/frontend/src/app/SurveyCountsContext.tsx apps/rux/frontend/src/app/AppShell.tsx
git commit -m "feat(gui): Overblik hero, KPI tiles, quick links and case-details editor" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 13: Overblik at `/`; the old Dashboard becomes Projektdata

**Files:**
- Create `src/routes/OverblikPage.tsx` and `OverblikPage.module.css`.
- Modify `src/app/App.tsx`, `src/app/navigation.ts`, `src/test/navigation.test.ts` and `src/routes/Dashboard.tsx` (doc comment only).

**Interfaces:**
- `/` renders `OverblikPage`.
- `/projektdata` renders the unchanged `Dashboard`.
- The navigation gains `{ to: '/projektdata', label: 'Projektdata', group: 'tools' }` as the first tool.

- [ ] **Step 1: Navigation test first.** In `src/test/navigation.test.ts`:
  - add `'/projektdata',` to the path list in `keeps every existing technical route reachable under Værktøjer`;
  - add:

```ts
  it('keeps the old project inventory reachable as the first tool', () => {
    const tools = entriesIn('tools');
    expect(tools[0]).toEqual({ to: '/projektdata', label: 'Projektdata', group: 'tools' });
  });
```

Run `npm --prefix apps/rux/frontend test -- navigation` → FAIL. In `src/app/navigation.ts`, insert `{ to: '/projektdata', label: 'Projektdata', group: 'tools' },` as the first `tools` entry, before `/viewport`. Run again → PASS.

- [ ] **Step 2: The page** — `src/routes/OverblikPage.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useRef, useState } from 'react';

import { api } from '../api/client';
import type { ProjectInfo } from '../api/types';
import { saveErrorMessage } from '../app/saveError';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { useSurveyCounts } from '../app/SurveyCountsContext';
import { useToast } from '../app/useToast';
import { CircularityBar } from '../components/CircularityBar';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { CaseHero } from '../components/overblik/CaseHero';
import { KpiRow } from '../components/overblik/KpiRow';
import { ProjectMetaForm } from '../components/overblik/ProjectMetaForm';
import { QuickLinks } from '../components/overblik/QuickLinks';
import {
  caseName,
  circularitySegments,
  heroSubline,
  INVALID_YEAR_TOAST,
  kpis,
  quickLinks,
  type ProjectPatch,
} from '../overblik/model';
import styles from './OverblikPage.module.css';

/**
 * Overblik — the case dashboard and the app's landing route: the case hero
 * with an in-place editor for its details, the KPI row, the circularity bar
 * and a link to each case screen with what waits there.
 *
 * Three reads at mount (project, survey summary, report versions). The busy
 * fix in `rux gui` (Phase 5 R1) is what keeps them from racing into 503s.
 * Metadata edits run on the page's mutation queue, and each settles by
 * re-reading the shell's project summary, so the sidebar name follows.
 */
export function OverblikPage() {
  const { data, error, loading, reload } = useAsync(
    (s) => Promise.all([api.projectSummary(s), api.surveySummary(s)]),
    [],
  );
  const versions = useAsync((s) => api.listReportVersions(s), []);
  const { refreshProject } = useSurveyCounts();
  const toast = useToast(2600);
  const { mutate } = useMutationQueue({
    onError: (cause) => toast.show(saveErrorMessage(cause)),
    onSettled: refreshProject,
  });

  // The record being edited. A ref mirror, so a queued commit reads the id the
  // previous commit's response produced, not a stale render's.
  const [project, setProjectState] = useState<ProjectInfo | undefined>(undefined);
  const projectRef = useRef<ProjectInfo | undefined>(undefined);
  const setProject = useCallback((p: ProjectInfo | undefined) => {
    projectRef.current = p;
    setProjectState(p);
  }, []);
  useEffect(() => {
    if (data) setProject(data[0].projects[0]);
  }, [data, setProject]);

  // PATCH /projects/{id} upserts, so a project with no record yet gets one on
  // its first edit, under an id minted once per visit.
  const newId = useRef<string>(crypto.randomUUID());
  const commit = useCallback(
    (patch: ProjectPatch) => {
      mutate(async () => {
        const id = projectRef.current?.id ?? newId.current;
        setProject(await api.patchProject(id, patch));
      });
    },
    [mutate, setProject],
  );

  const [editing, setEditing] = useState(false);
  const toggleRef = useRef<HTMLButtonElement>(null);
  const closeEditor = useCallback(() => {
    setEditing(false);
    toggleRef.current?.focus(); // never drop focus to <body>
  }, []);

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="sagsoverblikket" />
      </div>
    );
  }
  if (loading && !data) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser sagen…" />
      </div>
    );
  }
  if (!data) return null;

  const [summary, survey] = data;
  const segments = circularitySegments(survey.circularity);

  return (
    <div className={styles.page}>
      <CaseHero
        name={caseName(summary, project)}
        subline={heroSubline(project)}
        editing={editing}
        onToggle={() => (editing ? closeEditor() : setEditing(true))}
        toggleRef={toggleRef}
      />
      {editing && (
        <ProjectMetaForm
          project={project}
          onCommit={commit}
          onInvalidYear={() => toast.show(INVALID_YEAR_TOAST)}
          onClose={closeEditor}
        />
      )}

      <KpiRow kpis={kpis(survey)} />

      <section className={styles.panel} aria-labelledby="cirk-heading">
        <h3 id="cirk-heading" className={styles.panelHeading}>
          Cirkularitetsoversigt
        </h3>
        {segments.length > 0 ? (
          <CircularityBar segments={segments} />
        ) : (
          <EmptyState
            bare
            title="Ingen mængder endnu"
            detail="Tonnage pr. type sættes i Kortlægning; bjælken viser den fordelt på affaldshierarkiet."
          />
        )}
      </section>

      <QuickLinks links={quickLinks(survey, versions.error ? null : versions.data)} />
      <Toast message={toast.message} />
    </div>
  );
}
```

`src/routes/OverblikPage.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.page {
  composes: page from './viewHead.module.css';
}

.panel {
  composes: panel from '../components/surfaces.module.css';
  display: flex;
  flex-direction: column;
  gap: var(--space-3);
  padding: var(--space-4);
}

.panelHeading {
  composes: panelHeading from '../components/surfaces.module.css';
}
```

- [ ] **Step 3: Routes.** In `src/app/App.tsx`:
  - add `import { OverblikPage } from '../routes/OverblikPage';`;
  - change `<Route path="/" element={<Dashboard />} />` to `<Route path="/" element={<OverblikPage />} />`;
  - add `<Route path="/projektdata" element={<Dashboard />} />` directly after it.

  In `src/routes/Dashboard.tsx`, change the doc comment's first line "Project overview — the app's landing route." to "Project data (Værktøjer › Projektdata) — the technical inventory that was the landing route until Overblik replaced it (Phase 5)."

- [ ] **Step 4: Check and commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/routes/OverblikPage.module.css apps/rux/frontend/src/routes/OverblikPage.tsx --tsx
git add apps/rux/frontend/src/routes/OverblikPage.tsx apps/rux/frontend/src/routes/OverblikPage.module.css apps/rux/frontend/src/app/App.tsx apps/rux/frontend/src/app/navigation.ts apps/rux/frontend/src/test/navigation.test.ts apps/rux/frontend/src/routes/Dashboard.tsx
git commit -m "feat(gui): Overblik is the landing route; the inventory moves to Projektdata" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 14: Rapport (R7, R8, R13)

**Files:**
- Create `src/rapport/model.ts`, `src/test/rapport.model.test.ts`, `src/components/rapport/VersionList.tsx` + `.module.css` and `src/routes/RapportPage.tsx` + `.module.css`.
- Modify `src/app/App.tsx`, `src/app/navigation.ts`, `src/test/navigation.test.ts`, `src/routes/ExportPage.tsx` and `ExportPage.module.css`.

**Interfaces:**

```ts
// src/rapport/model.ts
export function parseServerTime(s: string): Date | null;
export function versionDate(s: string): string;             // "09.08.2026"
export function formatBytesDa(bytes: number): string;       // "2,4 MB"
export function versionTitle(v: ReportPdfVersion): string;  // "Ressourcekortlægning — v3"
export interface VersionStatus { tone: Tone; text: string; title: string }
export function versionStatus(v: ReportPdfVersion): VersionStatus | null;
export function reportHeroSub(s: SurveySummary): string;
export function draftNotice(f: SurveyFractions): string | null;
export function generatedToast(v: ReportPdfVersion): string;
export function generateErrorMessage(cause: unknown): string;
export const REPORT_FOOTNOTE: string;
```

- [ ] **Step 1: Failing tests** — `src/test/rapport.model.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import {
  draftNotice,
  formatBytesDa,
  generatedToast,
  generateErrorMessage,
  parseServerTime,
  REPORT_FOOTNOTE,
  reportHeroSub,
  versionDate,
  versionStatus,
  versionTitle,
} from '../rapport/model';
import { reportVersion, surveyFractions, surveySummary } from './surveyFixtures';

describe('server time', () => {
  it('reads sqlite datetime as UTC, and ISO as given', () => {
    expect(parseServerTime('2026-08-09 10:05:00')?.toISOString()).toBe('2026-08-09T10:05:00.000Z');
    expect(parseServerTime('2026-08-09T10:05:00Z')?.toISOString()).toBe('2026-08-09T10:05:00.000Z');
    expect(parseServerTime('igår')).toBeNull();
  });

  it('formats the version date like the prototype', () => {
    expect(versionDate('2026-08-09 10:05:00')).toBe('09.08.2026');
    expect(versionDate('igår')).toBe('igår');
  });
});

describe('version rows', () => {
  it('sizes in Danish units', () => {
    expect(formatBytesDa(15)).toBe('15 B');
    expect(formatBytesDa(86016)).toBe('84 KB');
    expect(formatBytesDa(2516582)).toBe('2,4 MB');
  });

  it('titles a version with its ordinal', () => {
    expect(versionTitle(reportVersion({ version: 3 }))).toBe('Ressourcekortlægning — v3');
    expect(versionTitle(reportVersion({ label: '', version: 1 }))).toBe('Ressourcekortlægning — v1');
  });

  it('marks complete, draft, and unknown', () => {
    expect(versionStatus(reportVersion({ blocking_types: 0 }))).toEqual({
      tone: 'good',
      text: 'Komplet',
      title: 'Alle typer var godkendt og afklaret, da versionen blev genereret.',
    });
    expect(versionStatus(reportVersion({ blocking_types: 7 }))).toEqual({
      tone: 'wait',
      text: 'Udkast',
      title: '7 typer var ikke godkendt eller afventede prøvesvar.',
    });
    expect(versionStatus(reportVersion({ blocking_types: 1 }))?.title).toBe(
      '1 type var ikke godkendt eller afventede prøvesvar.',
    );
    expect(versionStatus(reportVersion({ blocking_types: null }))).toBeNull();
  });
});

describe('hero and notices', () => {
  it('summarises the case like the prototype', () => {
    expect(reportHeroSub(surveySummary())).toBe('11 komponenter · 54 % bevaring/genbrug · 1 forurenet · 2 prøver afventer');
    expect(reportHeroSub(surveySummary({ reuse_share: null, pending_samples: 1 }))).toBe(
      '11 komponenter · — bevaring/genbrug · 1 forurenet · 1 prøve afventer',
    );
  });

  it('warns that a new version will be a draft while types block', () => {
    expect(draftNotice(surveyFractions())).toBe(
      '7 typer er ikke godkendt eller afventer prøvesvar — en ny version bliver et udkast, og de indgår ikke i mængderne.',
    );
    expect(draftNotice(surveyFractions({ ready: true, blocking: [], blocking_types: 0 }))).toBeNull();
  });

  it('confirms a generation and says why one failed', () => {
    expect(generatedToast(reportVersion({ version: 3, blocking_types: 0 }))).toBe('Ressourcekortlægning — v3 genereret');
    expect(generatedToast(reportVersion({ version: 1, blocking_types: 7 }))).toBe('Ressourcekortlægning — v1 genereret (udkast)');
    expect(generateErrorMessage(new ApiRequestError(409, 'job', '/r'))).toBe(
      'Kunne ikke generere — et pipeline-job kører. Prøv igen om lidt.',
    );
    expect(generateErrorMessage(new ApiRequestError(503, 'busy', '/r'))).toBe(
      'Kunne ikke generere — projektet skrives til lige nu. Prøv igen om lidt.',
    );
    expect(generateErrorMessage(new ApiRequestError(500, 'PDF generation failed: typst not found', '/r'))).toBe(
      'Kunne ikke generere rapporten: PDF generation failed: typst not found',
    );
  });

  it('states the report rules without the MRK signature', () => {
    expect(REPORT_FOOTNOTE).toBe(
      'Kun godkendte mængder indgår i rapportens kortlægningsafsnit. Versioner er uforanderlige — en ny generering giver en ny version med tidsstempel. Inventarlisten er en aktuel eksport og gemmes ikke som version.',
    );
  });
});
```

- [ ] **Step 2: Run → FAIL** with `npm --prefix apps/rux/frontend test -- rapport`.

- [ ] **Step 3: Model** — `src/rapport/model.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Rapport as data: how a stored version reads (date, size, title, complete or
 * draft), the hero line, the draft warning and the toasts. `blocking_types`
 * and `version` are the server's (schema v23); this only words them.
 */

import { ApiRequestError } from '../api/client';
import type { ReportPdfVersion, SurveyFractions, SurveySummary } from '../api/types';
import { errorMessage } from '../app/saveError';
import type { Tone } from '../kortlaegning/vocab';

const SQLITE_DATETIME = /^\d{4}-\d{2}-\d{2} \d{2}:\d{2}(:\d{2})?$/;

/** sqlite's `datetime('now')` ("2026-08-09 10:05:00", UTC, no zone) or ISO 8601. */
export function parseServerTime(s: string): Date | null {
  const iso = SQLITE_DATETIME.test(s) ? `${s.replace(' ', 'T')}Z` : s;
  const d = new Date(iso);
  return Number.isNaN(d.getTime()) ? null : d;
}

/** "09.08.2026" in local time; the raw string when it does not parse. */
export function versionDate(s: string): string {
  const d = parseServerTime(s);
  return d ? d.toLocaleDateString('da-DK', { day: '2-digit', month: '2-digit', year: 'numeric' }) : s;
}

export function formatBytesDa(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  const kb = bytes / 1024;
  if (kb < 1024) return `${Math.round(kb).toLocaleString('da-DK')} KB`;
  return `${(kb / 1024).toLocaleString('da-DK', { minimumFractionDigits: 1, maximumFractionDigits: 1 })} MB`;
}

export function versionTitle(v: ReportPdfVersion): string {
  return `${v.label.trim() || 'Ressourcekortlægning'} — v${v.version}`;
}

export interface VersionStatus {
  tone: Tone;
  text: string;
  title: string;
}

/** Complete / draft at generation time (R7); null for a pre-v23 version. */
export function versionStatus(v: ReportPdfVersion): VersionStatus | null {
  if (v.blocking_types === null) return null;
  if (v.blocking_types === 0) {
    return { tone: 'good', text: 'Komplet', title: 'Alle typer var godkendt og afklaret, da versionen blev genereret.' };
  }
  const n = v.blocking_types;
  return {
    tone: 'wait',
    text: 'Udkast',
    title: `${n === 1 ? '1 type' : `${n} typer`} var ikke godkendt eller afventede prøvesvar.`,
  };
}

/** The hero line: "11 komponenter · 54 % bevaring/genbrug · 1 forurenet · 2 prøver afventer". */
export function reportHeroSub(s: SurveySummary): string {
  const reuse = s.reuse_share === null ? '—' : `${Math.round(s.reuse_share * 100)} %`;
  return [
    `${s.counts.all} komponenter`,
    `${reuse} bevaring/genbrug`,
    `${s.contaminated_types} forurenet`,
    s.pending_samples === 1 ? '1 prøve afventer' : `${s.pending_samples} prøver afventer`,
  ].join(' · ');
}

export function draftNotice(f: SurveyFractions): string | null {
  if (f.ready) return null;
  const n = f.blocking_types;
  return `${n === 1 ? '1 type er' : `${n} typer er`} ikke godkendt eller afventer prøvesvar — en ny version bliver et udkast, og de indgår ikke i mængderne.`;
}

export function generatedToast(v: ReportPdfVersion): string {
  return `${versionTitle(v)} genereret${v.blocking_types ? ' (udkast)' : ''}`;
}

export function generateErrorMessage(cause: unknown): string {
  if (cause instanceof ApiRequestError) {
    if (cause.status === 409) return 'Kunne ikke generere — et pipeline-job kører. Prøv igen om lidt.';
    if (cause.status === 503) return 'Kunne ikke generere — projektet skrives til lige nu. Prøv igen om lidt.';
  }
  return `Kunne ikke generere rapporten: ${errorMessage(cause)}`;
}

/** The prototype's footnote without the MRK signature, which nothing models (R7). */
export const REPORT_FOOTNOTE =
  'Kun godkendte mængder indgår i rapportens kortlægningsafsnit. Versioner er uforanderlige — en ny generering giver en ny version med tidsstempel. Inventarlisten er en aktuel eksport og gemmes ikke som version.';
```

- [ ] **Step 4: Version list** — `src/components/rapport/VersionList.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ReportPdfVersion } from '../../api/types';
import { formatBytesDa, versionDate, versionStatus, versionTitle } from '../../rapport/model';
import { Pill } from '../Pill';
import styles from './VersionList.module.css';

export interface VersionListProps {
  /** Newest first, as the server lists them. */
  versions: ReportPdfVersion[];
  pdfUrl: (id: number) => string;
  /** The live material-passport CSV (`/exports/csv`): the prototype's Inventarliste. */
  inventoryUrl: string;
}

/** Stored PDF versions, then the live inventory export as a last, fixed row (R7). */
export function VersionList({ versions, pdfUrl, inventoryUrl }: VersionListProps) {
  return (
    <ul className={styles.list} aria-label="Rapportversioner">
      {versions.map((v) => {
        const status = versionStatus(v);
        const title = versionTitle(v);
        return (
          <li key={v.id} className={styles.row}>
            <span className={styles.format}>PDF</span>
            <span className={styles.main}>
              <span className={styles.name}>{title}</span>
              <span className={`${styles.meta} mono`}>
                {versionDate(v.created_at)} · {formatBytesDa(v.size_bytes)}
              </span>
            </span>
            <span className={styles.end}>
              {status && (
                <Pill tone={status.tone} title={status.title}>
                  {status.text}
                </Pill>
              )}
              <a
                className={styles.download}
                href={pdfUrl(v.id)}
                download={`ressourcekortlaegning-v${v.version}.pdf`}
                aria-label={`Hent ${title}`}
              >
                ↓
              </a>
            </span>
          </li>
        );
      })}
      <li className={styles.row}>
        <span className={styles.format}>CSV</span>
        <span className={styles.main}>
          <span className={styles.name}>Inventarliste</span>
          <span className={styles.meta}>Aktuel eksport af materialepassene</span>
        </span>
        <span className={styles.end}>
          <Pill tone="accent">Eksport</Pill>
          <a className={styles.download} href={inventoryUrl} download="inventarliste.csv" aria-label="Hent inventarliste (CSV)">
            ↓
          </a>
        </span>
      </li>
    </ul>
  );
}
```

`VersionList.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.list {
  composes: panel from '../surfaces.module.css';
  display: flex;
  flex-direction: column;
  margin: 0;
  padding: 0;
  list-style: none;
  overflow: hidden;
}

.row {
  display: flex;
  align-items: center;
  gap: var(--space-3);
  padding: var(--space-3) var(--space-4);
  font-size: var(--font-size-sm);
}

.row + .row {
  border-top: 1px solid var(--color-border);
}

.format {
  flex: none;
  padding: 0 var(--space-1);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-sm);
  font-size: var(--font-size-2xs);
  font-weight: var(--font-weight-bold);
  color: var(--color-text-muted);
}

.main {
  display: flex;
  flex-direction: column;
  min-width: 0;
}

.name {
  font-weight: var(--font-weight-bold);
  overflow-wrap: anywhere;
}

.meta {
  font-size: var(--font-size-xs);
  color: var(--color-text-faint);
}

.end {
  display: flex;
  flex: none;
  align-items: center;
  gap: var(--space-2);
  margin-left: auto;
}

.download {
  composes: btnGhost from '../controls.module.css';
  text-decoration: none;
}
```

- [ ] **Step 5: The page** — `src/routes/RapportPage.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useState } from 'react';

import { api } from '../api/client';
import type { ReportPdfVersion } from '../api/types';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { useToast } from '../app/useToast';
import { CircularityBar } from '../components/CircularityBar';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { VersionList } from '../components/rapport/VersionList';
import { caseName, circularitySegments } from '../overblik/model';
import { draftNotice, generatedToast, generateErrorMessage, REPORT_FOOTNOTE, reportHeroSub } from '../rapport/model';
import styles from './RapportPage.module.css';

/**
 * Rapport — the Ressourcekortlægning report versions (prototype 02d).
 *
 * The hero sums the case up, and the list holds every stored PDF, newest first,
 * marked complete or draft by how many types still blocked it when it was
 * generated. `Generér ny version` runs on the page's mutation queue. The same
 * queued task re-reads the list, so the new row and its number come from the
 * server.
 */
export function RapportPage() {
  const { data, error, loading, reload } = useAsync(
    (s) => Promise.all([api.projectSummary(s), api.surveySummary(s), api.surveyFractions(s)]),
    [],
  );
  const listed = useAsync((s) => api.listReportVersions(s), []);
  const [versions, setVersions] = useState<ReportPdfVersion[] | null>(null);
  useEffect(() => {
    if (listed.data) setVersions(listed.data);
  }, [listed.data]);

  const toast = useToast(3200);
  const { busy, mutate } = useMutationQueue({ onError: (cause) => toast.show(generateErrorMessage(cause)) });
  const generate = () =>
    mutate(async () => {
      const created = await api.generateReport();
      setVersions(await api.listReportVersions());
      toast.show(generatedToast(created));
    });

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="rapportgrundlaget" />
      </div>
    );
  }
  if (loading && !data) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser rapporten…" />
      </div>
    );
  }
  if (!data) return null;

  const [summary, survey, fractions] = data;
  const notice = draftNotice(fractions);
  const segments = circularitySegments(survey.circularity);

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h2 className={styles.title}>Rapport</h2>
        <span className={styles.sub}>Ressourcekortlægningsrapport</span>
        <div className={styles.actions}>
          <button type="button" className={styles.btnPrimary} onClick={generate} disabled={busy}>
            {busy ? 'Genererer…' : 'Generér ny version'}
          </button>
        </div>
      </header>

      {notice && <p className={styles.notice}>{notice}</p>}

      <section className={styles.hero} aria-labelledby="rapport-hero">
        <h3 id="rapport-hero" className={styles.heroTitle}>
          Ressourcekortlægning — {caseName(summary)}
        </h3>
        <p className={styles.heroSub}>{reportHeroSub(survey)}</p>
        {segments.length > 0 && <CircularityBar segments={segments} legend={false} />}
      </section>

      {listed.error ? (
        <ErrorBanner error={listed.error} onRetry={listed.reload} context="rapportversionerne" />
      ) : versions === null ? (
        <Spinner label="Indlæser versioner…" />
      ) : (
        <>
          {versions.length === 0 && (
            <EmptyState
              title="Ingen versioner endnu"
              detail="Generér den første version — den gemmes i projektet og kan hentes her igen."
            />
          )}
          <VersionList versions={versions} pdfUrl={(id) => api.reportPdfUrl(id)} inventoryUrl={api.csvExportUrl()} />
        </>
      )}

      <p className={styles.footnote}>{REPORT_FOOTNOTE}</p>
      <Toast message={toast.message} />
    </div>
  );
}
```

`src/routes/RapportPage.module.css`:

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
.actions {
  composes: actions from './viewHead.module.css';
}
.footnote {
  composes: footnote from './viewHead.module.css';
}
.btnPrimary {
  composes: btnPrimary from '../components/controls.module.css';
}
.notice {
  composes: notice from '../components/surfaces.module.css';
}

.hero {
  composes: panel from '../components/surfaces.module.css';
  display: flex;
  flex-direction: column;
  gap: var(--space-2);
  padding: var(--space-4);
}

.heroTitle {
  margin: 0;
  font-size: var(--font-size-xl);
  text-transform: uppercase;
}

.heroSub {
  margin: 0;
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
}
```

- [ ] **Step 6: Route and live entry.**
  - **`navigation.test.ts`:** add the test below, run it → FAIL, then make the entry change and run it → PASS. Also change the `gives pending entries a reason a user can read` test so that it still passes when no `sag` entry is pending. It already loops over whichever entries are pending, so it needs no change.

```ts
  it('makes Rapport a live destination', () => {
    expect(NAV_ENTRIES.find((e) => e.to === '/rapport')?.pending).toBeUndefined();
  });
```

  - **`navigation.ts`:** replace the `/rapport` entry with `{ to: '/rapport', label: 'Rapport', group: 'sag' },`.
  - **`App.tsx`:** import `RapportPage` and add `<Route path="/rapport" element={<RapportPage />} />` after the `/miljoe` route.

- [ ] **Step 7: Export keeps CSV and preview only (R13).** In `src/routes/ExportPage.tsx`:
  1. **Remove:**
     - from the types import: `ReportPdfVersion`;
     - the `WriteBanner` import;
     - the `describeWriteFailure` and `WriteFailure` imports;
     - the `formatBytes` and `formatTimestamp` functions;
     - in `ReportView`, the "Version history" state (`versionsKey`, `versionsAsync`) and the "Generate PDF state" block (`generating`, `generateFailure`, `handleGenerate`, `handleRetryGenerate`, `handleDismissFailure`).
  2. **In `ReportView`'s JSX:** replace everything from `{/* Screen-only toolbar */}` through the closing `</section>` of `{/* Version history */}` with:

```tsx
      <p className={styles.hint}>
        PDF reports (Ressourcekortlægning) are generated and stored on <Link to={RAPPORT_PATH}>Rapport →</Link>
      </p>
```

  3. **Imports:** add `import { Link } from 'react-router-dom';` and `import { RAPPORT_PATH } from '../app/links';`.
  4. **Doc comment:** replace the component's doc comment with: "CSV export (#459) and an on-page HTML preview of the passports. The server-side Typst PDF and its stored versions live on Rapport (`/rapport`, GUI Phase 5); this tool screen links there."
  5. **CSS:** delete the rules in `ExportPage.module.css` that nothing references any more. List them with the command below, delete each printed class's rule block, then run the command again: it must print nothing. (`printButton`, `hint` and `versionsHeading` are still used by the CSV section; keep them.)

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
for c in $(grep -o '^\.[a-zA-Z]*' apps/rux/frontend/src/routes/ExportPage.module.css | tr -d . | sort -u); do grep -q "styles\.$c\b" apps/rux/frontend/src/routes/ExportPage.tsx || echo "unused: $c"; done
```

- [ ] **Step 8: Check and commit**

```bash
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/routes/RapportPage.module.css apps/rux/frontend/src/components/rapport apps/rux/frontend/src/routes/RapportPage.tsx apps/rux/frontend/src/routes/ExportPage.module.css --tsx
git add apps/rux/frontend/src/rapport apps/rux/frontend/src/test/rapport.model.test.ts apps/rux/frontend/src/components/rapport apps/rux/frontend/src/routes/RapportPage.tsx apps/rux/frontend/src/routes/RapportPage.module.css apps/rux/frontend/src/app/App.tsx apps/rux/frontend/src/app/navigation.ts apps/rux/frontend/src/test/navigation.test.ts apps/rux/frontend/src/routes/ExportPage.tsx apps/rux/frontend/src/routes/ExportPage.module.css
git commit -m "feat(gui): Rapport lists report versions, complete or draft; Export keeps CSV" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 15: Indberetning (R3, R9)

**Files:**
- Create `src/indberetning/model.ts`, `src/test/indberetning.model.test.ts`, `src/components/indberetning/FractionTable.tsx` + `.module.css` and `src/routes/IndberetningPage.tsx` + `.module.css`.
- Modify `src/app/App.tsx`, `src/app/navigation.ts` and `src/test/navigation.test.ts`.

**Interfaces:**

```ts
export interface ReadyRow { kind: 'ready'; key: string; eak: string; fraction: string; contaminated: boolean; treatment: string; amount: string }
export interface BlockingRow { kind: 'blocking'; key: string; typeId: number; eak: string; name: string; treatment: string; amount: string; status: { tone: Tone; text: string } }
export type FractionRow = ReadyRow | BlockingRow;
export function tonnesText(t: number): string;               // "6,8 t"
export function fractionRows(f: SurveyFractions): FractionRow[];
export function footStatus(f: SurveyFractions): { tone: Tone; text: string };
export function fractionsCsv(f: SurveyFractions): string;
export const FRACTION_NOTE: string;
export const SEND_NOTICE: string;
```

- [ ] **Step 1: Failing tests** — `src/test/indberetning.model.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  footStatus,
  fractionRows,
  fractionsCsv,
  SEND_NOTICE,
  tonnesText,
  type BlockingRow,
  type ReadyRow,
} from '../indberetning/model';
import { blockingType, fraction, surveyFractions } from './surveyFixtures';

describe('fraction rows', () => {
  it('lists the ready fractions, then each blocking type with its reason', () => {
    const rows = fractionRows(surveyFractions());
    const ready = rows.filter((r): r is ReadyRow => r.kind === 'ready');
    expect(ready.map((r) => [r.eak, r.fraction, r.treatment, r.amount])).toEqual([
      ['17.01.01', 'Beton', 'Genanvendelse', '190 t'],
      ['17.04.05', 'Jern og stål', 'Genanvendelse', '6,8 t'],
      ['17.06.04', 'Isoleringsmateriale', 'Bortskaffelse', '2,4 t'],
    ]);
    const blocking = rows.filter((r): r is BlockingRow => r.kind === 'blocking');
    expect(rows.indexOf(blocking[0])).toBe(3); // blocking rows come last
    expect(blocking).toHaveLength(7);
    expect(blocking[0]).toMatchObject({
      typeId: 2,
      name: 'Betonsøjler, bærende',
      treatment: 'Genbrug',
      amount: '(58 t)',
      status: { tone: 'warn', text: 'Afventer godkendelse' },
    });
    expect(blocking.find((b) => b.typeId === 6)?.status).toEqual({ tone: 'wait', text: 'Afventer prøvesvar' });
  });

  it('flags contaminated tonnes and words the gaps', () => {
    const rows = fractionRows(
      surveyFractions({
        fractions: [
          fraction({ eak_code: '17.01.02', name: 'Mursten', treatment: 'bortskaffelse', mass_t: 38, contaminated: true }),
          fraction({ eak_code: '99.99.99', name: '' }),
        ],
        blocking: [blockingType({ mass_t: null, eak_code: '' })],
        blocking_types: 1,
      }),
    );
    expect(rows[0]).toMatchObject({ kind: 'ready', contaminated: true, amount: '38 t' });
    expect(rows[1]).toMatchObject({ kind: 'ready', fraction: 'Ukendt EAK-kode' });
    expect(rows[2]).toMatchObject({ kind: 'blocking', eak: '—', amount: '(—)' });
  });
});

describe('footer and totals', () => {
  it('says ready, or how many types block', () => {
    expect(footStatus(surveyFractions())).toEqual({ tone: 'warn', text: '7 typer blokerer' });
    expect(footStatus(surveyFractions({ blocking_types: 1 }))).toEqual({ tone: 'warn', text: '1 type blokerer' });
    expect(footStatus(surveyFractions({ ready: true, blocking: [], blocking_types: 0 }))).toEqual({
      tone: 'good',
      text: 'Klar til afsendelse',
    });
    expect(tonnesText(199.2)).toBe('199,2 t');
    expect(tonnesText(1334.8)).toBe('1.334,8 t');
  });
});

describe('the send gate and the CSV', () => {
  it('holds only the ready fractions, Danish Excel style', () => {
    expect(fractionsCsv(surveyFractions())).toBe(
      'EAK-kode;Fraktion;Behandling;Forurenet;Mængde (t)\r\n' +
        '17.01.01;Beton;Genanvendelse;Nej;190\r\n' +
        '17.04.05;Jern og stål;Genanvendelse;Nej;6,8\r\n' +
        '17.06.04;Isoleringsmateriale;Bortskaffelse;Nej;2,4\r\n',
    );
  });

  it('quotes a field with a separator or a quote', () => {
    const csv = fractionsCsv(surveyFractions({ fractions: [fraction({ name: 'Beton; "knust"', contaminated: true })] }));
    expect(csv.split('\r\n')[1]).toBe('17.01.01;"Beton; ""knust""";Genanvendelse;Ja;190');
  });

  it('says plainly that nothing was sent', () => {
    expect(SEND_NOTICE).toBe(
      'Ikke sendt. Direkte indberetning til bygningsaffald.dk er ikke koblet på endnu — hent tallene som CSV og indtast dem i portalen.',
    );
  });
});
```

- [ ] **Step 2: Run → FAIL** with `npm --prefix apps/rux/frontend test -- indberetning`.

- [ ] **Step 3: Model** — `src/indberetning/model.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Indberetning as data: the fraction table's rows (ready fractions, then the
 * types that block sending, both from `GET /survey/fractions`), the footer
 * status, the CSV for manual entry in bygningsaffald.dk, and the copy. The
 * rules (bevaring left out, pending types withheld) are the server's (R3).
 */

import type { SurveyFractions } from '../api/types';
import { formatNumber, TREATMENT_LABEL, type Tone } from '../kortlaegning/vocab';

export interface ReadyRow {
  kind: 'ready';
  key: string;
  eak: string;
  fraction: string;
  contaminated: boolean;
  treatment: string;
  amount: string;
}

export interface BlockingRow {
  kind: 'blocking';
  key: string;
  typeId: number;
  eak: string;
  name: string;
  treatment: string;
  amount: string;
  status: { tone: Tone; text: string };
}

export type FractionRow = ReadyRow | BlockingRow;

export function tonnesText(t: number): string {
  return `${formatNumber(t)} t`;
}

export function fractionRows(f: SurveyFractions): FractionRow[] {
  const ready: FractionRow[] = f.fractions.map((x) => ({
    kind: 'ready',
    key: `f:${x.eak_code}:${x.treatment}:${x.contaminated ? 1 : 0}`,
    eak: x.eak_code,
    fraction: x.name || 'Ukendt EAK-kode',
    contaminated: x.contaminated,
    treatment: TREATMENT_LABEL[x.treatment],
    amount: tonnesText(x.mass_t),
  }));
  const blocking: FractionRow[] = f.blocking.map((b) => ({
    kind: 'blocking',
    key: `b:${b.type_id}`,
    typeId: b.type_id,
    eak: b.eak_code || '—',
    name: b.name,
    treatment: TREATMENT_LABEL[b.treatment],
    amount: b.mass_t === null ? '(—)' : `(${tonnesText(b.mass_t)})`,
    status:
      b.reason === 'sample'
        ? { tone: 'wait', text: 'Afventer prøvesvar' }
        : { tone: 'warn', text: 'Afventer godkendelse' },
  }));
  return [...ready, ...blocking];
}

export function footStatus(f: SurveyFractions): { tone: Tone; text: string } {
  if (f.ready) return { tone: 'good', text: 'Klar til afsendelse' };
  return { tone: 'warn', text: f.blocking_types === 1 ? '1 type blokerer' : `${f.blocking_types} typer blokerer` };
}

function csvField(s: string): string {
  return /[";\r\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
}

/**
 * The ready fractions in the portal's columns: `;`-separated with a decimal
 * comma and CRLF, which Danish Excel opens without an import dialog. The page
 * prepends a BOM so the æøå survive.
 */
export function fractionsCsv(f: SurveyFractions): string {
  const lines = ['EAK-kode;Fraktion;Behandling;Forurenet;Mængde (t)'];
  for (const x of f.fractions) {
    lines.push(
      [
        x.eak_code,
        x.name,
        TREATMENT_LABEL[x.treatment],
        x.contaminated ? 'Ja' : 'Nej',
        x.mass_t.toLocaleString('da-DK', { useGrouping: false, maximumFractionDigits: 3 }),
      ]
        .map(csvField)
        .join(';'),
    );
  }
  return `${lines.join('\r\n')}\r\n`;
}

/** The prototype's note, with the bevaring rule said out loud (R3). */
export const FRACTION_NOTE =
  'Fraktionerne herunder er aggregeret pr. EAK-kode og behandling — kun godkendte mængder tælles med, og bevaring indgår ikke, da den bliver i bygningen. Rækker der afventer gennemsyn eller miljøsvar er vist nederst og blokerer afsendelse. Direkte indberetning kommer senere — v1 giver tallene i portalens struktur.';

export const SEND_NOTICE =
  'Ikke sendt. Direkte indberetning til bygningsaffald.dk er ikke koblet på endnu — hent tallene som CSV og indtast dem i portalen.';
```

- [ ] **Step 4: The table** — `src/components/indberetning/FractionTable.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link } from 'react-router-dom';

import type { SurveyFractions } from '../../api/types';
import { surveyTypeHref } from '../../app/links';
import { footStatus, fractionRows, tonnesText } from '../../indberetning/model';
import { Pill } from '../Pill';
import styles from './FractionTable.module.css';

export interface FractionTableProps {
  fractions: SurveyFractions;
}

/**
 * EAK-kode · Fraktion · Behandling · Mængde · Status. Ready fractions first,
 * then the types that block sending, muted, with a link to each in
 * Kortlægning. The footer totals the approved tonnes.
 */
export function FractionTable({ fractions }: FractionTableProps) {
  const foot = footStatus(fractions);
  return (
    <div className={styles.scroll}>
      <table className={styles.table}>
        <thead>
          <tr>
            <th scope="col">EAK-kode</th>
            <th scope="col">Fraktion</th>
            <th scope="col">Behandling</th>
            <th scope="col" className={styles.num}>
              Mængde
            </th>
            <th scope="col">Status</th>
          </tr>
        </thead>
        <tbody>
          {fractionRows(fractions).map((r) =>
            r.kind === 'ready' ? (
              <tr key={r.key}>
                <td className="mono">{r.eak}</td>
                <td>
                  {r.fraction}{' '}
                  {r.contaminated && <Pill tone="crit">Forurenet</Pill>}
                </td>
                <td>{r.treatment}</td>
                <td className={`${styles.num} ${styles.amount} mono`}>{r.amount}</td>
                <td>
                  <Pill tone="good">Klar ✓</Pill>
                </td>
              </tr>
            ) : (
              <tr key={r.key} className={styles.blocking}>
                <td className="mono">{r.eak}</td>
                <td>
                  <Link to={surveyTypeHref(r.typeId)} className={styles.typeLink}>
                    {r.name}
                  </Link>
                </td>
                <td>{r.treatment}</td>
                <td className={`${styles.num} mono`}>{r.amount}</td>
                <td>
                  <Pill tone={r.status.tone}>{r.status.text}</Pill>
                </td>
              </tr>
            ),
          )}
        </tbody>
        <tfoot>
          <tr>
            <td colSpan={3}>I alt (godkendt)</td>
            <td className={`${styles.num} mono`}>{tonnesText(fractions.total_t)}</td>
            <td>
              <Pill tone={foot.tone}>{foot.text}</Pill>
            </td>
          </tr>
        </tfoot>
      </table>
    </div>
  );
}
```

`FractionTable.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* The panel scrolls sideways on a narrow screen; the page never does. */
.scroll {
  composes: panel from '../surfaces.module.css';
  overflow-x: auto;
}

.table {
  width: 100%;
  min-width: calc(var(--space-7) * 11);
  border-collapse: collapse;
  font-size: var(--font-size-sm);
}

.table th {
  padding: var(--space-2) var(--space-3);
  border-bottom: 1px solid var(--color-border);
  background: var(--color-surface-sunken);
  font-size: var(--font-size-2xs);
  font-weight: var(--font-weight-bold);
  letter-spacing: var(--tracking-caps);
  text-align: left;
  text-transform: uppercase;
  color: var(--color-text-faint);
}

.table td {
  padding: var(--space-2) var(--space-3);
  border-bottom: 1px solid var(--color-border);
}

.num {
  text-align: right;
  white-space: nowrap;
}

.table th.num {
  text-align: right;
}

.amount {
  font-weight: var(--font-weight-bold);
}

.blocking td {
  color: var(--color-text-faint);
}

.typeLink {
  color: inherit;
  text-decoration: underline;
  text-decoration-color: var(--color-border-strong);
}

.typeLink:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: 2px;
}

.table tfoot td {
  border-bottom: none;
  background: var(--color-surface-sunken);
  font-weight: var(--font-weight-bold);
}
```

- [ ] **Step 5: The page** — `src/routes/IndberetningPage.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useMemo, useState } from 'react';

import { api } from '../api/client';
import { useAsync } from '../app/useAsync';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { FractionTable } from '../components/indberetning/FractionTable';
import { footStatus, FRACTION_NOTE, fractionsCsv, SEND_NOTICE } from '../indberetning/model';
import styles from './IndberetningPage.module.css';

/**
 * Indberetning — approved tonnes per EAK fraction for bygningsaffald.dk
 * (prototype 02e). Read-only: the fractions, the blocking types and readiness
 * are all `GET /survey/fractions`.
 *
 * The send gate is the prototype's: `Send til bygningsaffald.dk` is enabled
 * exactly when nothing blocks. v1 posts nothing, so a click only says so and
 * points to the CSV (R9). Submission is a follow-up.
 */
export function IndberetningPage() {
  const { data, error, loading, reload } = useAsync((s) => api.surveyFractions(s), []);
  const [sendNotice, setSendNotice] = useState(false);
  const csvHref = useMemo(
    () => (data ? `data:text/csv;charset=utf-8,${encodeURIComponent(`﻿${fractionsCsv(data)}`)}` : null),
    [data],
  );

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="affaldsfraktionerne" />
      </div>
    );
  }
  if (loading && !data) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser fraktioner…" />
      </div>
    );
  }
  if (!data || !csvHref) return null;

  const empty = data.fractions.length === 0 && data.blocking.length === 0;

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h2 className={styles.title}>Indberetning</h2>
        <span className={styles.sub}>Affaldsfraktioner til bygningsaffald.dk</span>
        <div className={styles.actions}>
          <a className={styles.btnGhost} href={csvHref} download="fraktioner.csv">
            Hent fraktioner (CSV)
          </a>
          <button
            type="button"
            className={styles.btnPrimary}
            disabled={!data.ready}
            title={data.ready ? undefined : footStatus(data).text}
            onClick={() => setSendNotice(true)}
          >
            Send til bygningsaffald.dk
          </button>
        </div>
      </header>

      {sendNotice && (
        <p className={styles.notice} role="status">
          {SEND_NOTICE}
        </p>
      )}

      <p className={styles.note}>{FRACTION_NOTE}</p>

      {empty ? (
        <EmptyState
          title="Ingen typer i kortlægningen endnu"
          detail="Fraktionerne opstår, når typerne i Kortlægning er godkendt med tonnage."
        />
      ) : (
        <FractionTable fractions={data} />
      )}
    </div>
  );
}
```

`src/routes/IndberetningPage.module.css`:

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
.actions {
  composes: actions from './viewHead.module.css';
}
.btnPrimary {
  composes: btnPrimary from '../components/controls.module.css';
}
.btnGhost {
  composes: btnGhost from '../components/controls.module.css';
  text-decoration: none;
}
.notice {
  composes: notice from '../components/surfaces.module.css';
}

.note {
  margin: 0;
  max-width: calc(var(--space-7) * 15);
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
}
```

- [ ] **Step 6: Route and live entry.** In `src/test/navigation.test.ts`, add the test below and run it → FAIL. In `navigation.ts`, replace the `/indberetning` entry with `{ to: '/indberetning', label: 'Indberetning', group: 'sag' },`. In `App.tsx`, import `IndberetningPage` and add `<Route path="/indberetning" element={<IndberetningPage />} />` after `/rapport`. Run again → PASS.

```ts
  it('makes Indberetning a live destination', () => {
    expect(NAV_ENTRIES.find((e) => e.to === '/indberetning')?.pending).toBeUndefined();
  });
```

- [ ] **Step 7: Check and commit**

```bash
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/routes/IndberetningPage.module.css apps/rux/frontend/src/components/indberetning apps/rux/frontend/src/routes/IndberetningPage.tsx --tsx
git add apps/rux/frontend/src/indberetning apps/rux/frontend/src/test/indberetning.model.test.ts apps/rux/frontend/src/components/indberetning apps/rux/frontend/src/routes/IndberetningPage.tsx apps/rux/frontend/src/routes/IndberetningPage.module.css apps/rux/frontend/src/app/App.tsx apps/rux/frontend/src/app/navigation.ts apps/rux/frontend/src/test/navigation.test.ts
git commit -m "feat(gui): Indberetning shows the EAK fractions, the blockers and the send gate" -m "The send button is enabled exactly when nothing blocks, as in the prototype, and posts nothing: a click says so and points to the CSV of the ready fractions. bygningsaffald.dk submission stays out of scope." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 16: Esc reverts in Kortlægning's detail panel (R10)

**Files:**
- Modify `src/components/kortlaegning/useQuantityNoteDrafts.ts`, `src/components/kortlaegning/DetailPanel.tsx` and `src/test/kortlaegning.detailPanel.test.ts`.

**Interfaces:**

```ts
// DetailPanel.tsx
export function fieldKeyAction(key: string, singleLine: boolean): 'revert' | 'commit' | null;
// useQuantityNoteDrafts return value gains
revertQuantity: (el: HTMLElement) => void;
revertNote: (el: HTMLElement) => void;
```

- [ ] **Step 1: Failing test.** In `src/test/kortlaegning.detailPanel.test.ts`, add `fieldKeyAction` to the import from `'../components/kortlaegning/DetailPanel'`, then append:

```ts
describe('fieldKeyAction (Esc convention, Phase 5 R10)', () => {
  it('reverts on Esc in any field and commits on Enter only in a single-line one', () => {
    expect(fieldKeyAction('Escape', true)).toBe('revert');
    expect(fieldKeyAction('Escape', false)).toBe('revert');
    expect(fieldKeyAction('Enter', true)).toBe('commit');
    expect(fieldKeyAction('Enter', false)).toBeNull();
    expect(fieldKeyAction('a', true)).toBeNull();
  });
});
```

Run `npm --prefix apps/rux/frontend test -- kortlaegning.detailPanel` → FAIL.

- [ ] **Step 2: Reverting drafts.** In `useQuantityNoteDrafts.ts`:
  1. After `const quantityFocusedRef = useRef(false);`, add:

```ts
  // Set by a revert so the blur it triggers commits nothing (Esc, R10).
  const skipQuantityCommit = useRef(false);
  const skipNoteCommit = useRef(false);
```

  2. In `quantityProps.onBlur`, after `quantityFocusedRef.current = false;`, insert:

```ts
        if (skipQuantityCommit.current) {
          skipQuantityCommit.current = false;
          return;
        }
```

  3. Replace `noteProps.onBlur: commitNote,` with:

```ts
      onBlur: () => {
        if (skipNoteCommit.current) {
          skipNoteCommit.current = false;
          return;
        }
        commitNote();
      },
```

  4. Add these two members to the returned object, after `noteProps`:

```ts
    /** Esc: drop the quantity draft and leave the field without committing. */
    revertQuantity: (el: HTMLElement) => {
      skipQuantityCommit.current = true;
      if (current) setQuantityDraft(formatQuantityInput(current.quantity));
      el.blur();
    },
    /** Esc: drop the note draft and leave the field without committing. */
    revertNote: (el: HTMLElement) => {
      skipNoteCommit.current = true;
      setNoteDraft(current?.note ?? '');
      el.blur();
    },
```

  5. Change the hook's doc comment sentence "both commit on blur, so a caller that wants Enter or Esc to commit just blurs the field." to "both commit on blur, so Enter commits by blurring; `revertQuantity` / `revertNote` are Esc, which drops the draft and blurs without committing."

  The EditDialog ignores the two new members, so its behaviour is unchanged (R10).

- [ ] **Step 3: DetailPanel uses them.** In `DetailPanel.tsx`:
  1. Replace `fieldDoneKey` with:

```ts
/**
 * What a key does in a detail-panel field (Phase 5 R10): Esc drops the
 * field's draft without committing, Enter in a single-line field commits it.
 * Either way focus goes back to the table ("Esc tilbage").
 */
export function fieldKeyAction(key: string, singleLine: boolean): 'revert' | 'commit' | null {
  if (key === 'Escape') return 'revert';
  if (singleLine && key === 'Enter') return 'commit';
  return null;
}
```

  2. Change the destructuring to `const { quantityProps, noteProps, revertQuantity, revertNote } = useQuantityNoteDrafts(current, { onQuantity, onNote });`.
  3. Directly after it, add:

```ts
  function onFieldKey(e: KeyboardEvent<HTMLElement>, singleLine: boolean, revert: ((el: HTMLElement) => void) | null) {
    const action = fieldKeyAction(e.key, singleLine);
    if (!action) return;
    e.preventDefault();
    if (action === 'revert' && revert) revert(e.currentTarget);
    else e.currentTarget.blur();
    onDone();
  }
```

  4. Use it in the three handlers:
     - the quantity input: `onKeyDown={(e) => onFieldKey(e, true, revertQuantity)}`;
     - the Behandling `select`: `onKeyDown={(e) => onFieldKey(e, false, null)}`. A select has no draft, because a change commits at once, so Esc just leaves it;
     - the note `textarea`: `onKeyDown={(e) => onFieldKey(e, false, revertNote)}`.
  5. In the `onDone` prop's doc comment, replace "The field has already been blurred (so its draft committed)" with "The field has already been left: on Enter its draft committed, on Esc it was dropped".

- [ ] **Step 4: Check and commit**

```bash
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
git add apps/rux/frontend/src/components/kortlaegning/useQuantityNoteDrafts.ts apps/rux/frontend/src/components/kortlaegning/DetailPanel.tsx apps/rux/frontend/src/test/kortlaegning.detailPanel.test.ts
git commit -m "fix(gui): Esc in a Kortlægning field drops the draft instead of saving it" -m "One Esc convention for every editor (Phase 5 R10): Esc in a text field reverts without committing, as in Miljø & prøver. The detail panel still hands focus back to the table; Enter in the quantity field still commits. The edit dialog is unchanged (follow-up)." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 17: Verify against the prototype and the flows

**Files:** None in the repo. The scripts and shots go to the scratchpad.

- [ ] **Step 1: Shot script.** Write `$SP/phase5_shots.py`. It waits on content, never on time, and fails if `<main>` overflows horizontally at 768px (R12):

```python
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Static shots of the Phase 5 screens in both themes; waits on content, not time."""
import sys

from playwright.sync_api import expect, sync_playwright

base, out = sys.argv[1], sys.argv[2]
only = set(sys.argv[3].split(",")) if len(sys.argv) > 3 else None
ROUTES = {
    "overblik": ("/", "Cirkularitetsoversigt"),
    "rapport": ("/rapport", "Inventarliste"),
    "indberetning": ("/indberetning", "I alt (godkendt)"),
    "miljoe": ("/miljoe", "P-01 · PCB i fugemasse"),
    "projektdata": ("/projektdata", "Clouds"),
}
VIEWPORTS = {"desktop": (1440, 1000), "tablet": (768, 1024), "mobile": (390, 844)}

with sync_playwright() as p:
    browser = p.chromium.launch()
    for theme in ("light", "dark"):
        for vp, (w, h) in VIEWPORTS.items():
            page = browser.new_page(viewport={"width": w, "height": h}, color_scheme=theme)
            page.add_init_script(f"localStorage.setItem('reusex-theme', '{theme}')")
            for name, (path, marker) in ROUTES.items():
                if only and name not in only:
                    continue
                page.goto(base + path)
                expect(page.get_by_text(marker).first).to_be_visible()
                page.evaluate("document.fonts.ready")
                if vp == "tablet":
                    overflow = page.evaluate(
                        "(() => { const m = document.querySelector('main'); return m.scrollWidth - m.clientWidth; })()"
                    )
                    assert overflow <= 0, f"{name}/{theme}: <main> overflows by {overflow}px at 768px"
                page.screenshot(path=f"{out}/{name}-{theme}-{vp}.png", full_page=True)
            page.close()
    browser.close()
print("shots OK")
```

Run it on the **plain** seed, which holds the prototype's data:

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/p5-demo.rux"
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/p5-demo.rux"
mkdir -p "$SP/shots/p5"
BROWSERS="$(nix build --no-link --print-out-paths nixpkgs#playwright-driver.browsers)"
PLAYWRIGHT_BROWSERS_PATH="$BROWSERS" nix shell --impure --expr 'let p = import (builtins.getFlake "nixpkgs") {}; in p.python3.withPackages (ps: [ ps.playwright ])' --command python3 "$SP/phase5_shots.py" http://localhost:5173 "$SP/shots/p5"
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

Expected: `shots OK`. Open the PNGs with Read and compare them with the three prototype shots.
- **`overblik-light-desktop`:**
  - the navy hero names the case;
  - the five KPIs read `11` / `—` (Klassificeret, hint `Kræver instansskyen`, unless the source project has an instance cloud) / `54 %` / `7` in warn ink / `2` in crit ink;
  - the bar's legend reads `Bevaring 48 % · Genbrug 6 % · Genanvendelse 43 % · Nyttiggørelse 0 % · Bortskaffelse 3 %`;
  - the four quick links read `7 til gennemsyn · 11 typer`, `2 prøver afventer svar`, `Ingen versioner endnu` and `4 af 11 typer godkendt`;
  - there is no BBR line (R6).
- **`rapport-light-desktop`:**
  - the head reads `RAPPORT` with `Generér ny version`;
  - the warn notice says 7 types;
  - the hero line reads `11 komponenter · 54 % bevaring/genbrug · 1 forurenet · 2 prøver afventer`, with the bar and no legend;
  - the empty state is shown, and the last row is `CSV Inventarliste · Eksport`;
  - the footnote has no MRK.
- **`indberetning-light-desktop`:**
  - three ready rows: `17.01.01 Beton Genanvendelse 190 t`, `17.04.05 Jern og stål 6,8 t` and `17.06.04 Isoleringsmateriale 2,4 t`, each with `Klar ✓`;
  - no Fundamenter row;
  - seven muted blocking rows in type order. Vinduespartier and Gulvbelægning carry `Afventer prøvesvar`, the rest `Afventer godkendelse`;
  - the footer reads `199,2 t` with `7 typer blokerer`;
  - the send button is disabled.
- **`miljoe-light-desktop`:** identical to the Phase 4 look (the Task 10 refactor).
- **The dark shots:** no light-only colour. The hero stays navy.
- **The tablet shots:** passed the overflow assertion. The fraction table scrolls inside its panel.
- **The mobile shots:** for the record only. The shell's own 390px overflow is R12's follow-up.

Fix what differs, then re-shoot.

- [ ] **Step 2: Varied seed — Rapport with versions**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh --varied "$SP/corridor-clouds.rux" "$SP/p5-varied.rux"
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/p5-varied.rux"
mkdir -p "$SP/shots/p5-varied"
PLAYWRIGHT_BROWSERS_PATH="$BROWSERS" nix shell --impure --expr 'let p = import (builtins.getFlake "nixpkgs") {}; in p.python3.withPackages (ps: [ ps.playwright ])' --command python3 "$SP/phase5_shots.py" http://localhost:5173 "$SP/shots/p5-varied" overblik,rapport
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

Expected, matching `rapport.png`'s list:
- three PDF rows, newest first:
  - `Ressourcekortlægning — v3` · `09.08.2026 · 15 B` · `Komplet`;
  - `— v2` · `21.05.2026` · `Udkast`;
  - `— v1` · `21.05.2026` · `Udkast`;
- then the CSV row;
- Overblik's Rapport link reads `3 versioner`.

- [ ] **Step 3: Flow script.** Write `$SP/phase5_flow.py`:

```python
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Drive the Phase 5 flows on the plain demo seed. Asserts on captured
requests and awaited responses; never sleeps."""
import json
import re
import sys

from playwright.sync_api import expect, sync_playwright

base, out = sys.argv[1], sys.argv[2]
API = re.compile(r"/api/v1/")
J = {"Content-Type": "application/json"}

with sync_playwright() as p:
    browser = p.chromium.launch()
    page = browser.new_page(viewport={"width": 1440, "height": 1000}, accept_downloads=True)
    page.add_init_script("localStorage.setItem('reusex-theme', 'light')")
    requests, statuses = [], []
    page.on("request", lambda r: requests.append((r.method, r.url, r.post_data)) if API.search(r.url) else None)
    page.on("response", lambda r: statuses.append((r.status, r.url)) if API.search(r.url) else None)

    def bodies(method, fragment):
        return [json.loads(d or "{}") for (m, u, d) in requests if m == method and fragment in u]

    def toast(text):
        # Several role=status regions exist (page Toast, JobToaster, notices).
        expect(page.get_by_role("status").filter(has_text=text)).to_be_visible()

    def is_patch(fragment):
        return lambda r: fragment in r.url and r.request.method == "PATCH"

    # --- Overblik: prototype figures on a full page load (no 503, R1) --------
    page.goto(f"{base}/")
    kpis = page.get_by_role("list", name="Nøgletal")
    for text in ("11", "Komponenter", "54 %", "Til gennemsyn", "Prøver afventer"):
        expect(kpis).to_contain_text(text)
    expect(page.get_by_text("Bevaring 48 %")).to_be_visible()
    quick = page.get_by_role("navigation", name="Sagens skærme")
    for text in ("7 til gennemsyn · 11 typer", "2 prøver afventer svar", "Ingen versioner endnu", "4 af 11 typer godkendt"):
        expect(quick).to_contain_text(text)
    page.screenshot(path=f"{out}/01-overblik.png")

    # --- Hero editor: untouched blur, Esc and a bad year send nothing --------
    page.get_by_role("button", name="Rediger sagsoplysninger").click()
    address = page.get_by_label("Adresse")
    name = page.get_by_label("Navn")
    year = page.get_by_label("Opført (år)")
    address.focus()
    address.blur()
    name.fill("Noget andet")
    name.press("Escape")
    expect(name).not_to_have_value("Noget andet")
    year.fill("nittenhalvfjerds")
    year.press("Tab")
    toast("Byggeår skal være et årstal")
    with page.expect_response(is_patch("/api/v1/projects/")) as resp:
        address.fill("Måløv Byvej 229, 2760 Måløv")
        address.press("Enter")
    assert resp.value.ok, resp.value.text()
    # Ordering proves absence: anything the earlier actions sent would have
    # been queued before this commit.
    assert bodies("PATCH", "/api/v1/projects/") == [{"building_address": "Måløv Byvej 229, 2760 Måløv"}], bodies(
        "PATCH", "/api/v1/projects/"
    )
    expect(page.locator("#case-name + p")).to_contain_text("Måløv Byvej 229, 2760 Måløv")

    # A name edit reaches the sidebar through refreshProject (R15).
    with page.expect_response(lambda r: r.url.endswith("/api/v1/project") and r.request.method == "GET"):
        with page.expect_response(is_patch("/api/v1/projects/")):
            name.fill("Måløv Byvej 229")
            name.press("Enter")
    expect(page.locator("aside [title='Måløv Byvej 229']")).to_be_visible()
    page.get_by_role("button", name="Luk redigering").click()
    expect(page.get_by_role("button", name="Rediger sagsoplysninger")).to_be_focused()
    page.screenshot(path=f"{out}/02-hero-edited.png")

    # --- Rapport: a draft version (7 types block) -----------------------------
    page.get_by_role("navigation", name="Sag").get_by_role("link", name="Rapport").click()
    expect(page.get_by_text("11 komponenter · 54 % bevaring/genbrug · 1 forurenet · 2 prøver afventer")).to_be_visible()
    expect(page.get_by_text("7 typer er ikke godkendt")).to_be_visible()
    is_post = lambda r: r.url.endswith("/api/v1/reports/ressourcekortlaegning") and r.request.method == "POST"
    with page.expect_response(is_post, timeout=120000) as gen:
        page.get_by_role("button", name="Generér ny version").click()
    assert gen.value.status == 201, gen.value.text()
    toast("Ressourcekortlægning — v1 genereret (udkast)")
    versions = page.get_by_role("list", name="Rapportversioner")
    expect(versions.locator("li").first).to_contain_text("Ressourcekortlægning — v1")
    expect(versions.locator("li").first).to_contain_text("Udkast")
    expect(versions.get_by_role("link", name="Hent Ressourcekortlægning — v1")).to_have_attribute(
        "href", re.compile(r"/api/v1/reports/ressourcekortlaegning/1$")
    )
    page.screenshot(path=f"{out}/03-rapport-draft.png")

    # --- Indberetning: the prototype's table, gate closed ---------------------
    page.get_by_role("navigation", name="Sag").get_by_role("link", name="Indberetning").click()
    table = page.get_by_role("table")
    expect(table.locator("tfoot")).to_contain_text("199,2 t")
    expect(table.locator("tfoot")).to_contain_text("7 typer blokerer")
    expect(table.locator("tbody tr")).to_have_count(10)
    expect(table).not_to_contain_text("Fundamenter")  # bevaring stays in the building
    expect(page.get_by_role("button", name="Send til bygningsaffald.dk")).to_be_disabled()
    with page.expect_download() as dl:
        page.get_by_role("link", name="Hent fraktioner (CSV)").click()
    with open(dl.value.path(), encoding="utf-8-sig") as f:
        lines = f.read().splitlines()
    assert lines[0] == "EAK-kode;Fraktion;Behandling;Forurenet;Mængde (t)", lines
    assert len(lines) == 4, lines
    page.screenshot(path=f"{out}/04-indberetning-blocked.png")

    # Answer the open samples and approve the queue (test setup, not UI).
    for sid in (1, 3):
        r = page.request.patch(f"{base}/api/v1/samples/{sid}", data=json.dumps({"stage": "svar", "result": "ren"}), headers=J)
        assert r.ok, r.text()
    for tid in (2, 3, 5, 6, 8, 9, 11):
        r = page.request.patch(f"{base}/api/v1/survey/types/{tid}", data=json.dumps({"review_status": "approved"}), headers=J)
        assert r.ok, r.text()

    page.reload()
    expect(table.locator("tfoot")).to_contain_text("Klar til afsendelse")
    expect(table.locator("tfoot")).to_contain_text("694,8 t")
    expect(table.locator("tbody tr")).to_have_count(9)
    expect(table.locator("tbody tr").filter(has_text="17.01.02")).to_contain_text("Forurenet")
    send = page.get_by_role("button", name="Send til bygningsaffald.dk")
    expect(send).to_be_enabled()
    before = len(requests)
    send.click()
    toast("Ikke sendt.")
    assert len(requests) == before, requests[before:]  # the gate posts nothing (R9)
    page.screenshot(path=f"{out}/05-indberetning-ready.png")

    # --- Rapport again: a complete version ------------------------------------
    page.get_by_role("navigation", name="Sag").get_by_role("link", name="Rapport").click()
    expect(page.get_by_text("Inventarliste")).to_be_visible()
    expect(page.get_by_text("typer er ikke godkendt")).to_have_count(0)
    with page.expect_response(is_post, timeout=120000) as gen2:
        page.get_by_role("button", name="Generér ny version").click()
    assert gen2.value.status == 201, gen2.value.text()
    toast("Ressourcekortlægning — v2 genereret")
    expect(versions.locator("li").first).to_contain_text("Ressourcekortlægning — v2")
    expect(versions.locator("li").first).to_contain_text("Komplet")
    page.screenshot(path=f"{out}/06-rapport-complete.png")

    # --- Kortlægning: Esc drops a draft, Enter/Tab still commits (R10) --------
    page.goto(f"{base}/kortlaegning?type=6")
    panel = page.locator("aside").last
    qty = panel.get_by_label(re.compile(r"^Mængde"))
    original = qty.input_value()
    qty.fill("999")
    qty.press("Escape")
    expect(qty).to_have_value(original)
    expect(qty).not_to_be_focused()
    note = panel.locator("textarea")
    with page.expect_response(is_patch("/api/v1/survey/types/6")):
        note.fill("Esc-test")
        note.press("Tab")
    assert bodies("PATCH", "/api/v1/survey/types/6") == [{"note": "Esc-test"}], bodies("PATCH", "/api/v1/survey/types/6")
    page.screenshot(path=f"{out}/07-kortlaegning-esc.png")

    busy = [s for s in statuses if s[0] == 503]
    assert not busy, busy  # R1: no 503 anywhere in the flow
    browser.close()
print("flow OK")
```

Run it on a fresh plain seed. `dev_env.sh` serves a copy, so the flow never dirties `$SP/p5-demo.rux`:

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/p5-demo.rux"
mkdir -p "$SP/shots/p5-flow"
PLAYWRIGHT_BROWSERS_PATH="$BROWSERS" nix shell --impure --expr 'let p = import (builtins.getFlake "nixpkgs") {}; in p.python3.withPackages (ps: [ ps.playwright ])' --command python3 "$SP/phase5_flow.py" http://localhost:5173 "$SP/shots/p5-flow"
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

Expected: `flow OK`. Open the seven PNGs with Read. A failing `expect` or `assert` names the broken step. Fix the code, not the script, unless the script contradicts this plan.

The `rux gui` started by `dev_env.sh` inherits the `nix develop` PATH, so `typst` is found. If the POST answers 500 with "typst", the server was started outside the devshell.

No commit (verification only).

---

### Task 18: Docs

**Files:**
- `docs/design/gui-kortlaegning-redesign.md`;
- `apps/rux/frontend/README.md`;
- `.claude/skills/design-studio/references/reusex-frontend.md`.

- [ ] **Step 1: Spec** (`docs/design/gui-kortlaegning-redesign.md`):
  - **§ Source:** after the screenshots sentence, add: "`rapport.png` and `indberetning.png` were added in Phase 5, rendered the same way from the artifact."
  - **§ Kortlægning domain model:** replace the **Fractions** bullet with: "**Fractions** (Indberetning): approved tonnes per (EAK code, behandling, contaminated). *Bevaring* never counts: it stays in the building, so it is not waste. A type awaiting a sample is withheld even when approved. Contaminated tonnes get their own row. The **blocking list** holds every non-rejected type still in the queue (`review`) or awaiting a sample (`sample`), and the report is ready when that list is empty."
  - **New § Overblik, Rapport and Indberetning screens**, after § Miljø & prøver screen. Summarise this plan's "The prototype, component by component", and state what v1 changes:
    - **Overblik:**
      - KPI `Klassificeret` in place of the unmeasured `Scanningsdækning`;
      - no aerial photo, sync chip, case number, MRK, deadline or BBR line;
      - the hero's in-place editor;
      - the old dashboard moved to `/projektdata`.
    - **Rapport:**
      - versions numbered `v<n>` and marked `Komplet`/`Udkast` from the stored blocking count (schema v23);
      - the PDF opens with the approved survey;
      - Inventarliste is the live CSV;
      - no MRK signature.
    - **Indberetning:**
      - the send button is gated like the prototype's and posts nothing, saying so;
      - CSV download of the ready fractions.
  - **§ Phases:** after item 5, add "(done in Phase 5)".
  - **§ Out of scope:**
    - **Remove:**
      - the `ErrorBanner` copy item;
      - the 503 item;
      - the Esc item.
    - **Replace the AppShell item with:** "The `AppShell` overflows horizontally at 390px (topbar + open sidebar, every route). Phase 6 (On-site) designs the phone layout and owns the collapsible sidebar."
    - **Add:**
      - "Case identity in project metadata: BFE number, case number (`RX-2026-0047`), MRK and the demolition deadline — schema, `PATCH /projects`, `rux set` — then Overblik's hero line and the BBR line with a verified BBR link. Until then BBR is not drawn."
      - "Esc in the Kortlægning edit dialog still closes *and saves* (the closing blur commits); every other editor drops the draft (Phase 5 R10)."
      - "Report approval (`Godkendt` / MRK signature on a version) and a stored XLS inventory version."
      - "A scan-coverage measure for the building, so Overblik can show the prototype's `Scanningsdækning` instead of `Klassificeret`."
- [ ] **Step 2: Frontend README** (`apps/rux/frontend/README.md` § Layout). In the same style as the Phase 4 lines, add:
  - `overblik/`, `rapport/` and `indberetning/` (pure models);
  - `components/overblik/` (CaseHero, KpiRow, QuickLinks, ProjectMetaForm), `components/rapport/` (VersionList) and `components/indberetning/` (FractionTable);
  - `components/CircularityBar.tsx`;
  - the shared `components/controls.module.css`, `components/surfaces.module.css` and `routes/viewHead.module.css`;
  - `app/errorCopy.ts`.

  Note that `/` is Overblik and the inventory lives at `/projektdata`.
- [ ] **Step 3: design-studio reference** (`.claude/skills/design-studio/references/reusex-frontend.md`):
  - add the same directories to the map;
  - add `OverblikPage`, `RapportPage`, `IndberetningPage` to the routes line, and `Dashboard (/projektdata)`;
  - add the rules:
    - "case-screen CSS composes from `controls` / `surfaces` / `viewHead` — never copy a button or panel rule";
    - "Esc in a text field reverts without committing (R10); Esc elsewhere closes";
    - "`ErrorBanner` `context` is a Danish definite noun phrase".
  - in the seed paragraph, add: "`--varied` also seeds three report versions".
- [ ] **Step 4: Check and commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase5
nix develop --command reuse lint
git add docs/design/gui-kortlaegning-redesign.md apps/rux/frontend/README.md .claude/skills/design-studio/references/reusex-frontend.md
git commit -m "docs(gui): Overblik, Rapport and Indberetning screens; fraction rules; follow-ups" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

## Phase exit criteria

- **Backend:**
  - `ctest --parallel` is green for every test this plan adds or touches: concurrency, fractions, summary, reports v23, report survey, Typst smoke (run, not skipped), version JSON, the schema-version assertion;
  - so are the existing `ProjectDb*`, `Survey*`, `RunningServer_*` and `gui_api_contract_parses`;
  - `scripts/check-openapi.py` passes.
- **Frontend:**
  - `test`, `typecheck` and `build` pass;
  - token lint is clean on every new or changed `.module.css`/`.tsx`;
  - `reuse lint` is compliant.
- **Busy fix:** the Task 2 concurrent-curl loop prints only `200`, and the Task 17 flow captures no 503.
- **`dev_env.sh`:**
  - after `start`/`stop`, `git status --porcelain tests/fixtures` is empty;
  - `status`/`stop` from a second `nix develop` find the servers;
  - no vite process survives `stop`.
- **Prototype match:** the shots of `/`, `/rapport` and `/indberetning` (light + dark, desktop + tablet) match the three prototype PNGs' structure on the plain seed. Deviations are only those ruled in R4–R9, R12 and R13.
- **Flow:** `phase5_flow.py` prints `flow OK`.
- **Follow-up issues filed:**
  1. Case identity metadata (BFE, case no., MRK, deadline) + BBR line (R5, R6).
  2. bygningsaffald.dk submission behind the existing gate (R9; spec out of scope).
  3. Report approval / MRK signature and a stored XLS inventory version (R7).
  4. A scan-coverage measure for Overblik (R4).
  5. AppShell 390px collapsible sidebar, assigned to Phase 6 (R12).
  6. EditDialog Esc saves on close (R10).

## Self-review

- **Spec coverage:**
  - KPIs: Tasks 5, 11, 12;
  - circularity: Tasks 11–13, plus the Rapport hero in Task 14;
  - report versions on the existing endpoints: Tasks 6, 14;
  - fraction table and send gate: Tasks 4, 15;
  - the fractions rule "approved tonnes per EAK code; blocking = queue or awaiting a sample": Task 4, which makes the prototype's bevaring rule explicit (R3);
  - BBR shown only with a BFE number: R6, not drawn because no BFE exists;
  - bygningsaffald.dk posts nowhere: R9;
  - IA `/` Overblik replacing Dashboard, `/rapport` replacing Export's report part: Tasks 13, 14 (R13).
- **The two known problems:**
  - the 503 has its own task (Task 2) with a root cause (R1), a deterministic Catch2 regression test, a concurrent socket test and a curl reproduction before and after, and no client retry;
  - `dev_env.sh` has its own task (Task 3) fixing both defects plus the vite orphan, with a verification that runs from a second `nix develop`.
- **The follow-ups the brief asked to rule on:** Esc (R10, Task 16), ErrorBanner (R11, Task 9), AppShell 390px (R12).
- **Backend gaps found, each with a task before its screen:**
  - fraction rules + `blocking` + `contaminated` (Task 4);
  - `classified_share` + `contaminated_types` (Task 5);
  - per-version `blocking_types` + `version`, and a survey section in the PDF (Task 6);
  - seed versions (Task 7).
- **Placeholders:** none. Every path, string and command is literal.
  - Two steps say what to do if the code differs from what this plan read: Task 8 Step 4 (another literal that builds the types) and Task 6 Step 9 (`ruxd` tests in the heavy binary). Both name the exact action.
  - Line numbers are avoided in favour of exact old strings.
- **Type consistency:**
  - `ReportPdfRecord.version: int` and `blocking_types: std::optional<int>` are serialised by `report_version_json` (rux gui) and `record_json` (ruxd), and typed `number` / `number | null` in `ReportPdfVersion`;
  - `BlockingType.mass_t: std::optional<double>` becomes `opt(...)` and then `number | null`;
  - `TypeTotals` keeps its five leading members in order, so the existing aggregate initialisers in `test_survey.cpp` still compile;
  - `ProjectPatch` is derived from `RuxApiClient['patchProject']`, so `metaPatch` / `yearCommit` bodies type-check against the client;
  - `useTextDraft`'s widened `onChange` stays assignable to an input's handler;
  - `fieldKeyAction` returns the union the handler switches on.
- **Global-constraint lessons:**
  - one serial queue per page: Overblik and Rapport use `useMutationQueue`;
  - field commits are never gated on busy: the hero fields call `onCommit` unconditionally, and only `Generér ny version` checks `busy`;
  - an untouched blur never commits: `textCommit` and `yearCommit`, proven by the flow's ordering assertion;
  - isField/isControl via `keyTargets.kindOf` in the editor;
  - tokens only, with lint in every UI task and `tokens.css` untouched;
  - Danish copy, given verbatim;
  - SPDX on every new file, including both `.png.license` sidecars;
  - pure logic in vitest under Node;
  - shared CSS through `composes` (Task 10);
  - no fixed sleeps: the browser scripts wait on `expect`/`expect_response`/`expect_download` and assert on captured requests.
