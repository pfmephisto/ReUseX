<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# GUI Phase 6 — Sager & On-site Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Load the project skill `design-studio` (`.claude/skills/design-studio/SKILL.md`) before touching any `.tsx`/`.css`.

**Goal:** Build the last two screens of prototype v2 and close the shell work that was parked for this phase.
- **Sager** at `/sager` is the case list. `rux gui` serves one project, so it shows that project as its one card and says how to open another. The sidebar's `← Alle sager` becomes a live link.
- **On-site** at `/on-site` is the phone capture sheet. A surveyor walks the stored bygningsdele room by room, sees each part's best photo, and writes three things against it: ★ (vigtig), a quick note, and a sample registered on the spot.
- **The shell becomes responsive.** Below 900px the sidebar turns into a drawer behind a `Menu` button and the title bar sheds its meta. This fixes the 390px overflow on every route (draft 25).
- **Two follow-ups land because Phase 6 makes them worse:** one app-wide write chain that the screens' first loads wait for (draft 30), and one cross-link style (draft 32).
- **Kortlægning and Miljø & prøver show what On-site wrote:** a ★ filter, ★ and note markers on part rows, and "Udtaget ved RX-008 · Office Zone" on a sample card.

**Architecture:** As in Phases 3–5, logic that can be pure is pure and unit-tested in Node:
- `src/sager/model.ts`: case status, card stats, sub line, date and the two command lines;
- `src/onsite/model.ts`: walk order, the current and next stop, the picker groups, the detection chip, the photo state, the reticle, the star button, the sample body and its toast;
- `src/miljoe/model.ts` gains `takenAt`, `src/kortlaegning/model.ts` gains the `starred` filter;
- `src/app/serialQueue.ts` gains `idle()`, and `src/app/writeChain.ts` is the one app-wide chain.

Components are presentational. Each route owns its state and writes through `src/app/useMutationQueue.ts`, which now enqueues on the app-wide chain. Every number on screen comes from a response body; the client only formats it.

The backend change is small and library-first: `samples` gains `part_code` (schema v24), `ProjectDB::add_sample` takes it, and `POST /samples` accepts `part_code` and an initial `stage` of `planlagt` or `udtaget`.

**Tech Stack:** C++20 (`reusex_core`, `rux_gui_lib`), sqlite3, Catch2 v3. React 19, react-router-dom 7, TypeScript, CSS Modules and vitest (Node, no DOM). Playwright via the design-studio scripts.

**Spec:** `docs/design/gui-kortlaegning-redesign.md`. The relevant sections are § Information architecture (the Sager paragraph: one project per server), § Kortlægning domain model (samples link to types; the approval gate), § Phases item 6 and § Out of scope (multi-project via `ruxd`, the AppShell 390px item, cross-screen staleness, cross-link styling, the miljørapport upload).

**Prototype:**
- `docs/gui/images/prototype-v2/sager.png` (1440×1000, light);
- `docs/gui/images/prototype-v2/onsite.png` (390×844, light). It is new: it was rendered on 2026-10-02 from the colleague's artifact `At7n5is3zp7cYX54faGJkn` (headless Chromium at the prototype's phone size, the `03 On-site` screen activated) and is committed with this plan, with a `.license` sidecar.
- The artifact's source was read directly for both screens; the copy quoted below is verbatim from it.

## The prototype, component by component

### `sager.png` (1440×1000, light)

- **Prototype chrome:** a navy top bar with `REUSEX` (the X in accent), the muted tag `prototype v2`, three pill tabs `01 Sager` (active, accent fill) · `02 Sag` · `03 On-site`, and on the right the muted hint `Klik rundt — eksempeldata (Måløv Byvej 229)`. There is no sidebar on this screen.
- **Page head:** `SAGER` in the display face, uppercase, 1.9rem, with a faint count `4` beside it. Right-aligned, the filled primary button `+ Nyt projekt`.
- **Card grid:** `repeat(auto-fill, minmax(15.5rem, 1fr))`, gap 1rem. Each card is a white raised panel (6px radius, the panel shadow) and a button; hover turns its border accent.
  - **Thumb**, 6.2rem tall. The first card has an aerial photo. The other three are a striped placeholder (sunken surface with 1px lines every 15px) with a centred, letter-spaced uppercase label in the display face: `SCANNING I GANG`, `AFVENTER UPLOAD`, `KLADDE`.
  - **Body** (`.8rem .9rem .95rem`, gap .45rem):
    - the name in the display face, uppercase, 1.1rem: `MÅLØV BYVEJ 229`;
    - a muted address line: `2760 Måløv · Bygherre: KBH Ejendomme A/S`;
    - a stats line of bold figures with muted labels: `11 komponenter`, `54 % bevaring/genbrug`, `7 til gennemsyn`. A card without a scan reads `Ingen scanning endnu`; one with no reuse figure shows `—`;
    - a foot row: a status pill on the left and a faint `Frist: 12. sep 2026` on the right (`—` when unset).
  - **The four cards and their pills:** `Måløv Byvej 229` — accent `Gennemgang`; `Industrivej 8` — wait `Scanning i gang`; `Skolegade 22` — warn `Afventer upload`; `Herlev Hovedgade 12` — wait `Kladde`.
- **Footnote:** `Prototype-note: luftfoto, plan, punktsky, 360° og rum-model stammer fra den tekniske prototype (statiske billeder — viewere kommer fra scanning-pipelinen). Al data er demodata.`
- **Behaviour (from the source):** clicking the first card opens the case (`02 Sag`, i.e. Overblik). The other cards and `+ Nyt projekt` do nothing. The first card's stats are computed live from the demo survey.

### `onsite.png` (390×844, light)

- **Prototype chrome:** the same top bar, wrapped onto two lines at phone width, `03 On-site` active.
- **The phone:** a 300px navy body with a 26px radius and a navy-line border.
  - **Camera stage**, 270px tall: a dark grey gradient standing in for the live camera. An accent-bordered **reticle** (2px, 6px radius) sits at 17 % / 26 % and covers 48 % × 40 %. A white **detection chip** sits at 12 % / 70 % with the bold line `Vinduesparti, aluminium — registreret` and the muted small line `Office Zone · sikkerhed 82 %`.
  - **Sheet** (navy, `.8rem .8rem 1rem`, gap .5rem). Four rows share one style (8px radius, navy-line border, navy-2 fill, on-navy text, 600 weight, an icon column 1rem wide):
    1. `☆ Markér som vigtig`. It is a toggle: on, the icon becomes `★` and the border and text take the star colour;
    2. `＋ Tilføj ekstra foto`;
    3. `◎ Registrér prøve her`;
    4. `✎` followed by a borderless text input with the placeholder `Hurtig note…`.

    Below them is the accent button `Videre →`, full width, bold, 8px radius.
- **Side notes** (beside the phone on a wide screen, below it on a phone): the heading `ON-SITE ER EN AFBRYDELSE, IKKE LISTEN` and three bullets:
  - `Scanningen kører — brugeren stopper kun for at markere noget som vigtigt, tage ekstra fotos, lægge en note — eller registrere en prøve på stedet (kobles til bygningsdel + position).`
  - `Mængder, metadata og godkendelse venter til gennemsynet i Kortlægning.`
  - `★ og prøver følger med ind i sagen og kan filtreres frem.`
- **Behaviour (from the source):** only the ★ row does anything; it toggles locally. The other rows, the note and `Videre →` are inert. In the prototype's demo data a type's note reads `★ fra on-site: 8–10 stk. skønnes direkte genbrugelige.`, so On-site's ★ is meant to reach Kortlægning.

## Rulings (spec silent, or contradicted by the code or the prototype)

- **R1 — Sager lists the one open project.**
  - The spec says: "`rux gui` serves one `.rux`. The Sager screen lists the open project as its card and says how to open another (`rux -p <fil>.rux gui`)." The grid keeps the prototype's `auto-fill` columns, so a longer list from `ruxd` drops in without a layout change.
  - The head reads `SAGER` with the count of cards (`1`).
  - **`+ Nyt projekt` is not drawn.** Creating a project is a `rux` command, and listing many is `ruxd`'s job (spec § Out of scope, #265 Phase 6). In its place a panel `Åbn en anden sag` gives the command and says the multi-case list belongs to the server edition. Follow-up: draft 37.
  - The prototype's footnote is a sketch note and is not drawn.
- **R2 — The card shows what the project stores.**
  - **Thumb:** a server-rendered floor plan (`GET /renders?view=plan`) stands in for the aerial photo, which no source provides. When the render fails (no renderer: 503; no cloud: 404), the card falls back to the prototype's striped thumb, labelled with the case status.
  - **Name:** `caseName` (the record's name, else the `.rux` stem), as in Overblik.
  - **Sub line:** the address and `udarbejdet af <organisation>`, whichever exist, else `Ingen adresse registreret`. There is no bygherre field; follow-up draft 41.
  - **Stats:** `<counts.all> komponenter`, `<reuse_share> % bevaring/genbrug` (`—` when null) and `<counts.queue> til gennemsyn`, all from `GET /survey/summary`. A project with no survey types reads `Ingen kortlægning endnu`.
  - **Foot:** the status pill (R3), and `Registreret <dd.mm.yyyy>` from `survey_date` in place of `Frist`, since no deadline is stored (draft 24). It shows `—` when unset. `survey_date` is a user-entered ISO date, not a server timestamp, so it goes through `danishDate`, not `parseServerUtc`.
  - The whole card is one link to Overblik (`/`), as in the prototype.
- **R3 — Case status is derived, never stored** (`caseStatus`, pure):
  - `Kladde` (wait): no survey types at all, rejected ones included;
  - `Gennemgang` (accent): any type in the queue, or any type blocking Indberetning (`GET /survey/fractions` `blocking_types`). It is also the answer while the fractions are unknown (loading or failed), because "done" must not be claimed without them;
  - `Klar til indberetning` (good): `fractions.ready`;
  - `Gennemgået` (good): nothing blocks, but there is no fraction to report (all bevaring, say).

  The prototype's `Scanning i gang` and `Afventer upload` describe capture and upload states that only a `ruxd` deployment has. They are not used.
- **R4 — The prototype's top tabs are prototype navigation, not app chrome.** `01 Sager / 02 Sag / 03 On-site` switch between the artifact's screens; the app already has its sidebar.
  - Sager is reached by the sidebar's `← Alle sager`, now a live `NavLink`. `ALL_CASES_PENDING` is deleted.
  - On-site gets a sidebar entry `On-site`, last in the Sag group, and a link on the Sager panel.
  - Sager renders inside the normal shell, sidebar included. The sidebar still names the one open project, which is accurate.
- **R5 — The whole shell becomes responsive; On-site gets no shell of its own** (draft 25).
  - **Below 900px** (the prototype's own `.sag` breakpoint):
    - the sidebar becomes an off-canvas drawer opened by a `☰` button (`aria-label="Menu"`, `aria-expanded`, `aria-controls`) at the left of the title bar;
    - a scrim covers the content while it is open;
    - the drawer closes on Esc (focus returns to the Menu button), on a scrim click and on any navigation;
    - when closed it is `visibility: hidden`, so its links leave the tab order;
    - the title bar hides its divider and its meta items (schema, backend, version). The product name, the project name (ellipsised), the job indicator and the theme toggle stay.
  - **Above 900px** nothing changes.
  - **Why one shell, not a dedicated phone shell:** a second shell would duplicate the title bar's job, connection and project state and the badge reads, and On-site needs exactly that state on the phone. The drawer is also what every other route needs at 390px.
  - **The guarantee:**
    - no document-level horizontal overflow at 390px on every route. `<main>` scrolls its own content (`overflow: auto`, `min-width: 0`), so once the title bar fits and the sidebar leaves the flow, no route can widen the document. The shots check the case screens and one tool route (`/projektdata`) to prove it;
    - no `<main>` overflow at 390px on Sager, On-site and the five case screens.

    The tool screens (Viewport, Posegraf, …) are not redesigned for a phone.
- **R6 — On-site walks the stored bygningsdele. It is not a live camera.**
  - `rux gui` has no live capture or detection stream. The prototype's camera stage therefore becomes the part's **best sensor-frame photo**: `GET /instances/{cloud}/{id}/frames`, then `GET /frames/{id}/image` for the top frame.
    - The stage takes the sensor frame's aspect ratio (`ProjectSummary.sensor_frames.width/height`), so the photo fills it undistorted.
    - The **reticle** is centred on the instance centroid's projection (`u`, `v` of the top frame), at the prototype's 48 % × 40 % size, clamped inside the stage.
  - **The detection chip** reads the type name over `RX-008 · Office Zone · sikkerhed 82 %`. "— registreret" is dropped: nothing is detected live.
  - **A part with no instance link**, like every part of the demo seed, gets a dark placeholder with `Intet foto — bygningsdelen er ikke koblet til en instans.` The chip stays.
  - **Which part:**
    - `?del=RX-008` selects one;
    - an unknown or missing code selects the first stop. An unknown code also shows the notice `RX-404 findes ikke — viser RX-002.`;
    - a `Bygningsdel` select above the stage lists every stop, grouped by room.
  - **Walk order:** rooms by Danish collation, then part code (numeric). Parts without a room come last, under `Uden rum`. Parts of rejected types are left out.
  - **`Videre →`:**
    - moves to the next stop, wrapping at the end, and names it: `Videre → RX-011`;
    - with one stop it reads `Ingen flere bygningsdele` and is disabled;
    - it navigates with `replace`, so Back leaves On-site instead of stepping back through parts;
    - it is navigation, not a write, so it is **not** gated on `busy`. The note's blur commits before the navigation, on the same queue.
  - **The sheet is re-keyed per part** (`key={part.code}`). Otherwise a note draft typed on RX-008 would still be showing on RX-011 when both stored notes are empty: `useTextDraft` only resets when `current` changes.
- **R7 — What the sheet writes** (all against the part; nothing touches quantities, metadata or approval):
  - **★:** `☆ Markér som vigtig` toggles the part's `starred` (`PATCH /survey/parts/{code}`). While on, it reads `★ Vigtig — tryk for at fjerne` in the star colour, with `aria-pressed`. It is a button, so it is gated on `busy`; the next value is computed from the current one, and two quick taps must not send the same value twice.
  - **Note:** the `Hurtig note…` input edits the part's `note` through `useTextDraft` + `fieldKeys`:
    - it commits on blur and on Enter;
    - Esc reverts without committing;
    - an untouched blur sends nothing;
    - it is never gated on `busy`;
    - it shows the stored note, so a second visit edits it rather than adding a second one.
  - **Sample:** `◎ Registrér prøve her` opens an inline form inside the sheet: `Prøve` (required, placeholder `fx PCB i fugemasse`), `Hvor præcist` (placeholder `fx fuge ved vindue mod nord`), `Annullér` and `Registrér prøve` (gated on `busy` and on an empty title).
    - Keys follow Miljø's `NewSampleForm`: Esc anywhere in the form cancels it, Ctrl/⌘+Enter submits, Enter in a field submits natively.
    - It posts `{title, what, type_ids: [part.type_id], part_code, stage: 'udtaget'}`, then re-reads `GET /survey` and `GET /samples`.
    - The toast is `✓ P-04 registreret ved RX-008`, extended with the gate effect when the type just started to wait: `✓ P-04 registreret ved RX-002 — Betonsøjler, bærende afventer nu prøvesvar`.
    - On failure the form stays open with its text.
  - **The type's samples** are listed under the sheet, `Prøver på typen`: one row per sample linked to the part's type, a cross-link to its Miljø card, its stage pill (`statusPill`), and `· her` when it was taken at this part. A surveyor sees that a sample is already planned before registering another.
  - **`＋ Tilføj ekstra foto` is not drawn** (R9).
  - **Footnote:** the prototype's second side note, verbatim: `Mængder, metadata og godkendelse venter til gennemsynet i Kortlægning.`, followed by the cross-link `Åbn <type> i Kortlægning` (`/kortlaegning?type=<id>`). The first side note is design rationale; the third is what R13 builds.
- **R8 — Samples record the part they were taken at (schema v24).**
  - The prototype says a sample registered on site is "kobles til bygningsdel + position". Position is not available, since the phone has no pose in the project. The bygningsdel is.
  - Spec decision 4 rules out writing it into the free-text `what`. So `samples` gains a nullable `part_code TEXT`, with `ProjectDB::SampleRecord::part_code` and `add_sample(title, what, part_code)`.
  - It is **not a foreign key**. Like `survey_parts.instance_guid`, it may outlive what it names, and a reader shows the code as stored. This also keeps the v23 rollback test possible: sqlite cannot `DROP COLUMN` a column with a foreign key.
  - A read-only open of a pre-v24 project selects `NULL` in its place (the `columnExists` gate Phase 5 used for `blocking_types`).
  - **`POST /samples` gains two fields:**
    - `part_code`: unknown → 404. The part's type is always added to `type_ids`, because the approval gate is a property of the type (spec § domain model);
    - `stage`: `planlagt` (the default) or `udtaget`. Anything else is 400, because later stages need a lab and go through `PATCH`, and `svar` without a result would count as clean (Phase 4).

    Every refusal is checked before the first write.
  - **Miljø & prøver** shows `Udtaget ved RX-008 · Office Zone` on such a card, as a cross-link to the part's type. Rejected types and unknown codes show plain text.
  - **`rux gui --help`** gains a NOTES line on phone access (R10).
- **R9 — Photo capture is out of scope.**
  - The spec's Phase 6 line names exactly three writes: "phone capture sheet writing ★ / note / sample against a bygningsdel". A photo is not one of them.
  - A photo would need blob storage, an upload endpoint, a capture path and a place in Kortlægning's evidence panel. That is the same storage gap that made Phase 4 leave out `Upload miljørapport (PDF)`, and the same rule applies: a button with nothing behind it is not drawn.
  - Follow-up: draft 36, which says its storage should be designed together with draft 22.
- **R10 — Phone access is a recipe, not a security change.**
  - `rux gui` binds loopback and refuses any non-loopback `Origin` with 403 (`SecurityMiddleware`). A phone on the LAN therefore needs both flags: `rux -p <fil>.rux gui --bind <din-ip> --allow-origin http://<din-ip>:<port>`.
  - Phase 6 does **not** relax the Origin check. A "same origin as Host" rule would look natural, but under DNS rebinding the attacker chooses the Host. It would reopen exactly the hole the middleware closes.
  - Sager's panel shows the recipe, with the port taken from the page's own address, and the warning `Serveren har ingen adgangskontrol — gør det kun på et netværk, du stoler på.` `rux gui --help` gets the same line.
  - Pairing or authentication for LAN access is a follow-up (draft 38).
- **R11 — One app-wide write chain (draft 30, in).**
  - On-site adds a third writer of survey state. "Write on the phone sheet, then open Kortlægning" is the same race the draft describes for Miljø.
  - `useMutationQueue` therefore enqueues on `appWriteChain` (`src/app/writeChain.ts`) by default. Kortlægning, Miljø & prøver, On-site, Overblik, Indberetning and Sager start their first load with `appWriteChain.idle()`, so a queued write from the screen just left lands before the GET.
  - `busy` stays per page, because it gates the page's own buttons.
  - **Rapport keeps a page chain** (`scope: 'page'`). Report generation can take minutes, changes no survey state, and must not hold up every other screen's first load.
  - The regression check holds a `POST /samples` in the browser, navigates to Kortlægning, and asserts that no `GET /survey` leaves until the POST is released, and that the gate then shows the new sample.
- **R12 — One cross-link style (draft 32, in).**
  - Phase 6 adds four more cross-links (On-site → Kortlægning, On-site → Miljø, Miljø's `Udtaget ved`, Sager → On-site).
  - `components/controls.module.css` gains `crossLink`: accent-deep, underlined, text colour on hover, the focus ring. The draft recommends the accent-deep style as the clearer link.
  - Kortlægning's `SampleLine` `.link`, Miljø's `.typeLink` and every new cross-link compose it.
- **R13 — Kortlægning shows what On-site wrote.**
  - The prototype says "★ og prøver følger med ind i sagen og kan filtreres frem". Samples already reach Kortlægning (its sample line).
  - For ★: the tools gain a `Kun vigtige ★` checkbox (`Filters.starred`), which keeps a type that is starred or has a starred part.
  - Part rows draw `★` before the code and a faint `✎` after it when the part has a note. The note is the marker's tooltip and accessible name.
  - The detail panel already shows and edits a part's ★ and note.
- **R14 — Drafts 26 and 31 stay out.**
  - Phase 6 does not touch `EditDialog` (draft 26, Esc saves on close) or Kortlægning's selection state (draft 31, selection not in the URL).
  - On-site links into Kortlægning by type (`?type=<id>`), which works on mount today. Both drafts stay open as they are.

## Global Constraints

- SPDX header on every new file:
  - TS/TSX: `// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `// SPDX-License-Identifier: GPL-3.0-or-later`;
  - C++: the same `//` form;
  - CSS: the `/* … */` block form;
  - shell and Python: `#`;
  - Markdown: the `<!-- … -->` block;
  - PNG: a `<name>.png.license` sidecar.
- **Tokens only.** No literal colour, radius, spacing or font size in `.module.css`, inline `style`, or TS strings/constants; only `var(--…)`. Three things are allowed:
  - 1px/2px hairline borders and focus outlines, as in Phases 3–5;
  - positions and aspect ratios computed from data (the reticle's percentages, the stage's `aspect-ratio`), which are not design values;
  - the media query breakpoint `900px`, because `var()` cannot appear in a media query. It is written once per file, with a comment naming R5.
  - **Never edit `src/tokens.css`.**
  - Check every changed CSS/TSX file with `python3 .claude/skills/design-studio/scripts/token_lint.py <files> --tsx`.
- **Token roles:**
  - pills use `<Pill tone>`;
  - filled primary buttons use `controls.module.css` `btnPrimary`;
  - field labels use `--font-size-2xs`, uppercase, `--tracking-caps`, `--color-text-muted` (`--color-on-chrome-muted` on the navy sheet);
  - headings use `--font-display`;
  - the sheet and the drawer use `--color-chrome`, `--color-chrome-raised`, `--color-chrome-border`, `--color-on-chrome(-muted)`;
  - the ★ uses `--color-star`;
  - the photo stage uses `--color-canvas`;
  - scrims use `--color-scrim` through `color-mix`.
- **Shared CSS goes through `composes`**, never copy-paste: `components/controls.module.css` (buttons, fields, checkbox, `crossLink`), `components/surfaces.module.css` (panel, heading, notice), `routes/viewHead.module.css` (page, head, title, sub, footnote).
- All UI copy on the case screens, Sager, On-site and the shell is Danish. Use the prototype's words where it has them; otherwise use the strings given in this plan verbatim. The tool screens stay English.
- **`src/kortlaegning/vocab.ts` is the single place where wire words become copy.** Stage labels come from `STAGE_LABEL` and sample status from Miljø's `statusPill`; no screen spells `udtaget` or `svar` itself.
- **Mutations go through `src/app/useMutationQueue.ts`**, now on the app-wide chain (R11). Never write a second ad-hoc chain.
- **Field commits are never dropped and never gated on `busy`.** That covers On-site's note blur. **Only buttons are gated on `busy`:** On-site's ★ and `Registrér prøve`. `Videre →` and the part select are navigation and are not gated.
- **An untouched field blur never commits.** On-site's note uses `useTextDraft` (`textCommit`): focusing and leaving the field sends nothing.
- **Esc in a text field reverts; Esc elsewhere closes** (`src/app/editorKeys.ts`, `useTextDraft`, `fieldKeys`).
  - The note field reverts on Esc and parks focus on the sheet (`tabIndex={-1}`).
  - The sample form cancels on Esc anywhere in it, as Miljø's create form does, because nothing in it is saved yet.
  - The open drawer closes on Esc and returns focus to the Menu button.
- **Keyboard handlers classify their target** with `src/app/keyTargets.ts` (`kindOf` / `isField` / `isControl`). Enter/Space on buttons, links and checkboxes keep their native activation. Phase 6 adds no document-level handlers: the drawer's Esc is a `keydown` on the drawer itself.
- Server state is never re-derived on the client. Miljøstatus, the gate, counts, reuse share, readiness and blocking counts come from response bodies. The client only words, orders and formats them. The case status (R3) is a wording of those server numbers.
- **`parseServerUtc` for every server timestamp.** Phase 6 shows none; `survey_date` is user-entered and goes through `danishDate` (R2).
- **CSV text cells get a formula-injection guard.** Phase 6 adds no CSV; any CSV added while executing this plan goes through `src/data/csvExport.ts`'s guard.
- No DOM test environment may be added: vitest runs in Node (`vite.config.ts` `environment: 'node'`). Testable logic is a pure exported function. Components are verified by screenshot in both themes and by the Playwright flow.
- **Browser checks use no fixed sleeps.**
  - They wait with Playwright `expect(...)` / `expect_response` / `expect_request` and assert on **captured requests** (`page.on("request")` / `page.on("response")`).
  - "Nothing was sent" is asserted by ordering. A later, real commit's response is awaited first, and then the captured list must hold only that later request.
  - The staleness check holds a request with `page.route` and releases it explicitly. It never delays one with a timer.
  - Static shots come from the same Playwright script, not from `shot.sh --wait`.
- **Backend builds run inside `nix develop`** with exactly this configure line:

  ```bash
  cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DCMAKE_CUDA_COMPILER=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9/bin/nvcc -DCUDAToolkit_ROOT=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9 -DCUDA_TOOLKIT_ROOT_DIR=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9
  ```

  - Builds run in the foreground. A build can run past the tool timeout; re-run the same `cmake --build …` command, which resumes where it stopped.
  - **Never touch `/home/mephisto/repos/ReUseX/build`**: the worktree has its own `build/`.
- Frontend commands run from the worktree root: `npm --prefix apps/rux/frontend test|run typecheck|run build`.
- Work happens in `/home/mephisto/repos/ReUseX/.worktrees/gui-phase6` (branch `gui-phase6-sager`). The dev servers use **gui port 8426 and vite port 5179**: `dev_env.sh start <project> 8426 5179`.
- Scratch projects live in `SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad`. `$SP/corridor-clouds.rux` is the cloud-bearing source project that Phases 3–5 used. Never seed or serve a tracked fixture in place; `dev_env.sh` serves a copy.
- Every commit carries the session trailers, written out in each command below. Never `--no-verify`.

## Review Focus

- **Schema v24 (R8):**
  - a sample round-trips `part_code`;
  - an unknown part code throws before anything is written;
  - a read-only open of a v23 project lists samples with `part_code` absent, through the `columnExists` gate;
  - a read-write open migrates and accepts a code;
  - the v22/v23 rollback tests delete `version >= 23` / `>= 22`, so they still exercise their migration.
- **`POST /samples` (R8):**
  - the part's type is always linked;
  - `stage` accepts only `planlagt`/`udtaget`;
  - every refusal (unknown part, empty code, bad stage, unknown type) writes nothing;
  - `GET /samples` serialises `part_code`, and openapi lists it as required and nullable.
- **The write chain (R11):**
  - `idle()` settles only after every earlier task, failing ones included;
  - every case-screen first load awaits it;
  - Rapport is the only `scope: 'page'` caller;
  - the browser check sees no `GET /survey` while the `POST` is held.
- **The drawer (R5):**
  - at 390px no route overflows the document;
  - the case screens, Sager and On-site do not overflow `<main>`;
  - a closed drawer is not focusable;
  - Esc closes it and focuses Menu;
  - navigation closes it;
  - above 900px the layout is pixel-identical to Phase 5 (compare the 1440 shots).
- **On-site:**
  - focusing and leaving the note sends nothing;
  - Esc reverts it without sending;
  - Enter commits once;
  - a note typed and followed straight by `Videre →` lands on the **old** part, and the new part's field is empty (the per-part key, R6);
  - ★ sends the negation of the current value and is disabled while busy;
  - the sample POST body is exactly `{title, what, type_ids:[type], part_code, stage:'udtaget'}`;
  - the toast names the gate effect only when the type just started to wait;
  - the walk order is rooms (`da` collation) then code, rejected types excluded.
- **Sager:**
  - the figures equal `GET /survey/summary`'s;
  - the status follows R3 (`Gennemgang` while fractions are unknown);
  - a failed plan render falls back to the striped thumb with the status label;
  - the card links to `/`, and `← Alle sager` links to `/sager`.
- **Kortlægning and Miljø (R13, R8):**
  - the ★ filter keeps starred types and types with a starred part;
  - part rows show ★ and ✎;
  - a P-## taken on site shows `Udtaget ved …` linking to its type.
- **Cross-links (R12):** `SampleLine`, `SampleCard`, On-site and Sager all compose `crossLink`. No other link rule colours a cross-screen link.
- **No 503** appears in any captured response during the Task 16 flow.

---

## File Structure

| File | Responsibility |
|---|---|
| `libs/reusex/include/core/ProjectDB.hpp` (modify) | `SampleRecord::part_code`; `add_sample(…, part_code)` |
| `libs/reusex/src/core/ProjectDB.cpp` (modify) | schema v24 (`samples.part_code`); sample reads/writes |
| `tests/unit/core/test_project_db_samples.cpp` (modify) | part-code round trip, unknown part, v23 → v24 migration |
| `tests/unit/core/test_project_db_reports.cpp`, `test_project_db_survey.cpp` (modify) | rollbacks delete every later version row |
| `apps/rux/src/gui/survey.cpp`, `apps/rux/include/gui/survey.hpp` (modify) | `POST /samples` `part_code` + `stage`; `sample_json` `part_code` |
| `apps/rux/src/gui.cpp` (modify) | `--help` NOTES: phone access (R10) |
| `tests/unit/rux_gui/test_gui_survey.cpp` (modify) | create-at-part, refusals |
| `docs/gui/openapi.yaml` (modify) | `Sample.part_code`; `POST /samples` body and 400 text |
| `apps/rux/frontend/dev/seed-survey-demo.sh` (modify) | `--varied`: a ★ part with a note, P-06 taken at RX-013 |
| `src/api/types.ts` (modify) | `Sample.part_code`, `SampleCreate.part_code/stage` |
| `src/test/surveyFixtures.ts`, `src/test/kortlaegning.page.test.ts`, `src/test/kortlaegning.detailPanel.test.ts`, `src/test/survey.client.test.ts` (modify) | builders; `surveyPart`; the create body |
| `src/app/serialQueue.ts`, `src/app/writeChain.ts` (new), `src/app/useMutationQueue.ts`, `src/test/serialQueue.test.ts` (modify) | R11 |
| `src/routes/{Kortlaegning,Miljoe,Overblik,Indberetning,Rapport}Page.tsx` (modify) | first loads await the chain; Rapport `scope: 'page'` |
| `src/components/controls.module.css`, `components/kortlaegning/SampleLine.module.css`, `components/miljoe/SampleCard.module.css` (modify) | `crossLink` (R12) |
| `src/app/AppShell.tsx` + css, `src/components/TitleBar.tsx` + css, `src/components/Sidebar.tsx` + css (modify) | the drawer (R5) |
| `src/app/navigation.ts`, `src/test/navigation.test.ts`, `src/app/links.ts`, `src/test/links.test.ts` (modify) | live `Alle sager`, `On-site` entry, `ONSITE_PATH`, `onsiteHref`, `parseOnsiteQuery` |
| `src/sager/model.ts`, `src/test/sager.model.test.ts` (new) | Sager logic |
| `src/components/sager/CaseCard.tsx` + css, `src/routes/SagerPage.tsx` + css (new) | Sager |
| `src/onsite/model.ts`, `src/test/onsite.model.test.ts` (new) | On-site logic |
| `src/components/onsite/CaptureStage.tsx`, `CaptureSheet.tsx`, `PartPicker.tsx` + css (new) | On-site pieces |
| `src/routes/OnsitePage.tsx` + css (new), `src/app/App.tsx` (modify) | `/sager`, `/on-site` |
| `src/kortlaegning/model.ts`, `src/components/kortlaegning/SurveyTable.tsx` + css, `src/test/kortlaegning.model.test.ts` (modify) | R13 |
| `src/miljoe/model.ts`, `src/components/miljoe/SampleCard.tsx`, `src/test/miljoe.model.test.ts` (modify) | `takenAt` (R8) |
| `docs/gui/images/prototype-v2/onsite.png` + `.license` (new, committed with this plan) | reference shot |
| docs: `docs/design/gui-kortlaegning-redesign.md`, `apps/rux/frontend/README.md`, `.claude/skills/design-studio/references/reusex-frontend.md`, `docs/DIRECTION.md` | screens, rules, follow-ups, changelog |
| `.github/issue-drafts/36-onsite-part-photos.md` to `41-case-client-bygherre.md` (new); `25-appshell-390px-overflow.md`, `30-cross-screen-staleness.md`, `32-cross-link-styling-inconsistency.md` (deleted, resolved) | follow-ups |

Unless a path is absolute or starts with a top-level directory, `src/…` is under `apps/rux/frontend/`.

---

### Task 1: Worktree build

**Files:** None in the repo.

**Interfaces:** Produces a worktree `build/apps/rux/rux` and `build/tests/reusex_unit_tests`, plus `apps/rux/frontend/node_modules`.

- [ ] **Step 1: Build** (inside `nix develop`, foreground, the CUDA-workaround configure line; re-run the same command if it stops at the tool timeout, since it resumes)

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
nix develop --command bash -c 'cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DCMAKE_CUDA_COMPILER=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9/bin/nvcc -DCUDAToolkit_ROOT=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9 -DCUDA_TOOLKIT_ROOT_DIR=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9 && cmake --build build --target rux reusex_unit_tests -j"$(nproc)"'
npm --prefix apps/rux/frontend ci
npm --prefix apps/rux/frontend test
```

Expected: `build/apps/rux/rux` exists, `npm ci` completes and the vitest suite is green on `main`'s code. That green suite is the baseline every later task keeps.

No commit (this task changes nothing in the repo).

---

### Task 2: Samples record the part they were taken at — schema v24 (R8)

**Files:**
- Modify `libs/reusex/include/core/ProjectDB.hpp` (`SampleRecord`, `add_sample`).
- Modify `libs/reusex/src/core/ProjectDB.cpp` (`LATEST_SCHEMA_VERSION`, the migration dispatch, `migrateToV24`, the sample reads, `add_sample`).
- Modify `tests/unit/core/test_project_db_samples.cpp`, `tests/unit/core/test_project_db_reports.cpp`.

**Interfaces:**

```cpp
struct SampleRecord {
  // … existing fields unchanged …
  std::optional<std::string> part_code; // schema v24; kept LAST
};
/// @throws std::out_of_range when part_code names no survey part.
SampleRecord add_sample(std::string_view title, std::string_view what,
                        const std::optional<std::string> &part_code = std::nullopt);
```

Existing callers of `add_sample(title, what)` compile unchanged.

- [ ] **Step 1: Failing tests.** In `tests/unit/core/test_project_db_samples.cpp`, replace the include block

```cpp
#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <stdexcept>
```

with

```cpp
#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <optional>
#include <stdexcept>
#include <string>
```

In its anonymous namespace, after `add_type`, add:

```cpp
void add_part(ProjectDB &db, const char *code, int64_t type_id) {
  db.add_survey_part({code, type_id, std::nullopt, std::nullopt, std::nullopt,
                      "Office Zone", 1, false, "", {}, {}});
}
```

Append:

```cpp
// GUI Phase 6 (On-site): a sample registered at a bygningsdel records it.

TEST_CASE("Samples_Add_WithPartCode_RoundTrips", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = add_type(db, "Vinduespartier, aluminium");
  add_part(db, "RX-008", t);
  const auto s =
      db.add_sample("Asbest i fugemasse", "Fuge mod nord", std::string("RX-008"));
  CHECK(s.part_code == std::optional<std::string>("RX-008"));
  CHECK(db.sample(s.id)->part_code == std::optional<std::string>("RX-008"));
  CHECK(db.samples().front().part_code == std::optional<std::string>("RX-008"));
  CHECK_FALSE(db.add_sample("Uden del", "").part_code.has_value());
}

TEST_CASE("Samples_Add_UnknownPart_ThrowsAndWritesNothing",
          "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK_THROWS_AS(db.add_sample("x", "", std::string("RX-404")),
                  std::out_of_range);
  CHECK(db.samples().empty());
  // The refused sample did not consume a code either.
  CHECK(db.add_sample("y", "").code == "P-01");
}

TEST_CASE("Samples_MigratesFromV23_PartCodeNull",
          "[ProjectDB][samples][migration]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
    db.add_sample("PCB i fugemasse", "Fugemasse");
  }
  {
    // Roll back to v23: drop the v24 column and every later version row.
    sqlite3 *raw = nullptr;
    REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
    const char *sql = "ALTER TABLE samples DROP COLUMN part_code;"
                      "DELETE FROM schema_version WHERE version >= 24;";
    REQUIRE(sqlite3_exec(raw, sql, nullptr, nullptr, nullptr) == SQLITE_OK);
    sqlite3_close(raw);
  }
  {
    // Read-only opens never migrate; the list must still work on v23.
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    const auto list = ro.samples();
    REQUIRE(list.size() == 1);
    CHECK_FALSE(list[0].part_code.has_value());
    CHECK_FALSE(ro.sample(list[0].id)->part_code.has_value());
  }
  ProjectDB db(tmp.path, /*readOnly=*/false);
  CHECK(db.schema_version() == ProjectDB::latest_schema_version());
  CHECK_FALSE(db.samples().front().part_code.has_value());
  const auto t = add_type(db, "Vinduer");
  add_part(db, "RX-001", t);
  CHECK(db.add_sample("Ny", "", std::string("RX-001")).part_code ==
        std::optional<std::string>("RX-001"));
}
```

In `tests/unit/core/test_project_db_reports.cpp`, test `ReportPdfs_MigratesFromV22_OldVersionsHaveNoBlockingCount`, replace

```cpp
    // Roll back to v22: drop the v23 column and its version row.
```

with

```cpp
    // Roll back to v22: drop the v23 column and every later version row
    // (deleting only 23 would leave the DB reading as 24, and migrateToV23
    // would never re-run).
```

and in the same test replace `"DELETE FROM schema_version WHERE version = 23;";` with `"DELETE FROM schema_version WHERE version >= 23;";`.

- [ ] **Step 2: Run → FAIL (compile error: `add_sample` takes two arguments, `part_code` is not a member)**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
nix develop --command bash -c 'cmake --build build --target reusex_unit_tests -j"$(nproc)"'
```

Expected: the build fails in `test_project_db_samples.cpp`.

- [ ] **Step 3: Header.** In `libs/reusex/include/core/ProjectDB.hpp`, in `struct SampleRecord`, after `std::string updated_at;`, add:

```cpp
    /// The bygningsdel (survey part code) the sample was taken at, set when
    /// it was registered on site (schema v24). Not a foreign key: like
    /// SurveyPartRecord::instance_guid it may outlive what it names, and a
    /// reader shows the code as stored. Kept LAST so positional aggregate
    /// initializers still compile.
    std::optional<std::string> part_code;
```

Replace

```cpp
  SampleRecord add_sample(std::string_view title, std::string_view what);
```

with

```cpp
  /// @throws std::out_of_range when part_code names no survey part; nothing
  /// is written then.
  SampleRecord
  add_sample(std::string_view title, std::string_view what,
             const std::optional<std::string> &part_code = std::nullopt);
```

- [ ] **Step 4: Migration.** In `libs/reusex/src/core/ProjectDB.cpp`:
  - replace `static constexpr int LATEST_SCHEMA_VERSION = 23;` with `static constexpr int LATEST_SCHEMA_VERSION = 24;`;
  - after the dispatch block

```cpp
    if (current < 23) {
      migrateToV23();
    }
```

    add

```cpp

    if (current < 24) {
      migrateToV24();
    }
```

  - directly after the closing `}` of `void migrateToV23()`, add:

```cpp

  void migrateToV24() {
    reusex::info("Migrating database to schema version 24");

    // GUI Phase 6 (On-site): the bygningsdel a sample was taken at. NULL for
    // a sample registered without one (Miljø & prøver, or before v24). Plain
    // TEXT, not a foreign key: see SampleRecord::part_code.
    if (!columnExists("samples", "part_code"))
      execOrThrow("ALTER TABLE samples ADD COLUMN part_code TEXT;");

    insertSchemaVersion(
        24, "Record the survey part a sample was taken at (Phase 6)");
    reusex::info("Migration to schema version 24 complete");
  }
```

- [ ] **Step 5: Reads and the write.** Replace the anonymous-namespace block that starts with `ProjectDB::SampleRecord read_sample_row(sqlite3_stmt *s) {` and ends with the `kSampleColumns` definition and `} // namespace` with:

```cpp
namespace {
ProjectDB::SampleRecord read_sample_row(sqlite3_stmt *s) {
  ProjectDB::SampleRecord r;
  r.id = sqlite3_column_int64(s, 0);
  r.code = column_text(s, 1);
  r.title = column_text(s, 2);
  r.what = column_text(s, 3);
  r.stage = core::sample_stage_from_string(column_text(s, 4))
                .value_or(core::SampleStage::planlagt);
  r.result = core::sample_result_from_string(column_text(s, 5))
                 .value_or(core::SampleResult::none);
  r.created_at = column_text(s, 6);
  r.updated_at = column_text(s, 7);
  if (sqlite3_column_type(s, 8) != SQLITE_NULL)
    r.part_code = column_text(s, 8);
  return r;
}
/// The sample columns read_sample_row expects. A read-only open of a pre-v24
/// project has no part_code column (read-only opens never migrate), so it
/// selects NULL in its place.
std::string sample_columns(bool has_part_code) {
  return std::string(
             "id, code, title, what, stage, result, created_at, updated_at, ") +
         (has_part_code ? "part_code" : "NULL");
}
} // namespace
```

In `ProjectDB::samples()`, replace

```cpp
  const std::string sql =
      std::string("SELECT ") + kSampleColumns + " FROM samples ORDER BY id;";
```

with

```cpp
  const std::string sql =
      "SELECT " +
      sample_columns(impl_->columnExists("samples", "part_code")) +
      " FROM samples ORDER BY id;";
```

In `ProjectDB::sample(int64_t id)`, replace

```cpp
  const std::string sql =
      std::string("SELECT ") + kSampleColumns + " FROM samples WHERE id = ?;";
```

with

```cpp
  const std::string sql =
      "SELECT " +
      sample_columns(impl_->columnExists("samples", "part_code")) +
      " FROM samples WHERE id = ?;";
```

Replace the whole `ProjectDB::add_sample` definition with:

```cpp
ProjectDB::SampleRecord
ProjectDB::add_sample(std::string_view title, std::string_view what,
                      const std::optional<std::string> &part_code) {
  impl_->checkWritable();
  // Refuse before the first write, so a bad code consumes no P-## code.
  if (part_code && !survey_part(*part_code))
    throw std::out_of_range("no survey part " + *part_code);
  // Codes are never reused: take the highest ever issued (AUTOINCREMENT's
  // sqlite_sequence survives deletes), not the current count.
  sqlite3_stmt *seq =
      prepare_or_throw(impl_->db,
                       "SELECT COALESCE((SELECT seq FROM sqlite_sequence WHERE "
                       "name = 'samples'), 0);",
                       "add_sample");
  StmtGuard seq_guard(seq);
  sqlite3_step(seq);
  const int next = sqlite3_column_int(seq, 0) + 1;

  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db,
                       "INSERT INTO samples (code, title, what, part_code) "
                       "VALUES (?,?,?,?) RETURNING id;",
                       "add_sample");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, core::sample_code(next));
  bind_text(stmt, 2, title);
  bind_text(stmt, 3, what);
  if (part_code)
    bind_text(stmt, 4, *part_code);
  else
    sqlite3_bind_null(stmt, 4);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    throw std::runtime_error("add_sample: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  return *sample(sqlite3_column_int64(stmt, 0));
}
```

- [ ] **Step 6: Run → PASS**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
nix develop --command bash -c 'cmake --build build --target reusex_unit_tests -j"$(nproc)" && ctest --test-dir build -R "Samples_|ReportPdfs_|SurveySchema_MigratesFromV21|ProjectDb" --output-on-failure --parallel "$(nproc)"'
```

Expected: all pass, including the existing `SurveySchema_MigratesFromV21`. That test drops `samples`, so v22 recreates it without `part_code` and v24 adds the column again, which is why `migrateToV24` checks `columnExists`.

- [ ] **Step 7: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
git add libs/reusex/include/core/ProjectDB.hpp libs/reusex/src/core/ProjectDB.cpp tests/unit/core/test_project_db_samples.cpp tests/unit/core/test_project_db_reports.cpp
git commit -m "feat(core): a sample records the bygningsdel it was taken at (schema v24)" -m "samples gains a nullable part_code; add_sample takes it and refuses an unknown part before writing. Read-only opens of a pre-v24 project select NULL in its place. The v22 rollback test now deletes every later version row, so its migration still runs." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 3: `POST /samples` takes the part and the stage (R8, R10)

**Files:**
- Modify `apps/rux/src/gui/survey.cpp`, `apps/rux/include/gui/survey.hpp`, `apps/rux/src/gui.cpp`.
- Modify `tests/unit/rux_gui/test_gui_survey.cpp`.
- Modify `docs/gui/openapi.yaml`.

**Interfaces:**
- `POST /samples` body: `{ title, what?, type_ids?, part_code?, stage?: 'planlagt' | 'udtaget' }`.
- `Sample` JSON gains `part_code: string | null`.

- [ ] **Step 1: Failing tests.** Append to `tests/unit/rux_gui/test_gui_survey.cpp` (after the existing `status_of` helper, i.e. at the end of the file):

```cpp
// ===========================================================================
// On-site (GUI Phase 6): a sample registered at a bygningsdel
// ===========================================================================

TEST_CASE("CreateSample_AtAPart_LinksItsTypeAndRecordsTheCode",
          "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t =
      add_type(db, "Vinduespartier, aluminium", core::Treatment::genbrug, 3.1);
  db.add_survey_part({"RX-008", t, std::nullopt, std::nullopt, std::nullopt,
                      "Office Zone", 26, false, "", {}, {}});
  const auto s = create_sample_json(
      db, R"({"title":"Asbest i fugemasse","what":"Fuge mod nord",)"
          R"("part_code":"RX-008","stage":"udtaget"})");
  CHECK(s.at("part_code") == "RX-008");
  CHECK(s.at("type_ids") == nlohmann::json::array({t}));
  CHECK(s.at("stage") == "udtaget");
  CHECK(s.at("result").is_null());
  // The part's type now waits for the sample: the gate is on the type.
  CHECK(core::environment_status_of(db, t) ==
        core::EnvironmentStatus::afventer);
  // The part's type is added once, even when the body already names it.
  const auto again = create_sample_json(
      db, R"({"title":"PCB","part_code":"RX-008","type_ids":[)" +
              std::to_string(t) + "]}");
  CHECK(again.at("type_ids") == nlohmann::json::array({t}));
  CHECK(again.at("stage") == "planlagt");
  // Without a part the field is null, also in the list.
  CHECK(create_sample_json(db, R"({"title":"Bly"})").at("part_code").is_null());
  CHECK(samples_json(db).at("samples").at(0).at("part_code") == "RX-008");
}

TEST_CASE("CreateSample_PartAndStage_RefusalsWriteNothing",
          "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK(status_of([&] {
          create_sample_json(db, R"({"title":"x","part_code":"RX-404"})");
        }) == 404);
  CHECK(status_of([&] {
          create_sample_json(db, R"({"title":"x","part_code":""})");
        }) == 400);
  CHECK(status_of([&] {
          create_sample_json(db, R"({"title":"x","part_code":7})");
        }) == 400);
  CHECK(status_of([&] {
          create_sample_json(db, R"({"title":"x","stage":"svar"})");
        }) == 400);
  CHECK(status_of([&] {
          create_sample_json(db, R"({"title":"x","stage":"lab"})");
        }) == 400);
  CHECK(status_of([&] {
          create_sample_json(db, R"({"title":"x","type_ids":[9999]})");
        }) == 404);
  CHECK(db.samples().empty());
}
```

- [ ] **Step 2: Run → FAIL**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
nix develop --command bash -c 'cmake --build build --target reusex_unit_tests -j"$(nproc)" && ctest --test-dir build -R "CreateSample_" --output-on-failure'
```

Expected: `part_code` is missing from the JSON (`at()` throws), and `stage:"svar"` answers 200 instead of 400.

- [ ] **Step 3: Implement.** In `apps/rux/src/gui/survey.cpp`, add `#include <algorithm>` as the first standard include (before `#include <cmath>`). In `sample_json`, replace

```cpp
          {"type_ids", s.type_ids},
          {"created_at", s.created_at},
          {"updated_at", s.updated_at}};
}
```

(the end of `sample_json`) with

```cpp
          {"type_ids", s.type_ids},
          {"part_code", opt(s.part_code)},
          {"created_at", s.created_at},
          {"updated_at", s.updated_at}};
}
```

Replace the whole `create_sample_json` definition with:

```cpp
json create_sample_json(reusex::ProjectDB &db, const std::string &body) {
  const auto j = parse_object(body);
  const auto title = opt_string(j, "title");
  if (!title || title->empty())
    throw HttpError(400, "'title' is required and must be non-empty");
  const auto what = opt_string(j, "what").value_or("");
  auto types = id_list(j, "type_ids");
  const auto part_code = opt_string(j, "part_code");
  if (part_code && part_code->empty())
    throw HttpError(400, "'part_code' must be non-empty");
  const auto stage =
      opt_enum<core::SampleStage>(j, "stage", core::sample_stage_from_string);
  // A sample is registered before it reaches a lab. Later stages are reached
  // through PATCH, and `svar` without a result would count as clean.
  if (stage && *stage != core::SampleStage::planlagt &&
      *stage != core::SampleStage::udtaget)
    throw HttpError(400, "'stage' on create must be 'planlagt' or 'udtaget'");
  return mapped([&] {
    // Every refusal is checked before the first write.
    for (auto t : types)
      if (!db.survey_type(t))
        throw std::out_of_range("no survey type " + std::to_string(t));
    if (part_code) {
      const auto part = db.survey_part(*part_code);
      if (!part)
        throw std::out_of_range("no survey part " + *part_code);
      // The approval gate is a property of the type, so a sample taken at a
      // part always covers that part's type.
      if (std::find(types.begin(), types.end(), part->type_id) == types.end())
        types.push_back(part->type_id);
    }
    const auto s = db.add_sample(*title, what, part_code);
    if (!types.empty())
      db.set_sample_links(s.id, types);
    if (stage == core::SampleStage::udtaget) {
      reusex::ProjectDB::SamplePatch p;
      p.stage = core::SampleStage::udtaget;
      core::update_sample_checked(db, s.id, p);
    }
    return sample_json(*db.sample(s.id));
  });
}
```

In `apps/rux/include/gui/survey.hpp`, replace the doc comment of `create_sample_json`

```cpp
/// `POST /samples`: register a new environmental sample. Body: `{ "title"
/// (required), "what"?, "type_ids"? }`.
/// @throws HttpError(400) when `title` is missing/empty.
/// @throws HttpError(404) when a given `type_ids` entry is unknown.
```

with

```cpp
/// `POST /samples`: register a new environmental sample. Body: `{ "title"
/// (required), "what"?, "type_ids"?, "part_code"?, "stage"? }`. A
/// `part_code` records the bygningsdel it was taken at (On-site) and always
/// links that part's type; `stage` is `planlagt` (default) or `udtaget`.
/// @throws HttpError(400) when `title` is missing/empty, `part_code` is empty
/// or not a string, or `stage` is not `planlagt`/`udtaget`.
/// @throws HttpError(404) when a `type_ids` entry or the part is unknown.
```

In `apps/rux/src/gui.cpp`, in the NOTES text, replace

```
  - Binds to 127.0.0.1 by default. There is NO authentication and the API can
    execute pipeline stages, so only change --bind if you know what that means.
```

with

```
  - Binds to 127.0.0.1 by default. There is NO authentication and the API can
    execute pipeline stages, so only change --bind if you know what that means.
  - To use On-site from a phone on the same network, bind the LAN address and
    allow the page's own origin, e.g.
    --bind 192.168.1.20 --allow-origin http://192.168.1.20:8420
```

- [ ] **Step 4: openapi.** In `docs/gui/openapi.yaml`:
  - In the `Sample` schema, replace `required: [id, code, title, what, stage, result, type_ids, created_at, updated_at]` with `required: [id, code, title, what, stage, result, type_ids, part_code, created_at, updated_at]`, and after `type_ids: { type: array, items: { type: integer } }` add:

```yaml
        part_code:
          type: string
          nullable: true
          example: RX-008
          description: The bygningsdel the sample was taken at, when it was registered on site (schema v24); null otherwise.
```

  - In `POST /samples`, after `type_ids: { type: array, items: { type: integer } }` in the request body properties, add:

```yaml
                part_code:
                  type: string
                  description: The bygningsdel the sample is taken at. Its survey type is always added to `type_ids` (the approval gate is on the type); an unknown code is 404.
                stage:
                  type: string
                  enum: [planlagt, udtaget]
                  default: planlagt
                  description: "`udtaget` when the sample is registered on site, already taken. Later stages are reached with PATCH."
```

  - In the same operation, replace `description: title is missing/empty` under `"400"` with `description: title is missing/empty, part_code is empty or not a string, or stage is not planlagt/udtaget`. Then give the operation a description: directly under `summary: Register an environmental sample`, add:

```yaml
      description: |
        Creates a sample at stage `planlagt` (or `udtaget`, see `stage`) with
        the next P-## code. Every refusal is checked before the first write.

        Performed under the project's writer lock; see `409`/`503`.
```

- [ ] **Step 5: Run → PASS**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
nix develop --command bash -c 'cmake --build build --target rux reusex_unit_tests -j"$(nproc)" && ctest --test-dir build -R "CreateSample_|SampleEndpoints_|SamplesJson_|gui_api_contract_parses" --output-on-failure --parallel "$(nproc)"'
python3 scripts/check-openapi.py
./build/apps/rux/rux gui --help | grep -A2 "On-site from a phone"
```

Expected: all pass; `check-openapi.py` exits 0; the help prints the new note.

- [ ] **Step 6: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
git add apps/rux/src/gui/survey.cpp apps/rux/include/gui/survey.hpp apps/rux/src/gui.cpp tests/unit/rux_gui/test_gui_survey.cpp docs/gui/openapi.yaml
git commit -m "feat(gui): register a sample at a bygningsdel, already taken" -m "POST /samples accepts part_code (its type is always linked) and an initial stage of planlagt or udtaget; GET /samples returns part_code. Every refusal is checked before the first write. rux gui --help documents phone access with --bind and --allow-origin." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 4: The varied demo seed gets On-site data

**Files:** Modify `apps/rux/frontend/dev/seed-survey-demo.sh`.

**Interfaces:** `--varied` now also stars RX-014 with a note and adds P-06, taken at RX-013. It needs schema v24. The plain seed is unchanged; it holds the prototype's numbers and is what the flow runs on.

- [ ] **Step 1: Edit.** Replace

```bash
# --varied adds P-04 (answered ren, linked to two approved types) and P-05 (planned, unlinked) for Miljø & prøver work, plus three report versions (two drafts, one complete) for Rapport.
```

with

```bash
# --varied adds P-04 (answered ren, linked to two approved types) and P-05 (planned, unlinked) for Miljø & prøver work, three report versions (two drafts, one complete) for Rapport, and — what On-site writes — a ★ part with a note (RX-014) and P-06 taken at RX-013.
```

Replace

```bash
[[ "$varied" -eq 0 || "$ver" -ge 23 ]] || { echo "schema v$ver < 23 — --varied needs a rux with report versions" >&2; exit 1; }
```

with

```bash
[[ "$varied" -eq 0 || "$ver" -ge 24 ]] || { echo "schema v$ver < 24 — --varied needs a rux with samples.part_code" >&2; exit 1; }
```

In the `--varied` SQL, replace

```sql
INSERT INTO sample_links (sample_id,type_id) VALUES (4,7),(4,10);
```

with

```sql
INSERT INTO sample_links (sample_id,type_id) VALUES (4,7),(4,10);
UPDATE survey_parts SET starred = 1, note = '8–10 stk. skønnes direkte genbrugelige.' WHERE code = 'RX-014';
INSERT INTO samples (id,code,title,what,stage,result,part_code) VALUES
 (6,'P-06','Asbest i linoleumslim','Prøve under vinduet mod gården','udtaget','','RX-013');
INSERT INTO sample_links (sample_id,type_id) VALUES (6,8);
```

- [ ] **Step 2: Run**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh --varied "$SP/corridor-clouds.rux" "$SP/p6-varied.rux"
sqlite3 "$SP/p6-varied.rux" "SELECT code, part_code, stage FROM samples WHERE id = 6; SELECT code, starred, note FROM survey_parts WHERE code = 'RX-014';"
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/p6-demo.rux"
sqlite3 "$SP/p6-demo.rux" "SELECT COUNT(*) FROM samples; SELECT MAX(version) FROM schema_version;"
```

Expected:
- `P-06|RX-013|udtaget` and `RX-014|1|8–10 stk. skønnes direkte genbrugelige.`;
- the plain seed prints `3` and `24`.

- [ ] **Step 3: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
git add apps/rux/frontend/dev/seed-survey-demo.sh
git commit -m "chore(gui): the varied demo seed carries On-site writes" -m "--varied now stars RX-014 with a note and adds P-06, taken at RX-013, so Kortlægning's star filter and Miljø's 'Udtaget ved' line have data. Needs schema v24." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 5: Frontend contract layer and test builders

**Files:**
- Modify `src/api/types.ts`.
- Modify `src/test/surveyFixtures.ts`, `src/test/kortlaegning.page.test.ts`, `src/test/kortlaegning.detailPanel.test.ts`, `src/test/survey.client.test.ts`.

**Interfaces** (in `src/api/types.ts`):

```ts
export interface Sample { /* … */ part_code: string | null }
export interface SampleCreate {
  title: string;
  what?: string;
  type_ids?: number[];
  part_code?: string;
  stage?: Extract<SampleStage, 'planlagt' | 'udtaget'>;
}
```

and in `src/test/surveyFixtures.ts`:

```ts
export function surveyPart(over?: Partial<SurveyPart>): SurveyPart;
```

- [ ] **Step 1: Failing test.** Append to the `describe('survey client', …)` block in `src/test/survey.client.test.ts`:

```ts
  it('registers a sample at a part, already taken', async () => {
    const { calls, api } = client({ id: 4, code: 'P-04' }, 201);
    await api.createSample({
      title: 'Asbest i fugemasse',
      what: 'Fuge mod nord',
      type_ids: [6],
      part_code: 'RX-008',
      stage: 'udtaget',
    });
    expect(calls[0]).toMatchObject({ url: '/api/v1/samples', method: 'POST' });
    expect(JSON.parse(calls[0].body!)).toEqual({
      title: 'Asbest i fugemasse',
      what: 'Fuge mod nord',
      type_ids: [6],
      part_code: 'RX-008',
      stage: 'udtaget',
    });
  });
```

Run `npm --prefix apps/rux/frontend run typecheck` → FAIL (`part_code` does not exist in type `SampleCreate`).

- [ ] **Step 2: Types.** In `src/api/types.ts`, in `interface Sample`, after `type_ids: number[];`, add:

```ts
  /** The bygningsdel the sample was taken at (On-site, schema v24); null otherwise. */
  part_code: string | null;
```

Replace

```ts
export interface SampleCreate {
  title: string;
  what?: string;
  type_ids?: number[];
}
```

with

```ts
export interface SampleCreate {
  title: string;
  what?: string;
  type_ids?: number[];
  /** The bygningsdel it is taken at; the server always adds that part's type to `type_ids`. */
  part_code?: string;
  /** `udtaget` when registered on site; later stages go through PATCH. The server's default is `planlagt`. */
  stage?: Extract<SampleStage, 'planlagt' | 'udtaget'>;
}
```

- [ ] **Step 3: Builders.** In `src/test/surveyFixtures.ts`:
  - in the type import list, add `SurveyPart,` after `SurveyFractions,`;
  - in `sample()`, after `type_ids: [],`, add `part_code: null,`;
  - after `surveyType`, add:

```ts
export function surveyPart(over: Partial<SurveyPart> = {}): SurveyPart {
  return {
    code: 'RX-008',
    type_id: 6,
    cloud: null,
    instance_id: null,
    room_id: 2,
    room_name: 'Office Zone',
    quantity: 26,
    starred: false,
    note: '',
    material_guid: null,
    instance_guid: null,
    orphaned: false,
    ...over,
  };
}
```

In `src/test/kortlaegning.page.test.ts` and in `src/test/kortlaegning.detailPanel.test.ts`, the local `sample()` builder: add `part_code: null,` after its `type_ids` line.

- [ ] **Step 4: Run → PASS**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend test
```

Expected: both pass. If `typecheck` names another object literal typed `Sample` that lacks `part_code`, add `part_code: null` to it; the compiler lists every one.

- [ ] **Step 5: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
git add apps/rux/frontend/src/api/types.ts apps/rux/frontend/src/test
git commit -m "feat(gui): contract types for a sample taken at a part" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 6: One app-wide write chain (R11)

**Files:**
- Modify `src/app/serialQueue.ts`, `src/app/useMutationQueue.ts`, `src/test/serialQueue.test.ts`.
- Create `src/app/writeChain.ts`.
- Modify `src/routes/KortlaegningPage.tsx`, `MiljoePage.tsx`, `OverblikPage.tsx`, `IndberetningPage.tsx`, `RapportPage.tsx`.

**Interfaces:**

```ts
export interface SerialQueue {
  enqueue(task: () => Promise<void>): Promise<void>;
  /** Settles once every task enqueued so far has settled. Never rejects. */
  idle(): Promise<void>;
}
export const appWriteChain: SerialQueue;           // src/app/writeChain.ts
export interface MutationQueueOptions {
  onError: (cause: unknown) => void;
  onSettled?: () => void;
  /** 'app' (default): the app-wide chain. 'page': this page's own chain. */
  scope?: 'app' | 'page';
}
```

- [ ] **Step 1: Failing test.** Append inside `describe('serial queue', …)` in `src/test/serialQueue.test.ts`:

```ts
  it('idle settles only after every task enqueued before it, failing ones included', async () => {
    const q = createSerialQueue();
    const log: string[] = [];
    const gate = deferred();
    void q.enqueue(async () => {
      await gate.promise;
      log.push('a');
    });
    void q
      .enqueue(async () => {
        log.push('b');
        throw new Error('boom');
      })
      .catch(() => log.push('b:caught'));
    const idle = q.idle().then(() => log.push('idle'));
    void q.enqueue(async () => {
      log.push('c'); // enqueued after idle(): idle does not wait for it
    });
    await Promise.resolve();
    expect(log).toEqual([]);
    gate.resolve();
    await idle;
    expect(log.slice(0, 3)).toEqual(['a', 'b', 'b:caught']);
    expect(log.indexOf('idle')).toBeGreaterThan(log.indexOf('b'));
    await expect(q.idle()).resolves.toBeUndefined();
    expect(log).toContain('c');
  });

  it('idle on an empty queue resolves at once', async () => {
    await expect(createSerialQueue().idle()).resolves.toBeUndefined();
  });
```

Run `npm --prefix apps/rux/frontend test -- serialQueue` → FAIL (`q.idle is not a function`).

- [ ] **Step 2: `idle`.** In `src/app/serialQueue.ts`, replace

```ts
export interface SerialQueue {
  enqueue(task: () => Promise<void>): Promise<void>;
}
```

with

```ts
export interface SerialQueue {
  enqueue(task: () => Promise<void>): Promise<void>;
  /**
   * Settles once every task enqueued so far has settled, failed ones
   * included. Never rejects. Tasks enqueued later are not waited for.
   */
  idle(): Promise<void>;
}
```

and in `createSerialQueue`, after the `enqueue(task) { … },` member, add:

```ts
    idle() {
      return tail;
    },
```

- [ ] **Step 3: The chain.** Create `src/app/writeChain.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The app's one write chain (GUI Phase 6, R11). Every screen's writes join
 * it through `useMutationQueue`, and the case screens start their first load
 * with `appWriteChain.idle()`. A write queued on the screen just left — a
 * sample registered on the phone sheet, a result recorded in Miljø & prøver —
 * therefore lands before the next screen reads the survey, so that screen
 * never shows the approval gate as it was before the write.
 *
 * Rapport opts out (`scope: 'page'`): report generation can take minutes,
 * changes no survey state, and must not hold up other screens' first loads.
 */

import { createSerialQueue, type SerialQueue } from './serialQueue';

export const appWriteChain: SerialQueue = createSerialQueue();
```

In `src/app/useMutationQueue.ts`:
  - add `import { appWriteChain } from './writeChain';` after the `serialQueue` import;
  - in `MutationQueueOptions`, after `onSettled?: () => void;`, add:

```ts
  /**
   * Which chain the writes join: 'app' (the default), the app-wide chain the
   * case screens' first loads wait for (R11), or 'page', a chain of this page's
   * own for writes that change no survey state and may run long (Rapport).
   */
  scope?: 'app' | 'page';
```

  - replace `if (queueRef.current === null) queueRef.current = createSerialQueue();` with:

```ts
  if (queueRef.current === null) {
    queueRef.current = options.scope === 'page' ? createSerialQueue() : appWriteChain;
  }
```

  - replace the doc sentence `A page's writes on one serial chain (\`createSerialQueue\`). Nothing is` with `A page's writes on one serial chain — the app-wide \`appWriteChain\` unless \`scope: 'page'\`. Nothing is`.

- [ ] **Step 4: First loads wait for the chain.** In each file, add `import { appWriteChain } from '../app/writeChain';` next to the `useAsync` import, then:
  - `src/routes/KortlaegningPage.tsx`: replace `(s) => Promise.all([api.survey(s), api.samples(s), api.surveySummary(s)]),` with `(s) => appWriteChain.idle().then(() => Promise.all([api.survey(s), api.samples(s), api.surveySummary(s)])),`.
  - `src/routes/MiljoePage.tsx`: replace `useAsync((s) => Promise.all([api.samples(s), api.survey(s)]), [])` with `useAsync((s) => appWriteChain.idle().then(() => Promise.all([api.samples(s), api.survey(s)])), [])`.
  - `src/routes/OverblikPage.tsx`: replace `(s) => Promise.all([api.projectSummary(s), api.surveySummary(s)]),` with `(s) => appWriteChain.idle().then(() => Promise.all([api.projectSummary(s), api.surveySummary(s)])),`, and `useAsync((s) => api.surveyFractions(s), [])` with `useAsync((s) => appWriteChain.idle().then(() => api.surveyFractions(s)), [])`.
  - `src/routes/IndberetningPage.tsx`: replace `(s) => Promise.all([api.surveyFractions(s), api.surveySummary(s)]),` with `(s) => appWriteChain.idle().then(() => Promise.all([api.surveyFractions(s), api.surveySummary(s)])),`.
  - `src/routes/RapportPage.tsx`: do not touch its loads. Replace `useMutationQueue({ onError: (cause) => toast.show(generateErrorMessage(cause)) })` with `useMutationQueue({ scope: 'page', onError: (cause) => toast.show(generateErrorMessage(cause)) })`.

- [ ] **Step 5: Run → PASS**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
grep -rn "scope: 'page'" apps/rux/frontend/src --include=*.tsx
```

Expected: tests and typecheck pass; the grep prints only `RapportPage.tsx`.

- [ ] **Step 6: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
git add apps/rux/frontend/src/app/serialQueue.ts apps/rux/frontend/src/app/writeChain.ts apps/rux/frontend/src/app/useMutationQueue.ts apps/rux/frontend/src/test/serialQueue.test.ts apps/rux/frontend/src/routes/KortlaegningPage.tsx apps/rux/frontend/src/routes/MiljoePage.tsx apps/rux/frontend/src/routes/OverblikPage.tsx apps/rux/frontend/src/routes/IndberetningPage.tsx apps/rux/frontend/src/routes/RapportPage.tsx
git commit -m "fix(gui): a screen's first read waits for writes queued on the one before" -m "Every page's writes now join one app-wide serial chain, and the case screens start their first load with appWriteChain.idle(). A sample result recorded in Miljø & prøver (or a sample registered on the phone sheet) can no longer land after Kortlægning's GET /survey and leave it showing a stale gate. Rapport keeps its own chain: generation runs long and changes no survey state." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 7: One cross-link style (R12)

**Files:** Modify `src/components/controls.module.css`, `src/components/kortlaegning/SampleLine.module.css`, `src/components/miljoe/SampleCard.module.css`.

**Interfaces:** `controls.module.css` exports `crossLink`. Every link that opens another case screen composes it.

- [ ] **Step 1: The class.** Append to `src/components/controls.module.css`:

```css
/* A link to another case screen (Kortlægning ↔ Miljø & prøver ↔ On-site,
 * Sager → On-site). One style everywhere, so "this opens the other screen"
 * reads the same on every screen (Phase 6 R12). */

.crossLink {
  color: var(--color-accent-deep);
  text-decoration: underline;
}

.crossLink:hover {
  color: var(--color-text);
}

.crossLink:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: 2px;
  border-radius: var(--radius-sm);
}
```

- [ ] **Step 2: Compose it.** In `src/components/kortlaegning/SampleLine.module.css`, replace the three rules `.link { … }`, `.link:hover { … }` and `.link:focus-visible { … }` with:

```css
.link {
  composes: crossLink from '../controls.module.css';
}
```

In `src/components/miljoe/SampleCard.module.css`, replace the three rules `.typeLink { … }`, `.typeLink:hover { … }` and `.typeLink:focus-visible { … }` with:

```css
.typeLink {
  composes: crossLink from '../controls.module.css';
}
```

- [ ] **Step 3: Check.** Start the servers on the plain seed and look at both links:

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/controls.module.css apps/rux/frontend/src/components/kortlaegning/SampleLine.module.css apps/rux/frontend/src/components/miljoe/SampleCard.module.css --tsx
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/p6-demo.rux" 8426 5179
bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5179/kortlaegning?type=6" --out "$SP/shots/p6-crosslink" --theme light --viewports desktop
bash .claude/skills/design-studio/scripts/shot.sh "http://localhost:5179/miljoe?sample=1" --out "$SP/shots/p6-crosslink" --theme dark --viewports desktop
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

Expected: the build and lint pass. Open both PNGs with Read: Kortlægning's `P-01 · PCB i fugemasse` and Miljø's `Koblet: Vinduespartier, aluminium` are both accent-deep and underlined. (These are quick visual checks of an unchanged layout. The asserted shots come from Task 16's script.)

- [ ] **Step 4: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
git add apps/rux/frontend/src/components/controls.module.css apps/rux/frontend/src/components/kortlaegning/SampleLine.module.css apps/rux/frontend/src/components/miljoe/SampleCard.module.css
git commit -m "fix(gui): one style for links between case screens" -m "Kortlægning's sample line and Miljø & prøver's 'Koblet:' links now compose the same crossLink rule (accent-deep, underlined), which Phase 6's new links use too." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 8: The shell becomes responsive (R5)

**Files:**
- Modify `src/app/AppShell.tsx`, `src/app/AppShell.module.css`.
- Modify `src/components/TitleBar.tsx`, `src/components/TitleBar.module.css`.
- Modify `src/components/Sidebar.tsx`, `src/components/Sidebar.module.css`.
- Modify `src/app/navigation.ts`, `src/test/navigation.test.ts`.

**Interfaces:**

```ts
// TitleBar
menuOpen?: boolean;                          // the drawer's state, for aria-expanded
onMenu?: () => void;                         // draws the ☰ button (shown below 900px only)
menuRef?: RefObject<HTMLButtonElement | null>;
menuControls?: string;                       // id of the drawer
// Sidebar
id?: string;
open?: boolean;                              // drawer open (below 900px)
onClose?: () => void;                        // Esc inside the drawer
navRef?: RefObject<HTMLElement | null>;
```

`ALL_CASES_PENDING` is deleted; `← Alle sager` is a `NavLink` to `/sager`. The route itself arrives in Task 10. Until then the link falls through to the catch-all redirect to `/`, which leaves the app working.

- [ ] **Step 1: Failing test.** In `src/test/navigation.test.ts`:
  - remove `ALL_CASES_PENDING,` from the named import list, and add `import * as navigation from '../app/navigation';` below it;
  - replace the test

```ts
  it('marks "Alle sager" pending until the case list exists', () => {
    expect(ALL_CASES_PENDING).toMatch(/fase 6/);
  });
```

    with

```ts
  it('makes "Alle sager" a live link: nothing is pending any more', () => {
    expect('ALL_CASES_PENDING' in navigation).toBe(false);
    expect(NAV_ENTRIES.filter((e) => e.pending)).toEqual([]);
  });
```

  - run `npm --prefix apps/rux/frontend test -- navigation` → FAIL (`ALL_CASES_PENDING` is still exported).

- [ ] **Step 2: `navigation.ts`.** Delete

```ts
/** Set to undefined once the `/sager` route lands; until then "Alle sager" renders inert. */
export const ALL_CASES_PENDING: string | undefined = 'Kommer i fase 6 — sagsliste';
```

- [ ] **Step 3: Sidebar.** Replace `src/components/Sidebar.tsx` with:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { RefObject } from 'react';
import { NavLink } from 'react-router-dom';

import { ALL_CASES_PATH, badgeText, entriesIn, type NavBadge, type NavEntry } from '../app/navigation';
import styles from './Sidebar.module.css';

export interface SidebarProps {
  /** Shown under the PROJEKT eyebrow; a placeholder dash while loading. */
  projectName?: string;
  /** Live counts for entries that carry a badge; absent or 0 hides it. */
  badges?: Partial<Record<NavBadge, number>>;
  /** The drawer's id, for the title bar's `aria-controls`. */
  id?: string;
  /** Below 900px the sidebar is a drawer (Phase 6 R5); this opens it. */
  open?: boolean;
  /** Esc inside the open drawer. */
  onClose?: () => void;
  /** The drawer takes focus when it opens. */
  navRef?: RefObject<HTMLElement | null>;
}

function Entry({ entry, count }: { entry: NavEntry; count?: number }) {
  const badge = badgeText(count);
  const inner = (
    <>
      <span className={styles.dot} aria-hidden="true" />
      <span className={styles.label}>{entry.label}</span>
      {badge && (
        <span className={`${styles.count} ${entry.badge === 'reviewQueue' ? styles.hot : ''}`}>{badge}</span>
      )}
    </>
  );
  if (entry.pending) {
    return (
      <span className={`${styles.item} ${styles.pending}`} title={entry.pending} aria-disabled="true">
        {inner}
      </span>
    );
  }
  return (
    <NavLink
      to={entry.to}
      end={entry.end}
      className={({ isActive }) => `${styles.item} ${isActive ? styles.active : ''}`}
    >
      {inner}
    </NavLink>
  );
}

/**
 * The navy case sidebar: which project, the case workflow, the technical tools,
 * and the way back to the case list. Below 900px it is a drawer the title
 * bar's Menu button opens (R5); closed, CSS hides it from the tab order.
 */
export function Sidebar({ projectName, badges = {}, id, open = false, onClose, navRef }: SidebarProps) {
  return (
    <aside
      id={id}
      ref={navRef}
      className={styles.sidebar}
      data-open={open || undefined}
      tabIndex={-1}
      aria-label="Navigation"
      onKeyDown={(e) => {
        if (open && e.key === 'Escape') {
          e.preventDefault();
          onClose?.();
        }
      }}
    >
      <div className={styles.eyebrow}>Projekt</div>
      <div className={styles.project} title={projectName}>
        {projectName ?? '—'}
      </div>
      <nav className={styles.nav} aria-label="Sag">
        {entriesIn('sag').map((e) => (
          <Entry key={e.to} entry={e} count={e.badge ? badges[e.badge] : undefined} />
        ))}
      </nav>
      <div className={styles.groupLabel}>Værktøjer</div>
      <nav className={styles.nav} aria-label="Værktøjer">
        {entriesIn('tools').map((e) => (
          <Entry key={e.to} entry={e} />
        ))}
      </nav>
      <div className={styles.back}>
        <NavLink to={ALL_CASES_PATH} className={({ isActive }) => (isActive ? styles.backActive : undefined)}>
          ← Alle sager
        </NavLink>
      </div>
    </aside>
  );
}
```

In `src/components/Sidebar.module.css`, replace the `.backPending { … }` rule with:

```css
.back .backActive {
  color: var(--color-on-chrome);
}

.sidebar:focus {
  outline: none; /* a keyboard anchor while the drawer is open, not a control */
}

/* R5: the shell's phone breakpoint, the prototype's own `.sag` query. Below it
   the sidebar is a drawer; closed, it is visibility:hidden, so its links leave
   the tab order. */
@media (max-width: 900px) {
  .sidebar {
    position: fixed;
    top: var(--layout-titlebar-height);
    bottom: 0;
    left: 0;
    z-index: var(--z-titlebar);
    box-shadow: var(--shadow-lg);
    transform: translateX(-100%);
    visibility: hidden;
    transition:
      transform var(--duration-normal) var(--easing-standard),
      visibility var(--duration-normal);
  }

  .sidebar[data-open] {
    transform: none;
    visibility: visible;
  }
}

@media (prefers-reduced-motion: reduce) {
  .sidebar {
    transition: none;
  }
}
```

- [ ] **Step 4: Title bar.** In `src/components/TitleBar.tsx`:
  - replace `import type { ConnectionStatus } from '../api/events';` with

```tsx
import type { RefObject } from 'react';

import type { ConnectionStatus } from '../api/events';
```

  - in `TitleBarProps`, after `unreachable?: boolean;`, add:

```tsx
  /** Below 900px: the sidebar drawer is open (R5). */
  menuOpen?: boolean;
  /** Below 900px: toggles the sidebar drawer. Without it no Menu button is drawn. */
  onMenu?: () => void;
  menuRef?: RefObject<HTMLButtonElement | null>;
  /** The drawer's id. */
  menuControls?: string;
```

  - add `menuOpen = false, onMenu, menuRef, menuControls,` to the destructured parameters after `unreachable = false,`;
  - directly inside `<div className={styles.identity}>`, before the product span, add:

```tsx
        {onMenu && (
          <button
            ref={menuRef}
            type="button"
            className={styles.menu}
            aria-label="Menu"
            aria-expanded={menuOpen}
            aria-controls={menuControls}
            onClick={onMenu}
          >
            ☰
          </button>
        )}
```

Append to `src/components/TitleBar.module.css`:

```css
.menu {
  display: none;
  align-items: center;
  justify-content: center;
  flex: none;
  width: var(--space-6);
  height: var(--space-6);
  padding: 0;
  border: 1px solid var(--color-chrome-border);
  border-radius: var(--radius-md);
  background: var(--color-chrome-raised);
  color: var(--color-on-chrome);
  font-size: var(--font-size-lg);
  cursor: pointer;
}

.menu:focus-visible {
  outline: 2px solid var(--color-on-chrome);
  outline-offset: 2px;
}

/* R5: the shell's phone breakpoint, the prototype's own `.sag` query. Below it
   the bar keeps product, project, job state and theme, and drops the rest. */
@media (max-width: 900px) {
  .bar {
    gap: var(--space-2);
    padding: 0 var(--space-3);
  }

  .identity {
    flex: 1 1 auto;
    gap: var(--space-2);
  }

  .menu {
    display: inline-flex;
  }

  .divider,
  .meta {
    display: none;
  }
}
```

- [ ] **Step 5: AppShell.** Replace `src/app/AppShell.tsx` with:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useRef, useState, type ReactNode } from 'react';
import { useLocation } from 'react-router-dom';

import { api } from '../api/client';
import type { Health, ProjectSummary, SurveySummary } from '../api/types';
import { TitleBar } from '../components/TitleBar';
import { Sidebar } from '../components/Sidebar';
import { JobToaster } from '../components/JobToaster';
import { useAsync } from './useAsync';
import { useJobs } from './JobsContext';
import { SurveyCountsProvider } from './SurveyCountsContext';
import { displayProjectName } from './navigation';
import styles from './AppShell.module.css';

const NAV_ID = 'app-nav';

/**
 * Title bar + sidebar + content region.
 *
 * The shell resolves the project identity once, from `GET /health`, rather than
 * from `/project`: health is the cheap call, it is the one the contract
 * designates for the version handshake, and it reports `project.open === false`
 * when the database could not be opened — which is exactly the state a title
 * bar must not render as if everything were fine.
 *
 * The Kortlægning and Miljø & prøver badges both come from
 * `GET /survey/summary` (`counts.queue`, `pending_samples`: samples not yet at
 * *svar*, i.e. those that can hold a type at *afventer prøve*). A server that
 * predates the survey routes answers 404; the shell then shows no badge rather
 * than an error — the badge is a hint, not something to block the app on.
 *
 * Below 900px (Phase 6 R5) the sidebar is a drawer behind the title bar's
 * Menu button, over a scrim. It closes on any navigation, on a scrim click and
 * on Esc inside it; Esc hands focus back to Menu, so it never drops to <body>.
 */
export function AppShell({ children }: { children: ReactNode }) {
  const { data: health, error } = useAsync<Health>((signal) => api.health(signal), []);
  const project = useAsync<ProjectSummary>((signal) => api.projectSummary(signal), []);
  const summary = project.data;
  const survey = useAsync<SurveySummary>((signal) => api.surveySummary(signal), []);
  const { active, status } = useJobs();
  const reviewQueue = survey.error ? undefined : survey.data?.counts.queue;
  const pendingSamples = survey.error ? undefined : survey.data?.pending_samples;
  const surveyCounts = useMemo(
    () => ({ refresh: survey.reload, refreshProject: project.reload }),
    [survey.reload, project.reload],
  );

  const [navOpen, setNavOpen] = useState(false);
  const menuRef = useRef<HTMLButtonElement>(null);
  const navRef = useRef<HTMLElement>(null);
  const location = useLocation();
  useEffect(() => {
    setNavOpen(false); // any navigation closes the drawer
  }, [location.pathname, location.search]);
  useEffect(() => {
    if (navOpen) navRef.current?.focus();
  }, [navOpen]);
  const closeNavToMenu = useCallback(() => {
    setNavOpen(false);
    menuRef.current?.focus();
  }, []);

  return (
    <div className={styles.shell}>
      <TitleBar
        projectName={health?.project.name}
        projectOpen={health?.project.open}
        schemaVersion={health?.project.schema_version}
        version={health?.version}
        implementation={health?.implementation}
        connection={status}
        activeJobCount={active.length}
        unreachable={Boolean(error)}
        menuOpen={navOpen}
        onMenu={() => setNavOpen((open) => !open)}
        menuRef={menuRef}
        menuControls={NAV_ID}
      />
      <div className={styles.body}>
        <Sidebar
          id={NAV_ID}
          open={navOpen}
          onClose={closeNavToMenu}
          navRef={navRef}
          projectName={displayProjectName(summary, health)}
          badges={{ reviewQueue, pendingSamples }}
        />
        <div className={styles.scrim} hidden={!navOpen} aria-hidden="true" onClick={() => setNavOpen(false)} />
        <main className={styles.content}>
          <SurveyCountsProvider value={surveyCounts}>{children}</SurveyCountsProvider>
        </main>
      </div>
      <JobToaster />
    </div>
  );
}
```

Append to `src/app/AppShell.module.css`:

```css
.scrim {
  display: none;
}

/* R5: the shell's phone breakpoint, the prototype's own `.sag` query. */
@media (max-width: 900px) {
  .scrim:not([hidden]) {
    display: block;
    position: fixed;
    inset: var(--layout-titlebar-height) 0 0 0;
    z-index: var(--z-panel);
    background: color-mix(in srgb, var(--color-scrim) 45%, transparent);
  }
}
```

- [ ] **Step 6: Check**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/app/AppShell.module.css apps/rux/frontend/src/app/AppShell.tsx apps/rux/frontend/src/components/TitleBar.module.css apps/rux/frontend/src/components/TitleBar.tsx apps/rux/frontend/src/components/Sidebar.module.css apps/rux/frontend/src/components/Sidebar.tsx --tsx
grep -rn ALL_CASES_PENDING apps/rux/frontend/src
```

Expected: all pass; the grep prints nothing. The drawer itself is asserted in Task 16 (it needs a browser).

- [ ] **Step 7: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
git add apps/rux/frontend/src/app/AppShell.tsx apps/rux/frontend/src/app/AppShell.module.css apps/rux/frontend/src/components/TitleBar.tsx apps/rux/frontend/src/components/TitleBar.module.css apps/rux/frontend/src/components/Sidebar.tsx apps/rux/frontend/src/components/Sidebar.module.css apps/rux/frontend/src/app/navigation.ts apps/rux/frontend/src/test/navigation.test.ts
git commit -m "fix(gui): the shell fits a phone — the sidebar becomes a drawer below 900px" -m "Below 900px the sidebar is an off-canvas drawer behind a Menu button, over a scrim; it closes on navigation, on a scrim click and on Esc (focus back to Menu), and is hidden from the tab order while closed. The title bar drops its meta there. Above 900px nothing changes. '← Alle sager' is now a live link." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 9: Sager model (R1–R3, R10)

**Files:** Create `src/sager/model.ts` and `src/test/sager.model.test.ts`.

**Interfaces:**

```ts
export interface CaseStatus { label: string; tone: Tone }
export function caseStatus(s: SurveySummary, f: SurveyFractions | null | undefined): CaseStatus;
export interface CaseStat { key: string; value: string; label: string }
export function caseStats(s: SurveySummary): CaseStat[] | null;   // null → NO_SURVEY_TEXT
export const NO_SURVEY_TEXT: string;                            // 'Ingen kortlægning endnu'
export function cardSubline(p: ProjectInfo | undefined): string;
export function cardDate(p: ProjectInfo | undefined): string;   // 'Registreret 09.08.2026' | '—'
export const OPEN_ANOTHER_COMMAND: string;                      // 'rux -p <fil>.rux gui'
export function phoneCommand(file: string, port: string): string;
export const NO_AUTH_WARNING: string;
```

- [ ] **Step 1: Failing tests** — `src/test/sager.model.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  cardDate,
  cardSubline,
  caseStats,
  caseStatus,
  NO_AUTH_WARNING,
  OPEN_ANOTHER_COMMAND,
  phoneCommand,
} from '../sager/model';
import { surveyFractions, surveySummary } from './surveyFixtures';

describe('case status (R3)', () => {
  it('is Kladde with no survey types at all', () => {
    const empty = surveySummary({ counts: { queue: 0, approved: 0, rejected: 0, all: 0 } });
    expect(caseStatus(empty, surveyFractions())).toEqual({ label: 'Kladde', tone: 'wait' });
  });

  it('is not Kladde when only rejected types exist', () => {
    const rejected = surveySummary({ counts: { queue: 0, approved: 0, rejected: 2, all: 0 } });
    const f = surveyFractions({ fractions: [], blocking: [], blocking_types: 0, ready: false });
    expect(caseStatus(rejected, f)).toEqual({ label: 'Gennemgået', tone: 'good' });
  });

  it('is Gennemgang while anything is queued or blocks', () => {
    expect(caseStatus(surveySummary(), surveyFractions())).toEqual({ label: 'Gennemgang', tone: 'accent' });
    const reviewed = surveySummary({ counts: { queue: 0, approved: 11, rejected: 0, all: 11 } });
    expect(caseStatus(reviewed, surveyFractions({ blocking_types: 2 })).label).toBe('Gennemgang');
  });

  it('never claims done while the fractions are unknown', () => {
    const reviewed = surveySummary({ counts: { queue: 0, approved: 11, rejected: 0, all: 11 } });
    expect(caseStatus(reviewed, undefined).label).toBe('Gennemgang');
    expect(caseStatus(reviewed, null).label).toBe('Gennemgang');
  });

  it('is ready when Indberetning is', () => {
    const reviewed = surveySummary({ counts: { queue: 0, approved: 11, rejected: 0, all: 11 } });
    const ready = surveyFractions({ blocking: [], blocking_types: 0, ready: true });
    expect(caseStatus(reviewed, ready)).toEqual({ label: 'Klar til indberetning', tone: 'good' });
  });
});

describe('card stats', () => {
  it('reads the summary: components, reuse share, queue', () => {
    expect(caseStats(surveySummary())).toEqual([
      { key: 'types', value: '11', label: 'komponenter' },
      { key: 'reuse', value: '54 %', label: 'bevaring/genbrug' },
      { key: 'queue', value: '7', label: 'til gennemsyn' },
    ]);
  });

  it('shows a dash for an unknown reuse share and nothing for an empty survey', () => {
    expect(caseStats(surveySummary({ reuse_share: null }))?.[1].value).toBe('—');
    expect(caseStats(surveySummary({ counts: { queue: 0, approved: 0, rejected: 0, all: 0 } }))).toBeNull();
  });
});

describe('card text', () => {
  it('joins address and organisation, else says none is registered', () => {
    const p = { id: 'p', name: 'Måløv Byvej 229' };
    expect(cardSubline({ ...p, building_address: 'Måløv Byvej 229, 2760 Måløv', survey_organisation: 'Link Arkitektur' })).toBe(
      'Måløv Byvej 229, 2760 Måløv · udarbejdet af Link Arkitektur',
    );
    expect(cardSubline({ ...p, building_address: '  ' })).toBe('Ingen adresse registreret');
    expect(cardSubline(undefined)).toBe('Ingen adresse registreret');
  });

  it('dates the card by the registration date, in Danish form', () => {
    expect(cardDate({ id: 'p', name: 'x', survey_date: '2026-08-09' })).toBe('Registreret 09.08.2026');
    expect(cardDate({ id: 'p', name: 'x' })).toBe('—');
  });
});

describe('commands (R1, R10)', () => {
  it('says how to open another case and how to reach this one from a phone', () => {
    expect(OPEN_ANOTHER_COMMAND).toBe('rux -p <fil>.rux gui');
    expect(phoneCommand('maaloev.rux', '8426')).toBe(
      'rux -p maaloev.rux gui --bind <din-ip> --allow-origin http://<din-ip>:8426',
    );
    expect(phoneCommand('maaloev.rux', '')).toBe(
      'rux -p maaloev.rux gui --bind <din-ip> --allow-origin http://<din-ip>:8420',
    );
    expect(NO_AUTH_WARNING).toMatch(/ingen adgangskontrol/);
  });
});
```

Run `npm --prefix apps/rux/frontend test -- sager` → FAIL (module not found).

- [ ] **Step 2: Implement** — `src/sager/model.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Sager as data: the one card `rux gui` can show (R1) and the commands that
 * open another case or reach this one from a phone (R10). Every figure is
 * read off a server response; this module only words it.
 */

import type { ProjectInfo, SurveyFractions, SurveySummary } from '../api/types';
import type { Tone } from '../kortlaegning/vocab';
import { danishDate, percentText } from '../overblik/model';

export interface CaseStatus {
  label: string;
  tone: Tone;
}

/**
 * The card's status pill (R3), a wording of server numbers. `f` is
 * `GET /survey/fractions`: `undefined` while loading, `null` when it failed.
 * Without it nothing can be called done, so it reads Gennemgang.
 */
export function caseStatus(s: SurveySummary, f: SurveyFractions | null | undefined): CaseStatus {
  if (s.counts.all === 0 && s.counts.rejected === 0) return { label: 'Kladde', tone: 'wait' };
  if (s.counts.queue > 0 || !f || f.blocking_types > 0) return { label: 'Gennemgang', tone: 'accent' };
  if (f.ready) return { label: 'Klar til indberetning', tone: 'good' };
  return { label: 'Gennemgået', tone: 'good' };
}

export interface CaseStat {
  key: string;
  value: string;
  label: string;
}

/** What the stats line says for a project with no survey types. */
export const NO_SURVEY_TEXT = 'Ingen kortlægning endnu';

/** The card's stats line (R2), or null when the project has no survey yet. */
export function caseStats(s: SurveySummary): CaseStat[] | null {
  if (s.counts.all === 0) return null;
  return [
    { key: 'types', value: String(s.counts.all), label: 'komponenter' },
    { key: 'reuse', value: s.reuse_share === null ? '—' : `${percentText(s.reuse_share)} %`, label: 'bevaring/genbrug' },
    { key: 'queue', value: String(s.counts.queue), label: 'til gennemsyn' },
  ];
}

/** Address and organisation, whichever the record has (no bygherre is stored). */
export function cardSubline(p: ProjectInfo | undefined): string {
  const parts: string[] = [];
  const address = p?.building_address?.trim();
  if (address) parts.push(address);
  const org = p?.survey_organisation?.trim();
  if (org) parts.push(`udarbejdet af ${org}`);
  return parts.length > 0 ? parts.join(' · ') : 'Ingen adresse registreret';
}

/** The foot's date: the registration date in place of the prototype's Frist (not stored). */
export function cardDate(p: ProjectInfo | undefined): string {
  const date = p?.survey_date?.trim();
  return date ? `Registreret ${danishDate(date)}` : '—';
}

/** How to open another case: `rux gui` serves the project it was started with. */
export const OPEN_ANOTHER_COMMAND = 'rux -p <fil>.rux gui';

/**
 * How to reach this case from a phone on the same network (R10): bind the LAN
 * address and allow the page's own origin. `port` is the page's own port;
 * empty means the default.
 */
export function phoneCommand(file: string, port: string): string {
  const p = port || '8420';
  return `rux -p ${file} gui --bind <din-ip> --allow-origin http://<din-ip>:${p}`;
}

export const NO_AUTH_WARNING = 'Serveren har ingen adgangskontrol — gør det kun på et netværk, du stoler på.';
```

- [ ] **Step 3: Run → PASS**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
npm --prefix apps/rux/frontend test -- sager
npm --prefix apps/rux/frontend run typecheck
```

- [ ] **Step 4: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
git add apps/rux/frontend/src/sager/model.ts apps/rux/frontend/src/test/sager.model.test.ts
git commit -m "feat(gui): Sager model — case status, card text and the open/phone commands" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 10: Sager at `/sager` (R1, R2, R4, R10)

**Files:**
- Create `src/components/sager/CaseCard.tsx` + `.module.css`, `src/routes/SagerPage.tsx` + `.module.css`.
- Modify `src/app/App.tsx`, `src/app/links.ts`, `src/test/links.test.ts`.

**Interfaces:**

```ts
// links.ts
export const ONSITE_PATH = '/on-site';
export function onsiteHref(code: string): string;          // '/on-site?del=RX-008'
export function parseOnsiteQuery(search: string): string | null;   // '?del=RX-008' → 'RX-008'
// CaseCard
export interface CaseCardProps {
  to: string; name: string; subline: string;
  stats: CaseStat[] | null; status: CaseStatus; date: string; thumbUrl: string;
}
```

- [ ] **Step 1: Failing test.** Append to `src/test/links.test.ts` (add `onsiteHref, parseOnsiteQuery, ONSITE_PATH` to its import from `../app/links`):

```ts
describe('on-site links', () => {
  it('builds and reads ?del=<part code>', () => {
    expect(ONSITE_PATH).toBe('/on-site');
    expect(onsiteHref('RX-008')).toBe('/on-site?del=RX-008');
    expect(parseOnsiteQuery('?del=RX-008')).toBe('RX-008');
    expect(parseOnsiteQuery('?del=RX-008&x=1')).toBe('RX-008');
  });

  it('ignores anything that is not a part code', () => {
    expect(parseOnsiteQuery('')).toBeNull();
    expect(parseOnsiteQuery('?del=')).toBeNull();
    expect(parseOnsiteQuery('?del=rx-008')).toBeNull();
    expect(parseOnsiteQuery('?del=RX-8a')).toBeNull();
    expect(parseOnsiteQuery('?del=%3Cscript%3E')).toBeNull();
  });
});
```

Run `npm --prefix apps/rux/frontend test -- links` → FAIL.

- [ ] **Step 2: Links.** Append to `src/app/links.ts`:

```ts
export const ONSITE_PATH = '/on-site';

/** On-site at one bygningsdel. */
export function onsiteHref(code: string): string {
  return `${ONSITE_PATH}?del=${encodeURIComponent(code)}`;
}

/** `?del=<RX-###>` on /on-site; anything that is not a part code is ignored. */
export function parseOnsiteQuery(search: string): string | null {
  const v = new URLSearchParams(search).get('del');
  return v !== null && /^RX-\d+$/.test(v) ? v : null;
}
```

Run `npm --prefix apps/rux/frontend test -- links` → PASS.

- [ ] **Step 3: Card.** Create `src/components/sager/CaseCard.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState } from 'react';
import { Link } from 'react-router-dom';

import type { CaseStat, CaseStatus } from '../../sager/model';
import { NO_SURVEY_TEXT } from '../../sager/model';
import { Pill } from '../Pill';
import styles from './CaseCard.module.css';

export interface CaseCardProps {
  to: string;
  name: string;
  subline: string;
  stats: CaseStat[] | null;
  status: CaseStatus;
  date: string;
  /** A server-rendered plan; on failure the striped thumb shows the status (R2). */
  thumbUrl: string;
}

/** One case on Sager: the prototype's card, linking to the case's Overblik. */
export function CaseCard({ to, name, subline, stats, status, date, thumbUrl }: CaseCardProps) {
  const [thumbFailed, setThumbFailed] = useState(false);
  return (
    <Link to={to} className={styles.card}>
      <div className={styles.thumb} data-plain={thumbFailed || undefined}>
        {thumbFailed ? (
          <span className={styles.thumbLabel}>{status.label}</span>
        ) : (
          <img
            className={styles.thumbImg}
            src={thumbUrl}
            alt="Plan af punktskyen"
            loading="lazy"
            onError={() => setThumbFailed(true)}
          />
        )}
      </div>
      <div className={styles.body}>
        <h3 className={styles.name}>{name}</h3>
        <p className={styles.addr}>{subline}</p>
        <p className={styles.stats}>
          {stats
            ? stats.map((s) => (
                <span key={s.key}>
                  <b className={styles.figure}>{s.value}</b> {s.label}
                </span>
              ))
            : NO_SURVEY_TEXT}
        </p>
        <div className={styles.foot}>
          <Pill tone={status.tone}>{status.label}</Pill>
          <span className={styles.date}>{date}</span>
        </div>
      </div>
    </Link>
  );
}
```

Create `src/components/sager/CaseCard.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.card {
  composes: panel from '../surfaces.module.css';
  display: flex;
  flex-direction: column;
  overflow: hidden;
  color: var(--color-text);
  text-decoration: none;
}

.card:hover {
  border-color: var(--color-accent);
}

.card:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: 2px;
}

/* The prototype's 6.2rem thumb: a plan render, or the striped placeholder. */
.thumb {
  position: relative;
  height: calc(var(--space-7) * 2 + var(--space-1));
  background: var(--color-canvas);
}

.thumb[data-plain] {
  background: repeating-linear-gradient(
    0deg,
    var(--color-surface-sunken) 0 calc(var(--space-3) + var(--space-1) - 1px),
    var(--color-border) calc(var(--space-3) + var(--space-1) - 1px) calc(var(--space-3) + var(--space-1))
  );
}

.thumbImg {
  display: block;
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.thumbLabel {
  position: absolute;
  inset: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  font-family: var(--font-display);
  font-size: var(--font-size-xs);
  letter-spacing: var(--tracking-wide);
  text-transform: uppercase;
  color: var(--color-text-faint);
}

.body {
  display: grid;
  gap: var(--space-2);
  padding: var(--space-3) var(--space-3) var(--space-4);
}

.name {
  margin: 0;
  font-family: var(--font-display);
  font-size: var(--font-size-lg);
  font-weight: var(--font-weight-bold);
  text-transform: uppercase;
}

.addr {
  margin: 0;
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
}

.stats {
  display: flex;
  flex-wrap: wrap;
  gap: var(--space-3);
  margin: 0;
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
}

.figure {
  color: var(--color-text);
  font-variant-numeric: tabular-nums;
}

.foot {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: var(--space-2);
  font-size: var(--font-size-xs);
  color: var(--color-text-faint);
}

.date {
  font-variant-numeric: tabular-nums;
}
```

- [ ] **Step 4: Page.** Create `src/routes/SagerPage.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link } from 'react-router-dom';

import { api } from '../api/client';
import { ONSITE_PATH, OVERBLIK_PATH } from '../app/links';
import { useAsync } from '../app/useAsync';
import { appWriteChain } from '../app/writeChain';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { CaseCard } from '../components/sager/CaseCard';
import { caseName } from '../overblik/model';
import {
  cardDate,
  cardSubline,
  caseStats,
  caseStatus,
  NO_AUTH_WARNING,
  OPEN_ANOTHER_COMMAND,
  phoneCommand,
} from '../sager/model';
import styles from './SagerPage.module.css';

/**
 * Sager — the case list (R1). `rux gui` serves one project, so the list is
 * that project's card, plus how to open another and how to reach this one
 * from a phone (R10). The grid is the prototype's, so a longer list from a
 * multi-case server drops in without a layout change.
 */
export function SagerPage() {
  const { data, error, loading, reload } = useAsync(
    (s) => appWriteChain.idle().then(() => Promise.all([api.projectSummary(s), api.surveySummary(s)])),
    [],
  );
  // Only the status needs the fractions; a failure reads as "not done" (R3).
  const fractions = useAsync((s) => appWriteChain.idle().then(() => api.surveyFractions(s)), []);

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="sagslisten" />
      </div>
    );
  }
  if (loading && !data) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser sager…" />
      </div>
    );
  }
  if (!data) return null;

  const [summary, survey] = data;
  const record = summary.projects[0];
  const cards = [
    {
      key: summary.path,
      name: caseName(summary, record),
      subline: cardSubline(record),
      stats: caseStats(survey),
      status: caseStatus(survey, fractions.error ? null : fractions.data),
      date: cardDate(record),
    },
  ];

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h1 className={styles.title}>Sager</h1>
        <span className={styles.count}>{cards.length}</span>
      </header>

      <ul className={styles.cards} aria-label="Sager">
        {cards.map((c) => (
          <li key={c.key}>
            <CaseCard
              to={OVERBLIK_PATH}
              name={c.name}
              subline={c.subline}
              stats={c.stats}
              status={c.status}
              date={c.date}
              thumbUrl={api.renderUrl({ view: 'plan', width: 640, height: 248 })}
            />
          </li>
        ))}
      </ul>

      <section className={styles.panel} aria-labelledby="sager-andre">
        <h3 id="sager-andre" className={styles.panelHeading}>
          Åbn en anden sag
        </h3>
        <p className={styles.text}>
          Denne server viser én sag — den projektfil, den blev startet med. Start den med en anden fil:
        </p>
        <code className={styles.command}>{OPEN_ANOTHER_COMMAND}</code>
        <p className={styles.muted}>En sagsliste på tværs af projekter hører til serverudgaven (ruxd).</p>

        <h3 className={styles.panelHeading}>På pladsen med telefonen</h3>
        <p className={styles.text}>
          Åbn{' '}
          <Link className={styles.crossLink} to={ONSITE_PATH}>
            On-site
          </Link>{' '}
          på telefonen. Serveren lytter kun på denne maskine; for at nå den fra en telefon på samme netværk:
        </p>
        <code className={styles.command}>{phoneCommand(summary.path, window.location.port)}</code>
        <p className={styles.warn}>{NO_AUTH_WARNING}</p>
      </section>
    </div>
  );
}
```

Create `src/routes/SagerPage.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.page {
  composes: page from './viewHead.module.css';
  max-width: calc(var(--space-7) * 25);
  margin: 0 auto;
  width: 100%;
}

.head {
  composes: head from './viewHead.module.css';
}

.title {
  composes: title from './viewHead.module.css';
}

.count {
  font-weight: var(--font-weight-bold);
  color: var(--color-text-faint);
}

/* The prototype's grid: a longer list drops in without a layout change. */
.cards {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(min(100%, calc(var(--space-7) * 5 + var(--space-2))), 1fr));
  gap: var(--space-4);
  margin: 0;
  padding: 0;
  list-style: none;
}

.panel {
  composes: panel from '../components/surfaces.module.css';
  display: flex;
  flex-direction: column;
  gap: var(--space-2);
  padding: var(--space-4);
  max-width: calc(var(--space-7) * 15);
}

.panelHeading {
  composes: panelHeading from '../components/surfaces.module.css';
  margin-top: var(--space-2);
}

.text,
.muted,
.warn {
  margin: 0;
  font-size: var(--font-size-sm);
}

.muted {
  color: var(--color-text-muted);
}

.warn {
  composes: notice from '../components/surfaces.module.css';
}

.command {
  display: block;
  overflow-x: auto;
  padding: var(--space-2) var(--space-3);
  border-radius: var(--radius-md);
  background: var(--color-surface-sunken);
  font-family: var(--font-mono);
  font-size: var(--font-size-sm);
  white-space: nowrap;
}

.crossLink {
  composes: crossLink from '../components/controls.module.css';
}
```

`minmax(min(100%, 15.5rem), 1fr)` keeps the prototype's 15.5rem column and still never overflows a 390px screen.

- [ ] **Step 5: Route.** In `src/app/App.tsx`:
  - add `import { SagerPage } from '../routes/SagerPage';` after the `RapportPage` import;
  - add `import { ALL_CASES_PATH } from './navigation';` after the `./links` import;
  - add `<Route path={ALL_CASES_PATH} element={<SagerPage />} />` directly before `<Route path={OVERBLIK_PATH} element={<OverblikPage />} />`.

- [ ] **Step 6: Check and commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/sager apps/rux/frontend/src/routes/SagerPage.module.css apps/rux/frontend/src/routes/SagerPage.tsx --tsx
git add apps/rux/frontend/src/components/sager apps/rux/frontend/src/routes/SagerPage.tsx apps/rux/frontend/src/routes/SagerPage.module.css apps/rux/frontend/src/app/App.tsx apps/rux/frontend/src/app/links.ts apps/rux/frontend/src/test/links.test.ts
git commit -m "feat(gui): Sager lists the open case and says how to open another" -m "One card for the project rux gui serves, linking to Overblik: a plan render as its thumb (the striped status thumb when that fails), the survey's figures and a derived status. A panel gives the command for another case and the --bind/--allow-origin recipe for a phone, with the no-authentication warning." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 11: On-site model (R6, R7)

**Files:** Create `src/onsite/model.ts` and `src/test/onsite.model.test.ts`.

**Interfaces:**

```ts
export interface Stop { code: string; typeId: number; room: string }
export const NO_ROOM: string;                                         // 'Uden rum'
export function walkOrder(types: readonly SurveyType[]): Stop[];
export function currentStop(order: readonly Stop[], asked: string | null): Stop | null;
export function unknownNotice(asked: string | null, stop: Stop | null): string | null;
export function stopAfter(order: readonly Stop[], code: string): Stop | null;
export function nextLabel(next: Stop | null): string;
export function partAt(types: readonly SurveyType[], stop: Stop | null): { type: SurveyType; part: SurveyPart } | null;
export interface PickerGroup { room: string; options: { code: string; label: string }[] }
export function pickerGroups(order: readonly Stop[], types: readonly SurveyType[]): PickerGroup[];
export function chipDetail(type: SurveyType, part: SurveyPart): string;
export interface PhotoLookup { key: string; frames: VisibleFrame[]; failed: boolean }
export type PhotoView =
  | { kind: 'unlinked' } | { kind: 'loading' } | { kind: 'failed' } | { kind: 'none' }
  | { kind: 'photo'; frame: VisibleFrame };
export function photoView(currentKey: string | null, lookup: PhotoLookup | undefined): PhotoView;
export const PHOTO_TEXT: Record<Exclude<PhotoView['kind'], 'photo'>, string>;
export interface Reticle { left: number; top: number; width: number; height: number }   // percent
export function reticleBox(frame: Pick<VisibleFrame, 'u' | 'v'> | undefined, size: { width?: number; height?: number }): Reticle | null;
export function starButton(starred: boolean): { icon: string; text: string };
export function onsiteSampleBody(title: string, what: string, part: SurveyPart): SampleCreate | null;
export function sampleToast(code: string, partCode: string, gate: GateChange): string;
export function typeSamples(type: SurveyType, part: SurveyPart, samples: readonly Sample[]): { sample: Sample; here: boolean }[];
```

- [ ] **Step 1: Failing tests** — `src/test/onsite.model.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { SurveyType } from '../api/types';
import type { GateChange } from '../miljoe/model';
import {
  chipDetail,
  currentStop,
  nextLabel,
  onsiteSampleBody,
  partAt,
  PHOTO_TEXT,
  photoView,
  pickerGroups,
  reticleBox,
  sampleToast,
  starButton,
  stopAfter,
  typeSamples,
  unknownNotice,
  walkOrder,
} from '../onsite/model';
import { sample, surveyPart, surveyType } from './surveyFixtures';

const room = (id: number, name: string) => ({ room_id: id, room_name: name });

const TYPES: SurveyType[] = [
  surveyType({
    id: 1,
    name: 'Fundamenter & terrændæk, beton',
    parts: [
      surveyPart({ code: 'RX-010', type_id: 1, ...room(1, 'Production Hall') }),
      surveyPart({ code: 'RX-011', type_id: 1, ...room(2, 'Office Zone') }),
    ],
  }),
  surveyType({
    id: 6,
    name: 'Vinduespartier, aluminium',
    parts: [
      surveyPart({ code: 'RX-009', type_id: 6, ...room(1, 'Production Hall') }),
      surveyPart({ code: 'RX-008', type_id: 6, ...room(2, 'Office Zone') }),
    ],
  }),
  surveyType({ id: 2, name: 'Betonsøjler, bærende', parts: [surveyPart({ code: 'RX-002', type_id: 2, ...room(4, 'Entrance') })] }),
  surveyType({ id: 3, name: 'Løse dele', parts: [surveyPart({ code: 'RX-030', type_id: 3, room_id: null, room_name: '' })] }),
  surveyType({ id: 4, name: 'Kælder', parts: [surveyPart({ code: 'RX-040', type_id: 4, ...room(7, 'Ældre fløj') })] }),
  surveyType({
    id: 9,
    name: 'Fejldetektion',
    review_status: 'rejected',
    parts: [surveyPart({ code: 'RX-020', type_id: 9, ...room(2, 'Office Zone') })],
  }),
];

const NO_GATE: GateChange = { unblocked: [], contaminated: [], blocked: [], reblocked: [], released: [] };

describe('walk order (R6)', () => {
  it('walks rooms in Danish order, then codes; roomless last; rejected types left out', () => {
    expect(walkOrder(TYPES).map((s) => s.code)).toEqual(['RX-002', 'RX-008', 'RX-011', 'RX-009', 'RX-010', 'RX-040', 'RX-030']);
    expect(walkOrder(TYPES).at(-1)).toEqual({ code: 'RX-030', typeId: 3, room: '' });
  });

  it('starts at the asked part, else the first, and says when the asked one is unknown', () => {
    const order = walkOrder(TYPES);
    expect(currentStop(order, 'RX-009')?.code).toBe('RX-009');
    expect(currentStop(order, null)?.code).toBe('RX-002');
    expect(currentStop(order, 'RX-404')?.code).toBe('RX-002');
    expect(currentStop([], 'RX-002')).toBeNull();
    expect(unknownNotice('RX-404', currentStop(order, 'RX-404'))).toBe('RX-404 findes ikke — viser RX-002.');
    expect(unknownNotice('RX-009', currentStop(order, 'RX-009'))).toBeNull();
    expect(unknownNotice(null, currentStop(order, null))).toBeNull();
    expect(unknownNotice('RX-020', currentStop(order, 'RX-020'))).toBe('RX-020 findes ikke — viser RX-002.');
  });

  it('moves on, wrapping at the end, and names the next part', () => {
    const order = walkOrder(TYPES);
    expect(stopAfter(order, 'RX-008')?.code).toBe('RX-011');
    expect(stopAfter(order, 'RX-030')?.code).toBe('RX-002');
    expect(stopAfter(order.slice(0, 1), 'RX-002')).toBeNull();
    expect(nextLabel(stopAfter(order, 'RX-008'))).toBe('Videre → RX-011');
    expect(nextLabel(null)).toBe('Ingen flere bygningsdele');
  });

  it('finds the stop’s type and part', () => {
    const at = partAt(TYPES, currentStop(walkOrder(TYPES), 'RX-008'));
    expect(at?.type.id).toBe(6);
    expect(at?.part.code).toBe('RX-008');
    expect(partAt(TYPES, null)).toBeNull();
  });

  it('groups the picker by room, in walk order', () => {
    const groups = pickerGroups(walkOrder(TYPES), TYPES);
    expect(groups.map((g) => g.room)).toEqual(['Entrance', 'Office Zone', 'Production Hall', 'Ældre fløj', 'Uden rum']);
    expect(groups[1].options).toEqual([
      { code: 'RX-008', label: 'RX-008 · Vinduespartier, aluminium' },
      { code: 'RX-011', label: 'RX-011 · Fundamenter & terrændæk, beton' },
    ]);
  });
});

describe('the stage (R6)', () => {
  it('words the detection chip from the part and its type', () => {
    const t = surveyType({ confidence: 0.82 });
    expect(chipDetail(t, surveyPart())).toBe('RX-008 · Office Zone · sikkerhed 82 %');
    expect(chipDetail(surveyType({ confidence: null }), surveyPart({ room_id: null, room_name: '' }))).toBe('RX-008');
  });

  it('shows the photo only for the current part’s own lookup', () => {
    const frame = { frame_id: 41, centrality: 0.1, score: 0.9, depth: 2.1, u: 320, v: 240 };
    expect(photoView(null, undefined)).toEqual({ kind: 'unlinked' });
    expect(photoView('instances/7', undefined)).toEqual({ kind: 'loading' });
    expect(photoView('instances/7', { key: 'instances/6', frames: [frame], failed: false })).toEqual({ kind: 'loading' });
    expect(photoView('instances/7', { key: 'instances/7', frames: [], failed: true })).toEqual({ kind: 'failed' });
    expect(photoView('instances/7', { key: 'instances/7', frames: [], failed: false })).toEqual({ kind: 'none' });
    expect(photoView('instances/7', { key: 'instances/7', frames: [frame], failed: false })).toEqual({ kind: 'photo', frame });
    expect(PHOTO_TEXT.unlinked).toBe('Intet foto — bygningsdelen er ikke koblet til en instans.');
  });

  it('centres the reticle on the projected centroid, inside the stage', () => {
    expect(reticleBox({ u: 320, v: 240 }, { width: 640, height: 480 })).toEqual({ left: 26, top: 30, width: 48, height: 40 });
    expect(reticleBox({ u: 0, v: 0 }, { width: 640, height: 480 })).toEqual({ left: 0, top: 0, width: 48, height: 40 });
    expect(reticleBox({ u: 640, v: 480 }, { width: 640, height: 480 })).toEqual({ left: 52, top: 60, width: 48, height: 40 });
    expect(reticleBox({ u: -5, v: 10 }, { width: 640, height: 480 })).toBeNull();
    expect(reticleBox({ u: 10, v: 10 }, {})).toBeNull();
    expect(reticleBox(undefined, { width: 640, height: 480 })).toBeNull();
  });
});

describe('the sheet (R7)', () => {
  it('words the star toggle', () => {
    expect(starButton(false)).toEqual({ icon: '☆', text: 'Markér som vigtig' });
    expect(starButton(true)).toEqual({ icon: '★', text: 'Vigtig — tryk for at fjerne' });
  });

  it('registers a sample at the part, already taken', () => {
    expect(onsiteSampleBody('  Asbest i fugemasse ', ' Fuge mod nord ', surveyPart())).toEqual({
      title: 'Asbest i fugemasse',
      what: 'Fuge mod nord',
      type_ids: [6],
      part_code: 'RX-008',
      stage: 'udtaget',
    });
    expect(onsiteSampleBody('   ', 'x', surveyPart())).toBeNull();
  });

  it('toasts the new code, and the gate effect only when the type just started to wait', () => {
    expect(sampleToast('P-04', 'RX-008', NO_GATE)).toBe('✓ P-04 registreret ved RX-008');
    const t = surveyType({ id: 2, name: 'Betonsøjler, bærende' });
    expect(sampleToast('P-04', 'RX-002', { ...NO_GATE, blocked: [t] })).toBe(
      '✓ P-04 registreret ved RX-002 — Betonsøjler, bærende afventer nu prøvesvar',
    );
    expect(sampleToast('P-04', 'RX-002', { ...NO_GATE, reblocked: [t] })).toBe(
      '✓ P-04 registreret ved RX-002 — Betonsøjler, bærende er godkendt, men afventer nu prøvesvar',
    );
  });

  it('lists the type’s samples and marks the ones taken here', () => {
    const t = surveyType({ id: 6 });
    const list = [
      sample({ id: 1, type_ids: [6] }),
      sample({ id: 2, code: 'P-02', type_ids: [11] }),
      sample({ id: 4, code: 'P-04', type_ids: [6], part_code: 'RX-008', stage: 'udtaget' }),
    ];
    expect(typeSamples(t, surveyPart(), list).map((r) => [r.sample.code, r.here])).toEqual([
      ['P-01', false],
      ['P-04', true],
    ]);
  });
});
```

Run `npm --prefix apps/rux/frontend test -- onsite` → FAIL (module not found).

- [ ] **Step 2: Implement** — `src/onsite/model.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * On-site as data: the walk through the stored bygningsdele (R6), the
 * detection chip and photo stage, and what the sheet sends (R7). Pure, so the
 * walk and every write's shape are testable without a DOM.
 */

import type { Sample, SampleCreate, SurveyPart, SurveyType, VisibleFrame } from '../api/types';
import { roomName } from '../kortlaegning/model';
import { confidencePercent } from '../kortlaegning/vocab';
import type { GateChange } from '../miljoe/model';

/** One bygningsdel on the walk. `room` is '' for a part without one. */
export interface Stop {
  code: string;
  typeId: number;
  room: string;
}

/** The picker's group for parts without a room. */
export const NO_ROOM = 'Uden rum';

/**
 * Every part of every non-rejected type, room by room (Danish collation),
 * then by code (numeric). Parts without a room come last.
 */
export function walkOrder(types: readonly SurveyType[]): Stop[] {
  const stops: Stop[] = [];
  for (const t of types) {
    if (t.review_status === 'rejected') continue;
    for (const p of t.parts) stops.push({ code: p.code, typeId: t.id, room: roomName(p) });
  }
  return stops.sort((a, b) => {
    if ((a.room === '') !== (b.room === '')) return a.room === '' ? 1 : -1;
    return a.room.localeCompare(b.room, 'da') || a.code.localeCompare(b.code, 'da', { numeric: true });
  });
}

/** The asked part when it is on the walk, else the first stop. */
export function currentStop(order: readonly Stop[], asked: string | null): Stop | null {
  if (order.length === 0) return null;
  return order.find((s) => s.code === asked) ?? order[0];
}

/** Said when `?del=` names a part that is not on the walk (unknown, or of a rejected type). */
export function unknownNotice(asked: string | null, stop: Stop | null): string | null {
  if (asked === null || stop === null || stop.code === asked) return null;
  return `${asked} findes ikke — viser ${stop.code}.`;
}

/** The stop after `code`, wrapping at the end; null when there is nothing else to go to. */
export function stopAfter(order: readonly Stop[], code: string): Stop | null {
  if (order.length < 2) return null;
  const at = order.findIndex((s) => s.code === code);
  return order[(at + 1) % order.length];
}

export function nextLabel(next: Stop | null): string {
  return next ? `Videre → ${next.code}` : 'Ingen flere bygningsdele';
}

export function partAt(
  types: readonly SurveyType[],
  stop: Stop | null,
): { type: SurveyType; part: SurveyPart } | null {
  if (!stop) return null;
  const type = types.find((t) => t.id === stop.typeId);
  const part = type?.parts.find((p) => p.code === stop.code);
  return type && part ? { type, part } : null;
}

export interface PickerGroup {
  room: string;
  options: { code: string; label: string }[];
}

/** The part select's option groups: one per room, in walk order. */
export function pickerGroups(order: readonly Stop[], types: readonly SurveyType[]): PickerGroup[] {
  const names = new Map(types.map((t) => [t.id, t.name]));
  const groups: PickerGroup[] = [];
  for (const s of order) {
    const room = s.room || NO_ROOM;
    let group = groups.at(-1);
    if (!group || group.room !== room) {
      group = { room, options: [] };
      groups.push(group);
    }
    group.options.push({ code: s.code, label: `${s.code} · ${names.get(s.typeId) ?? ''}` });
  }
  return groups;
}

/** The chip's small line: code, room and the type's AI confidence, whichever exist. */
export function chipDetail(type: SurveyType, part: SurveyPart): string {
  const bits = [part.code];
  const room = roomName(part);
  if (room) bits.push(room);
  const c = confidencePercent(type.confidence);
  if (c !== null) bits.push(`sikkerhed ${c} %`);
  return bits.join(' · ');
}

/** A frames lookup tagged with the instance it was made for (as Kortlægning's evidence panel does). */
export interface PhotoLookup {
  key: string;
  frames: VisibleFrame[];
  failed: boolean;
}

export type PhotoView =
  | { kind: 'unlinked' }
  | { kind: 'loading' }
  | { kind: 'failed' }
  | { kind: 'none' }
  | { kind: 'photo'; frame: VisibleFrame };

/**
 * What the stage shows. `currentKey` is the current part's instance key, or
 * null without an instance link. A lookup made for another key — the previous
 * part's, in the render right after `Videre` — counts as still loading, never
 * as this part's photo.
 */
export function photoView(currentKey: string | null, lookup: PhotoLookup | undefined): PhotoView {
  if (currentKey === null) return { kind: 'unlinked' };
  if (!lookup || lookup.key !== currentKey) return { kind: 'loading' };
  if (lookup.failed) return { kind: 'failed' };
  const frame = lookup.frames[0];
  return frame ? { kind: 'photo', frame } : { kind: 'none' };
}

export const PHOTO_TEXT: Record<Exclude<PhotoView['kind'], 'photo'>, string> = {
  unlinked: 'Intet foto — bygningsdelen er ikke koblet til en instans.',
  loading: 'Indlæser foto…',
  failed: 'Foto kunne ikke hentes.',
  none: 'Intet foto — der blev ikke fundet en ramme for denne instans.',
};

/** Percent of the stage. */
export interface Reticle {
  left: number;
  top: number;
  width: number;
  height: number;
}

/** The prototype's reticle size, as a share of the stage. */
const RETICLE_W = 0.48;
const RETICLE_H = 0.4;

function pct(share: number): number {
  return Math.round(share * 1000) / 10;
}

/**
 * The reticle around the instance centroid's projection (`u`, `v`, in the
 * frame's pixels), clamped inside the stage. Null without a frame, without
 * the frame size, or when the projection falls outside the frame.
 */
export function reticleBox(
  frame: Pick<VisibleFrame, 'u' | 'v'> | undefined,
  size: { width?: number; height?: number },
): Reticle | null {
  if (!frame || !size.width || !size.height) return null;
  const cx = frame.u / size.width;
  const cy = frame.v / size.height;
  if (!(cx >= 0 && cx <= 1 && cy >= 0 && cy <= 1)) return null;
  const left = Math.min(1 - RETICLE_W, Math.max(0, cx - RETICLE_W / 2));
  const top = Math.min(1 - RETICLE_H, Math.max(0, cy - RETICLE_H / 2));
  return { left: pct(left), top: pct(top), width: pct(RETICLE_W), height: pct(RETICLE_H) };
}

export function starButton(starred: boolean): { icon: string; text: string } {
  return starred ? { icon: '★', text: 'Vigtig — tryk for at fjerne' } : { icon: '☆', text: 'Markér som vigtig' };
}

/**
 * The `POST /samples` body for a sample registered at a part (R7, R8):
 * already taken, linked to the part's type. Null while the title is empty.
 */
export function onsiteSampleBody(title: string, what: string, part: SurveyPart): SampleCreate | null {
  const t = title.trim();
  if (t === '') return null;
  return { title: t, what: what.trim(), type_ids: [part.type_id], part_code: part.code, stage: 'udtaget' };
}

function names(types: SurveyType[]): string {
  return types.map((t) => t.name).join(' · ');
}

/**
 * The toast after a sample is registered. A sample that is not yet answered
 * can only make types wait, so only `blocked` / `reblocked` are worded.
 */
export function sampleToast(code: string, partCode: string, gate: GateChange): string {
  const base = `✓ ${code} registreret ved ${partCode}`;
  if (gate.blocked.length > 0) return `${base} — ${names(gate.blocked)} afventer nu prøvesvar`;
  if (gate.reblocked.length > 0) return `${base} — ${names(gate.reblocked)} er godkendt, men afventer nu prøvesvar`;
  return base;
}

/** The samples on the part's type, in id order, each marked when it was taken at this part. */
export function typeSamples(
  type: SurveyType,
  part: SurveyPart,
  samples: readonly Sample[],
): { sample: Sample; here: boolean }[] {
  return samples.filter((s) => s.type_ids.includes(type.id)).map((s) => ({ sample: s, here: s.part_code === part.code }));
}
```

- [ ] **Step 3: Run → PASS**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
npm --prefix apps/rux/frontend test -- onsite
npm --prefix apps/rux/frontend run typecheck
```

`'Ældre fløj'` sorting after `'Production Hall'` is the Danish-collation check: Æ sorts after Z in `da`. If the Node build lacks full ICU, that assertion fails while the others pass. In that case run `node -e "console.log(['Æ','Z'].sort(new Intl.Collator('da').compare))"`; it must print `[ 'Z', 'Æ' ]`. If it does not, the devshell's Node is missing ICU data, and the fix is the devshell, not the test.

- [ ] **Step 4: Commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
git add apps/rux/frontend/src/onsite/model.ts apps/rux/frontend/src/test/onsite.model.test.ts
git commit -m "feat(gui): On-site model — the walk, the stage and what the sheet sends" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 12: On-site pieces — stage, sheet, picker (R6, R7)

**Files:** Create `src/components/onsite/CaptureStage.tsx`, `CaptureSheet.tsx`, `PartPicker.tsx` and a `.module.css` for each.

**Interfaces:**

```ts
export interface CaptureStageProps {
  photoUrl: string | null;              // null: show `placeholder`
  placeholder: string;
  reticle: Reticle | null;
  title: string;                        // the type name
  detail: string;                       // chipDetail
  aspect: { width: number; height: number } | null;   // the sensor frame's
}
export interface CaptureSheetProps {
  part: SurveyPart;
  busy: boolean;                        // gates ★ and Registrér prøve only
  next: Stop | null;
  onStar: () => void;
  onNote: (note: string) => void;       // never gated
  onRegister: (body: SampleCreate, done: () => void) => void;  // done() closes the form on success
  onNext: () => void;                   // navigation, not gated
}
export interface PartPickerProps { groups: PickerGroup[]; value: string; onChange: (code: string) => void }
```

The page renders `<CaptureSheet key={part.code} … />` (R6). The sheet relies on that remount to reset its note draft and close its form.

- [ ] **Step 1: Stage.** `src/components/onsite/CaptureStage.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState } from 'react';

import { PHOTO_TEXT, type Reticle } from '../../onsite/model';
import styles from './CaptureStage.module.css';

export interface CaptureStageProps {
  photoUrl: string | null;
  placeholder: string;
  reticle: Reticle | null;
  title: string;
  detail: string;
  aspect: { width: number; height: number } | null;
}

/**
 * The prototype's camera frame, as a stored photo (R6): the part's best
 * sensor frame, the reticle on its instance, and the detection chip. Takes
 * the frame's aspect ratio, so the reticle's percentages land on the photo.
 */
export function CaptureStage({ photoUrl, placeholder, reticle, title, detail, aspect }: CaptureStageProps) {
  const [failedUrl, setFailedUrl] = useState<string | null>(null);
  const showPhoto = photoUrl !== null && failedUrl !== photoUrl;
  return (
    <div className={styles.stage} style={aspect ? { aspectRatio: `${aspect.width} / ${aspect.height}` } : undefined}>
      {showPhoto ? (
        <img className={styles.photo} src={photoUrl} alt={`Bedste foto af ${title}`} onError={() => setFailedUrl(photoUrl)} />
      ) : (
        <p className={styles.placeholder}>{photoUrl !== null ? PHOTO_TEXT.failed : placeholder}</p>
      )}
      {showPhoto && reticle && (
        <div
          className={styles.reticle}
          aria-hidden="true"
          style={{ left: `${reticle.left}%`, top: `${reticle.top}%`, width: `${reticle.width}%`, height: `${reticle.height}%` }}
        />
      )}
      <div className={styles.chip}>
        <span className={styles.chipTitle}>{title}</span>
        <small className={styles.chipDetail}>{detail}</small>
      </div>
    </div>
  );
}
```

`src/components/onsite/CaptureStage.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.stage {
  position: relative;
  aspect-ratio: 4 / 3; /* until the frame size is known */
  overflow: hidden;
  background: var(--color-canvas);
}

.photo {
  display: block;
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.placeholder {
  position: absolute;
  inset: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  margin: 0;
  padding: var(--space-4) var(--space-4) calc(var(--space-7) + var(--space-2));
  text-align: center;
  font-size: var(--font-size-sm);
  color: var(--color-on-chrome-muted);
}

.reticle {
  position: absolute;
  border: 2px solid var(--color-accent);
  border-radius: var(--radius-lg);
}

.chip {
  position: absolute;
  left: var(--space-3);
  bottom: var(--space-3);
  max-width: calc(100% - var(--space-3) * 2);
  padding: var(--space-1) var(--space-2);
  border-radius: var(--radius-lg);
  background: var(--color-surface-raised);
  box-shadow: var(--shadow-md);
  color: var(--color-text);
}

.chipTitle {
  display: block;
  font-size: var(--font-size-sm);
  font-weight: var(--font-weight-bold);
}

.chipDetail {
  display: block;
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
}
```

- [ ] **Step 2: Sheet.** `src/components/onsite/CaptureSheet.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useId, useRef, useState, type KeyboardEvent } from 'react';

import type { SampleCreate, SurveyPart } from '../../api/types';
import { editorKeyAction } from '../../app/editorKeys';
import { kindOf } from '../../app/keyTargets';
import { fieldKeys, useTextDraft } from '../../app/useTextDraft';
import { STAGE_LABEL } from '../../kortlaegning/vocab';
import { nextLabel, onsiteSampleBody, starButton, type Stop } from '../../onsite/model';
import styles from './CaptureSheet.module.css';

export interface CaptureSheetProps {
  part: SurveyPart;
  busy: boolean;
  next: Stop | null;
  onStar: () => void;
  onNote: (note: string) => void;
  onRegister: (body: SampleCreate, done: () => void) => void;
  onNext: () => void;
}

interface SampleFormProps {
  part: SurveyPart;
  busy: boolean;
  onSubmit: (body: SampleCreate) => void;
  onCancel: () => void;
}

/**
 * `Registrér prøve her`, opened in the sheet. Nothing is sent until
 * `Registrér prøve`; Esc anywhere in the form cancels it (nothing in it is
 * saved yet), Ctrl/⌘+Enter submits — the keys of Miljø's create form.
 */
function SampleForm({ part, busy, onSubmit, onCancel }: SampleFormProps) {
  const [title, setTitle] = useState('');
  const [what, setWhat] = useState('');
  const body = onsiteSampleBody(title, what, part);
  const id = useId();
  const titleRef = useRef<HTMLInputElement>(null);
  useEffect(() => {
    titleRef.current?.focus();
  }, []);

  function submit() {
    if (body && !busy) onSubmit(body);
  }

  function onKeyDown(e: KeyboardEvent<HTMLFormElement>) {
    const action = editorKeyAction({
      key: e.key,
      kind: kindOf(e.target),
      ctrlKey: e.ctrlKey,
      metaKey: e.metaKey,
      altKey: e.altKey,
    });
    if (action === 'revert' || action === 'close') {
      e.preventDefault();
      e.stopPropagation();
      onCancel();
    } else if (action === 'submit') {
      e.preventDefault();
      e.stopPropagation();
      submit();
    }
    // 'commit' (plain Enter in a field) falls through to the native submit.
  }

  return (
    <form
      className={styles.form}
      aria-label={`Ny prøve ved ${part.code}`}
      onSubmit={(e) => {
        e.preventDefault();
        submit();
      }}
      onKeyDown={onKeyDown}
    >
      <label className={styles.field} htmlFor={`${id}-titel`}>
        <span className={styles.label}>Prøve</span>
        <input
          ref={titleRef}
          id={`${id}-titel`}
          className={styles.input}
          value={title}
          onChange={(e) => setTitle(e.target.value)}
          placeholder="fx PCB i fugemasse"
          required
        />
      </label>
      <label className={styles.field} htmlFor={`${id}-hvor`}>
        <span className={styles.label}>Hvor præcist</span>
        <input
          id={`${id}-hvor`}
          className={styles.input}
          value={what}
          onChange={(e) => setWhat(e.target.value)}
          placeholder="fx fuge ved vindue mod nord"
        />
      </label>
      <p className={styles.hint}>
        Kobles til {part.code} og dens type. Prøven registreres som {STAGE_LABEL.udtaget}.
      </p>
      <div className={styles.formActions}>
        <button type="button" className={styles.ghost} onClick={onCancel}>
          Annullér
        </button>
        <button type="submit" className={styles.primary} disabled={busy || body === null}>
          Registrér prøve
        </button>
      </div>
    </form>
  );
}

/**
 * The prototype's navy sheet: ★, a sample registered here, a quick note, and
 * Videre. Writes go against the part only (R7). The note commits on blur and
 * Enter, never gated on `busy`; Esc reverts it. ★ and the sample's submit are
 * buttons, gated on `busy`. The page re-keys this per part (R6).
 */
export function CaptureSheet({ part, busy, next, onStar, onNote, onRegister, onNext }: CaptureSheetProps) {
  const home = useRef<HTMLDivElement>(null);
  const note = useTextDraft(part.note, onNote);
  const [form, setForm] = useState(false);
  const star = starButton(part.starred);
  const noteId = useId();

  const closeForm = () => {
    setForm(false);
    home.current?.focus(); // never drop focus to <body>
  };

  return (
    <div ref={home} className={styles.sheet} tabIndex={-1}>
      <button
        type="button"
        className={styles.row}
        data-on={part.starred || undefined}
        aria-pressed={part.starred}
        disabled={busy}
        onClick={onStar}
      >
        <span className={styles.icon} aria-hidden="true">
          {star.icon}
        </span>
        {star.text}
      </button>

      {form ? (
        <SampleForm part={part} busy={busy} onSubmit={(body) => onRegister(body, closeForm)} onCancel={closeForm} />
      ) : (
        <button type="button" className={styles.row} onClick={() => setForm(true)}>
          <span className={styles.icon} aria-hidden="true">
            ◎
          </span>
          Registrér prøve her
        </button>
      )}

      <label className={`${styles.row} ${styles.noteRow}`} htmlFor={noteId}>
        <span className={styles.icon} aria-hidden="true">
          ✎
        </span>
        <input
          id={noteId}
          className={styles.noteInput}
          placeholder="Hurtig note…"
          aria-label={`Note til ${part.code}`}
          {...note.props}
          onKeyDown={fieldKeys(note, home)}
        />
      </label>

      <button type="button" className={styles.next} disabled={next === null} onClick={onNext}>
        {nextLabel(next)}
      </button>
    </div>
  );
}
```

`src/components/onsite/CaptureSheet.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* The prototype's navy sheet. Rows are at least 44px tall: a thumb target. */

.sheet {
  display: grid;
  gap: var(--space-2);
  padding: var(--space-3) var(--space-3) var(--space-4);
  background: var(--color-chrome);
}

.sheet:focus {
  outline: none; /* a keyboard anchor, not a control */
}

.row {
  display: flex;
  align-items: center;
  gap: var(--space-2);
  min-height: calc(var(--space-6) + var(--space-3));
  padding: var(--space-2) var(--space-3);
  border: 1px solid var(--color-chrome-border);
  border-radius: var(--radius-xl);
  background: var(--color-chrome-raised);
  color: var(--color-on-chrome);
  font: inherit;
  font-size: var(--font-size-md);
  font-weight: var(--font-weight-bold);
  text-align: left;
  cursor: pointer;
}

.row[data-on] {
  border-color: var(--color-star);
  color: var(--color-star);
}

.row:disabled {
  cursor: not-allowed;
  opacity: 0.6;
}

.row:focus-visible,
.noteRow:focus-within {
  outline: 2px solid var(--color-on-chrome);
  outline-offset: 2px;
}

.icon {
  flex: none;
  width: var(--space-4);
  text-align: center;
}

.noteRow {
  cursor: text;
}

.noteInput {
  flex: 1;
  min-width: 0;
  padding: 0;
  border: 0;
  background: transparent;
  color: var(--color-on-chrome);
  font: inherit;
  font-weight: var(--font-weight-regular);
  outline: none; /* the row draws the ring (:focus-within) */
}

.noteInput::placeholder {
  color: var(--color-on-chrome-muted);
}

.next {
  composes: btnPrimary from '../controls.module.css';
  min-height: calc(var(--space-6) + var(--space-3));
  border-radius: var(--radius-xl);
  font-size: var(--font-size-md);
}

.form {
  display: grid;
  gap: var(--space-2);
  padding: var(--space-3);
  border: 1px solid var(--color-chrome-border);
  border-radius: var(--radius-xl);
  background: var(--color-chrome-raised);
  color: var(--color-on-chrome);
}

.field {
  display: grid;
  gap: var(--space-1);
}

.label {
  font-size: var(--font-size-2xs);
  font-weight: var(--font-weight-bold);
  text-transform: uppercase;
  letter-spacing: var(--tracking-caps);
  color: var(--color-on-chrome-muted);
}

.input {
  composes: input from '../controls.module.css';
  min-height: calc(var(--space-6) + var(--space-2));
  font-size: var(--font-size-md); /* at least 16px-ish: no zoom-on-focus on iOS */
}

.hint {
  margin: 0;
  font-size: var(--font-size-xs);
  color: var(--color-on-chrome-muted);
}

.formActions {
  display: flex;
  justify-content: flex-end;
  gap: var(--space-2);
}

.ghost {
  composes: btnGhost from '../controls.module.css';
}

.primary {
  composes: btnPrimary from '../controls.module.css';
}
```

- [ ] **Step 3: Picker.** `src/components/onsite/PartPicker.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useId } from 'react';

import type { PickerGroup } from '../../onsite/model';
import styles from './PartPicker.module.css';

export interface PartPickerProps {
  groups: PickerGroup[];
  value: string;
  onChange: (code: string) => void;
}

/** Jump to any bygningsdel, grouped by room in walk order (R6). */
export function PartPicker({ groups, value, onChange }: PartPickerProps) {
  const id = useId();
  return (
    <div className={styles.picker}>
      <label className={styles.label} htmlFor={id}>
        Bygningsdel
      </label>
      <select id={id} className={styles.select} value={value} onChange={(e) => onChange(e.target.value)}>
        {groups.map((g) => (
          <optgroup key={g.room} label={g.room}>
            {g.options.map((o) => (
              <option key={o.code} value={o.code}>
                {o.label}
              </option>
            ))}
          </optgroup>
        ))}
      </select>
    </div>
  );
}
```

`src/components/onsite/PartPicker.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.picker {
  composes: field from '../controls.module.css';
  min-width: 0;
}

.label {
  composes: fieldLabel from '../controls.module.css';
}

.select {
  composes: input from '../controls.module.css';
  min-height: calc(var(--space-6) + var(--space-2));
  font-size: var(--font-size-md);
}
```

- [ ] **Step 4: Check and commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/onsite --tsx
git add apps/rux/frontend/src/components/onsite
git commit -m "feat(gui): On-site stage, sheet and part picker" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

Expected: typecheck, build and lint pass. The components are exercised in Task 13's page and asserted in Task 16.

---

### Task 13: On-site at `/on-site` (R6, R7, R11)

**Files:**
- Create `src/routes/OnsitePage.tsx` + `.module.css`.
- Modify `src/app/App.tsx`, `src/app/navigation.ts`, `src/test/navigation.test.ts`.

**Interfaces:** the route `/on-site?del=RX-###` and a sidebar entry `On-site` (Sag group, last).

- [ ] **Step 1: Failing test.** Add to `src/test/navigation.test.ts`:

```ts
  it('lists On-site last in the case workflow', () => {
    const sag = NAV_ENTRIES.filter((e) => e.group === 'sag');
    expect(sag.at(-1)).toEqual({ to: '/on-site', label: 'On-site', group: 'sag' });
  });
```

Run `npm --prefix apps/rux/frontend test -- navigation` → FAIL.

- [ ] **Step 2: Entry.** In `src/app/navigation.ts`, add `ONSITE_PATH,` to the import from `./links` (alphabetical, after `MILJOE_PATH,`), and after `{ to: INDBERETNING_PATH, label: 'Indberetning', group: 'sag' },` add `{ to: ONSITE_PATH, label: 'On-site', group: 'sag' },`. Run the test → PASS.

- [ ] **Step 3: Page.** Create `src/routes/OnsitePage.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Link, useLocation, useNavigate } from 'react-router-dom';

import { api } from '../api/client';
import type { Sample, SampleCreate, SurveyPartPatch, SurveyType } from '../api/types';
import { KORTLAEGNING_PATH, onsiteHref, parseOnsiteQuery, sampleHref, surveyTypeHref } from '../app/links';
import { saveErrorMessage } from '../app/saveError';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { useSurveyCounts } from '../app/SurveyCountsContext';
import { useToast } from '../app/useToast';
import { appWriteChain } from '../app/writeChain';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Pill } from '../components/Pill';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { hasInstanceLink, instanceKey } from '../components/kortlaegning/EvidencePanel';
import { CaptureSheet } from '../components/onsite/CaptureSheet';
import { CaptureStage } from '../components/onsite/CaptureStage';
import { PartPicker } from '../components/onsite/PartPicker';
import { replacePart } from '../kortlaegning/model';
import { gateChanges, statusPill } from '../miljoe/model';
import {
  chipDetail,
  currentStop,
  partAt,
  PHOTO_TEXT,
  photoView,
  pickerGroups,
  reticleBox,
  sampleToast,
  stopAfter,
  typeSamples,
  unknownNotice,
  walkOrder,
  type PhotoLookup,
} from '../onsite/model';
import styles from './OnsitePage.module.css';

/**
 * On-site — the phone capture sheet (R6, R7). A walk through the stored
 * bygningsdele: the part's best photo with the reticle on its instance, and a
 * sheet that writes ★, a note and a sample against the part. Quantities,
 * metadata and approval stay in Kortlægning.
 *
 * Writes run on the app-wide chain (R11), so the next screen's first read
 * sees them. Responses are folded in through a ref mirror, so a queued write
 * reads the state its predecessor's response produced.
 */
export function OnsitePage() {
  const location = useLocation();
  const navigate = useNavigate();
  const { data, error, loading, reload } = useAsync(
    (s) =>
      appWriteChain.idle().then(() => Promise.all([api.survey(s), api.samples(s), api.projectSummary(s)])),
    [],
  );
  const { refresh } = useSurveyCounts();
  const toast = useToast(2600);
  const { busy, mutate } = useMutationQueue({
    onError: (cause) => toast.show(saveErrorMessage(cause)),
    onSettled: refresh,
  });

  const [types, setTypesState] = useState<SurveyType[]>([]);
  const typesRef = useRef<SurveyType[]>([]);
  const setTypes = useCallback((next: SurveyType[]) => {
    typesRef.current = next;
    setTypesState(next);
  }, []);
  const [samples, setSamples] = useState<Sample[]>([]);
  useEffect(() => {
    if (!data) return;
    setTypes(data[0].types);
    setSamples(data[1]);
  }, [data, setTypes]);

  const order = useMemo(() => walkOrder(types), [types]);
  const asked = parseOnsiteQuery(location.search);
  const stop = currentStop(order, asked);
  const at = partAt(types, stop);
  const next = stop ? stopAfter(order, stop.code) : null;
  const part = at?.part ?? null;
  const key = hasInstanceLink(part) ? instanceKey(part.cloud, part.instance_id) : null;

  const frames = useAsync<PhotoLookup>(
    async (signal) => {
      if (!hasInstanceLink(part)) return { key: '', frames: [], failed: false };
      const k = instanceKey(part.cloud, part.instance_id);
      try {
        return { key: k, frames: await api.instanceFrames(part.cloud, part.instance_id, signal), failed: false };
      } catch (cause) {
        if (signal.aborted) throw cause; // a superseded lookup; useAsync drops it
        return { key: k, frames: [], failed: true };
      }
    },
    [key],
  );
  const photo = photoView(key, frames.data);

  const go = (code: string) => navigate(onsiteHref(code), { replace: true });

  const patchPart = (code: string, patch: SurveyPartPatch) =>
    mutate(async () => {
      const updated = await api.patchSurveyPart(code, patch);
      setTypes(replacePart(typesRef.current, updated));
    });

  const register = (body: SampleCreate, done: () => void) =>
    mutate(async () => {
      const before = typesRef.current;
      const created = await api.createSample(body);
      const [survey, list] = await Promise.all([api.survey(), api.samples()]);
      setTypes(survey.types);
      setSamples(list);
      toast.show(sampleToast(created.code, created.part_code ?? body.part_code ?? '', gateChanges(before, survey.types)));
      done();
    });

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="bygningsdelene" />
      </div>
    );
  }
  if (loading && !data) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser bygningsdele…" />
      </div>
    );
  }
  if (!data) return null;

  if (!at || !stop) {
    return (
      <div className={styles.page}>
        <h1 className={styles.title}>On-site</h1>
        <EmptyState
          title="Ingen bygningsdele endnu"
          detail="Bygningsdele oprettes i Kortlægning (Opret kortlægning), ud fra projektets instanser."
          action={
            <Link className={styles.crossLink} to={KORTLAEGNING_PATH}>
              Åbn Kortlægning
            </Link>
          }
        />
      </div>
    );
  }

  const { type } = at;
  const frameSize = data[2].sensor_frames;
  const aspect = frameSize.width && frameSize.height ? { width: frameSize.width, height: frameSize.height } : null;
  const photoUrl = photo.kind === 'photo' ? api.frameImageUrl(photo.frame.frame_id, 'color', { maxSize: 960 }) : null;
  const notice = unknownNotice(asked, stop);
  const onType = typeSamples(type, at.part, samples);

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h1 className={styles.title}>On-site</h1>
        <PartPicker groups={pickerGroups(order, types)} value={stop.code} onChange={go} />
      </header>
      {notice && <p className={styles.notice}>{notice}</p>}

      <section className={styles.device} aria-label={`Bygningsdel ${at.part.code}`}>
        <CaptureStage
          photoUrl={photoUrl}
          placeholder={photo.kind === 'photo' ? '' : PHOTO_TEXT[photo.kind]}
          reticle={photo.kind === 'photo' ? reticleBox(photo.frame, frameSize) : null}
          title={type.name}
          detail={chipDetail(type, at.part)}
          aspect={aspect}
        />
        <CaptureSheet
          key={at.part.code}
          part={at.part}
          busy={busy}
          next={next}
          onStar={() => patchPart(at.part.code, { starred: !at.part.starred })}
          onNote={(note) => patchPart(at.part.code, { note })}
          onRegister={register}
          onNext={() => next && go(next.code)}
        />
      </section>

      <section className={styles.samples} aria-labelledby="onsite-proever">
        <h2 id="onsite-proever" className={styles.samplesHeading}>
          Prøver på typen
        </h2>
        {onType.length === 0 ? (
          <p className={styles.muted}>Ingen prøver på {type.name} endnu.</p>
        ) : (
          <ul className={styles.sampleList}>
            {onType.map(({ sample, here }) => {
              const pill = statusPill(sample);
              return (
                <li key={sample.id} className={styles.sampleRow}>
                  <Link className={styles.crossLink} to={sampleHref(sample.id)}>
                    {sample.code} · {sample.title}
                  </Link>
                  {here && <span className={styles.muted}>· her</span>}
                  <Pill tone={pill.tone}>{pill.label}</Pill>
                </li>
              );
            })}
          </ul>
        )}
      </section>

      <p className={styles.footnote}>
        Mængder, metadata og godkendelse venter til gennemsynet i Kortlægning.{' '}
        <Link className={styles.crossLink} to={surveyTypeHref(type.id)}>
          Åbn {type.name} i Kortlægning
        </Link>
      </p>
      <Toast message={toast.message} />
    </div>
  );
}
```

`hasInstanceLink(part)` is a type guard (`part is SurveyPart & { cloud: string; instance_id: number }`), so `part.cloud` and `part.instance_id` need no `!`. The frames closure repeats the check on purpose: narrowing from outside a closure does not carry into it.

Create `src/routes/OnsitePage.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/* Phone-first: one column the prototype's phone width (300px + margin), centred
   on a wide screen. */

.page {
  display: flex;
  flex-direction: column;
  gap: var(--space-3);
  width: 100%;
  max-width: calc(var(--space-7) * 8);
  margin: 0 auto;
  padding: var(--space-3) var(--space-3) var(--space-6);
  min-width: 0;
}

.head {
  display: grid;
  gap: var(--space-2);
}

.title {
  composes: title from './viewHead.module.css';
  font-size: var(--font-size-xl);
}

.notice {
  composes: notice from '../components/surfaces.module.css';
}

.device {
  overflow: hidden;
  border: 1px solid var(--color-chrome-border);
  border-radius: var(--radius-xl);
  background: var(--color-chrome);
  box-shadow: var(--shadow-panel);
}

.samples {
  display: grid;
  gap: var(--space-2);
}

.samplesHeading {
  composes: panelHeading from '../components/surfaces.module.css';
  font-size: var(--font-size-md);
}

.sampleList {
  display: grid;
  gap: var(--space-2);
  margin: 0;
  padding: 0;
  list-style: none;
}

.sampleRow {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: var(--space-2);
  font-size: var(--font-size-sm);
}

.muted {
  margin: 0;
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
}

.footnote {
  composes: footnote from './viewHead.module.css';
}

.crossLink {
  composes: crossLink from '../components/controls.module.css';
}
```

- [ ] **Step 4: Route.** In `src/app/App.tsx`:
  - add `ONSITE_PATH,` to the import from `./links` (after `MILJOE_PATH,`);
  - add `import { OnsitePage } from '../routes/OnsitePage';` after the `MiljoePage` import;
  - add `<Route path={ONSITE_PATH} element={<OnsitePage />} />` directly after `<Route path={INDBERETNING_PATH} element={<IndberetningPage />} />`.

- [ ] **Step 5: Check and commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/routes/OnsitePage.module.css apps/rux/frontend/src/routes/OnsitePage.tsx --tsx
git add apps/rux/frontend/src/routes/OnsitePage.tsx apps/rux/frontend/src/routes/OnsitePage.module.css apps/rux/frontend/src/app/App.tsx apps/rux/frontend/src/app/navigation.ts apps/rux/frontend/src/test/navigation.test.ts
git commit -m "feat(gui): On-site — the phone sheet writes ★, a note and a sample against a bygningsdel" -m "A walk through the stored parts, room by room: the part's best photo with the reticle on its instance, a sheet that stars the part, edits its note (commit on blur, Esc reverts) and registers a sample taken there, and Videre to the next part. The sheet is re-keyed per part so a draft never follows the walk. Photo capture is not drawn (out of scope)." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 14: Kortlægning shows what On-site wrote (R13)

**Files:**
- Modify `src/kortlaegning/model.ts`, `src/test/kortlaegning.model.test.ts`.
- Modify `src/components/kortlaegning/SurveyTable.tsx`, `src/components/kortlaegning/SurveyTable.module.css`.

**Interfaces:**

```ts
export interface Filters { search: string; roomId: number | null; env: EnvFilter | null; starred: boolean }
export const NO_FILTERS: Filters;   // starred: false
```

- [ ] **Step 1: Failing test.** Append to `src/test/kortlaegning.model.test.ts`:

```ts
describe('the ★ filter (Phase 6 R13)', () => {
  it('keeps starred types and types with a starred part', () => {
    const types = [
      type(1, 'Stålspær', { starred: true }),
      type(2, 'Vinduespartier', { parts: [{ ...part('RX-008', 2, [2, 'Office Zone'], 26), starred: true }] }),
      type(3, 'Betondæk', { parts: [part('RX-003', 3, [1, 'Production Hall'], 980)] }),
    ];
    expect(NO_FILTERS.starred).toBe(false);
    expect(visibleTypes(types, 'all', NO_FILTERS).map((t) => t.id)).toEqual([1, 2, 3]);
    expect(visibleTypes(types, 'all', { ...NO_FILTERS, starred: true }).map((t) => t.id)).toEqual([1, 2]);
  });
});
```

Run `npm --prefix apps/rux/frontend test -- kortlaegning.model` → FAIL (the filter does not exist; typecheck also flags `starred`).

- [ ] **Step 2: Model.** In `src/kortlaegning/model.ts`, replace

```ts
export interface Filters {
  search: string;
  roomId: number | null;
  env: EnvFilter | null;
}
export const NO_FILTERS: Filters = { search: '', roomId: null, env: null };
```

with

```ts
export interface Filters {
  search: string;
  roomId: number | null;
  env: EnvFilter | null;
  /** Only types that are ★, or have a ★ part — what On-site marks (Phase 6 R13). */
  starred: boolean;
}
export const NO_FILTERS: Filters = { search: '', roomId: null, env: null, starred: false };
```

and in `visibleTypes`, replace

```ts
      (f.env === null || envFilterOf(t.environment_status) === f.env),
```

with

```ts
      (f.env === null || envFilterOf(t.environment_status) === f.env) &&
      (!f.starred || t.starred || t.parts.some((p) => p.starred)),
```

- [ ] **Step 3: Table.** In `src/components/kortlaegning/SurveyTable.tsx`:
  - after the `Miljøstatus` `<select>`'s closing `</select>`, inside `<div className={styles.tools}>`, add:

```tsx
        <label className={styles.starFilter}>
          <input
            type="checkbox"
            className={styles.checkbox}
            checked={filters.starred}
            onChange={(e) => onFilters({ ...filters, starred: e.target.checked })}
          />
          Kun vigtige ★
        </label>
```

  - replace the part row's

```tsx
                      <div className={styles.partCell}>
                        {partLabel(part)}
```

    with

```tsx
                      <div className={styles.partCell}>
                        {part.starred && (
                          <span className={styles.star} role="img" aria-label="Vigtig" title="Vigtig">
                            ★
                          </span>
                        )}
                        {partLabel(part)}
                        {part.note && (
                          <span className={styles.noteMark} role="img" aria-label={`Note: ${part.note}`} title={part.note}>
                            ✎
                          </span>
                        )}
```

In `src/components/kortlaegning/SurveyTable.module.css`:
  - in the `.tools` rule, after `gap: var(--space-2);`, add `flex-wrap: wrap;`, so the tools wrap instead of widening the page at 390px;
  - append:

```css
.starFilter {
  display: inline-flex;
  align-items: center;
  gap: var(--space-1);
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
  white-space: nowrap;
}

.checkbox {
  composes: checkbox from '../controls.module.css';
}

.noteMark {
  color: var(--color-text-faint);
}
```

Kortlægning's table keys already skip fields (`isField` classifies a checkbox as `choice`), so G/A/V never fire from the checkbox.

- [ ] **Step 4: Check and commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/kortlaegning/SurveyTable.module.css apps/rux/frontend/src/components/kortlaegning/SurveyTable.tsx --tsx
git add apps/rux/frontend/src/kortlaegning/model.ts apps/rux/frontend/src/test/kortlaegning.model.test.ts apps/rux/frontend/src/components/kortlaegning/SurveyTable.tsx apps/rux/frontend/src/components/kortlaegning/SurveyTable.module.css
git commit -m "feat(gui): Kortlægning filters on ★ and marks starred and noted parts" -m "A 'Kun vigtige ★' checkbox keeps types that are starred or have a starred part, and part rows draw ★ and a ✎ note marker, so what On-site writes shows up in the review. The tools row wraps on a narrow screen." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 15: Miljø & prøver says where a sample was taken (R8)

**Files:** Modify `src/miljoe/model.ts`, `src/test/miljoe.model.test.ts`, `src/components/miljoe/SampleCard.tsx`.

**Interfaces:**

```ts
export interface TakenAt { text: string; typeId: number | null }   // typeId null: plain text, no link
export function takenAt(s: Pick<Sample, 'part_code'>, types: readonly SurveyType[]): TakenAt | null;
```

- [ ] **Step 1: Failing test.** In `src/test/miljoe.model.test.ts`, add `takenAt,` to the import from `../miljoe/model`, add `surveyPart` to the import from `./surveyFixtures`, and append:

```ts
describe('where a sample was taken (Phase 6 R8)', () => {
  const types = [
    surveyType({ id: 6, parts: [surveyPart()] }),
    surveyType({ id: 9, review_status: 'rejected', parts: [surveyPart({ code: 'RX-020', type_id: 9, room_id: null, room_name: '' })] }),
  ];

  it('names the part and its room, linking to the part’s type', () => {
    expect(takenAt({ part_code: 'RX-008' }, types)).toEqual({ text: 'Udtaget ved RX-008 · Office Zone', typeId: 6 });
  });

  it('is plain text for a rejected type or an unknown code, and absent without a part', () => {
    expect(takenAt({ part_code: 'RX-020' }, types)).toEqual({ text: 'Udtaget ved RX-020', typeId: null });
    expect(takenAt({ part_code: 'RX-404' }, types)).toEqual({ text: 'Udtaget ved RX-404', typeId: null });
    expect(takenAt({ part_code: null }, types)).toBeNull();
  });
});
```

Run `npm --prefix apps/rux/frontend test -- miljoe` → FAIL.

- [ ] **Step 2: Model.** In `src/miljoe/model.ts`, add `import { partLabel } from '../kortlaegning/model';` with the other imports, and append:

```ts
export interface TakenAt {
  text: string;
  /** The part's type to link to; null for a rejected type or an unknown code (plain text, R8). */
  typeId: number | null;
}

/** "Udtaget ved RX-008 · Office Zone" for a sample registered on site; null for any other. */
export function takenAt(s: Pick<Sample, 'part_code'>, types: readonly SurveyType[]): TakenAt | null {
  if (!s.part_code) return null;
  for (const t of types) {
    const p = t.parts.find((x) => x.code === s.part_code);
    if (p) return { text: `Udtaget ved ${partLabel(p)}`, typeId: t.review_status === 'rejected' ? null : t.id };
  }
  return { text: `Udtaget ved ${s.part_code}`, typeId: null };
}
```

If `Sample` or `SurveyType` is not yet imported as a type in `miljoe/model.ts`, add it to the existing `import type { … } from '../api/types';` line.

- [ ] **Step 3: Card.** In `src/components/miljoe/SampleCard.tsx`, add `takenAt,` to the import from `../../miljoe/model`. Directly after

```tsx
      {sample.what && <p className={styles.what}>{sample.what}</p>}
```

add

```tsx
      {taken && (
        <p className={styles.what}>
          {taken.typeId !== null ? (
            <Link className={styles.typeLink} to={surveyTypeHref(taken.typeId)}>
              {taken.text}
            </Link>
          ) : (
            taken.text
          )}
        </p>
      )}
```

and declare `const taken = takenAt(sample, types);` at the top of the component body that renders this markup (the card's main function, next to its other derived constants).

- [ ] **Step 4: Check and commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
npm --prefix apps/rux/frontend test
npm --prefix apps/rux/frontend run typecheck
npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/miljoe/SampleCard.tsx --tsx
git add apps/rux/frontend/src/miljoe/model.ts apps/rux/frontend/src/test/miljoe.model.test.ts apps/rux/frontend/src/components/miljoe/SampleCard.tsx
git commit -m "feat(gui): a sample card says which bygningsdel it was taken at" -m "Samples registered on site show 'Udtaget ved RX-008 · Office Zone', linking to the part's type in Kortlægning (plain text for a rejected type or an unknown code)." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 16: Verify against the prototype and the flows

**Files:** None in the repo. The scripts and shots go to the scratchpad.

- [ ] **Step 1: Shot script.** Write `$SP/phase6_shots.py`. It waits on content, never on time, and fails on horizontal overflow (R5):

```python
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Static shots of the Phase 6 screens in both themes; waits on content, not time.
Fails when a route overflows horizontally at phone or tablet width (R5)."""
import sys

from playwright.sync_api import expect, sync_playwright

base, out = sys.argv[1], sys.argv[2]
only = set(sys.argv[3].split(",")) if len(sys.argv) > 3 else None
# name: (path, marker, case screen — <main> must not overflow either)
ROUTES = {
    "sager": ("/sager", "Åbn en anden sag", True),
    "onsite": ("/on-site?del=RX-008", "Prøver på typen", True),
    "overblik": ("/", "Cirkularitetsoversigt", True),
    "kortlaegning": ("/kortlaegning", "Kun vigtige ★", True),
    "miljoe": ("/miljoe", "P-01 · PCB i fugemasse", True),
    "rapport": ("/rapport", "Inventarliste", True),
    "indberetning": ("/indberetning", "I alt (godkendt)", True),
    "projektdata": ("/projektdata", "Material passports", False),  # a tool screen: document overflow only
}
VIEWPORTS = {"desktop": (1440, 1000), "tablet": (768, 1024), "mobile": (390, 844)}
OVERFLOW = """(() => {
  const doc = document.documentElement.scrollWidth - window.innerWidth;
  const m = document.querySelector('main');
  const main = m ? m.scrollWidth - m.clientWidth : 0;
  let widest = '';
  if (doc > 0 || main > 0) {
    let max = 0;
    for (const el of document.querySelectorAll('main *')) {
      const r = el.getBoundingClientRect().right;
      if (r > max) { max = r; widest = el.tagName.toLowerCase() + '.' + [...el.classList].join('.'); }
    }
  }
  return { doc, main, widest };
})()"""

with sync_playwright() as p:
    browser = p.chromium.launch()
    for theme in ("light", "dark"):
        for vp, (w, h) in VIEWPORTS.items():
            mobile = vp == "mobile"
            page = browser.new_page(
                viewport={"width": w, "height": h}, color_scheme=theme, is_mobile=mobile, has_touch=mobile
            )
            page.add_init_script(f"localStorage.setItem('reusex-theme', '{theme}')")
            for name, (path, marker, case) in ROUTES.items():
                if only and name not in only:
                    continue
                page.goto(base + path)
                expect(page.get_by_text(marker).first).to_be_visible()
                page.evaluate("document.fonts.ready")
                if vp != "desktop":
                    o = page.evaluate(OVERFLOW)
                    assert o["doc"] <= 0, f"{name}/{theme}/{vp}: document overflows by {o['doc']}px ({o['widest']})"
                    if case:
                        assert o["main"] <= 0, f"{name}/{theme}/{vp}: <main> overflows by {o['main']}px ({o['widest']})"
                page.screenshot(path=f"{out}/{name}-{theme}-{vp}.png", full_page=True)
            if mobile:
                # The drawer, open, for the record.
                page.goto(base + "/on-site?del=RX-008")
                page.get_by_role("button", name="Menu").click()
                expect(page.get_by_role("navigation", name="Sag")).to_be_visible()
                page.screenshot(path=f"{out}/drawer-{theme}-{vp}.png")
            page.close()
    browser.close()
print("shots OK")
```

Run it on the **plain** seed, which holds the prototype's data:

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/p6-demo.rux"
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/p6-demo.rux" 8426 5179
mkdir -p "$SP/shots/p6"
BROWSERS="$(nix build --no-link --print-out-paths nixpkgs#playwright-driver.browsers)"
PLAYWRIGHT_BROWSERS_PATH="$BROWSERS" nix shell --impure --expr 'let p = import (builtins.getFlake "nixpkgs") {}; in p.python3.withPackages (ps: [ ps.playwright ])' --command python3 "$SP/phase6_shots.py" http://localhost:5179 "$SP/shots/p6"
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

Expected: `shots OK`. If an overflow assertion fails, its message names the widest element. Fix that element's CSS in the task that introduced it: let it wrap (`flex-wrap: wrap`), or let it scroll inside its own panel (`overflow-x: auto`, as the fraction table does). Then re-run. Open the PNGs with Read and compare them with the prototype shots:
- **`sager-light-desktop`** against `sager.png`:
  - `SAGER` with a faint `1`;
  - one card. Its thumb is a plan render, or the striped thumb labelled `GENNEMGANG` when the server has no renderer;
  - `MÅLØV BYVEJ 229` (or the `.rux` stem, if the seed sets no name);
  - `Ingen adresse registreret`, unless the record has one;
  - `11 komponenter · 54 % bevaring/genbrug · 7 til gennemsyn`;
  - an accent `Gennemgang` pill and `—` on the right;
  - below the grid, the `Åbn en anden sag` panel with both commands and the warn notice. No `+ Nyt projekt` (R1).
- **`onsite-light-mobile`** against `onsite.png`:
  - the navy title bar with `☰`, no sidebar;
  - `ON-SITE` and the `Bygningsdel` select on `RX-008 · Vinduespartier, aluminium`;
  - a dark stage with `Intet foto — bygningsdelen er ikke koblet til en instans.` (the demo parts have no instances) and the white chip `Vinduespartier, aluminium` / `RX-008 · Office Zone · sikkerhed 82 %`;
  - the navy sheet: `☆ Markér som vigtig`, `◎ Registrér prøve her`, `✎ Hurtig note…`, then the accent `Videre → RX-011`. No `Tilføj ekstra foto` (R9);
  - `Prøver på typen` with `P-01 · PCB i fugemasse` and a `Sendt til lab` pill;
  - the footnote with `Åbn Vinduespartier, aluminium i Kortlægning`.
- **`onsite-light-desktop`:** the same column, centred, at most 24rem wide; the full sidebar is shown.
- **`drawer-*-mobile`:** the drawer over a scrim, with every Sag entry including `On-site`, and `← Alle sager` at the bottom.
- **`kortlaegning-*`:** the tools row ends with `Kun vigtige ★`; at 390px it wraps.
- **The 1440 shots of `overblik`, `miljoe`, `rapport`, `indberetning`** are unchanged from Phase 5 (R5: nothing changes above 900px).
- **Dark shots:** no light-only colour. The sheet and the drawer stay navy, the stage near-black.

Fix what differs, then re-shoot.

- [ ] **Step 2: Varied seed — what On-site wrote, seen elsewhere**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh --varied "$SP/corridor-clouds.rux" "$SP/p6-varied.rux"
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/p6-varied.rux" 8426 5179
mkdir -p "$SP/shots/p6-varied"
PLAYWRIGHT_BROWSERS_PATH="$BROWSERS" nix shell --impure --expr 'let p = import (builtins.getFlake "nixpkgs") {}; in p.python3.withPackages (ps: [ ps.playwright ])' --command python3 "$SP/phase6_shots.py" http://localhost:5179 "$SP/shots/p6-varied" miljoe,kortlaegning
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

Expected:
- `miljoe-light-desktop`: the P-06 card reads `Udtaget ved RX-013 · Office Zone` as an accent-deep underlined link;
- the `kortlaegning` shots show the tools row and the ★ checkbox.

The ★ part marker is asserted in the flow, where the row is expanded.

- [ ] **Step 3: Flow script.** Write `$SP/phase6_flow.py`:

```python
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Drive the Phase 6 flows on the plain demo seed. Asserts on captured
requests and awaited responses; never sleeps. The one held request is held
with page.route and released explicitly."""
import json
import re
import sys

from playwright.sync_api import expect, sync_playwright

base, out = sys.argv[1], sys.argv[2]
API = re.compile(r"/api/v1/")

with sync_playwright() as p:
    browser = p.chromium.launch()
    phone = browser.new_page(viewport={"width": 390, "height": 844}, is_mobile=True, has_touch=True)
    phone.add_init_script("localStorage.setItem('reusex-theme', 'light')")
    requests, statuses = [], []

    def capture(page):
        page.on("request", lambda r: requests.append((r.method, r.url, r.post_data)) if API.search(r.url) else None)
        page.on("response", lambda r: statuses.append((r.status, r.url)) if API.search(r.url) else None)

    capture(phone)

    def bodies(method, fragment):
        return [json.loads(d or "{}") for (m, u, d) in requests if m == method and fragment in u]

    def toast(page, text):
        expect(page.get_by_role("status").filter(has_text=text)).to_be_visible()

    def is_patch(fragment):
        return lambda r: fragment in r.url and r.request.method == "PATCH"

    # --- Drawer at phone width (R5) -----------------------------------------
    phone.goto(f"{base}/on-site?del=RX-008")
    menu = phone.get_by_role("button", name="Menu")
    sag_nav = phone.get_by_role("navigation", name="Sag")
    expect(sag_nav).to_be_hidden()
    menu.click()
    expect(menu).to_have_attribute("aria-expanded", "true")
    expect(sag_nav).to_be_visible()
    phone.keyboard.press("Escape")
    expect(sag_nav).to_be_hidden()
    expect(menu).to_be_focused()
    menu.click()
    sag_nav.get_by_role("link", name="On-site").click()
    expect(sag_nav).to_be_hidden()  # navigation closes it

    # --- On-site: the chip and the stage (R6) --------------------------------
    phone.goto(f"{base}/on-site?del=RX-008")
    expect(phone.get_by_text("RX-008 · Office Zone · sikkerhed 82 %")).to_be_visible()
    expect(phone.get_by_text("Intet foto — bygningsdelen er ikke koblet til en instans.")).to_be_visible()
    phone.screenshot(path=f"{out}/01-onsite.png")

    # --- Note: untouched blur and Esc send nothing; Enter commits once ------
    note = phone.get_by_label("Note til RX-008")
    note.focus()
    note.blur()
    note.fill("Fuger revnede mod nord")
    note.press("Escape")
    expect(note).to_have_value("")
    with phone.expect_response(is_patch("/api/v1/survey/parts/RX-008")) as resp:
        note.fill("Fuger revnede mod nord")
        note.press("Enter")
    assert resp.value.ok, resp.value.text()
    # Ordering proves absence: anything the earlier actions sent would have
    # been queued before this commit.
    assert bodies("PATCH", "/survey/parts/RX-008") == [{"note": "Fuger revnede mod nord"}], bodies(
        "PATCH", "/survey/parts/RX-008"
    )

    # --- ★ toggles the part --------------------------------------------------
    star = phone.get_by_role("button", name="Markér som vigtig")
    with phone.expect_response(is_patch("/api/v1/survey/parts/RX-008")):
        star.click()
    assert bodies("PATCH", "/survey/parts/RX-008")[-1] == {"starred": True}
    expect(phone.get_by_role("button", name="Vigtig — tryk for at fjerne")).to_have_attribute("aria-pressed", "true")

    # --- A sample taken here (R7, R8) ----------------------------------------
    phone.get_by_role("button", name="Registrér prøve her").click()
    phone.get_by_label("Prøve", exact=True).fill("Asbest i fugemasse")
    phone.get_by_label("Hvor præcist").fill("Fuge ved vindue mod nord")
    is_create = lambda r: r.url.endswith("/api/v1/samples") and r.request.method == "POST"
    with phone.expect_response(is_create) as created:
        phone.get_by_role("button", name="Registrér prøve", exact=True).click()
    assert created.value.status == 201, created.value.text()
    assert bodies("POST", "/api/v1/samples")[-1] == {
        "title": "Asbest i fugemasse",
        "what": "Fuge ved vindue mod nord",
        "type_ids": [6],
        "part_code": "RX-008",
        "stage": "udtaget",
    }, bodies("POST", "/api/v1/samples")
    toast(phone, "✓ P-04 registreret ved RX-008")
    samples = phone.get_by_role("region", name="Prøver på typen")
    expect(samples.get_by_role("listitem").filter(has_text="P-04 · Asbest i fugemasse")).to_contain_text("her")
    expect(samples.get_by_role("listitem").filter(has_text="P-04")).to_contain_text("Udtaget")
    phone.screenshot(path=f"{out}/02-onsite-sample.png")

    # --- Videre: a typed note lands on the OLD part; the new part is clean ---
    with phone.expect_response(is_patch("/api/v1/survey/parts/RX-008")):
        note.fill("Fuger revnede mod nord · tjek ved prøvesvar")
        phone.get_by_role("button", name="Videre → RX-011").click()
    expect(phone).to_have_url(re.compile(r"/on-site\?del=RX-011$"))
    expect(phone.get_by_label("Note til RX-011")).to_have_value("")
    assert bodies("PATCH", "/survey/parts/RX-008")[-1] == {"note": "Fuger revnede mod nord · tjek ved prøvesvar"}
    assert not bodies("PATCH", "/survey/parts/RX-011")
    phone.screenshot(path=f"{out}/03-onsite-next.png")

    # --- Cross-screen staleness (R11): hold the POST, open Kortlægning -------
    desk = browser.new_page(viewport={"width": 1440, "height": 1000})
    desk.add_init_script("localStorage.setItem('reusex-theme', 'light')")
    capture(desk)
    held = []
    desk.route("**/api/v1/samples", lambda route: held.append(route) if route.request.method == "POST" else route.continue_())
    desk.goto(f"{base}/on-site?del=RX-002")
    desk.get_by_role("button", name="Registrér prøve her").click()
    desk.get_by_label("Prøve", exact=True).fill("Klorparaffiner i fuge")
    with desk.expect_request(lambda r: r.url.endswith("/api/v1/samples") and r.method == "POST"):
        desk.get_by_role("button", name="Registrér prøve", exact=True).click()
    marker = len(requests)
    desk.get_by_role("navigation", name="Sag").get_by_role("link", name="Kortlægning").click()
    expect(desk.get_by_role("status").filter(has_text="Indlæser kortlægning")).to_be_visible()
    assert len(held) == 1, held
    while_held = [u for (m, u, _) in requests[marker:] if m == "GET" and u.endswith("/api/v1/survey")]
    assert while_held == [], while_held  # Kortlægning's first read waits for the chain (R11)
    with desk.expect_response(lambda r: r.url.endswith("/api/v1/survey") and r.request.method == "GET"):
        held[0].continue_()
    row = desk.get_by_role("row").filter(has_text="Betonsøjler, bærende")
    expect(row).to_contain_text("Afventer prøve")
    desk.screenshot(path=f"{out}/04-kortlaegning-fresh-gate.png")

    # --- Kortlægning: the ★ filter and the part markers (R13) ----------------
    tools_star = desk.get_by_label("Kun vigtige ★")
    tools_star.check()
    names = desk.get_by_role("row").filter(has_text=re.compile(r"Vinduespartier|Stålspær|Indvendige døre"))
    expect(names).to_have_count(3)
    vindue = desk.get_by_role("row").filter(has_text="Vinduespartier, aluminium")
    vindue.get_by_role("button", name="Fold ud").click()
    part_row = desk.get_by_role("row").filter(has_text="RX-008 · Office Zone")
    expect(part_row.get_by_role("img", name="Vigtig")).to_be_visible()
    expect(part_row.get_by_role("img", name=re.compile(r"^Note: Fuger revnede"))).to_be_visible()
    desk.screenshot(path=f"{out}/05-kortlaegning-star.png")

    # --- Miljø: where P-04 was taken (R8) ------------------------------------
    desk.goto(f"{base}/miljoe?sample=4")
    taken = desk.get_by_role("link", name="Udtaget ved RX-008 · Office Zone")
    expect(taken).to_be_visible()
    taken.click()
    expect(desk).to_have_url(re.compile(r"/kortlaegning\?type=6$"))

    # --- Sager (R1–R3) -------------------------------------------------------
    desk.get_by_role("link", name="← Alle sager").click()
    expect(desk).to_have_url(re.compile(r"/sager$"))
    card = desk.get_by_role("list", name="Sager").get_by_role("link")
    expect(card).to_have_count(1)
    for text in ("komponenter", "bevaring/genbrug", "til gennemsyn", "Gennemgang"):
        expect(card).to_contain_text(text)
    desk.screenshot(path=f"{out}/06-sager.png")
    card.click()
    expect(desk).to_have_url(re.compile(r"/$"))
    expect(desk.get_by_text("Cirkularitetsoversigt")).to_be_visible()

    busy = [s for s in statuses if s[0] == 503]
    assert not busy, busy
    browser.close()
print("flow OK")
```

The staleness block asserts one claim: **no `GET /api/v1/survey` leaves between the held `POST` and its release.** Kortlægning's spinner (`<Spinner label="Indlæser kortlægning…">`, `role="status"`) is on screen the whole time, because its first load is waiting on `appWriteChain.idle()`. On-site's own re-read after the POST is part of the same queued task, so it, too, only leaves after the release.

Run it on a fresh plain seed. `dev_env.sh` serves a copy, so the flow never dirties `$SP/p6-demo.rux`:

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/p6-demo.rux" 8426 5179
mkdir -p "$SP/shots/p6-flow"
PLAYWRIGHT_BROWSERS_PATH="$BROWSERS" nix shell --impure --expr 'let p = import (builtins.getFlake "nixpkgs") {}; in p.python3.withPackages (ps: [ ps.playwright ])' --command python3 "$SP/phase6_flow.py" http://localhost:5179 "$SP/shots/p6-flow"
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

Expected: `flow OK`. Open the six PNGs with Read. A failing `expect` or `assert` names the broken step. Fix the code, not the script, unless the script contradicts this plan.

**Expected values, from the plain seed:**
- **The walk:** Office Zone holds RX-004, RX-008, RX-011, RX-013 and RX-014, so the stop after RX-008 is RX-011.
- **RX-002:** its type, Betonsøjler (2), has no sample. Registering one turns it `Afventer prøve`; that is what the staleness check reads.
- **The ★ filter:** it leaves Stålspær (5) and Indvendige døre (9), which the seed stars, plus Vinduespartier (6), now starred through RX-008. All three are in the queue tab.
- **Sample codes:** the seed's samples end at P-03, so the first new sample is P-04.

No commit (verification only).

---

### Task 17: Docs

**Files:**
- `docs/design/gui-kortlaegning-redesign.md`;
- `apps/rux/frontend/README.md`;
- `.claude/skills/design-studio/references/reusex-frontend.md`.

- [ ] **Step 1: Spec** (`docs/design/gui-kortlaegning-redesign.md`):
  - **§ Source:** after the sentence ending "rendered the same way from the artifact.", add: "`onsite.png` was added in Phase 6, rendered at the prototype's phone size (390×844) from the artifact's `03 On-site` screen."
  - **§ Kortlægning domain model**, the `samples` bullet: replace `stage (planlagt | udtaget | sendt | svar), result (null | ren | forurenet).` with `stage (planlagt | udtaget | sendt | svar), result (null | ren | forurenet), and — from schema v24 — part code: the bygningsdel it was taken at, set when it is registered on site (not a foreign key; it may outlive the part).`
  - **New § Sager and On-site screens (the prototype, component by component)**, after § Overblik, Rapport and Indberetning screens and before § Phases. Summarise this plan's "The prototype, component by component", then state what v1 changes:
    - **Sager:**
      - one card for the project the server was started with, linking to Overblik;
      - a plan render as its thumb (the striped status thumb when the render fails);
      - the survey's figures, and a status derived from the summary and the fractions (`Kladde`, `Gennemgang`, `Klar til indberetning`, `Gennemgået`);
      - the registration date in place of the deadline, which is not stored;
      - no `+ Nyt projekt` — instead an `Åbn en anden sag` panel with `rux -p <fil>.rux gui` and the `--bind`/`--allow-origin` recipe for a phone, with its no-authentication warning;
      - the prototype's top tabs are not built; `← Alle sager` and an `On-site` sidebar entry replace them.
    - **On-site:**
      - a walk through the stored bygningsdele (rooms in Danish order, then code; `?del=RX-###`);
      - the part's best sensor-frame photo with the reticle on its instance in place of a live camera;
      - the sheet writes ★ and the note to the part and registers a sample there (`POST /samples` with `part_code`, `stage: udtaget`; the part's type is always linked);
      - `Tilføj ekstra foto` is not drawn (no blob storage);
      - `Videre → RX-###` moves on, replacing the history entry;
      - Kortlægning gains a `Kun vigtige ★` filter and ★/✎ part markers, and a Miljø card says `Udtaget ved RX-### · <rum>`.
    - **Shell:** below 900px the sidebar is a drawer behind `Menu`, the title bar drops its meta, and no route overflows at 390px.
    - **Writes:** one app-wide chain. The case screens' first loads wait for it; Rapport keeps its own.
  - **§ Phases:** after item 6, add "(done in Phase 6)".
  - **§ Out of scope:**
    - **Remove:**
      - the `AppShell` 390px item;
      - the cross-screen staleness item;
      - the cross-links-styled-differently item.
    - **Edit** the multi-project bullet to read: "Multi-project case list and switching, and creating a project from the GUI (`ruxd`, #265 Phase 6; `.github/issue-drafts/37-multi-case-list-ruxd.md`)."
    - **Add:**
      - "Extra photos from the phone, stored against a part and shown in the evidence panel — needs the same blob storage as the miljørapport upload (`.github/issue-drafts/36-onsite-part-photos.md`, with `22-miljoerapport-pdf-upload.md`)."
      - "Pairing or authentication for opening `rux gui` from a phone on the LAN; v1 documents `--bind` + `--allow-origin` and warns (`.github/issue-drafts/38-lan-pairing-auth.md`)."
      - "On-site during a live capture: the sheet as an interruption while the scan runs, with a detection chip from live segmentation (`.github/issue-drafts/39-onsite-live-capture.md`)."
      - "An offline outbox for On-site, so writes made without signal are kept and sent later (`.github/issue-drafts/40-onsite-offline-outbox.md`)."
      - "The client (bygherre) in case metadata, for the Sager card and the Overblik hero (`.github/issue-drafts/41-case-client-bygherre.md`)."
- [ ] **Step 2: Frontend README** (`apps/rux/frontend/README.md` § Layout):
  - in the `app/` entry, replace `serialQueue.ts (one promise\n│                 chain per page, tasks settle in commit order),` with `serialQueue.ts (a promise\n│                 chain whose tasks settle in commit order; idle() waits for them),\n│                 writeChain.ts (the one app-wide chain every page's writes\n│                 join and the case screens' first loads wait for),`, and change `links.ts (the\n│                 Kortlægning ↔ Miljø & prøver ↔ Overblik deep links` to `links.ts (the\n│                 Kortlægning ↔ Miljø & prøver ↔ Overblik ↔ On-site deep links`;
  - in the `components/` entry, add `sager/ (CaseCard), onsite/ (CaptureStage, CaptureSheet, PartPicker)` after `indberetning/ (FractionTable)`, and `controls.module.css (buttons, fields and the crossLink every cross-screen link composes)` in place of `controls.module.css (buttons and fields)`;
  - add two pure-module entries after `indberetning/`:

```
├── sager/        Pure module for Sager: model.ts (case status, card stats and
│                 text, the open-another and phone commands)
├── onsite/       Pure module for On-site: model.ts (the walk order, the
│                 picker, the detection chip, photo state and reticle, the
│                 sample body and its toast)
```

  - in the `routes/` entry, add `SagerPage, OnsitePage` after `IndberetningPage`;
  - after the line `lives at \`/projektdata\` now (a \`Værktøjer\` entry).`, add: "`/sager` lists the one open case; `/on-site` is the phone sheet. Below 900px the sidebar is a drawer. To use On-site from a phone, start `rux gui --bind <LAN-IP> --allow-origin http://<LAN-IP>:<port>` (no authentication — trusted networks only)."
- [ ] **Step 3: design-studio reference** (`.claude/skills/design-studio/references/reusex-frontend.md`):
  - add `sager/  CaseCard — Sager's case card` and `onsite/  CaptureStage, CaptureSheet, PartPicker — On-site's phone sheet` to the `components/` map, after the `indberetning/` line;
  - add the `sager/` and `onsite/` pure modules to the map, worded as in the README;
  - add `SagerPage, OnsitePage` to the routes line;
  - add the rules:
    - "Page writes join the app-wide `appWriteChain` (`useMutationQueue`); a case screen's first load starts with `appWriteChain.idle()`. Only a page whose writes change no survey state and run long (Rapport) passes `scope: 'page'`."
    - "A link to another case screen composes `crossLink` from `controls.module.css` — never a local link colour."
    - "Below 900px the shell's sidebar is a drawer (`AppShell`, `Sidebar`, `TitleBar`); a new case screen must not overflow `<main>` at 390px — wrap it or scroll it inside its own panel."
  - in the seed paragraph, replace `--varied\` also seeds three report versions (v1 and v2 drafts, v3\ncomplete) for Rapport.` with `--varied\` also seeds three report versions (v1 and v2 drafts, v3\ncomplete) for Rapport, a ★ part with a note (RX-014) and P-06 taken at RX-013.`
- [ ] **Step 4: Check and commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
nix develop --command reuse lint
git add docs/design/gui-kortlaegning-redesign.md apps/rux/frontend/README.md .claude/skills/design-studio/references/reusex-frontend.md
git commit -m "docs(gui): Sager and On-site screens; samples' part code; the responsive shell; follow-ups" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 18: Follow-up drafts and the direction changelog

**Files:**
- Create `.github/issue-drafts/36-onsite-part-photos.md`, `37-multi-case-list-ruxd.md`, `38-lan-pairing-auth.md`, `39-onsite-live-capture.md`, `40-onsite-offline-outbox.md`, `41-case-client-bygherre.md`.
- Delete `.github/issue-drafts/25-appshell-390px-overflow.md`, `30-cross-screen-staleness.md`, `32-cross-link-styling-inconsistency.md` (resolved here).
- Modify `docs/DIRECTION.md`.

The drafts are **not** filed on GitHub. They follow the existing drafts' format (`title:`, `labels:`, `## Problem`, `## Proposed fix`, `category=… estimate=…`).

- [ ] **Step 1: Drafts.**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6/.github/issue-drafts
cat > 36-onsite-part-photos.md <<'DRAFT'
title: On-site: add extra photos to a bygningsdel from the phone
labels: gui, enhancement, backend, frontend

## Problem
The prototype's On-site sheet has `＋ Tilføj ekstra foto`. Phase 6 does
not draw it (R9): the spec's Phase 6 line names ★, note and sample only,
and a photo needs storage the project does not have — the same gap that
keeps Miljø & prøver's `Upload miljørapport (PDF)` undrawn (draft 22).
Today the only photos of a part are the sensor frames its instance was seen
in, which may not show what the surveyor stopped for (a label, a crack, a
fixing).

## Proposed fix
- [ ] Design the attachment storage once for both this and draft 22: a
      `part_photos` table (part code, blob, content type, taken_at), chunked
      like `report_pdfs` if a photo can be large, plus a size cap.
- [ ] `POST /api/v1/survey/parts/{code}/photos` (raw `image/jpeg` body, as
      the thumbnail PUT does) and `GET …/photos`, `GET …/photos/{id}`;
      openapi entries for all three.
- [ ] Draw `＋ Tilføj ekstra foto` on the sheet with
      `<input type="file" accept="image/*" capture="environment">`, through
      `useMutationQueue`; downscale on the client before upload.
- [ ] Show the photos in Kortlægning's evidence panel (Foto tab: stored
      photos first, then the best sensor frame) and in the edit dialog's
      photo strip, and fill the child rows' photo count the spec still owes.

category=I/O estimate=2d
DRAFT
cat > 37-multi-case-list-ruxd.md <<'DRAFT'
title: Sager: list, switch and create cases through ruxd
labels: gui, enhancement, backend, frontend

## Problem
`rux gui` serves one `.rux`, so Phase 6's Sager screen (`/sager`) shows that
project as its single card and a panel saying how to open another
(`rux -p <fil>.rux gui`). The prototype shows four cases with statuses that
only a server deployment has (`Scanning i gang`, `Afventer upload`) and a
`+ Nyt projekt` button. The spec assigns multi-project listing and switching
to `ruxd` (#265's own Phase 6 — a different numbering from the redesign's).

## Proposed fix
- [ ] A `GET /api/v1/cases` contract (id, name, address, the survey summary
      figures the card shows, a status, a thumbnail URL) that `ruxd`
      implements and `rux gui` answers with its one project, so the frontend
      has one code path.
- [ ] Case switching in the shell (the selected case scopes every other
      route's requests), and `+ Nyt projekt` creating one on the server.
- [ ] Map `ruxd`'s capture/upload job states onto the card's status pill,
      next to Phase 6's derived Kladde / Gennemgang / Klar til indberetning /
      Gennemgået.
- [ ] The card grid already uses the prototype's `auto-fill` columns, so a
      longer list needs no layout change.

category=CLI estimate=1w
DRAFT
cat > 38-lan-pairing-auth.md <<'DRAFT'
title: Open rux gui from a phone without disabling the origin policy by hand
labels: gui, enhancement, backend, security

## Problem
On-site is a phone screen, but `rux gui` binds loopback and refuses any
non-loopback `Origin` (SecurityMiddleware). Phase 6 (R10) documents the
workaround — `--bind <LAN-IP> --allow-origin http://<LAN-IP>:<port>` — and
warns that the server has no authentication. Anyone on that network can then
read the project and start pipeline stages. Relaxing the origin check to
"same as Host" is not an option: under DNS rebinding the attacker chooses
the Host.

## Proposed fix
- [ ] A pairing flow: `rux gui --lan` binds the LAN address, prints a
      one-time code (and a QR code in the terminal), and the phone exchanges
      it for a session token; every API request and the `/events` upgrade
      then require the token.
- [ ] Keep loopback requests token-free, so the desktop flow is unchanged.
- [ ] Derive the allowed origin from the bound address, so `--allow-origin`
      is no longer needed for the phone.
- [ ] Replace Sager's recipe panel with "Åbn på telefon" showing the code.

category=CLI estimate=3d
DRAFT
cat > 39-onsite-live-capture.md <<'DRAFT'
title: On-site during a live capture
labels: gui, enhancement, vision

## Problem
The prototype frames On-site as "en afbrydelse, ikke listen": the scan is
running and the surveyor stops only to mark something. `rux gui` has no live
capture, so Phase 6 (R6) builds On-site as a walk through the parts that a
finished scan produced, with the best stored sensor frame in place of the
camera and the type's stored AI confidence in the detection chip.

## Proposed fix
- [ ] Decide where live capture runs (a phone app feeding RTABMap, or a
      `ruxd` ingest) and how its frames reach the project while scanning.
- [ ] A live detection stream (SAM3 on the incoming frames) that names the
      object under the reticle, so the chip says what is being looked at now.
- [ ] Attach ★/notes/samples made during capture to the instance once
      `rux create instances` and `rux create survey` have run, by position.

category=Vision estimate=2w
DRAFT
cat > 40-onsite-offline-outbox.md <<'DRAFT'
title: On-site keeps writes made without signal
labels: gui, enhancement, frontend

## Problem
On-site is used in basements, plant rooms and stairwells, where the phone
loses the network. A ★, note or sample registered then fails, and the
toast says so. The note draft stays in its field until the next blur, but
moving on with `Videre →` remounts the sheet (R6) and the unsent draft is
gone. Nothing is queued for later.

## Proposed fix
- [ ] A persisted outbox (IndexedDB) behind `useMutationQueue` for On-site's
      three writes, replayed in order when the connection returns, with a
      visible "n ændringer venter på forbindelse" line.
- [ ] Idempotency for the sample POST (a client-generated key), so a replay
      after a lost response does not register the sample twice.
- [ ] A conflict rule for a note edited both on the phone and in Kortlægning
      meanwhile (last write wins, with a toast naming the overwritten text).

category=I/O estimate=3d
DRAFT
cat > 41-case-client-bygherre.md <<'DRAFT'
title: Store the client (bygherre) in case metadata
labels: gui, enhancement, backend, frontend

## Problem
The prototype's Sager card reads `2760 Måløv · Bygherre: KBH Ejendomme A/S`.
`ProjectInfo` has no client field, so Phase 6 (R2) shows the address and
`udarbejdet af <organisation>` (the surveying firm, which is not the client).
Draft 24 covers the other missing case identity fields (BFE, case number,
MRK, deadline); the client was not in it.

## Proposed fix
- [ ] Add `client` to the project metadata schema, `PATCH /projects/{id}`
      and `rux set`/`rux get`, together with draft 24's fields (one schema
      bump for all of them).
- [ ] Add it to `ProjectMetaForm`, Overblik's hero line and the Sager card's
      sub line (`cardSubline`), as `Bygherre: <name>`.

category=I/O estimate=4h
DRAFT
ls 3[6-9]-*.md 4[01]-*.md
```

Expected: the six files are listed.

- [ ] **Step 2: Retire the drafts this phase resolves.** Drafts 25 (AppShell 390px), 30 (cross-screen staleness) and 32 (cross-link styling) are fixed by Tasks 8, 6 and 7. They were never filed, so delete them rather than letting them be filed for a fixed problem. `filed/` holds drafts that *were* filed, so they do not belong there.

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
git rm .github/issue-drafts/25-appshell-390px-overflow.md .github/issue-drafts/30-cross-screen-staleness.md .github/issue-drafts/32-cross-link-styling-inconsistency.md
```

- [ ] **Step 3: Direction changelog.** In `docs/DIRECTION.md`, directly under `## Direction changelog` and its blank line, insert:

```markdown
- **2026-10-02** — **GUI application (#265): the prototype-v2 redesign is
  complete — Phases 1–6 have landed.** Phase 6 added Sager (the single-project
  case list `rux gui` can serve) and On-site (the phone sheet that writes ★, a
  note and a sample against a bygningsdel; schema v24 records the part a sample
  was taken at), made the shell responsive below 900px, and put every screen's
  writes on one app-wide chain. What remains are follow-ups, not phases: the
  multi-case list and switching belong to `ruxd` (#265's own Phase 6, a
  different numbering), and photo capture waits for blob storage — see the
  spec's Out of scope list and `.github/issue-drafts/36`–`41`.

```

- [ ] **Step 4: Check and commit**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase6
nix develop --command reuse lint
git add .github/issue-drafts/36-onsite-part-photos.md .github/issue-drafts/37-multi-case-list-ruxd.md .github/issue-drafts/38-lan-pairing-auth.md .github/issue-drafts/39-onsite-live-capture.md .github/issue-drafts/40-onsite-offline-outbox.md .github/issue-drafts/41-case-client-bygherre.md docs/DIRECTION.md
git commit -m "docs: GUI redesign phases 1–6 complete; Phase 6 follow-up drafts" -m "Drafts 36–41 hold what Phase 6 leaves out (photos, the ruxd case list, LAN pairing, live capture, an offline outbox, the client field). Drafts 25, 30 and 32 are deleted: this phase fixes them." --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

The drafts carry no SPDX header, like the existing ones: `REUSE.toml`'s `**/*.md` annotation covers them.

## Phase exit criteria

- **Backend:**
  - `ctest --parallel` is green for every test this plan adds or touches: `Samples_*`, `ReportPdfs_*`, `SurveySchema_MigratesFromV21`, `CreateSample_*`, `SampleEndpoints_*`, `SamplesJson_*`;
  - so are the existing `ProjectDb*`, `Survey*`, `RunningServer_*` and `gui_api_contract_parses`;
  - `scripts/check-openapi.py` passes;
  - `rux gui --help` prints the phone note.
- **Frontend:**
  - `test`, `typecheck` and `build` pass;
  - token lint is clean on every new or changed `.module.css`/`.tsx`;
  - `reuse lint` is compliant;
  - `grep -rn ALL_CASES_PENDING apps/rux/frontend/src` prints nothing.
- **Shell (R5):**
  - `phase6_shots.py` passes its overflow assertions at 390px and 768px in both themes;
  - the 1440 shots of the Phase 5 screens are unchanged;
  - the flow's drawer block passes.
- **Prototype match:** the shots of `/sager` (desktop) and `/on-site` (phone) match `sager.png` and `onsite.png` structurally on the plain seed. Deviations are only those ruled in R1–R10.
- **Flow:** `phase6_flow.py` prints `flow OK`. That includes:
  - the untouched-blur and Esc ordering proofs;
  - the old-part note on `Videre`;
  - the exact sample POST body;
  - the held-POST staleness check;
  - no 503.
- **Follow-ups drafted, not filed:** `.github/issue-drafts/36`–`41`. Drafts 26 (EditDialog Esc) and 31 (selection in the URL) stay open (R14). Drafts 25, 30 and 32 are deleted, because this phase resolves them.
- **Direction:** `docs/DIRECTION.md` carries the 2026-10-02 changelog line saying phases 1–6 are complete.

## Self-review

- **Spec coverage:**
  - "case list (single-project)": Tasks 9–10 (R1–R4), with the spec's own wording, a card plus how to open another, and the auto-fill grid for a longer list;
  - "phone capture sheet writing ★ / note / sample against a bygningsdel": Tasks 11–13 (R6, R7). ★ and note are written to the part (`PATCH /survey/parts/{code}`). The sample is written against the part through `part_code` (Tasks 2–3, R8), and its type link keeps the spec's gate on the type;
  - the out-of-scope items the spec hands to Phase 6: the AppShell 390px item (Task 8, R5), cross-screen staleness (Task 6, R11), cross-link styling (Task 7, R12);
  - multi-project switching stays out (R1, draft 37), as the spec says.
- **The prototype read:**
  - both screens were read from the artifact's source, not only from pixels: the Sager markup and CSS, the On-site sheet's four rows and its single working toggle, and the side notes;
  - `onsite.png` was rendered at 390×844 and committed with a `.license` sidecar;
  - every deviation has a ruling: the image, the top tabs, `+ Nyt projekt`, `Frist`, `Bygherre`, the photo button, "— registreret", the side notes.
- **Backend gaps found, each with a task before its screen:**
  - a sample could not record the bygningsdel it was taken at (Task 2, schema v24);
  - `POST /samples` could neither take a part nor start at `udtaget` (Task 3);
  - the seed had no On-site data for the other screens (Task 4).

  Photo capture is the gap deliberately **not** closed (R9, draft 36). No case-list endpoint is added: one project needs only `GET /project`, `/survey/summary` and `/survey/fractions` (draft 37 proposes the contract for `ruxd`).
- **Placeholders:** none. Every path, string and command is literal. Three steps carry a conditional, and each names the exact action:
  - Task 5 Step 4: another `Sample` literal flagged by `typecheck`;
  - Task 11 Step 3: a Node without ICU;
  - Task 16 Step 1: an overflow assertion naming the element to fix.
- **Type consistency:**
  - `SampleRecord::part_code` (`std::optional<std::string>`) is serialised by `sample_json` as `opt(…)` → `string | null`, and typed `part_code: string | null` in `Sample` and nullable-required in openapi;
  - `SampleCreate.stage` is `Extract<SampleStage, 'planlagt' | 'udtaget'>`, which matches the server's 400 rule;
  - `onsiteSampleBody` returns `SampleCreate`;
  - `PhotoLookup` matches the shape of `EvidencePanel`'s `FrameLookup` without importing it;
  - `reticleBox` takes `ProjectSummary.sensor_frames` (optional `width`/`height`) directly;
  - `Filters.starred` is added to `NO_FILTERS`, so every `{ ...NO_FILTERS, … }` literal still type-checks;
  - `useMutationQueue`'s `scope` defaults to `'app'`, so every existing caller joins the chain without an edit, and only Rapport opts out.
- **Global-constraint lessons:**
  - one serial queue: `useMutationQueue`, now app-wide (R11);
  - field commits never gated on busy: On-site's note calls `onCommit` unconditionally; only ★ and `Registrér prøve` read `busy`, and `Videre`/the picker are navigation;
  - an untouched blur never commits: `useTextDraft`/`textCommit`, proven by the flow's ordering assertion;
  - Esc: the note reverts and parks focus on the sheet; the sample form cancels; the drawer closes to Menu. All three go through `editorKeys`/`fieldKeys` or a handler on the element itself, never a document listener;
  - isField/isControl: `kindOf` in the sample form; Kortlægning's new checkbox is a `choice` field, so its shortcuts skip it;
  - tokens only, with lint in every UI task; `tokens.css` untouched; the only literals are the `900px` breakpoint (one per file, commented) and data-computed percentages;
  - Danish copy, given verbatim;
  - SPDX on every new file, including `onsite.png.license`;
  - pure logic in vitest under Node (`sager/model`, `onsite/model`, `takenAt`, the star filter, `serialQueue.idle`);
  - shared CSS through `composes` (`crossLink`, `btnPrimary`, `input`, `panel`, `notice`, `title`, `footnote`);
  - no fixed sleeps: the scripts wait on `expect`/`expect_response`/`expect_request`; the one held request is released explicitly;
  - `parseServerUtc`: no server timestamp is shown; `survey_date` goes through `danishDate` (R2);
  - `vocab.ts`: stage words come from `STAGE_LABEL`/`statusPill`;
  - CSV formula guard: no CSV is added.
