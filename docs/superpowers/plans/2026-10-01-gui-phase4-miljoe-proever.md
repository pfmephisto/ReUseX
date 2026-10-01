<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# GUI Phase 4 — Miljø & prøver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Load the project skill `design-studio` (`.claude/skills/design-studio/SKILL.md`) before touching any `.tsx`/`.css`.

**Goal:** Build the prototype-v2 **Miljø & prøver** screen at `/miljoe` on the Phase 2 sample API: one card per environmental sample with its stage chain (planlagt → udtaget → sendt → svar), "Næste trin →", result entry (Ren / Forurenet), registering a new sample, editing a sample and linking/unlinking it to survey types. Also: a live sidebar entry with the pending-samples badge, approval-gate feedback after every sample change, and two-way links between Kortlægning and Miljø & prøver.

**Architecture:** As in Phase 3, behaviour that can be pure is pure and unit-tested in Node:
- `src/miljoe/model.ts`: the stage chain, the card's action, the patches, link toggling, the gate-change diff and the toast copy.
- `src/app/links.ts`: cross-screen hrefs and query parsing.
- `src/app/keyTargets.ts`: what a key event's target is.
- `src/app/serialQueue.ts`: the ordered mutation chain.

The chain was inlined in `KortlaegningPage` and is now shared through `src/app/useMutationQueue.ts`. Kortlægning is migrated onto it without changing its behaviour. Components in `src/components/miljoe/` are presentational. `src/routes/MiljoePage.tsx` owns the state: samples, survey types, the link drafts, the editor and the create form. After each sample mutation, the same queued task re-reads `GET /survey`, so every miljøstatus shown comes from the server and is never re-derived. Each mutation then calls the shell's `refresh()`, so both sidebar badges follow. No backend change is needed (Task 1 verifies this against a running server).

**Tech Stack:** React 19, react-router-dom 7, TypeScript, CSS Modules, vitest (Node, no DOM). The Phase 2 client is used unchanged: `api.samples`, `api.createSample`, `api.patchSample`, `api.setSampleLinks`, `api.deleteSample`, `api.survey`. Also sqlite3 CLI for the dev seed and Playwright via the design-studio scripts.

**Spec:** `docs/design/gui-kortlaegning-redesign.md` (§ Kortlægning domain model, § Phases item 4, § Out of scope — sample-line link)

**Prototype:** `docs/gui/images/prototype-v2/miljoe.png`

## The prototype, component by component

`miljoe.png` (1440×1000, light theme):

- **Chrome:** the Phase 1 navy title bar and sidebar. **Miljø & prøver** is the active entry. Its count badge reads **2**, a neutral (not "hot") badge, unlike Kortlægning's accent **7**. Two samples, P-01 (sendt) and P-03 (udtaget), have not reached *svar*. These are the samples that can still hold a type at *afventer prøve*.
- **View head:**
  - left: `MILJØ & PRØVER` in the display face and the sub line `Prøver styrer miljøstatus på de koblede bygningsdele`;
  - right: a ghost button `Upload miljørapport (PDF)` and a filled primary button `+ Ny prøve`.
- **Sample cards:** a vertical stack of white raised cards with a hairline border, the Phase 3 panel look. Each card has four rows:
  1. **Title row:**
     - `P-01 · PCB i fugemasse` in the display face, bold;
     - a stage/result pill: `Sendt til lab` and `Udtaget` in the neutral *wait* tone, `Forurenet` in the *crit* tone;
     - right-aligned muted text `Koblet: Vinduespartier, aluminium`.
  2. **What row:** the muted line saying what was sampled and where, e.g. `Fugemasse omkring vinduespartier`.
  3. **Stage chain:**
     - the four steps `Planlagt — Udtaget — Sendt til lab — Svar modtaget`, each a small dot and a bold label, joined by faint em-dashes;
     - done steps have a filled dot in the good ink (dark green) with green text;
     - the current step has an accent-filled dot with accent text;
     - future steps have a hollow ring with faint text.
  4. **Action row, depending on the sample:**
     - *sendt* (P-01): primary `Registrér svar: Ren` and ghost `Registrér svar: Forurenet`;
     - *udtaget* (P-03): a ghost `Næste trin →`;
     - *svar* with a result (P-02): no buttons, only the small note `Svar registreret — miljøstatus opdateret på 1 type(r) i kortlægningen.`
- **Footnote:** faint small text under the cards: "Skitse-note: Svar fra laboratoriet flipper automatisk miljøstatus … Senere: direkte lab-integration (fx Milva) …". This is a designer's sketch note. The plan replaces it with a factual footnote (Ruling R5).
- **Not in the prototype:**
  - no list/detail split; the card *is* the detail;
  - no editing of title/what;
  - no link editor;
  - no delete;
  - no keyboard bar.

This plan adds those affordances inside the card with the least extra chrome: a `Rediger` text button after the `Koblet:` line opens an inline editor.

## Rulings (spec silent or contradicted by the code)

- **R1 — Samples link to survey types, not passports.**
  - The spec's domain model says `sample_links — many-to-many sample ↔ passport`.
  - The Phase 2 implementation links samples to **survey types**: `sample_links(sample_id, type_id)`, `Sample.type_ids`, `SurveyType.sample_ids`, `PUT /samples/{id}/links {type_ids}`.
  - That is the only reading consistent with the rest of the spec: miljøstatus and the approval gate are properties of a *type*, and the prototype's `Koblet:` names types.
  - Phase 4 builds on types, and Task 12 corrects the spec.
- **R2 — No backend task.** Everything the screen needs exists and is verified in Task 1:
  - create with links: `POST /samples {title, what, type_ids}`;
  - stage/result edits with the result-only-at-svar rule: `PATCH /samples/{id}` → `core::update_sample_checked` → 422;
  - link replacement: `PUT /samples/{id}/links`, with 404 on an unknown type;
  - delete: `DELETE /samples/{id}`, links cascade;
  - the pending count: `SurveySummary.pending_samples`.

  `PATCH /survey/types/{id}` cannot edit `sample_ids`. That is fine, because links are edited from the sample side, and this is the only screen that edits them.
- **R3 — Recording a result is one PATCH `{stage: 'svar', result}`.** The prototype offers `Registrér svar: …` while the sample is still at *sendt*. The backend checks the rule on the *merged* state (`update_sample_checked`), so a combined patch is legal. A result before *svar* is otherwise refused with 422. The UI never produces *svar without a result*, which `environment_status()` derives as *ren (prøvesvar)*: from *sendt* there is no `Næste trin`, only the two result buttons.
- **R4 — "Fortryd svar" returns the sample to *sendt* with no result** (`{stage: 'sendt', result: null}`), not to *svar* without a result. Per R3, the latter would silently count as clean and un-gate the types. This is the only backward step v1 offers. Arbitrary stage rewinds are a follow-up.
- **R5 — The sketch footnote is replaced by fact:** `Et prøvesvar opdaterer miljøstatus på alle koblede typer i kortlægningen — én prøve kan afklare mange bygningsdele.` Lab integration is already a spec follow-up.
- **R6 — `Upload miljørapport (PDF)` is not built.** There is no endpoint to store a lab report, and an inert button that does nothing is worse than none (cf. `navigation.ts`, which renders unbuilt places with a stated reason). The head shows only `+ Ny prøve`. The upload becomes a follow-up issue, which needs storage + an endpoint.
- **R7 — The badge counts `pending_samples`** (samples whose stage is not *svar*). This matches the prototype's **2** on its data (P-01, P-03). It also matches the spec's rule that a sample before *svar* is what holds a type at *afventer prøve*. The badge stays neutral (`Sidebar` styles only `reviewQueue` as hot), as in the prototype.
- **R8 — Rejected types:**
  - The link picker offers non-rejected types, plus any rejected type the sample is already linked to, marked `Afvist`, so it can be unlinked.
  - On the card, a rejected linked type is plain text `<name> (afvist)`, not a link: Kortlægning hides rejected types, so the link would land on nothing.
- **R9 — Deep links both ways:**
  - `/miljoe?sample=<id>` scrolls to and highlights a card;
  - `/miljoe?ny=<typeId>` opens the create form with that type pre-linked;
  - `/kortlaegning?type=<id>` selects that type in the tab it lives in.

  The spec's follow-up only asks for the first. The other two close the loop the approval gate creates, "this type needs a sample" → register it → back to the type, at the cost of two pure parsers.

## Global Constraints

- SPDX header on every new file:
  - TS/TSX: `// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `// SPDX-License-Identifier: GPL-3.0-or-later`;
  - CSS: the `/* … */` block form;
  - shell: `#`;
  - Markdown: the `<!-- … -->` block.
- **Tokens only.** No literal colour, radius, spacing or font-size in `.module.css`, inline `style`, or TS strings/constants; only `var(--…)`. 1px/2px hairline borders and focus outlines are the allowed exception, as in Phase 3. **Never edit `src/tokens.css`.** Check every changed CSS file with `python3 .claude/skills/design-studio/scripts/token_lint.py <files> --tsx`.
- Token roles:
  - pills use `<Pill tone>` (`--tone-*-bg/-ink`);
  - filled primary buttons use `--color-accent-deep` + `--color-on-accent`;
  - field labels use `--font-size-2xs`, uppercase, `--tracking-caps`, `--color-text-muted`;
  - headings use `--font-display`.
- All UI copy is Danish. Use the prototype's words where it has them; otherwise use the strings given in this plan verbatim.
- **Mutations go through one serialised promise chain per page** (`useMutationQueue`), so requests reach the server, and their responses are folded in, in the order they were made.
- **Field commits are never dropped and never gated on `busy`.** These are title/what blurs and link checkboxes; a commit made during a request waits its turn on the chain. **Only buttons are gated on `busy`:** `Næste trin →`, `Registrér svar: …`, `Fortryd svar`, `Registrér prøve`, `Slet prøve`.
- **An untouched field blur never commits.** Draft equal to the current value (after trim) → nothing is sent (`textCommit`). Esc in a text field reverts the draft and blurs *without* committing.
- **Keyboard handlers classify their target** with `src/app/keyTargets.ts` (`kindOf` / `isField` / `isControl`). Esc in a text field reverts that field; Esc elsewhere in an editor closes it; Enter/Space on buttons, links and checkboxes keep their native activation. This page adds no global (document-level) shortcuts.
- Server state is never re-derived on the client. Miljøstatus, `sample_ids`, `type_ids` and pending counts always come from a response body or a re-read. The gate-change toast *compares* two server snapshots; it does not compute statuses.
- No DOM test environment may be added: vitest runs in Node (`vite.config.ts` `environment: 'node'`). Testable logic is a pure exported function. Components are verified by screenshot in both themes.
- Frontend commands are run from the worktree root: `npm --prefix apps/rux/frontend test|run typecheck|run build`.
- Work happens in `/home/mephisto/repos/ReUseX/.worktrees/gui-phase4` (branch `gui-phase4-miljoe`). **Never touch `/home/mephisto/repos/ReUseX/build`.**
  - The worktree builds its own `rux` with the CUDA-workaround configure line in Task 1 (inside `nix develop`).
  - Scratch projects live in `SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad`. `$SP/corridor-clouds.rux` is the cloud-bearing source project Phase 3 used.
- Every commit carries the session trailers, written out in each command below. Never `--no-verify`.

## Review Focus

- **Result entry and the stage rule (R3/R4):**
  - `Registrér svar: …` sends exactly `{stage: 'svar', result}`;
  - `Fortryd svar` sends exactly `{stage: 'sendt', result: null}`;
  - no UI path leaves a sample at *svar* without a result;
  - a 422 shows the Danish toast and changes nothing on screen.
- **Approval-gate feedback after every sample mutation:** this covers advance, result, undo, link toggle, create and delete.
  - The survey is re-read *inside the same queued task*.
  - The toast names the types that became approvable, contaminated or newly blocked (an approved type newly *afventer* gets the "er godkendt, men afventer nu prøvesvar" warning).
  - Both sidebar badges refresh.
  - Navigating to Kortlægning shows `Godkend mængde ✓` enabled for an un-gated type.
- **Rapid link toggles:** five quick checkbox clicks all reach the server in order. The final server state equals the last on-screen state, and the checkbox never flickers back to an intermediate server response. A failed PUT reverts the checkboxes to the server's state and toasts.
- **Drafts:** focusing and leaving title/what sends nothing; Esc reverts without sending; an emptied title reverts.
- **The Kortlægning refactor (Task 2) is behaviour-preserving:** the same order, toasts, 422 handling and `busy` semantics. `kortlaegning.page.test.ts` still passes with only its import path changed.
- **Deep links:** unknown or rejected ids, non-numeric query values and `?sample=` for a deleted sample degrade to the plain screen, with no error and no stuck spinner.

---

## File Structure

| File | Responsibility |
|---|---|
| `apps/rux/frontend/dev/seed-survey-demo.sh` (modify) | `--varied` adds P-04 (multi-link, ren) and P-05 (unlinked, planlagt) |
| `src/app/serialQueue.ts` (new) | ordered promise chain, pure |
| `src/app/useMutationQueue.ts` (new) | React wrapper: `busy`, `mutate`, 422 routing, `onSettled` |
| `src/app/keyTargets.ts` (new) | `targetKind` / `kindOf` / `isField` / `isControl` (moved out of KortlaegningPage) |
| `src/app/saveError.ts` (new) | `saveErrorMessage` (moved out of KortlaegningPage) |
| `src/app/links.ts` (new) | `sampleHref`, `newSampleHref`, `surveyTypeHref`, `parseMiljoeQuery`, `parseTypeQuery` |
| `src/miljoe/model.ts` (new) | stage chain, card action, patches, links, gate diff, toasts, editor keys, text commits |
| `src/miljoe/useTextDraft.ts` (new) | blur-commit draft with revert, for title/what |
| `src/test/surveyFixtures.ts` (new) | `surveyType()` / `sample()` builders for the Phase 4 tests |
| `src/test/serialQueue.test.ts`, `keyTargets.test.ts`, `links.test.ts`, `miljoe.model.test.ts` (new) | their contracts |
| `src/components/miljoe/StageChain.tsx` + css (new) | the four-step chain |
| `src/components/miljoe/LinkPicker.tsx` + css (new) | checkbox list of survey types with miljø pills |
| `src/components/miljoe/SampleCard.tsx` + css (new) | one sample: title row, what, chain, actions, inline editor |
| `src/components/miljoe/NewSampleForm.tsx` + css (new) | register a sample |
| `src/routes/MiljoePage.tsx` + css (new) | state, data flow, gate feedback, deep link |
| `src/app/App.tsx`, `src/app/navigation.ts`, `src/app/AppShell.tsx`, `src/test/navigation.test.ts` (modify) | route, live entry, pending-samples badge |
| `src/routes/KortlaegningPage.tsx`, `src/test/kortlaegning.page.test.ts` (modify) | use the shared queue/targets/saveError; `?type=` deep link |
| `src/kortlaegning/model.ts`, `src/test/kortlaegning.model.test.ts` (modify) | `initialViewFor` |
| `src/components/kortlaegning/DetailPanel.tsx`, `EditDialog.tsx` (modify); `SampleLine.tsx` + css (new); `src/test/kortlaegning.detailPanel.test.ts` (modify) | sample line becomes links to `/miljoe` |
| docs: `docs/design/gui-kortlaegning-redesign.md`, `apps/rux/frontend/README.md`, `.claude/skills/design-studio/references/reusex-frontend.md` | R1 correction, screen description, layout maps |

All `src/…` paths are under `apps/rux/frontend/`.

---

### Task 1: Worktree build, varied seed, and backend contract check

**Files:**
- Modify `apps/rux/frontend/dev/seed-survey-demo.sh` and `apps/rux/frontend/README.md` (§ Development).

**Interfaces:**
- Produces a worktree `build/apps/rux/rux`.
- Produces `seed-survey-demo.sh [--varied] <source.rux> <dest.rux>`:
  - without the flag, the seed is unchanged: 3 samples, pending 2, exactly the prototype;
  - with `--varied`, it also seeds P-04 (*svar*, *ren*, linked to the two approved types 7 and 10) and P-05 (*planlagt*, unlinked), for 5 samples, pending 3.
- Produces a recorded confirmation that the Phase 2 sample API behaves as R2/R3/R4 assume.

- [ ] **Step 1: Build `rux` in the worktree** (only the `rux` target; this is the CUDA-workaround configure line)

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase4
nix develop --command bash -c 'cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DCMAKE_CUDA_COMPILER=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9/bin/nvcc -DCUDAToolkit_ROOT=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9 -DCUDA_TOOLKIT_ROOT_DIR=/nix/store/p49i1vrhcaw5nf2r3bwgmwfz5x8zgb14-cuda-merged-12.9 && cmake --build build --target rux -j"$(nproc)"'
npm --prefix apps/rux/frontend ci
```

Expected: `build/apps/rux/rux` exists. `npm ci` completes.

- [ ] **Step 2: Add `--varied` to the seed.** In `apps/rux/frontend/dev/seed-survey-demo.sh`:
  1. Change the usage comment to `# Usage: seed-survey-demo.sh [--varied] <source.rux> <dest.rux>`, and add the comment line `# --varied adds P-04 (answered ren, linked to two approved types) and P-05 (planned, unlinked) for Miljø & prøver work.`
  2. Directly after the `command -v sqlite3 …` line, insert:

```bash
varied=0
if [[ "${1:-}" == "--varied" ]]; then varied=1; shift; fi
```

  3. Replace the final `echo "seeded demo survey into $dst"` with:

```bash
if [[ "$varied" -eq 1 ]]; then
sqlite3 "$dst" <<'SQL'
BEGIN;
INSERT INTO samples (id,code,title,what,stage,result) VALUES
 (4,'P-04','Asbest i eternitplader','Tagplader over Roof, prøve fra nordfaldet','svar','ren'),
 (5,'P-05','PAH i tagpap','Tagpap under trapezplader — endnu ikke udtaget','planlagt','');
INSERT INTO sample_links (sample_id,type_id) VALUES (4,7),(4,10);
COMMIT;
SQL
fi
echo "seeded demo survey into $dst"
```

- [ ] **Step 3: Check the contract against a running server** (a throwaway copy; every expected value is on the right)

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase4
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/miljoe-smoke.rux"
./build/apps/rux/rux -p "$SP/miljoe-smoke.rux" gui --port 8441 --no-browser & S=$!
for i in $(seq 1 30); do curl -sf -o /dev/null localhost:8441/api/v1/health && break; sleep 1; done
A=localhost:8441/api/v1; J='Content-Type: application/json'
py() { python3 -c "import json,sys; d=json.load(sys.stdin); print($1)"; }
curl -s $A/survey/summary | py 'd["pending_samples"]'                                              # 2
curl -s -o /dev/null -w '%{http_code}\n' -X PATCH -H "$J" -d '{"result":"ren"}' $A/samples/3          # 422
curl -s -X PATCH -H "$J" -d '{"stage":"svar","result":"ren"}' $A/samples/1 | py 'd["stage"], d["result"]'   # ('svar', 'ren')
curl -s $A/survey | py '{t["id"]: t["environment_status"] for t in d["types"]}[6]'                   # ren_proevesvar
curl -s -X PATCH -H "$J" -d '{"stage":"sendt","result":null}' $A/samples/1 | py 'd["stage"], d["result"]'   # ('sendt', None)
curl -s $A/survey | py '{t["id"]: t["environment_status"] for t in d["types"]}[6]'                   # afventer
curl -s -X PUT -H "$J" -d '{"type_ids":[6,9]}' $A/samples/1/links | py 'd["type_ids"]'                 # [6, 9]
curl -s -o /dev/null -w '%{http_code}\n' -X PUT -H "$J" -d '{"type_ids":[999]}' $A/samples/1/links    # 404
curl -s -X POST -H "$J" -d '{"title":"Røgprøve","type_ids":[2]}' $A/samples | py 'd["code"], d["stage"], d["type_ids"]'  # ('P-04', 'planlagt', [2])
curl -s -o /dev/null -w '%{http_code}\n' -X DELETE $A/samples/4                                       # 204
curl -s $A/survey | py '{t["id"]: t["sample_ids"] for t in d["types"]}[2]'                           # []
kill $S
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh --varied "$SP/corridor-clouds.rux" "$SP/miljoe-varied.rux"
./build/apps/rux/rux -p "$SP/miljoe-varied.rux" gui --port 8441 --no-browser & S=$!
for i in $(seq 1 30); do curl -sf -o /dev/null localhost:8441/api/v1/health && break; sleep 1; done
curl -s $A/samples | py 'len(d["samples"])'                     # 5
curl -s $A/survey/summary | py 'd["pending_samples"]'           # 3
kill $S
```

If any line differs, stop: R2 no longer holds. Write the missing piece as a backend task first, following `docs/superpowers/plans/2026-09-30-gui-phase2-survey-backend.md`: a library function in `core/survey_service`, a Catch2 test in `tests/unit/core/test_survey_service.cpp` / `tests/unit/rux_gui/test_gui_survey.cpp`, the route in `apps/rux/src/gui/survey.cpp` + `Server.cpp`, and `docs/gui/openapi.yaml`.

- [ ] **Step 4: README.** In `apps/rux/frontend/README.md`, after the Kortlægning seed paragraph, add: "For Miljø & prøver work, add `--varied` (`bash dev/seed-survey-demo.sh --varied <project.rux> /tmp/miljoe-demo.rux`). This seeds two extra samples, one answered and linked to two types and one planned and unlinked, on top of the prototype's three. Screenshots against `miljoe.png` use the plain seed."

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/dev/seed-survey-demo.sh apps/rux/frontend/README.md
git commit -m "chore(gui): --varied sample seed for Miljø & prøver development" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 2: Shared mutation queue, key targets and save errors (Kortlægning migrated)

**Files:**
- Create:
  - `src/app/serialQueue.ts`, `src/app/useMutationQueue.ts`, `src/app/keyTargets.ts`, `src/app/saveError.ts`;
  - `src/test/serialQueue.test.ts`, `src/test/keyTargets.test.ts`.
- Modify `src/routes/KortlaegningPage.tsx` and `src/test/kortlaegning.page.test.ts`.

**Interfaces:**

```ts
// serialQueue.ts
export interface SerialQueue { enqueue(task: () => Promise<void>): Promise<void> }
export function createSerialQueue(): SerialQueue;
// useMutationQueue.ts
export interface MutationQueueOptions { onError: (cause: unknown) => void; onSettled?: () => void }
export interface MutationQueue {
  busy: boolean;
  mutate: (run: () => Promise<void>, onUnprocessable?: (cause: ApiRequestError) => void) => void;
}
export function useMutationQueue(options: MutationQueueOptions): MutationQueue;
// keyTargets.ts
export type TargetKind = 'text' | 'choice' | 'control' | 'other';
export interface TargetLike { tagName: string; type?: string }
export function targetKind(t: TargetLike | null | undefined): TargetKind;
export function kindOf(target: EventTarget | null): TargetKind;
export function isField(target: EventTarget | null): boolean;   // text | choice
export function isControl(target: EventTarget | null): boolean; // control
// saveError.ts
export function errorMessage(cause: unknown): string;
export function saveErrorMessage(cause: unknown): string;
```

- [ ] **Step 1: Failing tests.** Create `src/test/serialQueue.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { createSerialQueue } from '../app/serialQueue';

function deferred() {
  let resolve!: () => void;
  const promise = new Promise<void>((r) => {
    resolve = r;
  });
  return { promise, resolve };
}

describe('serial queue', () => {
  it('runs tasks one at a time, in the order they were enqueued', async () => {
    const q = createSerialQueue();
    const log: string[] = [];
    const gate = deferred();
    const first = q.enqueue(async () => {
      log.push('a:start');
      await gate.promise;
      log.push('a:end');
    });
    const second = q.enqueue(async () => {
      log.push('b');
    });
    await Promise.resolve();
    await Promise.resolve();
    expect(log).toEqual(['a:start']);
    gate.resolve();
    await first;
    await second;
    expect(log).toEqual(['a:start', 'a:end', 'b']);
  });

  it('keeps going after a failed task and reports the failure to its caller', async () => {
    const q = createSerialQueue();
    const log: string[] = [];
    const bad = q.enqueue(async () => {
      throw new Error('boom');
    });
    const good = q.enqueue(async () => {
      log.push('after');
    });
    await expect(bad).rejects.toThrow('boom');
    await good;
    expect(log).toEqual(['after']);
  });
});
```

Create `src/test/keyTargets.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { isControl, isField, targetKind } from '../app/keyTargets';

const t = (tagName: string, type?: string) => ({ tagName, type }) as unknown as EventTarget;

describe('key targets', () => {
  it('classifies typing, choosing and activating targets', () => {
    expect(targetKind({ tagName: 'INPUT', type: 'search' })).toBe('text');
    expect(targetKind({ tagName: 'INPUT' })).toBe('text');
    expect(targetKind({ tagName: 'TEXTAREA' })).toBe('text');
    expect(targetKind({ tagName: 'input', type: 'checkbox' })).toBe('choice');
    expect(targetKind({ tagName: 'SELECT' })).toBe('choice');
    expect(targetKind({ tagName: 'BUTTON' })).toBe('control');
    expect(targetKind({ tagName: 'A' })).toBe('control');
    expect(targetKind({ tagName: 'INPUT', type: 'submit' })).toBe('control');
    expect(targetKind({ tagName: 'DIV' })).toBe('other');
    expect(targetKind(null)).toBe('other');
  });

  it('keeps the Kortlægning meaning of isField / isControl', () => {
    expect(isField(t('INPUT', 'text'))).toBe(true);
    expect(isField(t('SELECT'))).toBe(true);
    expect(isField(t('TEXTAREA'))).toBe(true);
    expect(isField(t('BUTTON'))).toBe(false);
    expect(isControl(t('BUTTON'))).toBe(true);
    expect(isControl(t('A'))).toBe(true);
    expect(isControl(t('DIV'))).toBe(false);
    expect(isField(null)).toBe(false);
  });
});
```

- [ ] **Step 2: Run → FAIL** with `npm --prefix apps/rux/frontend test -- serialQueue keyTargets`.

- [ ] **Step 3: Implement.** `src/app/serialQueue.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * One promise chain for a page's writes: each task starts only after every
 * task enqueued before it has settled, so requests reach the server — and
 * their responses are applied — in the order the user made them. A failing
 * task rejects its own promise but never stops the chain.
 */

export interface SerialQueue {
  enqueue(task: () => Promise<void>): Promise<void>;
}

export function createSerialQueue(): SerialQueue {
  // Never rejects: it is re-pointed at a settled-either-way copy of each task.
  let tail: Promise<void> = Promise.resolve();
  return {
    enqueue(task) {
      const result = tail.then(task);
      tail = result.then(
        () => undefined,
        () => undefined,
      );
      return result;
    },
  };
}
```

`src/app/useMutationQueue.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useRef, useState } from 'react';

import { ApiRequestError } from '../api/client';
import { createSerialQueue, type SerialQueue } from './serialQueue';

export interface MutationQueueOptions {
  /** Any failure that has no `onUnprocessable` handler (shown as a toast). */
  onError: (cause: unknown) => void;
  /** After every mutation, success or not — e.g. re-read the sidebar badges. */
  onSettled?: () => void;
}

export interface MutationQueue {
  /** True while any mutation is queued or in flight. Gates buttons only. */
  busy: boolean;
  mutate: (run: () => Promise<void>, onUnprocessable?: (cause: ApiRequestError) => void) => void;
}

/**
 * A page's writes on one serial chain (`createSerialQueue`). Nothing is
 * dropped: a field commit made while another request is in flight waits its
 * turn. `busy` counts queued requests too, so two overlapping requests cannot
 * clear it early.
 */
export function useMutationQueue(options: MutationQueueOptions): MutationQueue {
  const queueRef = useRef<SerialQueue | null>(null);
  if (queueRef.current === null) queueRef.current = createSerialQueue();
  const optionsRef = useRef(options);
  optionsRef.current = options;
  const [inFlight, setInFlight] = useState(0);

  const mutate = useCallback(
    (run: () => Promise<void>, onUnprocessable?: (cause: ApiRequestError) => void) => {
      setInFlight((n) => n + 1);
      void queueRef.current!.enqueue(async () => {
        try {
          await run();
        } catch (cause) {
          if (cause instanceof ApiRequestError && cause.isUnprocessable && onUnprocessable) {
            onUnprocessable(cause);
          } else {
            optionsRef.current.onError(cause);
          }
        } finally {
          setInFlight((n) => n - 1);
          optionsRef.current.onSettled?.();
        }
      });
    },
    [],
  );

  return { busy: inFlight > 0, mutate };
}
```

`src/app/keyTargets.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * What a key event landed on, so page shortcuts never fire while the user is
 * typing and never steal Enter/Space from a button, link or checkbox.
 * Classifies by tag name and input type only, so it is testable in Node.
 */

export type TargetKind = 'text' | 'choice' | 'control' | 'other';

export interface TargetLike {
  tagName: string;
  type?: string;
}

const CHOICE_INPUTS = new Set(['checkbox', 'radio']);
const CONTROL_INPUTS = new Set(['button', 'submit', 'reset']);

export function targetKind(t: TargetLike | null | undefined): TargetKind {
  if (!t || typeof t.tagName !== 'string') return 'other';
  const tag = t.tagName.toUpperCase();
  if (tag === 'TEXTAREA') return 'text';
  if (tag === 'SELECT') return 'choice';
  if (tag === 'INPUT') {
    const type = (t.type ?? 'text').toLowerCase();
    if (CHOICE_INPUTS.has(type)) return 'choice';
    if (CONTROL_INPUTS.has(type)) return 'control';
    return 'text';
  }
  if (tag === 'BUTTON' || tag === 'A') return 'control';
  return 'other';
}

export function kindOf(target: EventTarget | null): TargetKind {
  return target !== null && typeof target === 'object' && 'tagName' in target
    ? targetKind(target as unknown as TargetLike)
    : 'other';
}

/** Typing or choosing targets: inputs, selects, textareas. */
export function isField(target: EventTarget | null): boolean {
  const k = kindOf(target);
  return k === 'text' || k === 'choice';
}

/** Buttons and links: Enter/Space belong to their native activation. */
export function isControl(target: EventTarget | null): boolean {
  return kindOf(target) === 'control';
}
```

`src/app/saveError.ts`: move `errorMessage` and `saveErrorMessage` out of `KortlaegningPage.tsx` verbatim, keeping the doc comment, add the SPDX header and `import { ApiRequestError } from '../api/client';`, and export both functions.

- [ ] **Step 4: Migrate KortlaegningPage** (the result must behave identically):
  - Delete its local `isField`, `isControl`, `errorMessage` and `saveErrorMessage`.
  - Add `import { isControl, isField } from '../app/keyTargets';`, `import { saveErrorMessage } from '../app/saveError';` and `import { useMutationQueue } from '../app/useMutationQueue';`.
  - Delete the `inFlight` state, `busy`, `chainRef` and the `mutate` function together with its comment block. In their place, after `const toast = useToast();`, put:

```ts
  // Every write runs on one serial chain (see useMutationQueue): two PATCHes
  // to one type — a note blur racing an approve — can never land out of order.
  const { busy, mutate } = useMutationQueue({
    onError: (cause) => toast.show(saveErrorMessage(cause)),
    onSettled: refresh,
  });
```

  - The existing calls `void mutate(async () => …, () => toast.show(…))` stay as they are, since `() => void` is assignable to the `onUnprocessable` parameter.
  - In `src/test/kortlaegning.page.test.ts`, change the import to `import { approvedMessage, blockedMessage, coverageParts } from '../routes/KortlaegningPage';` and add `import { saveErrorMessage } from '../app/saveError';`.

- [ ] **Step 5: Run → PASS.** Run `npm --prefix apps/rux/frontend test && npm --prefix apps/rux/frontend run typecheck`. All Phase 3 tests must pass unchanged apart from that import.

- [ ] **Step 6: Commit**

```bash
git add apps/rux/frontend/src/app/{serialQueue,useMutationQueue,keyTargets,saveError}.ts apps/rux/frontend/src/test/{serialQueue,keyTargets}.test.ts apps/rux/frontend/src/routes/KortlaegningPage.tsx apps/rux/frontend/src/test/kortlaegning.page.test.ts
git commit -m "refactor(gui): share the serial mutation queue and key-target checks" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 3: Cross-screen links and Kortlægning's initial view

**Files:**
- Create `src/app/links.ts`, `src/test/links.test.ts` and `src/test/surveyFixtures.ts`.
- Modify `src/kortlaegning/model.ts` and `src/test/kortlaegning.model.test.ts`.

**Interfaces:**

```ts
// links.ts
export const MILJOE_PATH = '/miljoe';
export const KORTLAEGNING_PATH = '/kortlaegning';
export function sampleHref(sampleId: number): string;    // '/miljoe?sample=1'
export function newSampleHref(typeId: number): string;   // '/miljoe?ny=6'
export function surveyTypeHref(typeId: number): string;  // '/kortlaegning?type=6'
export interface MiljoeQuery { sampleId: number | null; newForType: number | null }
export function parseMiljoeQuery(search: string): MiljoeQuery;
export function parseTypeQuery(search: string): number | null;
// kortlaegning/model.ts
export function initialViewFor(types: SurveyType[], typeId: number): { tab: Tab; selection: Selection } | null;
// test/surveyFixtures.ts
export function surveyType(over?: Partial<SurveyType>): SurveyType;
export function sample(over?: Partial<Sample>): Sample;
```

- [ ] **Step 1: Fixtures.** Create `src/test/surveyFixtures.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/** Builders for survey types and samples in the Miljø & prøver tests. */

import type { Sample, SurveyType } from '../api/types';

export function surveyType(over: Partial<SurveyType> = {}): SurveyType {
  return {
    id: 1,
    name: 'Vinduespartier, aluminium',
    eak_code: '17.04.02',
    eak_name: 'Aluminium',
    bim7aa_code: '312 Udv. vinduer',
    unit: 'stk',
    treatment: 'genbrug',
    review_status: 'queue',
    confidence: 0.82,
    mass_t: 3.1,
    note: '',
    starred: false,
    semantic_class: -2,
    environment_status: 'ren_screening',
    sample_ids: [],
    quantity: 38,
    parts: [],
    created_at: '',
    updated_at: '',
    ...over,
  };
}

export function sample(over: Partial<Sample> = {}): Sample {
  return {
    id: 1,
    code: 'P-01',
    title: 'PCB i fugemasse',
    what: 'Fugemasse omkring vinduespartier',
    stage: 'sendt',
    result: null,
    type_ids: [],
    created_at: '',
    updated_at: '',
    ...over,
  };
}
```

- [ ] **Step 2: Failing tests.** Create `src/test/links.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { newSampleHref, parseMiljoeQuery, parseTypeQuery, sampleHref, surveyTypeHref } from '../app/links';

describe('cross-screen links', () => {
  it('builds the hrefs both screens link with', () => {
    expect(sampleHref(1)).toBe('/miljoe?sample=1');
    expect(newSampleHref(6)).toBe('/miljoe?ny=6');
    expect(surveyTypeHref(11)).toBe('/kortlaegning?type=11');
  });

  it('parses the Miljø query, ignoring anything that is not a positive id', () => {
    expect(parseMiljoeQuery('?sample=3')).toEqual({ sampleId: 3, newForType: null });
    expect(parseMiljoeQuery('?ny=6')).toEqual({ sampleId: null, newForType: 6 });
    expect(parseMiljoeQuery('')).toEqual({ sampleId: null, newForType: null });
    expect(parseMiljoeQuery('?sample=abc&ny=-2')).toEqual({ sampleId: null, newForType: null });
    expect(parseMiljoeQuery('?sample=0')).toEqual({ sampleId: null, newForType: null });
    expect(parseMiljoeQuery('?sample=1.5')).toEqual({ sampleId: null, newForType: null });
  });

  it('parses the Kortlægning type query', () => {
    expect(parseTypeQuery('?type=6')).toBe(6);
    expect(parseTypeQuery('?type=')).toBeNull();
    expect(parseTypeQuery('?other=1')).toBeNull();
  });
});
```

Append to `src/test/kortlaegning.model.test.ts` (add `initialViewFor` to its model import and `import { surveyType } from './surveyFixtures';`):

```ts
describe('initialViewFor', () => {
  const types = [
    surveyType({ id: 2, review_status: 'queue' }),
    surveyType({ id: 3, review_status: 'approved' }),
    surveyType({ id: 4, review_status: 'rejected' }),
  ];

  it('opens a deep-linked type in the tab it lives in', () => {
    expect(initialViewFor(types, 2)).toEqual({ tab: 'queue', selection: { typeId: 2, partCode: null } });
    expect(initialViewFor(types, 3)).toEqual({ tab: 'approved', selection: { typeId: 3, partCode: null } });
  });

  it('ignores rejected and unknown types', () => {
    expect(initialViewFor(types, 4)).toBeNull();
    expect(initialViewFor(types, 99)).toBeNull();
  });
});
```

- [ ] **Step 3: Run → FAIL** with `npm --prefix apps/rux/frontend test -- links kortlaegning.model`.

- [ ] **Step 4: Implement.** `src/app/links.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The links between Kortlægning and Miljø & prøver, as data: a sample line in
 * Kortlægning opens its sample; a type with no sample opens the create form
 * pre-linked; a sample's linked type opens that type. Paths stay
 * extensionless (see App.tsx) and ids are positive integers.
 */

export const MILJOE_PATH = '/miljoe';
export const KORTLAEGNING_PATH = '/kortlaegning';

export function sampleHref(sampleId: number): string {
  return `${MILJOE_PATH}?sample=${sampleId}`;
}

export function newSampleHref(typeId: number): string {
  return `${MILJOE_PATH}?ny=${typeId}`;
}

export function surveyTypeHref(typeId: number): string {
  return `${KORTLAEGNING_PATH}?type=${typeId}`;
}

function positiveId(value: string | null): number | null {
  if (value === null || !/^\d+$/.test(value)) return null;
  const n = Number(value);
  return n > 0 && Number.isSafeInteger(n) ? n : null;
}

export interface MiljoeQuery {
  /** `?sample=<id>`: scroll to and highlight that card. */
  sampleId: number | null;
  /** `?ny=<typeId>`: open the create form with that type pre-linked. */
  newForType: number | null;
}

export function parseMiljoeQuery(search: string): MiljoeQuery {
  const q = new URLSearchParams(search);
  return { sampleId: positiveId(q.get('sample')), newForType: positiveId(q.get('ny')) };
}

/** `?type=<id>` on /kortlaegning. */
export function parseTypeQuery(search: string): number | null {
  return positiveId(new URLSearchParams(search).get('type'));
}
```

Append to `src/kortlaegning/model.ts`:

```ts
/**
 * Where a deep link to a type (`/kortlaegning?type=<id>`) lands: the tab the
 * type lives in, with it selected. Rejected types are hidden from every tab,
 * and an unknown id has nowhere to go — both give null (plain screen).
 */
export function initialViewFor(types: SurveyType[], typeId: number): { tab: Tab; selection: Selection } | null {
  const t = types.find((x) => x.id === typeId);
  if (!t || t.review_status === 'rejected') return null;
  return { tab: t.review_status === 'approved' ? 'approved' : 'queue', selection: { typeId, partCode: null } };
}
```

- [ ] **Step 5: Run → PASS. Commit.**

```bash
git add apps/rux/frontend/src/app/links.ts apps/rux/frontend/src/test/links.test.ts apps/rux/frontend/src/test/surveyFixtures.ts apps/rux/frontend/src/kortlaegning/model.ts apps/rux/frontend/src/test/kortlaegning.model.test.ts
git commit -m "feat(gui): links between Kortlægning and Miljø & prøver" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 4: Miljø model

**Files:** Create `src/miljoe/model.ts` and `src/test/miljoe.model.test.ts`.

**Interfaces:**

```ts
export const STAGES: readonly SampleStage[];                 // planlagt, udtaget, sendt, svar
export type StepState = 'done' | 'current' | 'todo';
export interface ChainStep { stage: SampleStage; label: string; state: StepState }
export function chainSteps(s: Pick<Sample, 'stage'>): ChainStep[];
export function nextStage(stage: SampleStage): SampleStage | null;
export function statusPill(s: Pick<Sample, 'stage' | 'result'>): { label: string; tone: Tone };
export type CardAction = 'advance' | 'answer' | 'answered';
export function cardAction(s: Pick<Sample, 'stage' | 'result'>): CardAction;
export function advancePatch(s: Pick<Sample, 'stage'>): SamplePatch | null;
export function resultPatch(result: SampleResult): SamplePatch;     // { stage: 'svar', result }   (R3)
export const UNDO_RESULT_PATCH: SamplePatch;                         // { stage: 'sendt', result: null } (R4)
export function answeredNote(s: Pick<Sample, 'type_ids'>): string;
export function linkedTypes(ids: readonly number[], types: SurveyType[]): SurveyType[];
export function linkableTypes(types: SurveyType[], linked: readonly number[]): SurveyType[];
export function toggleLink(ids: readonly number[], typeId: number): number[];
export function pendingCount(samples: Sample[]): number;
export function replaceSample(list: Sample[], updated: Sample): Sample[];
export function addSample(list: Sample[], created: Sample): Sample[];
export function removeSample(list: Sample[], id: number): Sample[];
export function textCommit(draft: string, current: string, required: boolean): string | null;
export function createBody(title: string, what: string, typeIds: readonly number[]): SampleCreate | null;
export interface GateChange { unblocked: SurveyType[]; contaminated: SurveyType[]; blocked: SurveyType[]; reblocked: SurveyType[] }
export function gateChanges(before: SurveyType[], after: SurveyType[]): GateChange;
export function gateMessage(code: string, change: GateChange): string | null;
export function resultToast(code: string, result: SampleResult): string;
export function deleteConfirmText(s: Pick<Sample, 'code' | 'title' | 'type_ids'>): string;
export type EditorKey = 'revert' | 'close' | 'commit' | 'submit';
export function editorKeyAction(k: { key: string; kind: TargetKind; ctrlKey?: boolean; metaKey?: boolean; altKey?: boolean }): EditorKey | null;
```

- [ ] **Step 1: Failing test** — `src/test/miljoe.model.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  addSample,
  advancePatch,
  answeredNote,
  cardAction,
  chainSteps,
  createBody,
  deleteConfirmText,
  editorKeyAction,
  gateChanges,
  gateMessage,
  linkableTypes,
  linkedTypes,
  nextStage,
  pendingCount,
  removeSample,
  replaceSample,
  resultPatch,
  resultToast,
  statusPill,
  textCommit,
  toggleLink,
  UNDO_RESULT_PATCH,
} from '../miljoe/model';
import { sample, surveyType } from './surveyFixtures';

describe('stage chain', () => {
  it('marks steps before the stage done, the stage current, the rest todo', () => {
    expect(chainSteps({ stage: 'sendt' }).map((s) => [s.label, s.state])).toEqual([
      ['Planlagt', 'done'],
      ['Udtaget', 'done'],
      ['Sendt til lab', 'current'],
      ['Svar modtaget', 'todo'],
    ]);
    expect(chainSteps({ stage: 'svar' }).map((s) => s.state)).toEqual(['done', 'done', 'done', 'current']);
    expect(chainSteps({ stage: 'planlagt' }).map((s) => s.state)).toEqual(['current', 'todo', 'todo', 'todo']);
  });

  it('advances one stage at a time and stops at svar', () => {
    expect(nextStage('planlagt')).toBe('udtaget');
    expect(nextStage('udtaget')).toBe('sendt');
    expect(nextStage('svar')).toBeNull();
    expect(advancePatch({ stage: 'udtaget' })).toEqual({ stage: 'sendt' });
    expect(advancePatch({ stage: 'svar' })).toBeNull();
  });
});

describe('card status and action', () => {
  it('pills the stage until a result exists, then the result', () => {
    expect(statusPill({ stage: 'sendt', result: null })).toEqual({ label: 'Sendt til lab', tone: 'wait' });
    expect(statusPill({ stage: 'udtaget', result: null })).toEqual({ label: 'Udtaget', tone: 'wait' });
    expect(statusPill({ stage: 'svar', result: 'forurenet' })).toEqual({ label: 'Forurenet', tone: 'crit' });
    expect(statusPill({ stage: 'svar', result: 'ren' })).toEqual({ label: 'Ren', tone: 'good' });
    expect(statusPill({ stage: 'svar', result: null })).toEqual({ label: 'Svar modtaget', tone: 'accent' });
  });

  it('offers Næste trin before the lab, the result buttons once sent, a note after', () => {
    expect(cardAction({ stage: 'planlagt', result: null })).toBe('advance');
    expect(cardAction({ stage: 'udtaget', result: null })).toBe('advance');
    expect(cardAction({ stage: 'sendt', result: null })).toBe('answer');
    expect(cardAction({ stage: 'svar', result: null })).toBe('answer');
    expect(cardAction({ stage: 'svar', result: 'ren' })).toBe('answered');
  });

  it('records a result together with the svar stage, and undoes back to sendt', () => {
    expect(resultPatch('ren')).toEqual({ stage: 'svar', result: 'ren' });
    expect(resultPatch('forurenet')).toEqual({ stage: 'svar', result: 'forurenet' });
    expect(UNDO_RESULT_PATCH).toEqual({ stage: 'sendt', result: null });
  });

  it('words the answered note like the prototype', () => {
    expect(answeredNote({ type_ids: [11] })).toBe(
      'Svar registreret — miljøstatus opdateret på 1 type(r) i kortlægningen.',
    );
    expect(answeredNote({ type_ids: [] })).toBe('Svar registreret — prøven er ikke koblet til nogen type.');
  });
});

describe('links', () => {
  const types = [
    surveyType({ id: 6, name: 'Vinduespartier, aluminium' }),
    surveyType({ id: 8, name: 'Gulvbelægning, linoleum' }),
    surveyType({ id: 4, name: 'Fejldetektion', review_status: 'rejected' }),
  ];

  it('resolves linked ids to types in id order, skipping unknown ones', () => {
    expect(linkedTypes([8, 6, 99], types).map((t) => t.id)).toEqual([8, 6]);
  });

  it('offers non-rejected types by name, plus rejected ones already linked', () => {
    expect(linkableTypes(types, []).map((t) => t.id)).toEqual([8, 6]);
    expect(linkableTypes(types, [4]).map((t) => t.id)).toEqual([4, 8, 6]);
  });

  it('toggles a link and keeps the set sorted', () => {
    expect(toggleLink([8], 6)).toEqual([6, 8]);
    expect(toggleLink([6, 8], 6)).toEqual([8]);
    expect(toggleLink([], 3)).toEqual([3]);
  });
});

describe('sample lists', () => {
  const list = [sample({ id: 1, stage: 'sendt' }), sample({ id: 2, code: 'P-02', stage: 'svar', result: 'forurenet' })];

  it('counts samples whose answer is outstanding, as the badge does', () => {
    expect(pendingCount(list)).toBe(1);
    expect(pendingCount([...list, sample({ id: 3, stage: 'planlagt' })])).toBe(2);
  });

  it('replaces, adds in id order and removes', () => {
    expect(replaceSample(list, sample({ id: 1, stage: 'svar', result: 'ren' }))[0].result).toBe('ren');
    expect(addSample(list, sample({ id: 0, code: 'P-00' })).map((s) => s.id)).toEqual([0, 1, 2]);
    expect(addSample(list, sample({ id: 2, title: 'ny' })).find((s) => s.id === 2)?.title).toBe('ny');
    expect(removeSample(list, 1).map((s) => s.id)).toEqual([2]);
  });
});

describe('drafts and the create body', () => {
  it('commits only a real change, trimmed; a required field never empties', () => {
    expect(textCommit('PCB i fugemasse', 'PCB i fugemasse', true)).toBeNull();
    expect(textCommit('  PCB i fugemasse ', 'PCB i fugemasse', true)).toBeNull();
    expect(textCommit('PCB i fuger', 'PCB i fugemasse', true)).toBe('PCB i fuger');
    expect(textCommit('   ', 'PCB i fugemasse', true)).toBeNull();
    expect(textCommit('', 'Fugemasse', false)).toBe('');
  });

  it('builds a POST /samples body, or null without a title', () => {
    expect(createBody(' Bly i maling ', ' Indervægge ', [11, 2])).toEqual({
      title: 'Bly i maling',
      what: 'Indervægge',
      type_ids: [2, 11],
    });
    expect(createBody('  ', 'x', [])).toBeNull();
  });
});

describe('gate feedback', () => {
  const before = [
    surveyType({ id: 6, name: 'Vinduespartier, aluminium', environment_status: 'afventer' }),
    surveyType({ id: 11, name: 'Indvendige murvægge, malet', environment_status: 'afventer' }),
    surveyType({ id: 7, name: 'Trapezplader, tag', review_status: 'approved', environment_status: 'ren_screening' }),
    surveyType({ id: 2, name: 'Betonsøjler, bærende', environment_status: 'ren_screening' }),
    surveyType({ id: 4, name: 'Fejldetektion', review_status: 'rejected', environment_status: 'afventer' }),
  ];

  it('sorts status changes into unblocked, contaminated, blocked and re-blocked', () => {
    const after = [
      { ...before[0], environment_status: 'ren_proevesvar' as const },
      { ...before[1], environment_status: 'forurenet' as const },
      { ...before[2], environment_status: 'afventer' as const },
      { ...before[3], environment_status: 'afventer' as const },
      { ...before[4], environment_status: 'ren_screening' as const },
    ];
    const c = gateChanges(before, after);
    expect(c.unblocked.map((t) => t.id)).toEqual([6]);
    expect(c.contaminated.map((t) => t.id)).toEqual([11]);
    expect(c.reblocked.map((t) => t.id)).toEqual([7]);
    expect(c.blocked.map((t) => t.id)).toEqual([2]);
  });

  it('words the toast, or says nothing when nothing changed', () => {
    const unblocked = gateChanges(before, [{ ...before[0], environment_status: 'ren_proevesvar' }]);
    expect(gateMessage('P-01', unblocked)).toBe('P-01: 1 type kan nu godkendes (Vinduespartier, aluminium)');
    const mixed = gateChanges(before, [
      { ...before[1], environment_status: 'forurenet' },
      { ...before[2], environment_status: 'afventer' },
      { ...before[3], environment_status: 'afventer' },
    ]);
    expect(gateMessage('P-02', mixed)).toBe(
      'P-02: Indvendige murvægge, malet er nu forurenet · Betonsøjler, bærende afventer nu prøvesvar · ' +
        'Trapezplader, tag er godkendt, men afventer nu prøvesvar',
    );
    expect(gateMessage('P-03', gateChanges(before, before))).toBeNull();
    const two = gateChanges(before, [
      { ...before[0], environment_status: 'ren_proevesvar' },
      { ...before[1], environment_status: 'ren_screening' },
    ]);
    expect(gateMessage('P-04', two)).toBe(
      'P-04: 2 typer kan nu godkendes (Vinduespartier, aluminium, Indvendige murvægge, malet)',
    );
  });

  it('has fallback toasts for a result and a delete prompt', () => {
    expect(resultToast('P-01', 'ren')).toBe('✓ P-01 · svar registreret: Ren');
    expect(deleteConfirmText({ code: 'P-01', title: 'PCB i fugemasse', type_ids: [6] })).toBe(
      'Slet P-01 · PCB i fugemasse? 1 koblet type mister prøven, og dens miljøstatus beregnes igen.',
    );
    expect(deleteConfirmText({ code: 'P-05', title: 'PAH i tagpap', type_ids: [] })).toBe('Slet P-05 · PAH i tagpap?');
  });
});

describe('editor keys', () => {
  it('reverts a text field on Esc and closes the editor from anywhere else', () => {
    expect(editorKeyAction({ key: 'Escape', kind: 'text' })).toBe('revert');
    expect(editorKeyAction({ key: 'Escape', kind: 'choice' })).toBe('close');
    expect(editorKeyAction({ key: 'Escape', kind: 'control' })).toBe('close');
  });

  it('commits a text field on Enter, submits on Ctrl/⌘+Enter, leaves controls alone', () => {
    expect(editorKeyAction({ key: 'Enter', kind: 'text' })).toBe('commit');
    expect(editorKeyAction({ key: 'Enter', kind: 'text', ctrlKey: true })).toBe('submit');
    expect(editorKeyAction({ key: 'Enter', kind: 'control', metaKey: true })).toBe('submit');
    expect(editorKeyAction({ key: 'Enter', kind: 'control' })).toBeNull();
    expect(editorKeyAction({ key: ' ', kind: 'choice' })).toBeNull();
    expect(editorKeyAction({ key: 'g', kind: 'other' })).toBeNull();
  });
});
```

- [ ] **Step 2: Run → FAIL** with `npm --prefix apps/rux/frontend test -- miljoe.model`.

- [ ] **Step 3: Implement `src/miljoe/model.ts`**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Miljø & prøver as data: the stage chain a card draws, what its action row
 * offers, the exact patches it sends, link toggling, and the approval-gate
 * feedback after a change. Pure, so the sample flow is testable without a DOM.
 *
 * The backend rule (core::update_sample_checked): a result can only be set
 * once the stage is `svar`, checked on the merged state — so recording a
 * result sends stage and result together, and undoing goes back to `sendt`
 * (a `svar` sample with no result counts as clean in environment_status()).
 */

import type { Sample, SampleCreate, SamplePatch, SampleResult, SampleStage, SurveyType } from '../api/types';
import type { TargetKind } from '../app/keyTargets';
import { RESULT_LABEL, STAGE_LABEL, type Tone } from '../kortlaegning/vocab';

export const STAGES: readonly SampleStage[] = ['planlagt', 'udtaget', 'sendt', 'svar'];

export type StepState = 'done' | 'current' | 'todo';

export interface ChainStep {
  stage: SampleStage;
  label: string;
  state: StepState;
}

export function chainSteps(s: Pick<Sample, 'stage'>): ChainStep[] {
  const at = STAGES.indexOf(s.stage);
  return STAGES.map((stage, i) => ({
    stage,
    label: STAGE_LABEL[stage],
    state: i < at ? 'done' : i === at ? 'current' : 'todo',
  }));
}

export function nextStage(stage: SampleStage): SampleStage | null {
  const i = STAGES.indexOf(stage);
  return i >= 0 && i < STAGES.length - 1 ? STAGES[i + 1] : null;
}

export function statusPill(s: Pick<Sample, 'stage' | 'result'>): { label: string; tone: Tone } {
  if (s.result === 'forurenet') return { label: RESULT_LABEL.forurenet, tone: 'crit' };
  if (s.result === 'ren') return { label: RESULT_LABEL.ren, tone: 'good' };
  if (s.stage === 'svar') return { label: STAGE_LABEL.svar, tone: 'accent' };
  return { label: STAGE_LABEL[s.stage], tone: 'wait' };
}

export type CardAction = 'advance' | 'answer' | 'answered';

/**
 * `advance` (Næste trin →) before the lab has the sample; `answer` (the two
 * result buttons) once it is sent — or answered without a result, which only
 * the API can produce; `answered` (the note) once a result exists.
 */
export function cardAction(s: Pick<Sample, 'stage' | 'result'>): CardAction {
  if (s.result !== null) return 'answered';
  if (s.stage === 'sendt' || s.stage === 'svar') return 'answer';
  return 'advance';
}

export function advancePatch(s: Pick<Sample, 'stage'>): SamplePatch | null {
  const next = nextStage(s.stage);
  return next ? { stage: next } : null;
}

/** One PATCH: the result is legal because the merged stage is `svar`. */
export function resultPatch(result: SampleResult): SamplePatch {
  return { stage: 'svar', result };
}

/** Back to awaiting the lab — never `svar` without a result (that counts as clean). */
export const UNDO_RESULT_PATCH: SamplePatch = { stage: 'sendt', result: null };

export function answeredNote(s: Pick<Sample, 'type_ids'>): string {
  const n = s.type_ids.length;
  return n === 0
    ? 'Svar registreret — prøven er ikke koblet til nogen type.'
    : `Svar registreret — miljøstatus opdateret på ${n} type(r) i kortlægningen.`;
}

export function linkedTypes(ids: readonly number[], types: SurveyType[]): SurveyType[] {
  const byId = new Map(types.map((t) => [t.id, t]));
  return ids.map((id) => byId.get(id)).filter((t): t is SurveyType => t !== undefined);
}

/** What the link picker offers: every non-rejected type, plus rejected ones still linked (so they can be unlinked). */
export function linkableTypes(types: SurveyType[], linked: readonly number[]): SurveyType[] {
  return types
    .filter((t) => t.review_status !== 'rejected' || linked.includes(t.id))
    .sort((a, b) => a.name.localeCompare(b.name, 'da'));
}

export function toggleLink(ids: readonly number[], typeId: number): number[] {
  const next = ids.includes(typeId) ? ids.filter((id) => id !== typeId) : [...ids, typeId];
  return next.sort((a, b) => a - b);
}

/** Samples whose answer is outstanding — the same rule as SurveySummary.pending_samples. */
export function pendingCount(samples: Sample[]): number {
  return samples.filter((s) => s.stage !== 'svar').length;
}

export function replaceSample(list: Sample[], updated: Sample): Sample[] {
  return list.map((s) => (s.id === updated.id ? updated : s));
}

export function addSample(list: Sample[], created: Sample): Sample[] {
  return [...list.filter((s) => s.id !== created.id), created].sort((a, b) => a.id - b.id);
}

export function removeSample(list: Sample[], id: number): Sample[] {
  return list.filter((s) => s.id !== id);
}

/**
 * The value a text draft commits on blur, or null to send nothing: unchanged
 * after trimming (an untouched blur), or emptied when the field is required.
 */
export function textCommit(draft: string, current: string, required: boolean): string | null {
  const value = draft.trim();
  if (value === current.trim()) return null;
  if (required && value === '') return null;
  return value;
}

export function createBody(title: string, what: string, typeIds: readonly number[]): SampleCreate | null {
  const t = title.trim();
  if (t === '') return null;
  return { title: t, what: what.trim(), type_ids: [...typeIds].sort((a, b) => a - b) };
}

export interface GateChange {
  /** Queued types that were awaiting a sample and now can be approved. */
  unblocked: SurveyType[];
  /** Types that just became forurenet. */
  contaminated: SurveyType[];
  /** Queued types that now await a sample. */
  blocked: SurveyType[];
  /** Approved types that now await a sample — approved, but blocking Indberetning. */
  reblocked: SurveyType[];
}

/** Compares two server snapshots of the survey; never derives a status itself. Rejected types are ignored. */
export function gateChanges(before: SurveyType[], after: SurveyType[]): GateChange {
  const was = new Map(before.map((t) => [t.id, t.environment_status]));
  const out: GateChange = { unblocked: [], contaminated: [], blocked: [], reblocked: [] };
  for (const t of after) {
    const prev = was.get(t.id);
    if (prev === undefined || prev === t.environment_status || t.review_status === 'rejected') continue;
    if (t.environment_status === 'forurenet') out.contaminated.push(t);
    else if (t.environment_status === 'afventer') (t.review_status === 'approved' ? out.reblocked : out.blocked).push(t);
    else if (prev === 'afventer' && t.review_status === 'queue') out.unblocked.push(t);
  }
  return out;
}

function names(types: SurveyType[]): string {
  return types.map((t) => t.name).join(', ');
}

export function gateMessage(code: string, c: GateChange): string | null {
  const parts: string[] = [];
  if (c.unblocked.length > 0) {
    const n = c.unblocked.length;
    parts.push(`${n === 1 ? '1 type kan' : `${n} typer kan`} nu godkendes (${names(c.unblocked)})`);
  }
  if (c.contaminated.length > 0) parts.push(`${names(c.contaminated)} er nu forurenet`);
  if (c.blocked.length > 0) parts.push(`${names(c.blocked)} afventer nu prøvesvar`);
  if (c.reblocked.length > 0) parts.push(`${names(c.reblocked)} er godkendt, men afventer nu prøvesvar`);
  return parts.length > 0 ? `${code}: ${parts.join(' · ')}` : null;
}

export function resultToast(code: string, result: SampleResult): string {
  return `✓ ${code} · svar registreret: ${RESULT_LABEL[result]}`;
}

export function deleteConfirmText(s: Pick<Sample, 'code' | 'title' | 'type_ids'>): string {
  const n = s.type_ids.length;
  if (n === 0) return `Slet ${s.code} · ${s.title}?`;
  const who = n === 1 ? '1 koblet type mister prøven, og dens' : `${n} koblede typer mister prøven, og deres`;
  return `Slet ${s.code} · ${s.title}? ${who} miljøstatus beregnes igen.`;
}

export type EditorKey = 'revert' | 'close' | 'commit' | 'submit';

/**
 * Keys inside a sample editor or the create form. Esc in a text field drops
 * that field's draft (and must not commit it); Esc anywhere else closes.
 * Enter in a single-line text field commits it; Ctrl/⌘+Enter submits from
 * anywhere. Enter/Space on buttons, links and checkboxes stay native.
 */
export function editorKeyAction(k: {
  key: string;
  kind: TargetKind;
  ctrlKey?: boolean;
  metaKey?: boolean;
  altKey?: boolean;
}): EditorKey | null {
  if (k.key === 'Escape') return k.kind === 'text' ? 'revert' : 'close';
  if (k.key === 'Enter' && (k.ctrlKey || k.metaKey)) return 'submit';
  if (k.key === 'Enter' && k.kind === 'text' && !k.altKey) return 'commit';
  return null;
}
```

- [ ] **Step 4: Run → PASS. Commit.**

```bash
git add apps/rux/frontend/src/miljoe/model.ts apps/rux/frontend/src/test/miljoe.model.test.ts
git commit -m "feat(gui): Miljø & prøver model — stage chain, patches, gate feedback" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 5: Text draft hook

**Files:** Create `src/miljoe/useTextDraft.ts`.

**Interfaces:**

```ts
export interface TextDraft {
  props: { value: string; onChange: (e: ChangeEvent<HTMLInputElement>) => void; onFocus: () => void; onBlur: () => void };
  /** Esc: drop the draft and leave the field without committing. */
  revert: (el: HTMLElement) => void;
}
export function useTextDraft(current: string, onCommit: (value: string) => void, required?: boolean): TextDraft;
```

The rules are pure and already tested (`textCommit`). The hook is the thin React shell around them.

- [ ] **Step 1: Implement**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState, type ChangeEvent } from 'react';

import { textCommit } from './model';

export interface TextDraft {
  props: {
    value: string;
    onChange: (e: ChangeEvent<HTMLInputElement>) => void;
    onFocus: () => void;
    onBlur: () => void;
  };
  /** Esc: drop the draft and leave the field without committing. */
  revert: (el: HTMLElement) => void;
}

/**
 * A text field that commits on blur. An untouched blur sends nothing
 * (`textCommit`); a required field that was emptied snaps back. While the
 * field is not focused it follows the server value, so a response that
 * changed it shows up; while focused it never clobbers what is being typed.
 */
export function useTextDraft(current: string, onCommit: (value: string) => void, required = false): TextDraft {
  const [draft, setDraft] = useState(current);
  const focused = useRef(false);
  const skipNextCommit = useRef(false);

  useEffect(() => {
    if (!focused.current) setDraft(current);
  }, [current]);

  return {
    props: {
      value: draft,
      onChange: (e) => setDraft(e.target.value),
      onFocus: () => {
        focused.current = true;
      },
      onBlur: () => {
        focused.current = false;
        if (skipNextCommit.current) {
          skipNextCommit.current = false;
          return;
        }
        const value = textCommit(draft, current, required);
        if (value === null) setDraft(current);
        else onCommit(value);
      },
    },
    revert: (el) => {
      skipNextCommit.current = true;
      setDraft(current);
      el.blur();
    },
  };
}
```

- [ ] **Step 2: Typecheck; commit.**

```bash
npm --prefix apps/rux/frontend run typecheck
git add apps/rux/frontend/src/miljoe/useTextDraft.ts
git commit -m "feat(gui): blur-commit text draft with Esc revert" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 6: StageChain and LinkPicker

**Files:** Create `src/components/miljoe/StageChain.tsx` + `StageChain.module.css` and `src/components/miljoe/LinkPicker.tsx` + `LinkPicker.module.css`.

**Interfaces:**

```tsx
export function StageChain(props: { sample: Pick<Sample, 'stage'> }): JSX.Element;
export interface LinkPickerProps { types: SurveyType[]; selected: readonly number[]; onToggle: (typeId: number) => void; legend?: string }
export function LinkPicker(props: LinkPickerProps): JSX.Element;
```

- [ ] **Step 1: StageChain** — `StageChain.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Sample } from '../../api/types';
import { chainSteps } from '../../miljoe/model';
import styles from './StageChain.module.css';

/** Planlagt — Udtaget — Sendt til lab — Svar modtaget, with the sample's stage current. */
export function StageChain({ sample }: { sample: Pick<Sample, 'stage'> }) {
  return (
    <ol className={styles.chain} aria-label="Prøveforløb">
      {chainSteps(sample).map((step) => (
        <li
          key={step.stage}
          className={`${styles.step} ${styles[step.state]}`}
          aria-current={step.state === 'current' ? 'step' : undefined}
        >
          <span className={styles.dot} aria-hidden="true" />
          {step.label}
        </li>
      ))}
    </ol>
  );
}
```

`StageChain.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.chain {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: var(--space-2);
  margin: 0;
  padding: 0;
  list-style: none;
  font-size: var(--font-size-xs);
  font-weight: var(--font-weight-bold);
}

.step {
  display: inline-flex;
  align-items: center;
  gap: var(--space-1);
}

.step + .step::before {
  content: '—';
  margin-right: var(--space-1);
  color: var(--color-text-faint);
  font-weight: var(--font-weight-regular);
}

.dot {
  width: var(--space-2);
  height: var(--space-2);
  border-radius: var(--radius-pill);
  border: 1px solid currentColor;
}

.done {
  color: var(--tone-good-ink);
}

.done .dot {
  background: currentColor;
}

.current {
  color: var(--color-accent-deep);
}

.current .dot {
  background: var(--color-accent);
  border-color: var(--color-accent);
}

.todo {
  color: var(--color-text-faint);
  font-weight: var(--font-weight-regular);
}
```

- [ ] **Step 2: LinkPicker** — `LinkPicker.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { SurveyType } from '../../api/types';
import { ENV_LABEL, ENV_TONE } from '../../kortlaegning/vocab';
import { linkableTypes } from '../../miljoe/model';
import { Pill } from '../Pill';
import styles from './LinkPicker.module.css';

export interface LinkPickerProps {
  types: SurveyType[];
  /** The linked type ids as the user last set them. */
  selected: readonly number[];
  onToggle: (typeId: number) => void;
  legend?: string;
}

/**
 * The survey types a sample can cover, each with its current miljøstatus, as
 * checkboxes. Each toggle is a field commit — the caller sends it at once.
 */
export function LinkPicker({ types, selected, onToggle, legend = 'Koblet til typer' }: LinkPickerProps) {
  const options = linkableTypes(types, selected);
  return (
    <fieldset className={styles.picker}>
      <legend className={styles.legend}>{legend}</legend>
      {options.length === 0 ? (
        <p className={styles.empty}>Ingen typer i kortlægningen endnu — prøven kan kobles senere.</p>
      ) : (
        <ul className={styles.list}>
          {options.map((t) => (
            <li key={t.id}>
              <label className={styles.option}>
                <input type="checkbox" checked={selected.includes(t.id)} onChange={() => onToggle(t.id)} />
                <span className={styles.name}>{t.name}</span>
                {t.review_status === 'rejected' ? (
                  <Pill tone="wait">Afvist</Pill>
                ) : (
                  <Pill tone={ENV_TONE[t.environment_status]}>{ENV_LABEL[t.environment_status]}</Pill>
                )}
              </label>
            </li>
          ))}
        </ul>
      )}
    </fieldset>
  );
}
```

`LinkPicker.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.picker {
  margin: 0;
  padding: 0;
  border: none;
  min-width: 0;
}

.legend {
  padding: 0;
  margin-bottom: var(--space-1);
  font-size: var(--font-size-2xs);
  font-weight: var(--font-weight-bold);
  text-transform: uppercase;
  letter-spacing: var(--tracking-caps);
  color: var(--color-text-muted);
}

.list {
  margin: 0;
  padding: var(--space-1);
  list-style: none;
  max-height: calc(var(--space-7) * 5);
  overflow: auto;
  border: 1px solid var(--color-border);
  border-radius: var(--radius-md);
  background: var(--color-surface-sunken);
}

.option {
  display: flex;
  align-items: center;
  gap: var(--space-2);
  padding: var(--space-1) var(--space-2);
  border-radius: var(--radius-sm);
  font-size: var(--font-size-sm);
  cursor: pointer;
}

.option:hover {
  background: var(--color-surface-raised);
}

.option input:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: 2px;
}

.name {
  flex: 1 1 auto;
  min-width: 0;
}

.empty {
  margin: 0;
  font-size: var(--font-size-sm);
  color: var(--color-text-faint);
}
```

- [ ] **Step 3: Lint, typecheck, commit.**

```bash
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/miljoe/{StageChain,LinkPicker}.module.css apps/rux/frontend/src/components/miljoe/{StageChain,LinkPicker}.tsx --tsx
npm --prefix apps/rux/frontend run typecheck
git add apps/rux/frontend/src/components/miljoe/{StageChain,LinkPicker}.{tsx,module.css}
git commit -m "feat(gui): sample stage chain and survey-type link picker" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 7: SampleCard

**Files:** Create `src/components/miljoe/SampleCard.tsx` and `SampleCard.module.css`.

**Interfaces:**

```tsx
export interface SampleCardProps {
  sample: Sample;
  types: SurveyType[];
  /** The linked type ids to show: the pending link draft, else the server's `type_ids`. */
  linkedIds: readonly number[];
  busy: boolean;
  /** Deep-linked (`?sample=`) or just created: highlighted. */
  focused: boolean;
  editing: boolean;
  onEditing: (open: boolean) => void;
  onAdvance: () => void;
  onResult: (result: SampleResult) => void;
  onUndoResult: () => void;
  onTitle: (title: string) => void;
  onWhat: (what: string) => void;
  onToggleLink: (typeId: number) => void;
  onDelete: () => void;
  cardRef?: (el: HTMLElement | null) => void;
}
export function SampleCard(props: SampleCardProps): JSX.Element;
```

- [ ] **Step 1: Implement `SampleCard.tsx`**

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { KeyboardEvent } from 'react';
import { Link } from 'react-router-dom';

import type { Sample, SampleResult, SurveyType } from '../../api/types';
import { kindOf } from '../../app/keyTargets';
import { surveyTypeHref } from '../../app/links';
import { STAGE_LABEL } from '../../kortlaegning/vocab';
import {
  answeredNote,
  cardAction,
  deleteConfirmText,
  editorKeyAction,
  linkedTypes,
  nextStage,
  statusPill,
} from '../../miljoe/model';
import { useTextDraft, type TextDraft } from '../../miljoe/useTextDraft';
import { Pill } from '../Pill';
import { LinkPicker } from './LinkPicker';
import { StageChain } from './StageChain';
import styles from './SampleCard.module.css';

export interface SampleCardProps {
  sample: Sample;
  types: SurveyType[];
  linkedIds: readonly number[];
  busy: boolean;
  focused: boolean;
  editing: boolean;
  onEditing: (open: boolean) => void;
  onAdvance: () => void;
  onResult: (result: SampleResult) => void;
  onUndoResult: () => void;
  onTitle: (title: string) => void;
  onWhat: (what: string) => void;
  onToggleLink: (typeId: number) => void;
  onDelete: () => void;
  cardRef?: (el: HTMLElement | null) => void;
}

/** Enter commits (blurs) a text field, Esc reverts it without committing. */
function fieldKeys(draft: TextDraft) {
  return (e: KeyboardEvent<HTMLInputElement>) => {
    const action = editorKeyAction({ key: e.key, kind: 'text', ctrlKey: e.ctrlKey, metaKey: e.metaKey, altKey: e.altKey });
    if (action === 'revert') {
      e.preventDefault();
      e.stopPropagation(); // the editor's Esc would otherwise close it
      draft.revert(e.currentTarget);
    } else if (action === 'commit') {
      e.preventDefault();
      e.currentTarget.blur();
    }
  };
}

function LinkedLine({ types }: { types: SurveyType[] }) {
  if (types.length === 0) return <>Ikke koblet til en type</>;
  return (
    <>
      Koblet:{' '}
      {types.map((t, i) => (
        <span key={t.id}>
          {i > 0 && ', '}
          {t.review_status === 'rejected' ? (
            `${t.name} (afvist)`
          ) : (
            <Link className={styles.typeLink} to={surveyTypeHref(t.id)}>
              {t.name}
            </Link>
          )}
        </span>
      ))}
    </>
  );
}

function SampleEditor(props: SampleCardProps) {
  const { sample, types, linkedIds, busy, onTitle, onWhat, onToggleLink, onDelete, onEditing } = props;
  const title = useTextDraft(sample.title, onTitle, true);
  const what = useTextDraft(sample.what, onWhat);
  const titleId = `sample-${sample.id}-edit-title`;
  const whatId = `sample-${sample.id}-edit-what`;

  function onKeyDown(e: KeyboardEvent<HTMLDivElement>) {
    const action = editorKeyAction({ key: e.key, kind: kindOf(e.target), ctrlKey: e.ctrlKey, metaKey: e.metaKey });
    if (action === 'close') {
      e.preventDefault();
      onEditing(false);
    }
  }

  return (
    <div className={styles.editor} onKeyDown={onKeyDown}>
      <div className={styles.fields}>
        <div className={styles.field}>
          <label className={styles.label} htmlFor={titleId}>
            Titel
          </label>
          <input id={titleId} className={styles.input} {...title.props} onKeyDown={fieldKeys(title)} />
        </div>
        <div className={styles.field}>
          <label className={styles.label} htmlFor={whatId}>
            Hvad er udtaget, og hvor
          </label>
          <input id={whatId} className={styles.input} {...what.props} onKeyDown={fieldKeys(what)} />
        </div>
      </div>
      <LinkPicker types={types} selected={linkedIds} onToggle={onToggleLink} />
      <div className={styles.editorActions}>
        <button
          type="button"
          className={styles.btnDanger}
          disabled={busy}
          onClick={() => {
            if (window.confirm(deleteConfirmText(sample))) onDelete();
          }}
        >
          Slet prøve
        </button>
        <button type="button" className={styles.btnGhost} onClick={() => onEditing(false)}>
          Luk
        </button>
      </div>
    </div>
  );
}

/**
 * One environmental sample (prototype `miljoe.png`): title + stage/result
 * pill + linked types, what was sampled, the stage chain, and the action the
 * stage allows. `Rediger` opens an inline editor for title, what, links and
 * delete.
 */
export function SampleCard(props: SampleCardProps) {
  const { sample, types, linkedIds, busy, focused, editing, onEditing, onAdvance, onResult, onUndoResult, cardRef } =
    props;
  const pill = statusPill(sample);
  const action = cardAction(sample);
  const next = nextStage(sample.stage);
  const headingId = `sample-${sample.id}-title`;

  return (
    <article
      ref={cardRef}
      tabIndex={-1}
      className={`${styles.card} ${focused ? styles.focused : ''}`}
      aria-labelledby={headingId}
    >
      <header className={styles.head}>
        <h3 id={headingId} className={styles.title}>
          {sample.code} · {sample.title}
        </h3>
        <Pill tone={pill.tone}>{pill.label}</Pill>
        <p className={styles.linked}>
          <LinkedLine types={linkedTypes(linkedIds, types)} />
          <button
            type="button"
            className={styles.textBtn}
            aria-expanded={editing}
            onClick={() => onEditing(!editing)}
          >
            {editing ? 'Luk redigering' : 'Rediger'}
          </button>
        </p>
      </header>

      {sample.what && <p className={styles.what}>{sample.what}</p>}

      <StageChain sample={sample} />

      {action === 'advance' && (
        <div className={styles.actions}>
          <button
            type="button"
            className={styles.btnGhost}
            disabled={busy}
            onClick={onAdvance}
            title={next ? `Markér som ${STAGE_LABEL[next]}` : undefined}
          >
            Næste trin →
          </button>
        </div>
      )}
      {action === 'answer' && (
        <div className={styles.actions}>
          <button type="button" className={styles.btnPrimary} disabled={busy} onClick={() => onResult('ren')}>
            Registrér svar: Ren
          </button>
          <button type="button" className={styles.btnGhost} disabled={busy} onClick={() => onResult('forurenet')}>
            Registrér svar: Forurenet
          </button>
        </div>
      )}
      {action === 'answered' && (
        <p className={styles.note}>
          {answeredNote({ type_ids: linkedIds as number[] })}{' '}
          <button type="button" className={styles.textBtn} disabled={busy} onClick={onUndoResult}>
            Fortryd svar
          </button>
        </p>
      )}

      {editing && <SampleEditor {...props} />}
    </article>
  );
}
```

Because `LinkedLine` and the answered note read `linkedIds`, a pending link toggle shows at once in both. The editor is keyed implicitly by the card (`key={sample.id}` in the page), so its drafts can never carry over to another sample.

- [ ] **Step 2: `SampleCard.module.css`**

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.card {
  display: flex;
  flex-direction: column;
  gap: var(--space-3);
  padding: var(--space-4);
  background: var(--color-surface-raised);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-lg);
  box-shadow: var(--shadow-sm);
}

.card:focus {
  outline: none;
}

.focused {
  box-shadow: inset 3px 0 0 var(--color-accent), var(--shadow-sm);
  border-color: var(--color-accent);
}

.head {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: var(--space-2);
}

.title {
  margin: 0;
  font-family: var(--font-display);
  font-size: var(--font-size-md);
  font-weight: var(--font-weight-bold);
}

.linked {
  margin: 0 0 0 auto;
  display: inline-flex;
  flex-wrap: wrap;
  align-items: baseline;
  gap: var(--space-2);
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
}

.typeLink {
  color: inherit;
  text-decoration: underline;
  text-decoration-color: var(--color-border-strong);
}

.typeLink:hover {
  color: var(--color-accent-deep);
}

.what {
  margin: 0;
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
}

.actions,
.editorActions {
  display: flex;
  flex-wrap: wrap;
  gap: var(--space-2);
}

.editorActions {
  justify-content: space-between;
}

.note {
  margin: 0;
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
}

.btnGhost,
.btnPrimary,
.btnDanger {
  padding: var(--space-2) var(--space-3);
  border-radius: var(--radius-md);
  font-size: var(--font-size-sm);
  font-weight: var(--font-weight-bold);
  white-space: nowrap;
  cursor: pointer;
}

.btnGhost,
.btnDanger {
  background: var(--color-surface-raised);
  border: 1px solid var(--color-border);
  color: var(--color-text);
}

.btnDanger {
  color: var(--tone-crit-ink);
}

.btnGhost:hover,
.btnDanger:hover {
  background: var(--color-surface-sunken);
}

.btnPrimary {
  background: var(--color-accent-deep);
  color: var(--color-on-accent);
  border: none;
}

.btnGhost:disabled,
.btnDanger:disabled,
.btnPrimary:disabled {
  cursor: not-allowed;
  opacity: 0.6;
}

.btnPrimary:disabled {
  background: var(--color-text-faint);
  opacity: 1;
}

.textBtn {
  padding: 0;
  border: none;
  background: none;
  font: inherit;
  color: var(--color-accent-deep);
  text-decoration: underline;
  cursor: pointer;
}

.textBtn:disabled {
  color: var(--color-text-faint);
  cursor: not-allowed;
}

.btnGhost:focus-visible,
.btnPrimary:focus-visible,
.btnDanger:focus-visible,
.textBtn:focus-visible,
.typeLink:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: 2px;
}

.editor {
  display: flex;
  flex-direction: column;
  gap: var(--space-3);
  padding-top: var(--space-3);
  border-top: 1px solid var(--color-border);
}

.fields {
  display: grid;
  grid-template-columns: minmax(0, 1fr) minmax(0, 2fr);
  gap: var(--space-3);
}

@media (max-width: 720px) {
  .fields {
    grid-template-columns: minmax(0, 1fr);
  }
}

.field {
  display: flex;
  flex-direction: column;
  gap: var(--space-1);
}

.label {
  font-size: var(--font-size-2xs);
  font-weight: var(--font-weight-bold);
  text-transform: uppercase;
  letter-spacing: var(--tracking-caps);
  color: var(--color-text-muted);
}

.input {
  padding: var(--space-1) var(--space-2);
  background: var(--color-surface-sunken);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-md);
  font-size: var(--font-size-sm);
  color: var(--color-text);
}

.input:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: 0;
}
```

(`opacity: 0.6` is a unitless state value, not a colour, size or radius. If `token_lint.py` flags it, replace both disabled rules with `color: var(--color-text-faint)`.)

- [ ] **Step 3: Lint, typecheck, commit.**

```bash
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/miljoe/SampleCard.module.css apps/rux/frontend/src/components/miljoe/SampleCard.tsx --tsx
npm --prefix apps/rux/frontend run typecheck
git add apps/rux/frontend/src/components/miljoe/SampleCard.{tsx,module.css}
git commit -m "feat(gui): sample card with stage actions and inline editor" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 8: NewSampleForm

**Files:** Create `src/components/miljoe/NewSampleForm.tsx` and `NewSampleForm.module.css`.

**Interfaces:**

```tsx
export interface NewSampleFormProps {
  types: SurveyType[];
  /** Pre-linked types (`/miljoe?ny=<typeId>`). */
  initialTypeIds: readonly number[];
  busy: boolean;
  onSubmit: (body: SampleCreate) => void;
  onCancel: () => void;
}
export function NewSampleForm(props: NewSampleFormProps): JSX.Element;
```

- [ ] **Step 1: Implement `NewSampleForm.tsx`**

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState, type KeyboardEvent } from 'react';

import type { SampleCreate, SurveyType } from '../../api/types';
import { kindOf } from '../../app/keyTargets';
import { createBody, editorKeyAction, toggleLink } from '../../miljoe/model';
import { LinkPicker } from './LinkPicker';
import styles from './NewSampleForm.module.css';

export interface NewSampleFormProps {
  types: SurveyType[];
  initialTypeIds: readonly number[];
  busy: boolean;
  onSubmit: (body: SampleCreate) => void;
  onCancel: () => void;
}

/**
 * `+ Ny prøve`: title, what was sampled, and the types it covers. The server
 * assigns the P-## code and starts the sample at Planlagt. Nothing is sent
 * until `Registrér prøve`; Esc cancels from anywhere in the form.
 */
export function NewSampleForm({ types, initialTypeIds, busy, onSubmit, onCancel }: NewSampleFormProps) {
  const [title, setTitle] = useState('');
  const [what, setWhat] = useState('');
  const [typeIds, setTypeIds] = useState<number[]>(() => [...initialTypeIds]);
  const body = createBody(title, what, typeIds);

  function submit() {
    if (body && !busy) onSubmit(body);
  }

  function onKeyDown(e: KeyboardEvent<HTMLFormElement>) {
    const action = editorKeyAction({ key: e.key, kind: kindOf(e.target), ctrlKey: e.ctrlKey, metaKey: e.metaKey, altKey: e.altKey });
    if (action === 'revert' || action === 'close') {
      e.preventDefault();
      onCancel();
    } else if (action === 'submit') {
      e.preventDefault();
      submit();
    }
    // 'commit' (plain Enter in a text input) falls through to the native submit.
  }

  return (
    <form
      className={styles.form}
      aria-labelledby="ny-proeve-title"
      onSubmit={(e) => {
        e.preventDefault();
        submit();
      }}
      onKeyDown={onKeyDown}
    >
      <h3 id="ny-proeve-title" className={styles.title}>
        Ny prøve
      </h3>
      <div className={styles.fields}>
        <div className={styles.field}>
          <label className={styles.label} htmlFor="ny-proeve-titel">
            Titel
          </label>
          <input
            id="ny-proeve-titel"
            className={styles.input}
            value={title}
            onChange={(e) => setTitle(e.target.value)}
            placeholder="fx PCB i fugemasse"
            autoFocus
            required
          />
        </div>
        <div className={styles.field}>
          <label className={styles.label} htmlFor="ny-proeve-hvad">
            Hvad er udtaget, og hvor
          </label>
          <input
            id="ny-proeve-hvad"
            className={styles.input}
            value={what}
            onChange={(e) => setWhat(e.target.value)}
            placeholder="fx Fugemasse omkring vinduespartier, Office Zone"
          />
        </div>
      </div>
      <LinkPicker types={types} selected={typeIds} onToggle={(id) => setTypeIds((ids) => toggleLink(ids, id))} />
      <p className={styles.hint}>Koden (P-##) tildeles automatisk. Prøven starter som Planlagt.</p>
      <div className={styles.actions}>
        <button type="button" className={styles.btnGhost} onClick={onCancel}>
          Annullér
        </button>
        <button type="submit" className={styles.btnPrimary} disabled={busy || body === null}>
          Registrér prøve
        </button>
      </div>
    </form>
  );
}
```

- [ ] **Step 2: `NewSampleForm.module.css`.** Copy the `.field`, `.label`, `.input`, `.input:focus-visible`, `.btnGhost`, `.btnPrimary`, disabled and focus-visible rules from `SampleCard.module.css` verbatim, and add:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.form {
  display: flex;
  flex-direction: column;
  gap: var(--space-3);
  padding: var(--space-4);
  background: var(--color-surface-raised);
  border: 1px solid var(--color-accent);
  border-radius: var(--radius-lg);
  box-shadow: var(--shadow-sm);
}

.title {
  margin: 0;
  font-family: var(--font-display);
  font-size: var(--font-size-md);
  text-transform: uppercase;
}

.fields {
  display: grid;
  grid-template-columns: minmax(0, 1fr) minmax(0, 2fr);
  gap: var(--space-3);
}

@media (max-width: 720px) {
  .fields {
    grid-template-columns: minmax(0, 1fr);
  }
}

.hint {
  margin: 0;
  font-size: var(--font-size-xs);
  color: var(--color-text-faint);
}

.actions {
  display: flex;
  justify-content: flex-end;
  gap: var(--space-2);
}
```

(The copied rules stay local on purpose: CSS Modules are per component, as in Phase 3's `KortlaegningPage.module.css`. A shared button module is a cross-screen refactor, out of scope.)

- [ ] **Step 3: Lint, typecheck, commit.**

```bash
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/miljoe/NewSampleForm.module.css apps/rux/frontend/src/components/miljoe/NewSampleForm.tsx --tsx
npm --prefix apps/rux/frontend run typecheck
git add apps/rux/frontend/src/components/miljoe/NewSampleForm.{tsx,module.css}
git commit -m "feat(gui): register-a-sample form" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 9: Miljø & prøver page, route, live navigation and badge

**Files:**
- Create `src/routes/MiljoePage.tsx` and `MiljoePage.module.css`.
- Modify `src/app/App.tsx`, `src/app/navigation.ts`, `src/app/AppShell.tsx` and `src/test/navigation.test.ts`.

**Interfaces:**
- Produces the `/miljoe` route.
- `navigation.ts`: the Miljø & prøver entry loses `pending`.
- `AppShell` passes `badges={{ reviewQueue, pendingSamples }}`, with `pendingSamples = survey.data?.pending_samples` (no badge on error) (R7).

- [ ] **Step 1: Navigation test first.** In `src/test/navigation.test.ts` add:

```ts
  it('makes Miljø & prøver a live destination', () => {
    const entry = NAV_ENTRIES.find((e) => e.to === '/miljoe');
    expect(entry).toBeDefined();
    expect(entry?.pending).toBeUndefined();
  });
```

Run `npm --prefix apps/rux/frontend test -- navigation` → FAIL. In `src/app/navigation.ts`, replace the `/miljoe` entry with `{ to: '/miljoe', label: 'Miljø & prøver', group: 'sag', badge: 'pendingSamples' },`. Run again → PASS.

- [ ] **Step 2: The page** — `src/routes/MiljoePage.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useRef, useState } from 'react';
import { useLocation } from 'react-router-dom';

import { api } from '../api/client';
import type { Sample, SampleCreate, SamplePatch, SampleResult, SurveyType } from '../api/types';
import { parseMiljoeQuery } from '../app/links';
import { saveErrorMessage } from '../app/saveError';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { useSurveyCounts } from '../app/SurveyCountsContext';
import { useToast } from '../app/useToast';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { NewSampleForm } from '../components/miljoe/NewSampleForm';
import { SampleCard } from '../components/miljoe/SampleCard';
import {
  addSample,
  advancePatch,
  gateChanges,
  gateMessage,
  removeSample,
  replaceSample,
  resultPatch,
  resultToast,
  toggleLink,
  UNDO_RESULT_PATCH,
} from '../miljoe/model';
import styles from './MiljoePage.module.css';

/**
 * Miljø & prøver — the environmental samples that gate approval in
 * Kortlægning. One card per sample: its stage chain, the next step or the
 * result buttons, and the survey types it covers.
 *
 * Every write runs on one serial chain. After each one, the same queued task
 * re-reads `GET /survey`, so the linked types' miljøstatus is the server's,
 * and the toast says what the change did to the approval gate. `refresh()`
 * then re-reads the summary behind both sidebar badges.
 */
export function MiljoePage() {
  const location = useLocation();
  const query = parseMiljoeQuery(location.search);
  const { data, error, loading, reload } = useAsync((s) => Promise.all([api.samples(s), api.survey(s)]), []);
  const { refresh } = useSurveyCounts();
  const toast = useToast(2600);
  const { busy, mutate } = useMutationQueue({
    onError: (cause) => toast.show(saveErrorMessage(cause)),
    onSettled: refresh,
  });

  const [samples, setSamplesState] = useState<Sample[]>([]);
  const samplesRef = useRef<Sample[]>([]);
  const setSamples = useCallback((update: (prev: Sample[]) => Sample[]) => {
    samplesRef.current = update(samplesRef.current);
    setSamplesState(samplesRef.current);
  }, []);
  const [types, setTypesState] = useState<SurveyType[]>([]);
  // Mirrors `types` synchronously: a queued task's gate diff compares against
  // the snapshot the previous task produced, not a stale render's.
  const typesRef = useRef<SurveyType[]>([]);
  const setTypes = useCallback((next: SurveyType[]) => {
    typesRef.current = next;
    setTypesState(next);
  }, []);

  const [loadedOnce, setLoadedOnce] = useState(false);
  const [editingId, setEditingId] = useState<number | null>(null);
  const [creating, setCreating] = useState(query.newForType !== null);
  const [focusedId, setFocusedId] = useState<number | null>(query.sampleId);

  // Link toggles are field commits: each sends the full desired set at once
  // (PUT replaces the set, so the last request wins), and the checkboxes show
  // that desired set until the last queued PUT for the sample has settled.
  const [linkDrafts, setLinkDrafts] = useState<ReadonlyMap<number, readonly number[]>>(() => new Map());
  const linkDraftsRef = useRef<ReadonlyMap<number, readonly number[]>>(new Map());
  const linkSeq = useRef(new Map<number, number>());
  const setLinkDraft = useCallback((id: number, ids: readonly number[] | null) => {
    const next = new Map(linkDraftsRef.current);
    if (ids === null) next.delete(id);
    else next.set(id, ids);
    linkDraftsRef.current = next;
    setLinkDrafts(next);
  }, []);

  const cardRefs = useRef(new Map<number, HTMLElement>());

  useEffect(() => {
    if (!data) return;
    setSamples(() => data[0]);
    setTypes(data[1].types);
    setLoadedOnce(true);
  }, [data, setSamples, setTypes]);

  // A new deep link on the mounted page (e.g. the sidebar after ?sample=) re-targets.
  useEffect(() => {
    const q = parseMiljoeQuery(location.search);
    setFocusedId(q.sampleId);
    if (q.newForType !== null) setCreating(true);
  }, [location.search]);

  // Scroll to and focus the deep-linked (or just created) card. A stale id
  // (deleted sample) simply finds no card.
  useEffect(() => {
    if (!loadedOnce || focusedId === null) return;
    const el = cardRefs.current.get(focusedId);
    el?.scrollIntoView({ block: 'center' });
    el?.focus({ preventScroll: true });
  }, [loadedOnce, focusedId]);

  /** Re-read the survey and say what the change did to the approval gate. */
  async function refreshGate(code: string): Promise<string | null> {
    const before = typesRef.current;
    try {
      const survey = await api.survey();
      setTypes(survey.types);
      return gateMessage(code, gateChanges(before, survey.types));
    } catch {
      return 'Gemt — men miljøstatus kunne ikke genindlæses. Åbn Kortlægning for at se den.';
    }
  }

  function patch(sample: Sample, body: SamplePatch, fallback: string | null) {
    mutate(
      async () => {
        const updated = await api.patchSample(sample.id, body);
        setSamples((prev) => replaceSample(prev, updated));
        const message = (await refreshGate(updated.code)) ?? fallback;
        if (message) toast.show(message);
      },
      () => toast.show(`${sample.code}: svaret kan først registreres, når prøven er sendt til lab.`),
    );
  }

  // Buttons: gated on `busy`, so a double click cannot stack requests.
  function advance(sample: Sample) {
    const body = advancePatch(sample);
    if (busy || !body) return;
    patch(sample, body, null);
  }

  function recordResult(sample: Sample, result: SampleResult) {
    if (busy) return;
    patch(sample, resultPatch(result), resultToast(sample.code, result));
  }

  function undoResult(sample: Sample) {
    if (busy) return;
    patch(sample, UNDO_RESULT_PATCH, `${sample.code}: svaret er fortrudt — afventer igen svar fra lab`);
  }

  function remove(sample: Sample) {
    if (busy) return;
    mutate(async () => {
      await api.deleteSample(sample.id);
      setSamples((prev) => removeSample(prev, sample.id));
      setEditingId((id) => (id === sample.id ? null : id));
      const message = await refreshGate(sample.code);
      toast.show(message ?? `${sample.code} slettet`);
    });
  }

  function create(body: SampleCreate) {
    if (busy) return;
    mutate(async () => {
      const created = await api.createSample(body);
      setSamples((prev) => addSample(prev, created));
      setCreating(false);
      setFocusedId(created.id);
      const message = await refreshGate(created.code);
      toast.show(message ?? `${created.code} registreret`);
    });
  }

  // Field commits: never gated on `busy`, never dropped — they wait their turn.
  function editText(sample: Sample, body: Pick<SamplePatch, 'title' | 'what'>) {
    patch(sample, body, null);
  }

  function onToggleLink(sample: Sample, typeId: number) {
    const base = linkDraftsRef.current.get(sample.id) ?? sample.type_ids;
    const next = toggleLink(base, typeId);
    setLinkDraft(sample.id, next);
    const seq = (linkSeq.current.get(sample.id) ?? 0) + 1;
    linkSeq.current.set(sample.id, seq);
    mutate(async () => {
      try {
        const updated = await api.setSampleLinks(sample.id, next);
        setSamples((prev) => replaceSample(prev, updated));
        const message = await refreshGate(updated.code);
        if (message) toast.show(message);
      } finally {
        // Only the last toggle hands the checkboxes back to the server's set;
        // on failure that set is whatever the last successful PUT left.
        if (linkSeq.current.get(sample.id) === seq) setLinkDraft(sample.id, null);
      }
    });
  }

  // --------------------------------------------------------------- render --

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="Miljø & prøver" />
      </div>
    );
  }
  if ((loading && !data) || (data && !loadedOnce)) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser prøver…" />
      </div>
    );
  }

  const preLinked =
    query.newForType !== null && types.some((t) => t.id === query.newForType && t.review_status !== 'rejected')
      ? [query.newForType]
      : [];

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h2 className={styles.title}>Miljø & prøver</h2>
        <span className={styles.sub}>Prøver styrer miljøstatus på de koblede bygningsdele</span>
        <button type="button" className={styles.btnPrimary} onClick={() => setCreating(true)} disabled={creating}>
          + Ny prøve
        </button>
      </header>

      {creating && (
        <NewSampleForm
          key={query.newForType ?? 'ny'}
          types={types}
          initialTypeIds={preLinked}
          busy={busy}
          onSubmit={create}
          onCancel={() => setCreating(false)}
        />
      )}

      {samples.length === 0 && !creating ? (
        <EmptyState
          title="Ingen prøver endnu"
          detail="Registrér en miljøprøve og kobl den til de typer i kortlægningen, den dækker. Indtil svaret foreligger, kan typerne ikke godkendes."
          action={
            <button type="button" className={styles.btnPrimary} onClick={() => setCreating(true)}>
              + Ny prøve
            </button>
          }
        />
      ) : (
        <ul className={styles.list} aria-label="Prøver">
          {samples.map((s) => (
            <li key={s.id}>
              <SampleCard
                sample={s}
                types={types}
                linkedIds={linkDrafts.get(s.id) ?? s.type_ids}
                busy={busy}
                focused={focusedId === s.id}
                editing={editingId === s.id}
                onEditing={(open) => setEditingId(open ? s.id : null)}
                onAdvance={() => advance(s)}
                onResult={(r) => recordResult(s, r)}
                onUndoResult={() => undoResult(s)}
                onTitle={(title) => editText(s, { title })}
                onWhat={(what) => editText(s, { what })}
                onToggleLink={(typeId) => onToggleLink(s, typeId)}
                onDelete={() => remove(s)}
                cardRef={(el) => {
                  if (el) cardRefs.current.set(s.id, el);
                  else cardRefs.current.delete(s.id);
                }}
              />
            </li>
          ))}
        </ul>
      )}

      {samples.length > 0 && (
        <p className={styles.footnote}>
          Et prøvesvar opdaterer miljøstatus på alle koblede typer i kortlægningen — én prøve kan afklare mange
          bygningsdele.
        </p>
      )}

      <Toast message={toast.message} />
    </div>
  );
}
```

Callbacks inside `patch`/`create` read the sample passed in only for its `id` and `code`, which never change. Responses always replace the local copy (`replaceSample`), so a field commit queued behind a result shows the merged server state.

- [ ] **Step 3: `MiljoePage.module.css`**

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.page {
  display: flex;
  flex-direction: column;
  gap: var(--space-4);
  padding: var(--space-5);
}

.head {
  display: flex;
  flex-wrap: wrap;
  align-items: baseline;
  gap: var(--space-3);
}

.title {
  font-size: var(--font-size-3xl);
  text-transform: uppercase;
}

.sub {
  font-size: var(--font-size-sm);
  color: var(--color-text-muted);
}

.btnPrimary {
  align-self: center;
  margin-left: auto;
  padding: var(--space-2) var(--space-3);
  border: none;
  border-radius: var(--radius-md);
  background: var(--color-accent-deep);
  color: var(--color-on-accent);
  font-size: var(--font-size-sm);
  font-weight: var(--font-weight-bold);
  white-space: nowrap;
  cursor: pointer;
}

.btnPrimary:disabled {
  background: var(--color-text-faint);
  cursor: not-allowed;
}

.btnPrimary:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: 2px;
}

.list {
  display: flex;
  flex-direction: column;
  gap: var(--space-3);
  margin: 0;
  padding: 0;
  list-style: none;
}

.footnote {
  margin: 0;
  font-size: var(--font-size-xs);
  color: var(--color-text-faint);
}
```

- [ ] **Step 4: Route.** In `src/app/App.tsx`, import `MiljoePage` from `'../routes/MiljoePage'` and add `<Route path="/miljoe" element={<MiljoePage />} />` directly after the `/kortlaegning` route.

- [ ] **Step 5: Badge.** In `src/app/AppShell.tsx`:
  - after the `reviewQueue` line, add `const pendingSamples = survey.error ? undefined : survey.data?.pending_samples;`;
  - change the Sidebar prop to `badges={{ reviewQueue, pendingSamples }}`;
  - extend the doc comment's badge paragraph to: "The Kortlægning and Miljø & prøver badges both come from `GET /survey/summary` (`counts.queue`, `pending_samples`: samples not yet at *svar*, i.e. those that can hold a type at *afventer prøve*)."

- [ ] **Step 6: Gates.** Run:

```bash
npm --prefix apps/rux/frontend test && npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend run build
python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/routes/MiljoePage.module.css apps/rux/frontend/src/routes/MiljoePage.tsx --tsx
```

- [ ] **Step 7: Commit**

```bash
git add apps/rux/frontend/src/routes/MiljoePage.{tsx,module.css} apps/rux/frontend/src/app/{App.tsx,navigation.ts,AppShell.tsx} apps/rux/frontend/src/test/navigation.test.ts
git commit -m "feat(gui): Miljø & prøver at /miljoe with the pending-samples badge" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 10: Kortlægning ↔ Miljø links

**Files:**
- Create `src/components/kortlaegning/SampleLine.tsx` and `SampleLine.module.css`.
- Modify `src/components/kortlaegning/DetailPanel.tsx`, `src/components/kortlaegning/EditDialog.tsx`, `src/routes/KortlaegningPage.tsx` and `src/test/kortlaegning.detailPanel.test.ts`.

**Interfaces:**

```ts
// DetailPanel.tsx (new export; sampleLineText keeps its exact output)
export type SampleLineModel =
  | { kind: 'none'; typeId: number }
  | { kind: 'linked'; items: { id: number; text: string }[] };
export function sampleLineModel(type: SurveyType, samples: Sample[]): SampleLineModel;
// SampleLine.tsx
export function SampleLine(props: { type: SurveyType; samples: Sample[]; className?: string }): JSX.Element;
```

- [ ] **Step 1: Failing test.** Append to `src/test/kortlaegning.detailPanel.test.ts` (add `sampleLineModel` to the DetailPanel import; use the file's existing type/sample builders):

```ts
describe('sampleLineModel', () => {
  it('links each linked sample by id with the same text the line shows', () => {
    const t = { ...type(), id: 6, sample_ids: [1] };
    const s = [{ ...sample(), id: 1, code: 'P-01', title: 'PCB i fugemasse', stage: 'sendt' as const, result: null }];
    expect(sampleLineModel(t, s)).toEqual({
      kind: 'linked',
      items: [{ id: 1, text: 'P-01 · PCB i fugemasse — Sendt til lab' }],
    });
    expect(sampleLineText(t, s)).toBe('Miljøstatus styres af P-01 · PCB i fugemasse — Sendt til lab');
  });

  it('offers registering a sample for a type with none', () => {
    expect(sampleLineModel({ ...type(), id: 9, sample_ids: [] }, [])).toEqual({ kind: 'none', typeId: 9 });
  });
});
```

If that file's builders have other names, use them. The values that matter are `id`, `sample_ids`, `code`, `title`, `stage` and `result`.

Run → FAIL.

- [ ] **Step 2: Implement the model** in `DetailPanel.tsx`. Replace the body of `sampleLineText` and add `sampleLineModel` above it:

```ts
/** The sample line as data: each linked sample with its text, or the type to register one for. */
export type SampleLineModel =
  | { kind: 'none'; typeId: number }
  | { kind: 'linked'; items: { id: number; text: string }[] };

export function sampleLineModel(type: SurveyType, samples: Sample[]): SampleLineModel {
  const linked = linkedSamples(type, samples);
  if (linked.length === 0) return { kind: 'none', typeId: type.id };
  return {
    kind: 'linked',
    items: linked.map((s) => {
      const line = `${s.code} · ${s.title} — ${STAGE_LABEL[s.stage]}`;
      return { id: s.id, text: s.result ? `${line} · ${RESULT_LABEL[s.result]}` : line };
    }),
  };
}

export function sampleLineText(type: SurveyType, samples: Sample[]): string {
  const m = sampleLineModel(type, samples);
  return m.kind === 'none'
    ? 'Ingen prøve koblet — miljøstatus fra screening: ren.'
    : `Miljøstatus styres af ${m.items.map((i) => i.text).join(', ')}`;
}
```

Run → PASS: the old `sampleLineText` tests still pass.

- [ ] **Step 3: SampleLine component** — `SampleLine.tsx`:

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link } from 'react-router-dom';

import type { Sample, SurveyType } from '../../api/types';
import { newSampleHref, sampleHref } from '../../app/links';
import { sampleLineModel } from './DetailPanel';
import styles from './SampleLine.module.css';

/**
 * "Miljøstatus styres af …" with each sample a link to its card in Miljø &
 * prøver; for a type with no sample, a link that opens the create form with
 * the type pre-linked. Shared by DetailPanel and EditDialog.
 */
export function SampleLine({ type, samples, className }: { type: SurveyType; samples: Sample[]; className?: string }) {
  const m = sampleLineModel(type, samples);
  if (m.kind === 'none') {
    return (
      <p className={className}>
        Ingen prøve koblet — miljøstatus fra screening: ren.{' '}
        <Link className={styles.link} to={newSampleHref(m.typeId)}>
          Registrér prøve
        </Link>
      </p>
    );
  }
  return (
    <p className={className}>
      Miljøstatus styres af{' '}
      {m.items.map((item, i) => (
        <span key={item.id}>
          {i > 0 && ', '}
          <Link className={styles.link} to={sampleHref(item.id)}>
            {item.text}
          </Link>
        </span>
      ))}
    </p>
  );
}
```

`SampleLine.module.css`:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.link {
  color: var(--color-accent-deep);
  text-decoration: underline;
}

.link:hover {
  color: var(--color-text);
}

.link:focus-visible {
  outline: 2px solid var(--color-border-focus);
  outline-offset: 2px;
  border-radius: var(--radius-sm);
}
```

- [ ] **Step 4: Use it.**
  - `DetailPanel.tsx`: replace `<p className={styles.sampleLine}>{sampleLineText(type, samples)}</p>` with `<SampleLine className={styles.sampleLine} type={type} samples={samples} />` and import `SampleLine` from `'./SampleLine'`. Because `SampleLine.tsx` imports from `DetailPanel.tsx`, the cycle is type-and-function only and resolves at call time. If the bundler warns, move `linkedSamples`/`sampleLineModel`/`sampleLineText` into `src/kortlaegning/samples.ts` and re-export them from `DetailPanel.tsx` so the existing tests keep their import.
  - `EditDialog.tsx`: make the same replacement and drop `sampleLineText` from its `./DetailPanel` import.
  - Update the DetailPanel doc comment: replace "plain text (Miljø & prøver is Phase 4)", or its equivalent, with "links to the sample in Miljø & prøver".

- [ ] **Step 5: `?type=` deep link in KortlaegningPage.** Add `import { useLocation } from 'react-router-dom';`, `import { parseTypeQuery } from '../app/links';`, and `initialViewFor` to the `../kortlaegning/model` import. After `const toast = …`/`useMutationQueue`, add:

```ts
  const location = useLocation();
  // `/kortlaegning?type=<id>` (from a sample's "Koblet:" link) selects that
  // type once, in the tab it lives in. Applied in the same effect that seeds
  // `types`, so the first render with data already has the right tab and the
  // keep-selection-visible effect below finds the selection shown.
  const deepLinkType = useRef(parseTypeQuery(location.search));
```

Then replace the data-seeding effect with:

```ts
  useEffect(() => {
    if (!data) return;
    setTypes(() => data[0].types);
    const want = deepLinkType.current;
    if (want !== null) {
      deepLinkType.current = null;
      const view = initialViewFor(data[0].types, want);
      if (view) {
        setTab(view.tab);
        setFilters(NO_FILTERS);
        select(view.selection);
      }
    }
    setLoadedOnce(true);
  }, [data, setTypes, select]);
```

- [ ] **Step 6: Gates.** Run `npm --prefix apps/rux/frontend test && npm --prefix apps/rux/frontend run typecheck && npm --prefix apps/rux/frontend run build`, then `python3 .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/kortlaegning/SampleLine.module.css apps/rux/frontend/src/components/kortlaegning/SampleLine.tsx --tsx`.

- [ ] **Step 7: Commit**

```bash
git add apps/rux/frontend/src/components/kortlaegning/{SampleLine.tsx,SampleLine.module.css,DetailPanel.tsx,EditDialog.tsx} apps/rux/frontend/src/routes/KortlaegningPage.tsx apps/rux/frontend/src/test/kortlaegning.detailPanel.test.ts
git commit -m "feat(gui): Kortlægning sample line links to Miljø & prøver; ?type= deep link" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

---

### Task 11: Verify against the prototype and the flows

**Files:** None in the repo. The scripts and shots go to the scratchpad.

- [ ] **Step 1: Static shots in both themes** on the plain seed (prototype data)

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase4
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/miljoe-demo.rux"
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/miljoe-demo.rux"
bash .claude/skills/design-studio/scripts/shot.sh http://localhost:5173/miljoe --out "$SP/shots/p4" --theme light --viewports desktop --wait 2500
bash .claude/skills/design-studio/scripts/shot.sh http://localhost:5173/miljoe --out "$SP/shots/p4" --theme dark --viewports desktop,mobile --wait 2500
```

Open the PNGs with Read and compare with `docs/gui/images/prototype-v2/miljoe.png`:
- the sidebar's Miljø & prøver entry is active and shows a neutral **2**, while Kortlægning shows **7**;
- the head reads `MILJØ & PRØVER` with the sub line and `+ Ny prøve` (no PDF button, R6);
- three cards in order P-01 / P-02 / P-03 with pills `Sendt til lab` / `Forurenet` / `Udtaget`;
- `Koblet: Vinduespartier, aluminium` / `Indvendige murvægge, malet` / `Gulvbelægning, linoleum` right-aligned, followed by `Rediger`;
- the stage chains' green/accent/hollow states as described above;
- P-01's two result buttons, P-02's answered note with `Fortryd svar`, P-03's `Næste trin →`;
- the factual footnote;
- in dark, no light-only colour;
- on mobile, the `Koblet:` line wraps under the title without horizontal scroll.

Fix what differs, then re-shoot.

- [ ] **Step 2: Flow script.** Write `$SP/miljoe_flow.py`:

```python
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Drive the Miljø & prøver flows on the seeded demo and screenshot each step."""
import re
import sys

from playwright.sync_api import expect, sync_playwright

base, out = sys.argv[1], sys.argv[2]

with sync_playwright() as p:
    browser = p.chromium.launch()
    page = browser.new_page(viewport={"width": 1440, "height": 900})
    page.add_init_script("localStorage.setItem('reusex-theme', 'light')")
    page.on("dialog", lambda d: d.accept())
    nav = lambda: page.get_by_role("link", name=re.compile("Miljø & prøver"))

    def toast(text):
        # Two role=status regions exist (the page Toast and the JobToaster);
        # pick the one carrying the message.
        expect(page.get_by_role("status").filter(has_text=text)).to_be_visible()

    page.goto(f"{base}/miljoe")
    p01 = page.get_by_role("article", name="P-01 · PCB i fugemasse")
    expect(p01).to_be_visible()
    expect(nav()).to_contain_text("2")

    # Result entry from 'sendt': one PATCH, gate feedback names the unblocked type.
    p01.get_by_role("button", name="Registrér svar: Ren").click()
    toast("1 type kan nu godkendes (Vinduespartier, aluminium)")
    expect(p01).to_contain_text("Svar registreret")
    expect(nav()).to_contain_text("1")
    page.screenshot(path=f"{out}/01-result.png")

    # Advance a stage.
    p03 = page.get_by_role("article", name="P-03 · Asbest i linoleumslim")
    p03.get_by_role("button", name="Næste trin →").click()
    expect(p03.locator("[aria-current=step]")).to_have_text("Sendt til lab")
    page.screenshot(path=f"{out}/02-advance.png")

    # Rapid link toggles: all land, last state wins.
    p03.get_by_role("button", name="Rediger").click()
    box = p03.get_by_role("checkbox", name=re.compile("Indvendige døre, træ"))
    for _ in range(5):
        box.click()
    expect(box).to_be_checked()
    toast("P-03: Indvendige døre, træ afventer nu prøvesvar")
    page.wait_for_timeout(1500)  # every queued PUT has settled; no flicker back
    expect(box).to_be_checked()
    expect(p03).to_contain_text("Indvendige døre, træ")
    page.screenshot(path=f"{out}/03-links.png")

    # Untouched blur sends nothing; Esc reverts.
    title = p03.get_by_label("Titel")
    title.focus()
    title.blur()
    title.fill("Ændret titel")
    title.press("Escape")
    expect(title).to_have_value("Asbest i linoleumslim")

    # Register a new sample, pre-linked from Kortlægning's link.
    page.goto(f"{base}/miljoe?ny=2")
    page.get_by_label("Titel").fill("Røgprøve")
    expect(page.get_by_role("checkbox", name=re.compile("Betonsøjler, bærende"))).to_be_checked()
    page.get_by_role("button", name="Registrér prøve").click()
    p04 = page.get_by_role("article", name="P-04 · Røgprøve")
    expect(p04).to_be_visible()
    toast("Betonsøjler, bærende afventer nu prøvesvar")
    page.screenshot(path=f"{out}/04-created.png")

    # Delete it again (confirm accepted).
    p04.get_by_role("button", name="Rediger").click()
    p04.get_by_role("button", name="Slet prøve").click()
    expect(p04).to_have_count(0)

    # Kortlægning: the un-gated type can be approved; its sample line links back.
    page.goto(f"{base}/kortlaegning?type=6")
    expect(page.get_by_role("button", name="Godkend mængde ✓")).to_be_enabled()
    page.screenshot(path=f"{out}/05-kortlaegning.png")
    page.get_by_role("link", name=re.compile("P-01 · PCB i fugemasse")).first.click()
    expect(page).to_have_url(re.compile(r"/miljoe\?sample=1$"))
    expect(page.get_by_role("article", name="P-01 · PCB i fugemasse")).to_be_focused()
    page.screenshot(path=f"{out}/06-deeplink.png")

    # Undo goes back to sendt and re-gates the type.
    page.get_by_role("article", name="P-01 · PCB i fugemasse").get_by_role("button", name="Fortryd svar").click()
    toast("Vinduespartier, aluminium afventer nu prøvesvar")
    browser.close()
print("flow OK")
```

Run it on a fresh seed, because the flow mutates:

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase4
SP=/tmp/claude-1001/-home-mephisto-repos-ReUseX/3f5d811b-b3af-447f-a3ba-505a5e85dab5/scratchpad
bash .claude/skills/design-studio/scripts/dev_env.sh stop
PATH="$PWD/build/apps/rux:$PATH" bash apps/rux/frontend/dev/seed-survey-demo.sh "$SP/corridor-clouds.rux" "$SP/miljoe-flow.rux"
RUX_BIN="$PWD/build/apps/rux/rux" nix develop --command bash .claude/skills/design-studio/scripts/dev_env.sh start "$SP/miljoe-flow.rux"
mkdir -p "$SP/shots/p4-flow"
BROWSERS="$(nix build --no-link --print-out-paths nixpkgs#playwright-driver.browsers)"
PLAYWRIGHT_BROWSERS_PATH="$BROWSERS" nix shell --impure --expr 'let p = import (builtins.getFlake "nixpkgs") {}; in p.python3.withPackages (ps: [ ps.playwright ])' --command python3 "$SP/miljoe_flow.py" http://localhost:5173 "$SP/shots/p4-flow"
bash .claude/skills/design-studio/scripts/dev_env.sh stop
```

Expected: `flow OK`. Open the six PNGs with Read. A failing `expect` names the broken step: fix the code, not the script, unless the script contradicts this plan. In the Network panel equivalent, every PATCH body must be one of `{stage}`, `{stage:'svar',result}`, `{stage:'sendt',result:null}`, `{title}` or `{what}`. To check, add `page.on("request", …)` logging to the script while debugging.

- [ ] **Step 3: Varied data shot.** Seed with `--varied` and shoot `/miljoe` once in light. Confirm:
  - P-04 shows `Ren` and `Koblet: Trapezplader, tag, Isolering, mineraluld`, with the note "opdateret på 2 type(r)";
  - P-05 shows `Ikke koblet til en type` and `Næste trin →`;
  - the badge reads **3**.

No commit (verification only).

---

### Task 12: Docs

**Files:** `docs/design/gui-kortlaegning-redesign.md`, `apps/rux/frontend/README.md`, `.claude/skills/design-studio/references/reusex-frontend.md`.

- [ ] **Step 1: Spec** (`docs/design/gui-kortlaegning-redesign.md`):
  - In § Kortlægning domain model, change `` `sample_links` — many-to-many sample ↔ passport. `` to `` `sample_links` — many-to-many sample ↔ survey type. `` Add the sentence: "(Originally written as "passport"; miljøstatus and the approval gate are properties of a type, so the link is to the type — as implemented in Phase 2.)"
  - After § Kortlægning screen, add § **Miljø & prøver screen**, summarising the component list from this plan's "The prototype, component by component" plus what v1 adds:
    - the inline `Rediger` editor;
    - `Fortryd svar` back to *sendt*;
    - the combined `{stage: 'svar', result}` patch;
    - the gate-feedback toast;
    - the deep links `?sample=`, `?ny=` and `/kortlaegning?type=`;
    - the badge = `pending_samples`.
  - In the Detail panel bullet, change "sample line (plain text today — … not yet a link to Miljø & prøver, since that screen doesn't exist until Phase 4)" to "sample line, each sample a link to its card in Miljø & prøver (or `Registrér prøve` when none is linked)".
  - In § Out of scope, remove the "Linking the detail panel's sample line …" item, and add:
    - "Uploading the lab's miljørapport (PDF) to a sample — needs blob storage and an endpoint; the prototype's `Upload miljørapport (PDF)` button is not drawn until then."
    - "Rewinding a sample's stage beyond `Fortryd svar` (back to *sendt*)."
- [ ] **Step 2: Frontend README** (`apps/rux/frontend/README.md` § Layout):
  - add `miljoe/` to the tree: "Pure modules for Miljø & prøver: model.ts (stage chain, patches, link toggling, gate feedback), useTextDraft.ts";
  - add `components/miljoe/` (StageChain, LinkPicker, SampleCard, NewSampleForm);
  - add the shared `app/serialQueue.ts` / `useMutationQueue.ts` / `keyTargets.ts` / `links.ts`, with one line each.
- [ ] **Step 3: design-studio reference** (`.claude/skills/design-studio/references/reusex-frontend.md`):
  - in the directory map, add `miljoe/` and `components/miljoe/` in the same style as the kortlaegning lines;
  - add `MiljoePage` to the routes line;
  - add one sentence under the seed paragraph: "`--varied` adds two samples for Miljø & prøver (multi-link answered, unlinked planned)";
  - add the rule: "page writes go through `app/useMutationQueue` — never a second ad-hoc chain".
- [ ] **Step 4: Check and commit.**

```bash
cd /home/mephisto/repos/ReUseX/.worktrees/gui-phase4
nix develop --command reuse lint
git add docs/design/gui-kortlaegning-redesign.md apps/rux/frontend/README.md .claude/skills/design-studio/references/reusex-frontend.md
git commit -m "docs(gui): Miljø & prøver screen; samples link to survey types" --trailer "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>" --trailer "Claude-Session: https://claude.ai/code/session_01E7P8haSHHtqGupsuGGwzrf"
```

## Phase exit criteria

- Frontend `test`, `typecheck` and `build` pass. Token lint is clean on every new or changed `.module.css`/`.tsx`. `reuse lint` is compliant.
- Shots of `/miljoe` (light + dark, desktop + mobile) match `miljoe.png`'s structure on the plain seed, with deviations only as ruled (R4–R6 and the Rediger editor).
- `miljoe_flow.py` prints `flow OK`:
  - result entry un-gates Vinduespartier, and Kortlægning then enables `Godkend mængde ✓`;
  - the badge goes 2 → 1;
  - rapid link toggles settle on the last state;
  - Esc reverts a draft;
  - `?ny=` pre-links;
  - the sample-line link lands on a focused card;
  - `Fortryd svar` re-gates.
- Kortlægning behaves as in Phase 3: its tests pass and the Phase 3 dialog/approve flows are unchanged.
- Follow-up issues filed: miljørapport PDF upload (R6); stage rewind beyond `Fortryd svar` (R4).

## Self-review

- **Spec coverage:**
  - the samples screen (Task 9), stage chain (Tasks 4, 6), advance (Tasks 4, 7, 9), result entry restricted by the svar rule (R3, Tasks 4, 7, 9);
  - new sample (Task 8), link/unlink (Tasks 6, 7, 9), live nav + badge (Task 9, R7);
  - sample line → `/miljoe` (Task 10), approval-gate feedback and count refresh (Task 9 `refreshGate` + `onSettled: refresh`);
  - screenshots in both themes (Task 11), docs (Task 12).
- **Backend:** Task 1 Step 3 verifies each assumed endpoint behaviour, with a stop-and-add-a-backend-task rule if one fails.
- **Placeholders:** none. Every string, path and command is literal. The two "if the name differs" notes (the detailPanel test builders, a possible import cycle) name the exact fallback.
- **Type consistency:**
  - `SampleCardProps.linkedIds: readonly number[]` is fed `linkDrafts.get(id) ?? s.type_ids`;
  - `toggleLink` returns `number[]`, which `api.setSampleLinks(id, number[])` accepts;
  - `useMutationQueue.mutate`'s optional `(cause: ApiRequestError) => void` accepts Kortlægning's existing `() => void` callbacks;
  - `editorKeyAction` takes `TargetKind` from `keyTargets.ts`;
  - `initialViewFor` returns the model's own `Tab`/`Selection`.
- **Phase 3 lessons:**
  - serial chain (Task 2, used in Tasks 9/10);
  - commits never gated: title/what/link toggles call `mutate` without a `busy` check, and only buttons check `busy`;
  - untouched blur (`textCommit`, Task 5);
  - isField/isControl via `keyTargets` (Tasks 2, 7, 8);
  - tokens only, with lint in every UI task;
  - Danish copy; SPDX on every new file; pure, Node-tested logic.
