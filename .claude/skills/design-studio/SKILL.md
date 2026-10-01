---
name: design-studio
description: The design workflow for the ReUseX rux GUI — its web frontend (React/Vite/TS + three.js, in apps/rux/frontend) and its planned native Qt client, which share one token-based design system. Use this whenever you design, build, restyle, redesign, or "make it look better/more polished" for any rux GUI screen, page, panel, route or component (Dashboard, Viewport, Pipeline, Graph view, Image/annotation view, Materials, …); whenever you touch a *.module.css or add a new component; whenever you work from docs/Design Review.md; and for one-off mockups, prototypes, slide decks or one-pagers. Also use when matching the existing rux GUI look. Reach for it even when the user only says "build a page", "add a panel" or "wire up a view" — visual quality and token discipline still apply.
---

# Design Studio — ReUseX rux GUI

Claude Code writes good frontend code. What this skill adds is *process* and the
project's *house rules*: work inside the design system that already exists,
build in the real stack, **look** at the result, and iterate. The single most
important step is §5 — never ship a screen you haven't seen, in both themes.

This project has **two surfaces, one design system**:
- **Now:** the web frontend in `apps/rux/frontend/` — React 19 + Vite + TypeScript
  + three.js, CSS Modules.
- **Later:** a native **Qt 6** client (issue #265 runner-up). It does not exist
  yet, but every visual decision you make today is expressed as a *token* so the
  Qt port inherits the same look instead of re-picking it. See §Qt.

The system itself — every colour, size, radius, type step — lives in
**`apps/rux/frontend/src/tokens.css`**, which is **owned by the Claude Design
project "ReUseX GUI"** and synced with the `DesignSync` tool / `/design-sync`.
That ownership boundary drives almost everything below.

Deep project specifics (full token inventory, directory map, dev/proxy details,
Qt mirroring plan) live in `references/reusex-frontend.md`. Read it before your
first build in this repo.

## The loop

Brief → Locate in the system → (Direction, only if greenfield) → Build → Screenshot → Critique & fix → Hand off

### 1. Pin down the brief

If the request is vague, ask up to 3 short questions, and only ones whose
answers change the design:
- Which surface, screen and route? (e.g. a new panel on `ViewportPage`, the
  Graph view in `docs/Design Review.md`, a fresh route.)
- Fidelity: throwaway mockup to agree a layout, or production React to merge?
- Is there a contract for the data? New views usually need `docs/gui/openapi.yaml`
  + `docs/gui/websocket-events.md` support — the frontend is a *pure client*, so
  if the endpoint doesn't exist yet, say so and design against the intended shape.

If the user wants speed or has already said enough, state your assumptions in one
line and proceed. `docs/Design Review.md` is the current design backlog — treat
its entries as briefs.

### 2. Locate the work in the existing system (this replaces "invent a design system")

The design system already exists. Your job is to *conform*, not to create one.

- **Read `src/tokens.css` for the token names and their roles** (surfaces,
  border, text, accent, status, `--label-*`, spacing, type, radius, shadow,
  layout, motion, z-index). `references/reusex-frontend.md` has the full list.
- **Find the nearest existing component and copy its patterns.** Grep
  `src/components/` and `src/routes/` for something similar — a card, a table, a
  panel, a form. Reuse `DataTable`, `StatCard`, `Sidebar`, `EmptyState`,
  `ErrorBanner`, `ParameterForm`, `SelectDropdown`, etc. rather than inventing a
  parallel one. Consistency with the app beats novelty.
- **Hard rules that come from that ownership boundary:**
  - **Never hand-edit `src/tokens.css`.** Its values are placeholders until a
    `/design-sync`; the repo owns token *names/roles*, the design project owns
    every *value*. If a colour or size feels wrong, that's a re-sync, not an
    edit. Never create a `design-tokens.md` — this project already has its source
    of truth.
  - **No literal colour, radius, spacing or type size anywhere in `src/`** — only
    `var(--…)`. This is what makes a token change a one-line value swap instead
    of a refactor. `scripts/token_lint.py` enforces it; run it before you finish.
  - **SPDX header on every new file** (`.tsx`, `.ts`, `.css`) — copy the two/three
    line header from any neighbour. The REUSE lint job fails without it.

### 3. Choose a direction — only for genuinely greenfield work

Most work here is *additive*: a new panel or route that must look like the rest
of the app. In that case **skip this step** — the direction is "match the app",
and §2 is the whole job.

Only when the brief is open-ended (a brand-new kind of surface, or the user is
explicitly exploring) sketch 2–3 directions, a few lines each — palette drawn
from the *existing* tokens, layout as a one-line ASCII wireframe, and the one
memorable move. Even then, stay inside the established feel:
- **Light-default.** The workbench (`:root`) is light; `[data-theme='dark']`
  re-points the themed roles. The 3D viewport canvas stays near-black in
  *both* themes — that one surface is dark-first regardless of the workbench
  theme.
- **Dense, technical, calm.** This is an engineering tool over a `.rux` project —
  tabular numbers (`.mono`, `tabular-nums`), tight rhythm, restraint. Not
  marketing polish.

Avoid the usual AI tells (identical rounded cards with the same soft shadow,
tracked-out ALL-CAPS eyebrows on everything, fade-up on every section, gradient
hero). They read as templated and clash with a tool UI.

### 4. Build

**Production deliverable = the real stack**, not a self-contained HTML file:
- One `Foo.tsx` + one `Foo.module.css` per component, in the right directory
  (`components/` = presentational + contract-agnostic; `routes/` = page
  compositions; `viewport/` = three.js; `pipeline/` = pure stage logic; `api/` =
  contract layer). See the layout in `references/reusex-frontend.md`.
- **CSS Modules, tokens only.** Every colour/spacing/radius/type value is a
  `var(--…)`. Class names are local; compose with `styles.foo`.
- **Real copy**, specific to the domain ("Back-project depth frames", "Loop
  closures", "Unlabeled points") — not "Feature one". Buttons name the action.
- **Contract-faithful.** Types mirror `docs/gui/openapi.yaml` in `src/api/types.ts`;
  never invent fields. If the data doesn't exist yet, stub it behind the intended
  shape and flag the missing endpoint.
- **Quality floor:** keyboard focus visible (`:focus-visible` is themed), body
  contrast ≥ 4.5:1 in *both* themes, `prefers-reduced-motion` respected (the
  motion tokens already zero out under it), tap targets ≥ 44px, no horizontal
  overflow.

**Throwaway mockups are allowed** to agree a layout fast — a single HTML file is
fine — but it **must `@import "../src/tokens.css"`** so it uses the real system,
and it is a sketch, not the deliverable. Convert the agreed layout to React
before handing off.

### 5. Look at it (mandatory), in both themes

Run the app and screenshot it. The frontend is a client of `rux gui`, so bring
the backend up first (fixture project provided):

```bash
# one command: starts `rux gui` on a COPY of the fixture (never the tracked file) + the Vite dev server, prints URLs; state in .superpowers/dev-env/
bash <skill-dir>/scripts/dev_env.sh start
# then capture the route you changed, BOTH themes:
bash <skill-dir>/scripts/shot.sh http://localhost:5173/viewport --out shots/ --theme dark
bash <skill-dir>/scripts/shot.sh http://localhost:5173/viewport --out shots/ --theme light
bash <skill-dir>/scripts/dev_env.sh stop
```

`--theme` sets the app's own `reusex-theme` preference before load, so you see
the real resolved theme — not just `prefers-color-scheme`. For a standalone
mockup file, pass the file path instead of the URL.

Then **open every PNG with your Read tool and actually look.** Defaults: desktop
1440×900, tablet 768×1024, mobile 390×844, full-page. `--help` for `--selector`,
`--viewports`, `--pdf`, `--wait`.

**Headless:** Playwright's headless Chromium renders without any display, so this
works over SSH or in a worktree with no monitor. A monitor being attached is
fine but never required — do not skip the screenshot because "there's no
display". `shot.sh` wraps `screenshot.py` (same arguments) and, when the host python
cannot import Playwright, borrows nixpkgs' Playwright + matching browsers — so
no pip install is needed on NixOS. If that's genuinely impossible, say so and ask the
user for a screenshot — never skip review silently.

### 6. Critique and fix

Review the screenshots against `references/critique-checklist.md` (it has a
ReUseX section). Write down the 3–5 biggest problems, fix them, re-screenshot.
Expect 2–3 rounds; stop when what's left is taste, not defects. Before finishing:
- Run `python <skill-dir>/scripts/token_lint.py <the files you changed>` and fix
  every literal it flags. (Some older components carry pre-existing violations, so
  lint your changed files, not the whole tree — fixing unrelated drift is a
  separate, opt-in cleanup. It also catches typo'd tokens like a
  `var(--color-status-ok, …)` that no longer exists.)
- Run `npm --prefix apps/rux/frontend run typecheck` and, if you touched
  pure-logic modules, `npm --prefix apps/rux/frontend test`.
- Remove one decorative element that isn't earning its place.

### 7. Hand off

In a few lines: what changed and where, which existing components you reused,
which tokens the look depends on (so a future `/design-sync` knows what it
drives), any missing API endpoint the view needs, and known gaps. Leave
screenshots in `shots/`. If the look needs a *new* token name or role, note it
for the design project rather than hardcoding a value.

## Iterating on feedback

Treat comments like inline notes on a canvas: find the exact element, change it,
re-screenshot, confirm. If a comment implies a *system* change ("this accent is
too loud", "cards need more air"), it is a token concern — do **not** patch the
one instance with a literal. Either it's already a token (re-sync territory,
flag it) or it reveals a missing token (note it for the design project). Keeping
the fix at the token layer is what keeps the two surfaces (web + Qt) in step.

## Qt (the second surface)

The native Qt client does not exist yet, but design *for* it now by keeping every
decision in a token. When it lands, its palette/QSS is meant to be **generated
from the same token names**, never re-chosen by eye — that is the entire reason
the web side forbids literals. So when you add a visual concept the system can't
yet express, add it as a *named token/role* (and flag it for the design
project), not as a one-off value that only the web build knows about. A colour or
radius that lives only in a `.module.css` literal is invisible to the Qt
generator and breaks the shared look. See `references/reusex-frontend.md` §Qt.

## Correctness constraints (not taste)

Two token groups are correctness, not aesthetics — do not "improve" them away:
- **The viewport canvas stays near-black** (`--color-canvas`) in *both* themes.
  Point clouds are additive light on a dark field; a light canvas destroys the
  depth read.
- **`--label-0..7` is a colourblind-safe categorical scale** (Okabe-Ito). Users
  read semantic segmentation classes off these colours. If a redesign touches
  them, the replacement must stay distinguishable under deuteranopia/protanopia/
  tritanopia.

## Slides and print

For the occasional deck or one-pager (design reviews, issue write-ups): one HTML
file, `@import "../src/tokens.css"` so it's on-brand, each slide a 1920×1080
section, arrow-key nav, one idea per slide. Screenshot with `--selector .slide`;
export a one-pager with `--pdf`.
