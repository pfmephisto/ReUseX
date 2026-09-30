<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# GUI Phase 1 — Identity & Shell Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Also load the project skill `design-studio` (`.claude/skills/design-studio/SKILL.md`) before touching any `.tsx`/`.css`.

**Goal:** Give the whole `rux gui` frontend the prototype-v2 identity — light-default palette, Oswald/Archivo type, navy title bar and sidebar, Danish chrome — without changing any screen's behaviour.

**Architecture:** Values change in `src/tokens.css` only (new token *roles* added where the old set could not express the prototype); components keep reading `var(--…)`. The nav rail is replaced by a navy `Sidebar` driven by a pure, unit-tested navigation model. New case-workflow destinations appear as *pending* entries (the existing inert-entry mechanism) until their phase lands.

**Tech Stack:** React 19, Vite, TypeScript, CSS Modules, vitest (Node env, no DOM), `@fontsource/oswald`, `@fontsource/archivo`, Nix (`pkgs/reusex-gui-frontend/package.nix`).

**Spec:** `docs/design/gui-kortlaegning-redesign.md`

## Global Constraints

- Every new file starts with the SPDX header used by its neighbours (`// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` / `// SPDX-License-Identifier: GPL-3.0-or-later`; CSS uses the `/* … */` block form).
- No literal colour, radius, spacing or font-size in any `.module.css` or JSX `style` — only `var(--…)`. Check with `python .claude/skills/design-studio/scripts/token_lint.py <changed files>`.
- Fonts are self-hosted from `@fontsource/*`; never load Google Fonts (the app must render offline).
- `--color-canvas` stays near-black (RGB channel sum < 90) in both themes; `--label-0..7` stays the Okabe-Ito set and is never overridden in the dark block.
- Filled primary buttons use `--color-accent-deep` as background with `--color-on-accent` text (white on `#5980A6` is only 4.15:1 — deviation from the prototype for WCAG AA).
- Table headers use `--color-text-muted`, not `--color-text-faint` (faint is 3.1:1 on white — decoration only).
- UI copy on screens touched by this plan is Danish.
- Frontend tests run with `npm --prefix apps/rux/frontend test`; typecheck with `npm --prefix apps/rux/frontend run typecheck`. No DOM test environment may be added.
- Run from inside `nix develop` (provides node 22).

## Review Focus

- A user with a stored `reusex-theme=dark` from before this change keeps dark after upgrade — the FOUC guard and `readStoredPreference` must honour stored values; only the *absent* case changes to light.
- Every existing screen stays legible in the new light default — components that hardcoded dark-theme colours (e.g. `rgba(255,255,255,.12)` hovers, `#e6e9ef` text) turn invisible on white. Task 6 owns this; its screenshot sweep is the test.
- The viewport keep-alive (`display: contents`/`none` wrapper in `App.tsx`) must survive the shell swap — the viewport must still fill the content area and resize on return.
- Keyboard users can reach every sidebar entry with Tab and see a focus ring on navy (`--color-border-focus` must contrast with `--color-chrome`).
- A pending sidebar entry must not be focusable as a link and must expose its reason (`title`) — a click on it does nothing and does not navigate.

---

## File Structure

| File | Responsibility |
|---|---|
| `apps/rux/frontend/package.json`, `package-lock.json` | add `@fontsource/oswald`, `@fontsource/archivo` |
| `pkgs/reusex-gui-frontend/package.nix` | refresh `npmDepsHash` |
| `apps/rux/frontend/src/fonts.ts` (new) | the only place fonts are imported |
| `apps/rux/frontend/src/main.tsx` | import `./fonts` |
| `apps/rux/frontend/src/tokens.css` | light `:root`, `[data-theme='dark']` override, new roles |
| `apps/rux/frontend/src/base.css` | headings in `--font-display`, uppercase |
| `apps/rux/frontend/src/test/themeTokens.test.ts` | rewritten token contract (light root, dark override, new roles) |
| `apps/rux/frontend/src/test/contrast.test.ts` (new) | WCAG contrast for text/tone/chrome pairs in both themes |
| `apps/rux/frontend/src/theme.ts`, `index.html`, `test/theme.test.ts` | default preference becomes `light` |
| `apps/rux/frontend/src/components/ThemeToggle.tsx` | Danish labels |
| `apps/rux/frontend/src/app/navigation.ts` (new) | pure navigation model |
| `apps/rux/frontend/src/test/navigation.test.ts` (new) | its contract |
| `apps/rux/frontend/src/components/Sidebar.tsx` + `.module.css` (new) | navy sidebar |
| `apps/rux/frontend/src/components/NavRail.tsx` + `.module.css` | deleted |
| `apps/rux/frontend/src/components/TitleBar.tsx` + `.module.css` | navy, Danish |
| `apps/rux/frontend/src/app/AppShell.tsx` | uses `Sidebar` |
| `apps/rux/frontend/index.html` | `lang="da"` |
| components listed in Task 6 | colour literals → tokens |
| `docs/gui/images/prototype-v2/*.png` + `.license` | reference screenshots |

---

### Task 1: Self-hosted fonts

**Files:**
- Modify: `apps/rux/frontend/package.json`, `apps/rux/frontend/package-lock.json`
- Create: `apps/rux/frontend/src/fonts.ts`
- Modify: `apps/rux/frontend/src/main.tsx`
- Modify: `pkgs/reusex-gui-frontend/package.nix:32`

**Interfaces:**
- Produces: font families `"Oswald"` (500/600/700) and `"Archivo"` (400/500/600/700, 400 italic) available to CSS.

- [ ] **Step 1: Install the packages**

Run: `npm --prefix apps/rux/frontend install @fontsource/oswald@^5 @fontsource/archivo@^5`
Expected: both appear under `dependencies` in `package.json`; `package-lock.json` updated.

- [ ] **Step 2: Create `src/fonts.ts`**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The GUI's two typefaces, bundled rather than fetched: `rux gui` is
 * local-first and must render the same on an offline laptop on site.
 *
 * Oswald (display) carries headings and large figures; Archivo (text) is the
 * body face. Both are SIL OFL-1.1. Only the weights the design uses are
 * imported, so the bundle does not carry unused faces. `tokens.css` names the
 * families in `--font-display` and `--font-sans`.
 */
import '@fontsource/oswald/500.css';
import '@fontsource/oswald/600.css';
import '@fontsource/oswald/700.css';
import '@fontsource/archivo/400.css';
import '@fontsource/archivo/400-italic.css';
import '@fontsource/archivo/500.css';
import '@fontsource/archivo/600.css';
import '@fontsource/archivo/700.css';
```

- [ ] **Step 3: Import it first in `src/main.tsx`**

Add `import './fonts';` on the line directly above `import './tokens.css';`.

- [ ] **Step 4: Build to confirm the fonts bundle**

Run: `npm --prefix apps/rux/frontend run build`
Expected: PASS; `ls apps/rux/frontend/dist/assets | grep -c woff2` prints a number ≥ 8.

- [ ] **Step 5: Refresh the Nix `npmDepsHash`**

Set `npmDepsHash = lib.fakeHash;` in `pkgs/reusex-gui-frontend/package.nix`, then run
`git add apps/rux/frontend/package.json apps/rux/frontend/package-lock.json && nix build .#reusex-gui-frontend 2>&1 | grep 'got:'`
and paste the printed `sha256-…` into `npmDepsHash`. Re-run `nix build .#reusex-gui-frontend`.
Expected: build succeeds. (Flakes only see git-tracked files — hence the `git add`. If the attribute name differs, find it with `grep -rn reusex-gui-frontend flake.nix default.nix`.)

- [ ] **Step 6: Commit**

```bash
git add apps/rux/frontend/package.json apps/rux/frontend/package-lock.json \
  apps/rux/frontend/src/fonts.ts apps/rux/frontend/src/main.tsx pkgs/reusex-gui-frontend/package.nix
git commit -m "feat(gui): bundle Oswald and Archivo via @fontsource"
```

---

### Task 2: Token values and roles (light default, dark override)

**Files:**
- Modify: `apps/rux/frontend/src/tokens.css` (full rewrite below)
- Modify: `apps/rux/frontend/src/base.css`
- Modify: `apps/rux/frontend/src/test/themeTokens.test.ts` (full rewrite below)
- Create: `apps/rux/frontend/src/test/contrast.test.ts`

**Interfaces:**
- Produces (new token names later phases rely on): `--color-chrome`, `--color-chrome-raised`, `--color-chrome-border`, `--color-on-chrome`, `--color-on-chrome-muted`, `--color-accent-deep`, `--color-star`, `--tone-{good,warn,wait,crit,accent}-{bg,ink}`, `--circ-{bevaring,genbrug,genanvendelse,nyttiggoerelse,bortskaffelse}`, `--font-display`, `--font-size-2xs`, `--font-size-3xl`, `--radius-xl`, `--tracking-caps`, `--tracking-wide`, `--shadow-panel`, `--layout-bench-aside-width`, `--color-scrim`.
- Existing names keep their roles; `--layout-nav-width` becomes the sidebar width.

- [ ] **Step 1: Rewrite the token contract test (it will fail against the old file)**

Replace `apps/rux/frontend/src/test/themeTokens.test.ts` with:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * `tokens.css` theme contract.
 *
 * Light is the default (`:root`); `[data-theme='dark']` re-points it. Values
 * are the design project's, but some properties are correctness, not taste:
 * the viewport canvas stays near-black in both themes (point clouds are
 * additive light on a dark field), and the categorical --label-* palette is
 * the colourblind-safe Okabe-Ito set in both. Every role the redesign relies
 * on must be defined, so a component never falls back to an undefined var.
 */

import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';

const css = readFileSync(new URL('../tokens.css', import.meta.url), 'utf8');

/** Declaration body of the first top-level `selector { ... }` rule. */
function ruleBody(selector: string): string {
  const at = css.indexOf(`${selector} {`);
  expect(at, `${selector} rule is missing`).toBeGreaterThanOrEqual(0);
  return css.slice(at, css.indexOf('}', at));
}

const DARK = "[data-theme='dark']";

/** Roles introduced by the prototype-v2 identity; both themes must set them. */
const THEMED_ROLES = [
  '--color-chrome',
  '--color-chrome-raised',
  '--color-chrome-border',
  '--color-on-chrome',
  '--color-on-chrome-muted',
  '--color-accent-deep',
  '--color-star',
  '--tone-good-bg',
  '--tone-good-ink',
  '--tone-warn-bg',
  '--tone-warn-ink',
  '--tone-wait-bg',
  '--tone-wait-ink',
  '--tone-crit-bg',
  '--tone-crit-ink',
  '--tone-accent-bg',
  '--tone-accent-ink',
  '--circ-bevaring',
  '--circ-genbrug',
  '--circ-genanvendelse',
  '--circ-nyttiggoerelse',
  '--circ-bortskaffelse',
  '--shadow-panel',
  '--color-scrim',
];

/** Theme-independent roles; defined once on :root. */
const STATIC_ROLES = [
  '--font-display',
  '--font-size-2xs',
  '--font-size-3xl',
  '--radius-xl',
  '--tracking-caps',
  '--tracking-wide',
  '--layout-bench-aside-width',
];

describe('tokens.css themes', () => {
  it('declares a [data-theme=dark] block after :root', () => {
    const root = css.indexOf(':root {');
    const dark = css.indexOf(`${DARK} {`);
    expect(root).toBeGreaterThanOrEqual(0);
    // Equal specificity: the dark block only wins on source order.
    expect(dark).toBeGreaterThan(root);
  });

  it('makes light the default and opts each theme into its color-scheme', () => {
    expect(ruleBody(':root')).toMatch(/color-scheme:\s*light/);
    expect(ruleBody(DARK)).toMatch(/color-scheme:\s*dark/);
  });

  it('re-points the chrome surfaces and text for dark', () => {
    const dark = ruleBody(DARK);
    for (const token of ['--color-surface', '--color-surface-raised', '--color-text']) {
      expect(dark, `${token} must be overridden for dark`).toMatch(new RegExp(`${token}:`));
    }
  });

  it('defines every themed role in both themes', () => {
    const root = ruleBody(':root');
    const dark = ruleBody(DARK);
    for (const token of THEMED_ROLES) {
      expect(root, `${token} missing from :root`).toMatch(new RegExp(`${token}:`));
      expect(dark, `${token} missing from dark`).toMatch(new RegExp(`${token}:`));
    }
  });

  it('defines every static role on :root', () => {
    const root = ruleBody(':root');
    for (const token of STATIC_ROLES) {
      expect(root, `${token} missing from :root`).toMatch(new RegExp(`${token}:`));
    }
  });

  it('keeps the viewport canvas near-black in both themes', () => {
    const canvas = /--color-canvas:\s*(#[0-9a-f]{6})/gi;
    const values = [...css.matchAll(canvas)].map((m) => m[1].toLowerCase());
    expect(values.length).toBeGreaterThanOrEqual(2);
    for (const value of values) {
      const sum =
        parseInt(value.slice(1, 3), 16) +
        parseInt(value.slice(3, 5), 16) +
        parseInt(value.slice(5, 7), 16);
      expect(sum, `canvas ${value} is not near-black`).toBeLessThan(90);
    }
  });

  it('keeps the Okabe-Ito label palette and never overrides it for dark', () => {
    const root = ruleBody(':root');
    const okabeIto = ['#e69f00', '#56b4e9', '#009e73', '#f0e442', '#0072b2', '#d55e00', '#cc79a7', '#999999'];
    okabeIto.forEach((hex, i) => {
      expect(root.toLowerCase()).toMatch(new RegExp(`--label-${i}:\\s*${hex}`));
    });
    expect(ruleBody(DARK)).not.toMatch(/--label-[0-7]:/);
  });

  it('loads the display and text faces by name', () => {
    const root = ruleBody(':root');
    expect(root).toMatch(/--font-display:[^;]*Oswald/);
    expect(root).toMatch(/--font-sans:[^;]*Archivo/);
  });
});
```

- [ ] **Step 2: Write the contrast test**

Create `apps/rux/frontend/src/test/contrast.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * WCAG 2.x contrast for the pairs the UI actually puts text on.
 *
 * Read from `tokens.css` itself, so a re-sync from the design project that
 * breaks legibility fails here rather than in a user's eyes. `--color-text-faint`
 * is deliberately absent: it is for decoration and disabled states only.
 */

import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';

const css = readFileSync(new URL('../tokens.css', import.meta.url), 'utf8');

function block(selector: string): string {
  const at = css.indexOf(`${selector} {`);
  return css.slice(at, css.indexOf('}', at));
}

/** Token → hex for one theme; dark inherits anything it does not override. */
function palette(selector: string | null): Map<string, string> {
  const out = new Map<string, string>();
  const read = (body: string) => {
    for (const m of body.matchAll(/(--[a-z0-9-]+):\s*(#[0-9a-fA-F]{6})\b/g)) {
      out.set(m[1], m[2].toLowerCase());
    }
  };
  read(block(':root'));
  if (selector) read(block(selector));
  return out;
}

function luminance(hex: string): number {
  const [r, g, b] = [1, 3, 5].map((i) => {
    const c = parseInt(hex.slice(i, i + 2), 16) / 255;
    return c <= 0.03928 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4;
  });
  return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}

function ratio(a: string, b: string): number {
  const [hi, lo] = [luminance(a), luminance(b)].sort((x, y) => y - x);
  return (hi + 0.05) / (lo + 0.05);
}

/** [foreground, background] pairs that must reach 4.5:1 (normal text). */
const PAIRS: [string, string][] = [
  ['--color-text', '--color-surface'],
  ['--color-text', '--color-surface-raised'],
  ['--color-text', '--color-surface-sunken'],
  ['--color-text-muted', '--color-surface'],
  ['--color-text-muted', '--color-surface-raised'],
  ['--color-text-muted', '--color-surface-sunken'],
  ['--color-accent-deep', '--color-surface-raised'],
  ['--color-on-accent', '--color-accent-deep'],
  ['--color-on-chrome', '--color-chrome'],
  ['--color-on-chrome-muted', '--color-chrome'],
  ['--color-on-chrome', '--color-chrome-raised'],
  ['--color-on-chrome-muted', '--color-chrome-raised'],
  ['--tone-good-ink', '--tone-good-bg'],
  ['--tone-warn-ink', '--tone-warn-bg'],
  ['--tone-wait-ink', '--tone-wait-bg'],
  ['--tone-crit-ink', '--tone-crit-bg'],
  ['--tone-accent-ink', '--tone-accent-bg'],
];

describe.each([
  ['light', null],
  ['dark', "[data-theme='dark']"],
] as const)('contrast (%s)', (_name, selector) => {
  const p = palette(selector);
  it.each(PAIRS)('%s on %s ≥ 4.5:1', (fg, bg) => {
    const f = p.get(fg);
    const b = p.get(bg);
    expect(f, `${fg} is not a hex token`).toBeDefined();
    expect(b, `${bg} is not a hex token`).toBeDefined();
    expect(ratio(f!, b!)).toBeGreaterThanOrEqual(4.5);
  });

  it('focus ring is visible on the navy chrome (≥ 3:1)', () => {
    expect(ratio(p.get('--color-border-focus')!, p.get('--color-chrome')!)).toBeGreaterThanOrEqual(3);
  });
});
```

- [ ] **Step 3: Run both tests to verify they fail**

Run: `npm --prefix apps/rux/frontend test -- themeTokens contrast`
Expected: FAIL — `:root` is `color-scheme: dark`, the dark block is missing, the new roles are undefined.

- [ ] **Step 4: Rewrite `src/tokens.css`**

Replace the whole file with:

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ============================================================================
 * OWNED BY the Claude Design project "ReUseX GUI"
 * (project id 19f0cf0f-1f7c-43a7-8311-1fd67e34bbb7).
 *
 * Values: the prototype-v2 identity (docs/design/gui-kortlaegning-redesign.md),
 * adopted 2026-09-30 ahead of the first /design-sync. The repo owns the token
 * *names* and roles; the design project owns every value on the right-hand
 * side. Push these to the design project with /design-sync rather than
 * letting the two drift.
 *
 * Rule for everything else in src/: no colour, radius, spacing or type size is
 * ever written literally in a component. Only var(--...).
 * ============================================================================
 *
 * Correctness constraints for whoever authors values:
 *  - --color-canvas stays near-black in BOTH themes. Point clouds are additive
 *    light on a dark field; a light canvas destroys the depth read.
 *  - --label-N is a CATEGORICAL, colourblind-safe scale (Okabe-Ito). A
 *    replacement must keep that property; it is never overridden per theme.
 *  - --circ-* is the waste hierarchy (affaldshierarki), ordered from best
 *    (bevaring) to worst (bortskaffelse). It is read as a ranked category.
 */

:root {
  /* Light is the default theme. `data-theme` on <html> carries the resolved
     value; the dark block below overrides at equal specificity and wins on
     source order when it applies. */
  color-scheme: light;

  /* ---------------------------------------------------------- surfaces -- */
  --color-canvas: #171c24; /* the 3D viewport — near-black in both themes */
  --color-surface: #e9e9e7; /* the workbench (page background) */
  --color-surface-raised: #ffffff; /* panels, cards */
  --color-surface-overlay: #ffffff; /* menus, toasts, hover */
  --color-surface-sunken: #f2f2f0; /* inputs, table headers, wells */
  --color-scrim: #1d2d3d; /* modal backdrop, used at reduced opacity */

  /* ------------------------------------------------------------ chrome -- */
  /* The navy title bar and sidebar. */
  --color-chrome: #1d2d3d;
  --color-chrome-raised: #24374b; /* active nav row, chrome buttons */
  --color-chrome-border: #35495e;
  --color-on-chrome: #f2f2f3;
  --color-on-chrome-muted: #93a3b3;

  /* ------------------------------------------------------------ border -- */
  --color-border: #d8d8d4;
  --color-border-strong: #b9bcbf;
  --color-border-focus: #5980a6;

  /* -------------------------------------------------------------- text -- */
  --color-text: #1c2530;
  --color-text-muted: #5a6470;
  --color-text-faint: #8b939d; /* decoration and disabled only (3.1:1) */
  --color-text-inverse: #f2f2f3;

  /* ------------------------------------------------------------ accent -- */
  --color-accent: #5980a6; /* bars, markers, focus, active dots */
  --color-accent-hover: #3e5f80;
  --color-accent-muted: #e4ebf2; /* selected row, soft fills */
  --color-accent-deep: #3e5f80; /* accent text; filled primary buttons */
  --color-on-accent: #ffffff; /* text on --color-accent-deep */
  --color-star: #b98a1d; /* "vigtig" ★ */

  /* -------------------------------------------------------------- tone -- */
  /* Background/ink pairs for pills and notices. */
  --tone-good-bg: #deebe2;
  --tone-good-ink: #2c5c40;
  --tone-warn-bg: #f3e9cb;
  --tone-warn-ink: #6e5313;
  --tone-wait-bg: #e3e6ea;
  --tone-wait-ink: #4a5563;
  --tone-crit-bg: #f3dcd7;
  --tone-crit-ink: #8a2f23;
  --tone-accent-bg: #e4ebf2;
  --tone-accent-ink: #3e5f80;

  /* ----------------------------------------- categorical: affaldshierarki */
  --circ-bevaring: #24374b;
  --circ-genbrug: #5980a6;
  --circ-genanvendelse: #6f9080;
  --circ-nyttiggoerelse: #c09a46;
  --circ-bortskaffelse: #8a6a55;

  /* ------------------------------------------------------------ status -- */
  /* Job/stage lifecycle. Distinct in luminance as well as hue. */
  --color-status-queued: #55606f;
  --color-status-running: #0072b2;
  --color-status-succeeded: #007a59;
  --color-status-failed: #b8460b;
  --color-status-cancelled: #838d9c;

  /* --------------------------------------------- categorical: labels ---- */
  /* Okabe-Ito. Consumed by the viewport's label colour mode and the legend. */
  --label-0: #e69f00;
  --label-1: #56b4e9;
  --label-2: #009e73;
  --label-3: #f0e442;
  --label-4: #0072b2;
  --label-5: #d55e00;
  --label-6: #cc79a7;
  --label-7: #999999;
  --label-count: 8;
  /* Points whose label is 0 (unlabeled, STANDARDS §3) render in this. */
  --label-unlabeled: #4a505c;

  /* ------------------------------------------------- viewport: geometry -- */
  --mesh-surface: #8a8f99;

  /* ------------------------------------------------------------ typography */
  --font-display: 'Oswald', 'Arial Narrow', sans-serif;
  --font-sans: 'Archivo', 'Helvetica Neue', Arial, sans-serif;
  /* Data tables are monospace by design brief — figures must align. */
  --font-mono:
    ui-monospace, 'SF Mono', 'JetBrains Mono', 'Fira Code', Menlo, Consolas, monospace;

  --font-size-2xs: 0.625rem; /* uppercase field labels, table headers */
  --font-size-xs: 0.6875rem; /* pills, captions */
  --font-size-sm: 0.75rem; /* secondary text, controls */
  --font-size-md: 0.84375rem; /* body (13.5px) */
  --font-size-lg: 1rem; /* card titles */
  --font-size-xl: 1.45rem; /* view headings */
  --font-size-2xl: 1.65rem; /* KPI figures */
  --font-size-3xl: 1.9rem; /* page headings */

  --font-weight-regular: 400;
  --font-weight-medium: 500;
  --font-weight-bold: 600;

  --line-height-tight: 1.15;
  --line-height-normal: 1.45;

  --tracking-caps: 0.08em; /* uppercase labels */
  --tracking-wide: 0.18em; /* the sidebar's PROJEKT eyebrow */

  /* -------------------------------------------------------------- space -- */
  --space-0: 0;
  --space-1: 0.25rem;
  --space-2: 0.5rem;
  --space-3: 0.75rem;
  --space-4: 1rem;
  --space-5: 1.5rem;
  --space-6: 2rem;
  --space-7: 3rem;

  /* ------------------------------------------------------------- radius -- */
  --radius-sm: 3px; /* pills, kbd */
  --radius-md: 4px; /* buttons, inputs */
  --radius-lg: 6px; /* panels, cards */
  --radius-xl: 8px; /* dialogs */
  --radius-pill: 999px;

  /* ------------------------------------------------------------ shadow --- */
  --shadow-sm: 0 1px 2px rgb(29 45 61 / 10%);
  --shadow-md: 0 4px 12px rgb(29 45 61 / 12%);
  --shadow-lg: 0 14px 44px rgb(0 0 0 / 40%);
  --shadow-panel: 0 1px 2px rgb(29 45 61 / 10%), 0 4px 12px rgb(29 45 61 / 7%);

  /* ------------------------------------------------------------ layout --- */
  --layout-titlebar-height: 44px;
  --layout-nav-width: 13.5rem; /* the navy sidebar */
  --layout-panel-width: 280px;
  --layout-bench-aside-width: 21.5rem; /* Kortlægning's evidence/detail column */

  /* ------------------------------------------------------------- motion -- */
  --duration-fast: 120ms;
  --duration-normal: 200ms;
  --easing-standard: cubic-bezier(0.2, 0, 0.2, 1);

  /* ------------------------------------------------------------ z-index -- */
  --z-panel: 10;
  --z-titlebar: 20;
  --z-toast: 30;
}

/*
 * Dark theme — derived from the navy chrome. Re-points the themed roles only:
 * the canvas and the --label-* scale are left alone (see the header).
 * `[data-theme='dark']` and `:root` share specificity (0,1,0); this block wins
 * only because it comes later in the file. Keep it after `:root`.
 */
[data-theme='dark'] {
  color-scheme: dark;

  --color-surface: #141c25;
  --color-surface-raised: #1b2530;
  --color-surface-overlay: #22303d;
  --color-surface-sunken: #10161d;
  --color-scrim: #05080c;

  --color-chrome: #0f1822;
  --color-chrome-raised: #172433;
  --color-chrome-border: #26384a;
  --color-on-chrome: #eef1f4;
  --color-on-chrome-muted: #8698aa;

  --color-border: #2c3a48;
  --color-border-strong: #3e4f60;
  --color-border-focus: #7fa3c6;

  --color-text: #e6eaef;
  --color-text-muted: #a3aebb;
  --color-text-faint: #74808d;
  --color-text-inverse: #141c25;

  --color-accent: #7fa3c6;
  --color-accent-hover: #9bb8d4;
  --color-accent-muted: #1f3346;
  --color-accent-deep: #9bb8d4;
  --color-on-accent: #0f1720;
  --color-star: #e0b34a;

  --tone-good-bg: #1d3527;
  --tone-good-ink: #8fd1a6;
  --tone-warn-bg: #3a3017;
  --tone-warn-ink: #e3c779;
  --tone-wait-bg: #26303b;
  --tone-wait-ink: #b4bfcb;
  --tone-crit-bg: #3d2220;
  --tone-crit-ink: #f0a094;
  --tone-accent-bg: #1f3346;
  --tone-accent-ink: #9bb8d4;

  /* Lifted so the ranked scale still reads on a dark field. */
  --circ-bevaring: #8fa7c0;
  --circ-genbrug: #7fa3c6;
  --circ-genanvendelse: #8fb3a1;
  --circ-nyttiggoerelse: #d6b366;
  --circ-bortskaffelse: #b08e77;

  --color-status-queued: #9aa3b2;
  --color-status-running: #56b4e9;
  --color-status-succeeded: #009e73;
  --color-status-failed: #d55e00;
  --color-status-cancelled: #7a828f;

  --shadow-sm: 0 1px 2px rgb(0 0 0 / 40%);
  --shadow-md: 0 4px 12px rgb(0 0 0 / 45%);
  --shadow-lg: 0 12px 32px rgb(0 0 0 / 55%);
  --shadow-panel: 0 1px 2px rgb(0 0 0 / 35%), 0 4px 12px rgb(0 0 0 / 25%);
}

@media (prefers-reduced-motion: reduce) {
  :root {
    --duration-fast: 0ms;
    --duration-normal: 0ms;
  }
}
```

- [ ] **Step 5: Headings in the display face**

In `apps/rux/frontend/src/base.css`, replace the `h1, h2, h3, h4 { … }` rule with:

```css
h1,
h2,
h3,
h4 {
  margin: 0;
  font-family: var(--font-display);
  font-weight: var(--font-weight-bold);
  line-height: var(--line-height-tight);
  letter-spacing: 0.01em;
  text-transform: uppercase;
}
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `npm --prefix apps/rux/frontend test -- themeTokens contrast`
Expected: PASS (all pairs; values were checked when the plan was written).

- [ ] **Step 7: Lint the changed CSS, typecheck, commit**

Run: `python .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/base.css --include-base` — expect only the pre-existing scrollbar/outline widths, no colour findings.
Run: `npm --prefix apps/rux/frontend run typecheck` — PASS.

```bash
git add apps/rux/frontend/src/tokens.css apps/rux/frontend/src/base.css \
  apps/rux/frontend/src/test/themeTokens.test.ts apps/rux/frontend/src/test/contrast.test.ts
git commit -m "feat(gui): prototype-v2 token values, light default, new token roles"
```

---

### Task 3: Light is the default theme preference

**Files:**
- Modify: `apps/rux/frontend/src/theme.ts`
- Modify: `apps/rux/frontend/index.html`
- Modify: `apps/rux/frontend/src/test/theme.test.ts`
- Modify: `apps/rux/frontend/src/components/ThemeToggle.tsx`
- Modify: `apps/rux/frontend/src/app/useTheme.ts` (doc comment only)

**Interfaces:**
- Produces: `DEFAULT_THEME_PREFERENCE: ThemePreference = 'light'`; `THEME_PREFERENCES = ['light', 'dark', 'system']`; `readStoredPreference()` returns `'light'` when nothing valid is stored.

- [ ] **Step 1: Update the tests first**

In `apps/rux/frontend/src/test/theme.test.ts`:
- replace the test `'lists the three choices with system (the default) first'` with:

```ts
  it('lists the three choices with light (the default) first', () => {
    expect([...THEME_PREFERENCES]).toEqual(['light', 'dark', 'system']);
    expect(THEME_PREFERENCES[0]).toBe(DEFAULT_THEME_PREFERENCE);
    expect(DEFAULT_THEME_PREFERENCE).toBe('light');
  });
```

- change `'defaults to system when nothing is stored'` to expect `'light'`, and `'defaults to system when the stored value is not a preference'` to expect `'light'` (rename both titles to "defaults to light …").
- add, next to them:

```ts
  it('keeps a stored dark or system choice across the default change', () => {
    expect(readStoredPreference(fakeStorage({ 'reusex-theme': 'dark' }))).toBe('dark');
    expect(readStoredPreference(fakeStorage({ 'reusex-theme': 'system' }))).toBe('system');
  });
```

- add `DEFAULT_THEME_PREFERENCE` to the import from `'../theme'`. If `fakeStorage` does not accept an initial map, extend it: `function fakeStorage(init: Record<string, string> = {})` seeding its backing `Map` from `init`.

- [ ] **Step 2: Run to verify failure**

Run: `npm --prefix apps/rux/frontend test -- theme.test`
Expected: FAIL — `DEFAULT_THEME_PREFERENCE` is not exported; defaults are `system`.

- [ ] **Step 3: Implement in `src/theme.ts`**

Replace the `THEME_PREFERENCES` declaration and `readStoredPreference` with:

```ts
/** What a first-time user sees: the light workbench of the prototype-v2 identity. */
export const DEFAULT_THEME_PREFERENCE: ThemePreference = 'light';

/** The three choices, in the order the toggle presents them (default first). */
export const THEME_PREFERENCES = ['light', 'dark', 'system'] as const;
```

```ts
/**
 * The stored preference, defaulting to {@link DEFAULT_THEME_PREFERENCE} when
 * missing or unrecognised. A stored `dark` or `system` from before light became
 * the default is honoured unchanged.
 */
export function readStoredPreference(storage = defaultStorage()): ThemePreference {
  try {
    const raw = storage?.getItem(THEME_STORAGE_KEY);
    return isThemePreference(raw) ? raw : DEFAULT_THEME_PREFERENCE;
  } catch {
    return DEFAULT_THEME_PREFERENCE;
  }
}
```

Update the module doc comment's sentence about `system` being the default to say light is.

- [ ] **Step 4: Mirror it in the FOUC guard (`index.html`)**

Change `<html lang="en">` to `<html lang="da">` and replace the guard's body with:

```js
      (function () {
        try {
          var choice = localStorage.getItem('reusex-theme');
          if (choice !== 'dark' && choice !== 'system') choice = 'light';
          var dark =
            choice === 'dark' ||
            (choice === 'system' &&
              window.matchMedia('(prefers-color-scheme: dark)').matches);
          document.documentElement.dataset.theme = dark ? 'dark' : 'light';
        } catch (e) {
          document.documentElement.dataset.theme = 'light';
        }
      })();
```

Keep the explanatory comment above it, changing "system" defaults to "light".

- [ ] **Step 5: Danish toggle labels**

In `ThemeToggle.tsx` set the label map to `{ light: 'Lys', dark: 'Mørk', system: 'System' }` and the radiogroup `aria-label="Farvetema"`. If the toggle iterates `THEME_PREFERENCES`, the new order follows automatically.

- [ ] **Step 6: Verify and commit**

Run: `npm --prefix apps/rux/frontend test -- theme` → PASS. `npm --prefix apps/rux/frontend run typecheck` → PASS.

```bash
git add apps/rux/frontend/src/theme.ts apps/rux/frontend/index.html \
  apps/rux/frontend/src/test/theme.test.ts apps/rux/frontend/src/components/ThemeToggle.tsx \
  apps/rux/frontend/src/app/useTheme.ts
git commit -m "feat(gui): light is the default theme; Danish theme toggle"
```

---

### Task 4: Navigation model

**Files:**
- Create: `apps/rux/frontend/src/app/navigation.ts`
- Create: `apps/rux/frontend/src/test/navigation.test.ts`

**Interfaces:**
- Produces:

```ts
export type NavGroup = 'sag' | 'tools';
export type NavBadge = 'reviewQueue' | 'pendingSamples';
export interface NavEntry {
  to: string;
  label: string;
  group: NavGroup;
  end?: boolean;
  pending?: string;
  badge?: NavBadge;
}
export const NAV_ENTRIES: readonly NavEntry[];
export const ALL_CASES_PATH = '/sager';
export function entriesIn(group: NavGroup): NavEntry[];
export function badgeText(count: number | undefined): string | null;
```

Later phases remove an entry's `pending` when its screen lands and fill badge counts.

- [ ] **Step 1: Write the failing test**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ALL_CASES_PATH, NAV_ENTRIES, badgeText, entriesIn } from '../app/navigation';

describe('navigation model', () => {
  it('lists the case workflow in the prototype order', () => {
    expect(entriesIn('sag').map((e) => e.label)).toEqual([
      'Overblik',
      'Kortlægning',
      'Miljø & prøver',
      'Rapport',
      'Indberetning',
    ]);
  });

  it('keeps every existing technical route reachable under Værktøjer', () => {
    const tools = entriesIn('tools').map((e) => e.to);
    for (const path of [
      '/viewport',
      '/graph-view',
      '/pipeline',
      '/pipeline/log',
      '/frames',
      '/geometry',
      '/instances',
      '/materials',
      '/labels',
      '/export',
    ]) {
      expect(tools).toContain(path);
    }
  });

  it('uses unique, extensionless paths', () => {
    const paths = NAV_ENTRIES.map((e) => e.to);
    expect(new Set(paths).size).toBe(paths.length);
    for (const p of paths) expect(p, p).not.toMatch(/\./);
  });

  it('matches /pipeline exactly so /pipeline/log does not light it up', () => {
    expect(NAV_ENTRIES.find((e) => e.to === '/pipeline')?.end).toBe(true);
  });

  it('gives pending entries a reason a user can read', () => {
    for (const e of NAV_ENTRIES.filter((x) => x.pending !== undefined)) {
      expect(e.pending!.length, e.label).toBeGreaterThan(10);
    }
  });

  it('attaches the review-queue and sample badges to their entries', () => {
    expect(NAV_ENTRIES.find((e) => e.to === '/kortlaegning')?.badge).toBe('reviewQueue');
    expect(NAV_ENTRIES.find((e) => e.to === '/miljoe')?.badge).toBe('pendingSamples');
  });

  it('points "Alle sager" at the case list', () => {
    expect(ALL_CASES_PATH).toBe('/sager');
  });

  it('hides a badge for no count and zero, caps it at 99+', () => {
    expect(badgeText(undefined)).toBeNull();
    expect(badgeText(0)).toBeNull();
    expect(badgeText(7)).toBe('7');
    expect(badgeText(120)).toBe('99+');
  });
});
```

- [ ] **Step 2: Run it to verify it fails**

Run: `npm --prefix apps/rux/frontend test -- navigation`
Expected: FAIL — cannot resolve `../app/navigation`.

- [ ] **Step 3: Implement `src/app/navigation.ts`**

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The navigation model for the whole app, as data.
 *
 * Two groups: the case workflow a surveyor works through (`sag`, in the
 * prototype-v2 order) and the technical tools that existed before it
 * (`tools`). Not-yet-built destinations are listed with a `pending` reason and
 * rendered inert rather than hidden — a user who cannot see that a place will
 * exist reasonably concludes the GUI cannot do that thing at all.
 *
 * Kept free of React so the contract is unit-testable in Node.
 */

export type NavGroup = 'sag' | 'tools';

/** Live counts the sidebar can show next to an entry. */
export type NavBadge = 'reviewQueue' | 'pendingSamples';

export interface NavEntry {
  to: string;
  label: string;
  group: NavGroup;
  /** Match exactly, for a route with a longer route nested under it. */
  end?: boolean;
  /** Set while the destination does not exist yet; says when it arrives. */
  pending?: string;
  badge?: NavBadge;
}

export const ALL_CASES_PATH = '/sager';

export const NAV_ENTRIES: readonly NavEntry[] = [
  { to: '/', label: 'Overblik', group: 'sag', end: true },
  {
    to: '/kortlaegning',
    label: 'Kortlægning',
    group: 'sag',
    badge: 'reviewQueue',
    pending: 'Kommer i fase 3 — brug Materialedata indtil da',
  },
  {
    to: '/miljoe',
    label: 'Miljø & prøver',
    group: 'sag',
    badge: 'pendingSamples',
    pending: 'Kommer i fase 4 — prøver og miljøstatus',
  },
  { to: '/rapport', label: 'Rapport', group: 'sag', pending: 'Kommer i fase 5 — rapportversioner' },
  {
    to: '/indberetning',
    label: 'Indberetning',
    group: 'sag',
    pending: 'Kommer i fase 5 — fraktioner til bygningsaffald.dk',
  },

  { to: '/viewport', label: 'Viewport', group: 'tools' },
  { to: '/graph-view', label: 'Posegraf', group: 'tools' },
  { to: '/pipeline', label: 'Pipeline', group: 'tools', end: true },
  { to: '/pipeline/log', label: 'Kørselslog', group: 'tools' },
  { to: '/frames', label: 'Billeder', group: 'tools' },
  { to: '/geometry', label: 'Geometri', group: 'tools' },
  { to: '/instances', label: 'Instanser', group: 'tools' },
  { to: '/materials', label: 'Materialedata', group: 'tools' },
  { to: '/labels', label: 'Labels', group: 'tools' },
  { to: '/export', label: 'Eksport', group: 'tools' },
];

export function entriesIn(group: NavGroup): NavEntry[] {
  return NAV_ENTRIES.filter((e) => e.group === group);
}

/** The badge label for a count, or null when there is nothing to flag. */
export function badgeText(count: number | undefined): string | null {
  if (count === undefined || count <= 0) return null;
  return count > 99 ? '99+' : String(count);
}
```

- [ ] **Step 4: Run to verify it passes, then commit**

Run: `npm --prefix apps/rux/frontend test -- navigation` → PASS.

```bash
git add apps/rux/frontend/src/app/navigation.ts apps/rux/frontend/src/test/navigation.test.ts
git commit -m "feat(gui): case-workflow navigation model"
```

---

### Task 5: Navy sidebar, title bar and shell

**Files:**
- Create: `apps/rux/frontend/src/components/Sidebar.tsx`, `Sidebar.module.css`
- Delete: `apps/rux/frontend/src/components/NavRail.tsx`, `NavRail.module.css`
- Modify: `apps/rux/frontend/src/components/TitleBar.tsx`, `TitleBar.module.css`
- Modify: `apps/rux/frontend/src/app/AppShell.tsx`
- Modify: `apps/rux/frontend/src/app/navigation.ts` (add `displayProjectName`)
- Modify: `apps/rux/frontend/src/test/navigation.test.ts`

**Interfaces:**
- Consumes: `NAV_ENTRIES`, `entriesIn`, `badgeText`, `ALL_CASES_PATH`, `NavBadge` (Task 4); tokens from Task 2.
- Produces: `<Sidebar projectName?: string; badges?: Partial<Record<NavBadge, number>> />`; `displayProjectName(summary?: ProjectSummary, health?: Health): string | undefined`.

- [ ] **Step 1: Test the project-name rule**

In `navigation.test.ts`, add `displayProjectName` to the existing `'../app/navigation'` import, add `import type { Health, ProjectSummary } from '../api/types';` under it, and append:

```ts
describe('displayProjectName', () => {
  const health = { project: { name: 'scan.rux', open: true } } as Health;
  it('prefers the first project record name', () => {
    const summary = { projects: [{ id: 'a', name: 'Måløv Byvej 229' }] } as ProjectSummary;
    expect(displayProjectName(summary, health)).toBe('Måløv Byvej 229');
  });
  it('falls back to the file name when the record has no name', () => {
    const summary = { projects: [{ id: 'a', name: '' }] } as ProjectSummary;
    expect(displayProjectName(summary, health)).toBe('scan.rux');
  });
  it('is undefined while nothing has loaded', () => {
    expect(displayProjectName(undefined, undefined)).toBeUndefined();
  });
});
```

Run: `npm --prefix apps/rux/frontend test -- navigation` → FAIL (`displayProjectName` missing).

- [ ] **Step 2: Implement `displayProjectName` in `navigation.ts`**

```ts
import type { Health, ProjectSummary } from '../api/types';

/**
 * The name the sidebar shows under PROJEKT: the building's record name when the
 * project metadata has one, else the `.rux` file name from `/health`.
 */
export function displayProjectName(
  summary: ProjectSummary | undefined,
  health: Health | undefined,
): string | undefined {
  const recordName = summary?.projects?.[0]?.name?.trim();
  return recordName ? recordName : health?.project.name;
}
```

Run the test → PASS.

- [ ] **Step 3: Create `Sidebar.tsx`**

```tsx
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link, NavLink } from 'react-router-dom';

import { ALL_CASES_PATH, badgeText, entriesIn, type NavBadge, type NavEntry } from '../app/navigation';
import styles from './Sidebar.module.css';

export interface SidebarProps {
  /** Shown under the PROJEKT eyebrow; a placeholder dash while loading. */
  projectName?: string;
  /** Live counts for entries that carry a badge; absent or 0 hides it. */
  badges?: Partial<Record<NavBadge, number>>;
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
 * and the way back to the case list.
 */
export function Sidebar({ projectName, badges = {} }: SidebarProps) {
  return (
    <aside className={styles.sidebar}>
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
        <Link to={ALL_CASES_PATH}>← Alle sager</Link>
      </div>
    </aside>
  );
}
```

- [ ] **Step 4: Create `Sidebar.module.css`**

```css
/*
 * SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

.sidebar {
  display: flex;
  flex-direction: column;
  flex: none;
  width: var(--layout-nav-width);
  padding: var(--space-4) 0 var(--space-5);
  overflow-y: auto;
  background: var(--color-chrome);
  color: var(--color-on-chrome);
}

.eyebrow,
.groupLabel {
  padding: 0 var(--space-4);
  font-size: var(--font-size-2xs);
  letter-spacing: var(--tracking-wide);
  text-transform: uppercase;
  color: var(--color-on-chrome-muted);
}

.groupLabel {
  margin-top: var(--space-5);
  margin-bottom: var(--space-1);
}

.project {
  padding: var(--space-1) var(--space-4) var(--space-3);
  font-family: var(--font-display);
  font-weight: var(--font-weight-bold);
  font-size: var(--font-size-lg);
  text-transform: uppercase;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.nav {
  display: grid;
}

.item {
  display: flex;
  align-items: center;
  gap: var(--space-2);
  padding: var(--space-2) var(--space-4);
  font-size: var(--font-size-md);
  font-weight: var(--font-weight-medium);
  color: var(--color-on-chrome-muted);
  text-decoration: none;
}

.item:hover {
  color: var(--color-on-chrome);
}

.item:focus-visible {
  outline-offset: -2px;
}

.dot {
  flex: none;
  width: 7px;
  height: 7px;
  border: 1.5px solid currentColor;
  border-radius: var(--radius-pill);
}

.label {
  min-width: 0;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.active {
  background: var(--color-chrome-raised);
  color: var(--color-on-chrome);
  box-shadow: inset 3px 0 0 var(--color-accent);
}

.active .dot {
  background: var(--color-accent);
  border-color: var(--color-accent);
}

.count {
  margin-left: auto;
  padding: 0 var(--space-2);
  border-radius: var(--radius-pill);
  background: var(--color-chrome-border);
  font-size: var(--font-size-xs);
}

.hot {
  background: var(--color-accent-deep);
  color: var(--color-on-accent);
}

.pending {
  cursor: default;
  opacity: 0.55;
}

.pending:hover {
  color: var(--color-on-chrome-muted);
}

.back {
  margin-top: auto;
  padding: var(--space-3) var(--space-4) 0;
  font-size: var(--font-size-sm);
}

.back a {
  color: var(--color-on-chrome-muted);
}

.back a:hover {
  color: var(--color-on-chrome);
}
```

(`1.5px`/`7px`/`3px` are hairline/indicator sizes, not scale values; `token_lint.py` does not flag width/height/border widths or box-shadow offsets without colour literals.)

- [ ] **Step 5: Navy title bar with Danish copy**

In `TitleBar.tsx`: `'Server unreachable'` → `'Server utilgængelig'`; `'Loading…'` → `'Indlæser…'`; badge text `not open` → `ikke åben`, its title → `'Serveren kunne ikke åbne databasen'`; `schema v{schemaVersion}` → `skema v{schemaVersion}`. Render the product name as `ReUse<em className={styles.x}>X</em>`.

In `TitleBar.module.css`: set `.bar` to `height: var(--layout-titlebar-height); background: var(--color-chrome); color: var(--color-on-chrome); border-bottom: 1px solid var(--color-chrome-border);`; `.product` to `font-family: var(--font-display); font-weight: var(--font-weight-bold); font-size: var(--font-size-lg); text-transform: uppercase; letter-spacing: 0.04em;`; add `.x { color: var(--color-accent); font-style: normal; }`; set `.meta` and `.project` secondary text to `var(--color-on-chrome-muted)`; `.divider` background `var(--color-chrome-border)`. Keep every existing class name the component uses.

- [ ] **Step 6: Wire the shell (`AppShell.tsx`)**

Replace the `NavRail` import/use with `Sidebar`, and load the summary for the project name:

```tsx
import { Sidebar } from '../components/Sidebar';
import { displayProjectName } from './navigation';
import type { Health, ProjectSummary } from '../api/types';
```

```tsx
  const { data: summary } = useAsync<ProjectSummary>((signal) => api.projectSummary(signal), []);
```

```tsx
      <div className={styles.body}>
        <Sidebar projectName={displayProjectName(summary, health)} />
        <main className={styles.content}>{children}</main>
      </div>
```

Delete `components/NavRail.tsx` and `components/NavRail.module.css`; `grep -rn NavRail apps/rux/frontend/src` must return nothing (update the comment in `routes/Dashboard.module.css:16` that mentions the rail to say "sidebar").

- [ ] **Step 7: Verify**

Run: `npm --prefix apps/rux/frontend test` → PASS. `npm --prefix apps/rux/frontend run typecheck` → PASS.
Run: `python .claude/skills/design-studio/scripts/token_lint.py apps/rux/frontend/src/components/Sidebar.module.css apps/rux/frontend/src/components/TitleBar.module.css` → `OK`.
Screenshot (design-studio §5):

```bash
bash .claude/skills/design-studio/scripts/dev_env.sh start
python .claude/skills/design-studio/scripts/screenshot.py http://localhost:5173/ --out shots/p1 --theme light --viewports desktop
python .claude/skills/design-studio/scripts/screenshot.py http://localhost:5173/ --out shots/p1 --theme dark --viewports desktop
python .claude/skills/design-studio/scripts/screenshot.py http://localhost:5173/viewport --out shots/p1-viewport --theme light --viewports desktop
```

Open each PNG with Read. Expected: navy title bar + navy sidebar with PROJEKT / project name / Overblik active with accent inset bar / four dimmed pending entries / Værktøjer group / "← Alle sager" at the bottom; the viewport fills the content area. Compare against `m-overblik.png` from the prototype (Task 7 commits it). If Playwright is missing: `pip install playwright && python -m playwright install chromium`.

- [ ] **Step 8: Commit**

```bash
git add -A apps/rux/frontend/src/components/Sidebar.tsx apps/rux/frontend/src/components/Sidebar.module.css \
  apps/rux/frontend/src/components/NavRail.tsx apps/rux/frontend/src/components/NavRail.module.css \
  apps/rux/frontend/src/components/TitleBar.tsx apps/rux/frontend/src/components/TitleBar.module.css \
  apps/rux/frontend/src/app/AppShell.tsx apps/rux/frontend/src/app/navigation.ts \
  apps/rux/frontend/src/test/navigation.test.ts apps/rux/frontend/src/routes/Dashboard.module.css
git commit -m "feat(gui): navy sidebar and title bar replace the nav rail"
```

---

### Task 6: Make every existing screen legible on the light default

**Files (colour literals the token linter found; each becomes a token):**
- `apps/rux/frontend/src/components/InstanceList.module.css` — `var(--color-border, #444)`/`#333` fallbacks → drop fallbacks; `var(--color-border-faint, #2a2a2a)` → `var(--color-border)` (`--color-border-faint` does not exist)
- `apps/rux/frontend/src/components/LabelQueuePanel.module.css:370` — `var(--color-status-ok, #4caf50)` → `var(--tone-good-bg)` background with `var(--tone-good-ink)` text (`--color-status-ok` does not exist)
- `apps/rux/frontend/src/components/MultiSelectDropdown.module.css:70`, `SelectDropdown.module.css:70` — `rgba(255,255,255,.12)` → `var(--color-accent-muted)`
- `apps/rux/frontend/src/components/MaterialTable.module.css:118`, `PeekPanel.module.css:25` — shadow literals → `var(--shadow-md)` / `var(--shadow-lg)`
- `apps/rux/frontend/src/components/SourceImagePanel.module.css:136-157` — overlay `rgba(0,0,0,.65)` + `#e6e9ef`/`#9aa3b2` → overlays sit on images, so use `background: color-mix(in srgb, var(--color-chrome) 85%, transparent); color: var(--color-on-chrome);` and `var(--color-on-chrome-muted)`
- `apps/rux/frontend/src/routes/ExportPage.module.css:490-518` — inside `@media print`: leave as is (print ink is literal black by intent); add a one-line comment saying so.

**Interfaces:** none (visual only).

- [ ] **Step 1: Capture the before state of every route in light**

With `dev_env.sh start` running:

```bash
for r in "" viewport graph-view pipeline pipeline/log frames geometry instances materials labels export; do
  python .claude/skills/design-studio/scripts/screenshot.py "http://localhost:5173/$r" \
    --out "shots/p1-before/${r//\//-}" --theme light --viewports desktop
done
```

Open each PNG. List every element that is invisible, low-contrast or dark-on-dark (e.g. dropdown hover, image overlays, table headers).

- [ ] **Step 2: Replace the literals listed above**

Edit each file as listed. Do not touch spacing/radius literals in these files — they are pre-existing drift outside this plan.

- [ ] **Step 3: Lint the touched files for colour**

Run: `python .claude/skills/design-studio/scripts/token_lint.py <each touched .module.css> | grep "literal colour"`
Expected: no output except ExportPage's print block.

- [ ] **Step 4: After screenshots, both themes**

Repeat Step 1's loop with `--theme light` and `--theme dark` into `shots/p1-after-light/` and `shots/p1-after-dark/`. Open every PNG. Expected: each problem listed in Step 1 is fixed; no route regresses in dark. Fix anything new, re-shoot.

- [ ] **Step 5: Verify and commit**

Run: `npm --prefix apps/rux/frontend test && npm --prefix apps/rux/frontend run build` → PASS.

```bash
git add apps/rux/frontend/src/components/*.module.css apps/rux/frontend/src/routes/ExportPage.module.css
git commit -m "fix(gui): replace dark-only colour literals so every screen reads on light"
```

---

### Task 7: Reference screenshots and docs

**Files:**
- Create: `docs/gui/images/prototype-v2/{sager,overblik,kortlaegning,dialog,miljoe}.png` and one `.png.license` per image
- Modify: `apps/rux/frontend/README.md` (§ Design tokens)

- [ ] **Step 1: Copy the prototype screenshots**

Already done while writing this plan: `docs/gui/images/prototype-v2/` holds `sager.png`, `overblik.png`, `kortlaegning.png`, `dialog.png`, `miljoe.png`, each with a `<name>.png.license` containing the text below. Only verify they are present and `git add` them. (To re-render: `Artifact` read the prototype → save HTML → `chromium --headless=new --screenshot=… --window-size=1440,1000`.)

```
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
```

- [ ] **Step 2: README note**

In `apps/rux/frontend/README.md` § Design tokens, replace "Its values are placeholders until the first `/design-sync`." with: "Its values are the prototype-v2 identity (`docs/design/gui-kortlaegning-redesign.md`), adopted before the first `/design-sync`; push them to the design project with `/design-sync` rather than hand-tuning." Add a sentence: "Fonts (Oswald, Archivo) are bundled from `@fontsource/*` in `src/fonts.ts`; the app never fetches fonts at runtime."

- [ ] **Step 3: REUSE and commit**

Run: `reuse lint` → compliant.

```bash
git add docs/gui/images/prototype-v2 apps/rux/frontend/README.md
git commit -m "docs(gui): prototype-v2 reference screenshots; token provenance"
```

---

## Phase exit criteria

- `npm --prefix apps/rux/frontend test`, `run typecheck`, `run build` pass; `nix build .#reusex-gui-frontend` passes; `reuse lint` compliant.
- Screenshots of `/` and `/viewport` in both themes match the prototype chrome; every route legible in both themes.
- Follow-up for the maintainer: run `/design-sync` to push the new values to the "ReUseX GUI" design project.
