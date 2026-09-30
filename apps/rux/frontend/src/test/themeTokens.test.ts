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
  '--chip-blue-bg',
  '--chip-blue-ink',
  '--chip-red-bg',
  '--chip-red-ink',
  '--chip-green-bg',
  '--chip-green-ink',
  '--chip-purple-bg',
  '--chip-purple-ink',
  '--chip-yellow-bg',
  '--chip-yellow-ink',
  '--chip-gray-bg',
  '--chip-gray-ink',
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
