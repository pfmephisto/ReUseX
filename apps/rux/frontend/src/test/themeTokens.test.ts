// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * `tokens.css` light-theme contract.
 *
 * The light block is placeholder values, but two properties are correctness
 * constraints, not taste: the viewport canvas must stay near-black even in
 * light mode (point clouds are additive light on a dark field), and the
 * categorical --label-* palette must stay the colourblind-safe Okabe-Ito set in
 * both themes. Those are asserted against the stylesheet itself, the way the
 * JobIndicator colour contract is — a swapped value here is invisible to any
 * DOM-only test.
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

describe('tokens.css light theme', () => {
  it('declares a [data-theme=light] block after :root', () => {
    const root = css.indexOf(':root {');
    const light = css.indexOf("[data-theme='light'] {");
    expect(root).toBeGreaterThanOrEqual(0);
    // Equal specificity: the light block only wins on source order.
    expect(light).toBeGreaterThan(root);
  });

  it('opts each theme into the matching native color-scheme', () => {
    expect(ruleBody(':root')).toMatch(/color-scheme:\s*dark/);
    expect(ruleBody("[data-theme='light']")).toMatch(/color-scheme:\s*light/);
  });

  it('re-points the chrome surfaces and text to a light palette', () => {
    const light = ruleBody("[data-theme='light']");
    for (const token of ['--color-surface', '--color-surface-raised', '--color-text']) {
      expect(light, `${token} must be overridden for light`).toMatch(
        new RegExp(`${token}:`),
      );
    }
  });

  it('keeps the viewport canvas near-black in light mode', () => {
    const canvas = /--color-canvas:\s*(#[0-9a-f]{6})/gi;
    const values = [...css.matchAll(canvas)].map((m) => m[1].toLowerCase());
    expect(values.length).toBeGreaterThanOrEqual(2); // :root and the light block
    for (const value of values) {
      // Sum of the RGB channels stays low — a light canvas would blow past this.
      const sum =
        parseInt(value.slice(1, 3), 16) +
        parseInt(value.slice(3, 5), 16) +
        parseInt(value.slice(5, 7), 16);
      expect(sum, `canvas ${value} is not near-black`).toBeLessThan(90);
    }
  });

  it('does not override the categorical --label-N palette for light', () => {
    const light = ruleBody("[data-theme='light']");
    // --label-unlabeled (the neutral) may move; the semantic scale must not.
    expect(light).not.toMatch(/--label-[0-7]:/);
  });
});
