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
  ['--chip-blue-ink', '--chip-blue-bg'],
  ['--chip-red-ink', '--chip-red-bg'],
  ['--chip-green-ink', '--chip-green-bg'],
  ['--chip-purple-ink', '--chip-purple-bg'],
  ['--chip-yellow-ink', '--chip-yellow-bg'],
  ['--chip-gray-ink', '--chip-gray-bg'],
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
