// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Label → colour lookup.
 *
 * Labels are 1-based and `0` means unlabeled (STANDARDS §3), so the palette
 * index is `(label - 1) % size`. Getting that wrong does not crash anything —
 * it just shifts every semantic class by one relative to the legend and to any
 * other view of the same data, which is why it is pinned here explicitly.
 */

import { describe, expect, it } from 'vitest';
import {
  labelColorIndex,
  paletteToFloats,
  parseColor,
  readLabelPalette,
} from '../viewport/labelColors';
import { srgbToLinear } from '../viewport/decode';

describe('labelColorIndex', () => {
  it('gives slot 0 to label 1, not to label 0', () => {
    expect(labelColorIndex(1, 8)).toBe(0);
    expect(labelColorIndex(2, 8)).toBe(1);
    expect(labelColorIndex(8, 8)).toBe(7);
  });

  it('never assigns a palette entry to unlabeled', () => {
    // Label 0 is unlabeled and is coloured from `--label-unlabeled` instead.
    expect(labelColorIndex(0, 8)).toBe(-1);
    expect(labelColorIndex(0, 1)).toBe(-1);
  });

  it('wraps modulo the palette size', () => {
    expect(labelColorIndex(9, 8)).toBe(0);
    expect(labelColorIndex(10, 8)).toBe(1);
    expect(labelColorIndex(17, 8)).toBe(0);
    expect(labelColorIndex(3, 2)).toBe(0);
  });

  it('rejects negative, fractional-below-one and non-finite labels', () => {
    expect(labelColorIndex(-1, 8)).toBe(-1);
    expect(labelColorIndex(0.5, 8)).toBe(-1);
    expect(labelColorIndex(Number.NaN, 8)).toBe(-1);
    expect(labelColorIndex(Number.POSITIVE_INFINITY, 8)).toBe(-1);
    expect(labelColorIndex(Number.NEGATIVE_INFINITY, 8)).toBe(-1);
  });

  it('rejects an empty palette rather than dividing by zero', () => {
    expect(labelColorIndex(1, 0)).toBe(-1);
    expect(labelColorIndex(5, -1)).toBe(-1);
  });

  it('floors a fractional label above one', () => {
    expect(labelColorIndex(3.9, 8)).toBe(2);
  });
});

describe('parseColor', () => {
  it('parses #rrggbb', () => {
    expect(parseColor('#000000')).toEqual([0, 0, 0]);
    expect(parseColor('#ffffff')).toEqual([1, 1, 1]);
    const [r, g, b] = parseColor('#e69f00');
    expect(r).toBeCloseTo(0xe6 / 255, 10);
    expect(g).toBeCloseTo(0x9f / 255, 10);
    expect(b).toBe(0);
  });

  it('parses the #rgb shorthand by doubling each digit', () => {
    expect(parseColor('#fff')).toEqual([1, 1, 1]);
    expect(parseColor('#000')).toEqual([0, 0, 0]);
    const [r, g, b] = parseColor('#1a2');
    expect(r).toBeCloseTo(0x11 / 255, 10);
    expect(g).toBeCloseTo(0xaa / 255, 10);
    expect(b).toBeCloseTo(0x22 / 255, 10);
  });

  it('is case-insensitive', () => {
    expect(parseColor('#E69F00')).toEqual(parseColor('#e69f00'));
    expect(parseColor('#ABC')).toEqual(parseColor('#abc'));
  });

  it('tolerates surrounding whitespace, as getPropertyValue returns it', () => {
    expect(parseColor('  #e69f00  ')).toEqual(parseColor('#e69f00'));
  });

  it('falls back to mid-grey — never NaN — for anything it cannot parse', () => {
    // A NaN component makes three.js drop the whole draw call, so a design sync
    // that switches the tokens to `oklch()` must degrade, not break.
    for (const value of [
      'transparent',
      'oklch(0.72 0.15 60)',
      'rgb(230, 159, 0)',
      'rebeccapurple',
      'var(--label-1)',
      '#ggg',
      '#12345',
      '#',
      '',
    ]) {
      const parsed = parseColor(value);
      expect(parsed).toEqual([0.5, 0.5, 0.5]);
      expect(parsed.every((component) => Number.isFinite(component))).toBe(true);
    }
  });
});

describe('readLabelPalette', () => {
  it('returns the Okabe-Ito fallback with no document available', () => {
    // Node has no `getComputedStyle`; the palette must degrade rather than
    // throw, which is what lets this module be unit-tested at all.
    const palette = readLabelPalette();
    expect(palette.colors).toHaveLength(8);
    expect(palette.colors[0]).toBe('#e69f00');
    expect(palette.unlabeled).toBe('#4a505c');
  });

  it('returns a colourblind-safe palette of distinct, parseable colours', () => {
    // The user reads semantic classes off these colours, so distinctness is a
    // correctness property, not a stylistic one.
    const palette = readLabelPalette();
    expect(new Set(palette.colors).size).toBe(palette.colors.length);
    for (const color of palette.colors) {
      expect(color).toMatch(/^#[0-9a-f]{6}$/i);
      expect(parseColor(color)).not.toEqual([0.5, 0.5, 0.5]);
    }
  });

  it('ignores an element argument when there is no styling engine', () => {
    expect(readLabelPalette(null)).toEqual(readLabelPalette());
  });
});

/**
 * `paletteToFloats` is the boundary where a token stops being CSS and becomes
 * geometry, so it owes the palette the SAME sRGB→linear transfer `decode.ts`
 * applies to sensor RGB. three.js applies the inverse on output; handing it
 * sRGB renders every label lighter and less saturated than the legend swatch
 * beside it — and the legend is right, because it uses the CSS string directly.
 * These assertions are what stop the two views drifting apart again.
 */
describe('paletteToFloats', () => {
  const toLinear = ([r, g, b]: [number, number, number]) =>
    [srgbToLinear(r), srgbToLinear(g), srgbToLinear(b)] as const;

  it('converts the whole palette to linear, unlabeled included', () => {
    const palette = readLabelPalette();
    const floats = paletteToFloats(palette);
    expect(floats.colors).toHaveLength(palette.colors.length);
    expect(floats.colors[0]).toEqual(toLinear(parseColor(palette.colors[0])));
    expect(floats.unlabeled).toEqual(toLinear(parseColor(palette.unlabeled)));
    for (const triple of [...floats.colors, floats.unlabeled]) {
      expect(triple.every((component) => component >= 0 && component <= 1)).toBe(true);
    }
  });

  it('lines up with labelColorIndex, so label 1 gets the first palette colour', () => {
    const floats = paletteToFloats(readLabelPalette());
    const slot = labelColorIndex(1, floats.colors.length);
    expect(floats.colors[slot]).toEqual(toLinear(parseColor('#e69f00')));
  });

  it('is not the identity — sRGB written straight through would be the bug', () => {
    const floats = paletteToFloats(readLabelPalette());
    // #e69f00's red channel is 0.902 in sRGB and 0.7912 linear. If these ever
    // compare equal, the transfer has been dropped.
    expect(floats.colors[0][0]).toBeCloseTo(0.79130, 4);
    expect(floats.colors[0][0]).not.toBeCloseTo(parseColor('#e69f00')[0], 4);
  });
});
