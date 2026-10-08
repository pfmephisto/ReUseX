// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Label → colour, read out of the design tokens.
 *
 * The categorical scale lives in `tokens.css` as `--label-0 … --label-N` plus
 * `--label-count` and `--label-unlabeled`, and this module is the only place
 * that reads it. That indirection is the point: the viewport colours points
 * from the same palette the legend and any future label editor use, and a
 * `/design-sync` that changes the palette changes all of them at once with no
 * code edit.
 *
 * The palette must stay colourblind-safe. The user is reading semantic classes
 * off these colours, so that is a correctness property, not a stylistic one —
 * see the note in `tokens.css`.
 */

import { srgbToLinear } from './decode';

export interface LabelPalette {
  /** Colours for labels 1..N, as CSS colour strings. */
  colors: string[];
  /** Colour for label 0 — unlabeled (STANDARDS §3). */
  unlabeled: string;
  /** Colour for an out-of-contract label — a -1 that wrapped to 0xFFFFFFFF in
   *  a uint32 cloud (STANDARDS §3.1). Not a class, so not on the scale. */
  invalid: string;
}

/** labelColorIndex() for label 0 (unlabeled). */
export const UNLABELED_SLOT = -1;
/** labelColorIndex() for an out-of-contract label (> INT32_MAX). */
export const INVALID_SLOT = -2;

/** Palette used when no document is available (tests, SSR). */
const FALLBACK: LabelPalette = {
  // Okabe-Ito, mirroring the tokens.css placeholders. Only ever reached
  // headless — in the browser the tokens are the single source of truth.
  colors: [
    '#e69f00',
    '#56b4e9',
    '#009e73',
    '#f0e442',
    '#0072b2',
    '#d55e00',
    '#cc79a7',
    '#999999',
  ],
  unlabeled: '#4a505c',
  // Dim, recedes against the near-black 3D canvas (mirrors tokens.css's dark
  // `--label-invalid`, the value actually picked for this surface — see
  // PointCloudScene's canvas-scoped read). #e9e9e7 (the light panel surface
  // colour) was wrong here: only ever reached headless, but it shouted at
  // the same brightness the light-theme token used to (integration review
  // finding 3, residue 2).
  invalid: '#2c3a48',
};

/**
 * Pull the categorical scale out of the live CSS custom properties.
 *
 * Reads `--label-count` first rather than probing until a property comes back
 * empty, so that a design sync which legitimately sets a token to `transparent`
 * cannot silently truncate the scale.
 */
export function readLabelPalette(element?: Element | null): LabelPalette {
  if (typeof getComputedStyle !== 'function') return FALLBACK;
  const target = element ?? (typeof document !== 'undefined' ? document.documentElement : null);
  if (!target) return FALLBACK;

  const style = getComputedStyle(target);
  const declared = Number.parseInt(style.getPropertyValue('--label-count').trim(), 10);
  const count = Number.isFinite(declared) && declared > 0 ? declared : FALLBACK.colors.length;

  const colors: string[] = [];
  for (let index = 0; index < count; index += 1) {
    const value = style.getPropertyValue(`--label-${index}`).trim();
    colors.push(value || FALLBACK.colors[index % FALLBACK.colors.length]);
  }

  const unlabeled = style.getPropertyValue('--label-unlabeled').trim() || FALLBACK.unlabeled;
  const invalid = style.getPropertyValue('--label-invalid').trim() || FALLBACK.invalid;
  return { colors, unlabeled, invalid };
}

/**
 * Palette slot for a label value.
 *
 * `0` means unlabeled and is never a palette entry — it returns `-1`. Label
 * values are 1-based, so the slot is `(label - 1) % size`: indexing the palette
 * directly with `label` would leave slot 0 permanently unused and shift every
 * class by one relative to any other view of the same data. STANDARDS §3 warns
 * about exactly this off-by-one.
 */
export function labelColorIndex(label: number, paletteSize: number): number {
  // A uint32 label above INT32_MAX is a wrapped -1 (STANDARDS §3.1): it is
  // neither unlabeled nor a class, and must not borrow a class's colour.
  if (Number.isFinite(label) && label > 0x7fffffff) return INVALID_SLOT;
  if (!Number.isFinite(label) || label < 1 || paletteSize <= 0) return UNLABELED_SLOT;
  return (Math.floor(label) - 1) % paletteSize;
}

/**
 * Parse a `#rgb` / `#rrggbb` colour into linear-ish 0..1 components.
 *
 * Only hex is handled, because that is what the token file declares. Anything
 * else (a named colour, `oklch(...)`, `transparent`) falls back to mid-grey
 * rather than producing `NaN` components, which would make three.js drop the
 * whole draw call rather than one point.
 */
export function parseColor(value: string): [number, number, number] {
  const text = value.trim();
  const short = /^#([0-9a-f])([0-9a-f])([0-9a-f])$/i.exec(text);
  if (short) {
    return [
      Number.parseInt(short[1] + short[1], 16) / 255,
      Number.parseInt(short[2] + short[2], 16) / 255,
      Number.parseInt(short[3] + short[3], 16) / 255,
    ];
  }
  const long = /^#([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})$/i.exec(text);
  if (long) {
    return [
      Number.parseInt(long[1], 16) / 255,
      Number.parseInt(long[2], 16) / 255,
      Number.parseInt(long[3], 16) / 255,
    ];
  }
  return [0.5, 0.5, 0.5];
}

/**
 * The palette as **linear** float triples, ready for a three.js colour attribute.
 *
 * `parseColor` yields sRGB, because that is the space the tokens are authored
 * in. three.js applies the inverse transfer on output, so writing sRGB straight
 * into the attribute renders every label lighter and less saturated than its
 * swatch in the legend — the legend uses the CSS string directly and so is
 * always correct, which makes the mismatch a visible disagreement between two
 * views of the same label. Converting here keeps the single conversion at the
 * boundary where colour stops being CSS and becomes geometry.
 */
export function paletteToFloats(palette: LabelPalette): {
  colors: [number, number, number][];
  unlabeled: [number, number, number];
  invalid: [number, number, number];
} {
  const toLinear = ([r, g, b]: [number, number, number]): [number, number, number] => [
    srgbToLinear(r),
    srgbToLinear(g),
    srgbToLinear(b),
  ];
  return {
    colors: palette.colors.map((color) => toLinear(parseColor(color))),
    unlabeled: toLinear(parseColor(palette.unlabeled)),
    invalid: toLinear(parseColor(palette.invalid)),
  };
}

/** The colour of a labelColorIndex() slot, sentinels included. */
export function slotColor<T>(palette: { colors: T[]; unlabeled: T; invalid: T }, slot: number): T {
  if (slot === INVALID_SLOT) return palette.invalid;
  return slot < 0 ? palette.unlabeled : palette.colors[slot];
}
