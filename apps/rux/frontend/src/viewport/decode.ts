// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { CloudPointsPage } from '../api/types';
import { fieldIndices, hasColors, hasPositions } from './pagination';

/**
 * Decode a JSON points page into typed arrays.
 *
 * Pure, and separate from the scene, for two reasons: it is the step #283
 * replaces wholesale when the binary transport lands (the page-walking above it
 * and the GPU upload below it both stay), and it is where the two easy silent
 * corruptions live — reading columns positionally without checking `fields`,
 * and treating 0..255 bytes as if they were 0..1 floats.
 */

/**
 * sRGB → linear, tabulated over the 256 byte values.
 *
 * `p.r/g/b` on a `pcl::PointXYZRGB` are `uint8` sRGB samples straight off the
 * sensor, and the server serialises them as integers 0..255
 * (`apps/rux/src/gui/api.cpp`). three.js expects colour attributes in linear
 * space and applies the inverse transfer on output, so handing it `r / 255`
 * double-encodes: the whole cloud comes out visibly washed out and flat. There
 * are only 256 possible inputs, so this is exact and costs one array.
 */
/**
 * The sRGB electro-optical transfer, for a single 0..1 channel.
 *
 * Exported because the label palette needs the identical conversion: token
 * colours are authored in sRGB (they are CSS hex), and a swatch in the legend
 * would not match the points it describes if only one of the two went through
 * this. Same transfer, one definition.
 */
export function srgbToLinear(channel: number): number {
  return channel <= 0.04045 ? channel / 12.92 : Math.pow((channel + 0.055) / 1.055, 2.4);
}

const SRGB_TO_LINEAR = (() => {
  const table = new Float32Array(256);
  for (let value = 0; value < 256; value += 1) table[value] = srgbToLinear(value / 255);
  return table;
})();

/** xyz triples, or `null` when the page carries no positional geometry. */
export function decodePositions(page: CloudPointsPage): Float32Array | null {
  if (!hasPositions(page.fields)) return null;
  const index = fieldIndices(page.fields);
  const rows = page.points;
  const out = new Float32Array(rows.length * 3);
  for (let i = 0; i < rows.length; i += 1) {
    const row = rows[i];
    out[i * 3] = row[index.x];
    out[i * 3 + 1] = row[index.y];
    out[i * 3 + 2] = row[index.z];
  }
  return out;
}

/** Linear rgb triples in 0..1, or `null` for a cloud with no colour. */
export function decodeColors(page: CloudPointsPage): Float32Array | null {
  if (!hasColors(page.fields)) return null;
  const index = fieldIndices(page.fields);
  const rows = page.points;
  const out = new Float32Array(rows.length * 3);
  for (let i = 0; i < rows.length; i += 1) {
    const row = rows[i];
    out[i * 3] = SRGB_TO_LINEAR[clampByte(row[index.r])];
    out[i * 3 + 1] = SRGB_TO_LINEAR[clampByte(row[index.g])];
    out[i * 3 + 2] = SRGB_TO_LINEAR[clampByte(row[index.b])];
  }
  return out;
}

/**
 * Label values, or `null` when this page is not from a `Label` cloud.
 *
 * `0` is kept as `0` — it means unlabeled, and collapsing it into the palette
 * here would lose the distinction the whole label contract rests on
 * (STANDARDS §3). The colouring step is what maps it to `--label-unlabeled`.
 */
export function decodeLabels(page: CloudPointsPage): Uint32Array | null {
  const index = fieldIndices(page.fields);
  if (index.label === undefined) return null;
  const rows = page.points;
  const out = new Uint32Array(rows.length);
  for (let i = 0; i < rows.length; i += 1) {
    const value = rows[i][index.label];
    out[i] = Number.isFinite(value) && value > 0 ? Math.floor(value) : 0;
  }
  return out;
}

/**
 * A colour component, forced into a valid 0..255 table index.
 *
 * `NaN` (and anything non-numeric that rounds to it) has no position on the
 * scale, so it becomes 0. `±Infinity` does: it is out of range in a known
 * direction and clamps to that end. Rounding first and testing the *result*
 * is what makes both true at once — `Number.isFinite(value)` alone would send
 * a blown-out `+Infinity` component to black.
 *
 * Nothing may escape this returning `NaN`: one `NaN` in a colour attribute
 * makes three.js drop the entire draw call, not the one point.
 */
function clampByte(value: number): number {
  const rounded = Math.round(value);
  if (Number.isNaN(rounded)) return 0;
  return Math.min(255, Math.max(0, rounded));
}
