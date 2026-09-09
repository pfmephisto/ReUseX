// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { CloudPointsPage } from '../api/types';
import type { RuxpPage } from './binaryPoints';
import { ruxpField } from './binaryPoints';
import { fieldIndices, hasColors, hasPositions } from './pagination';
import type { PageBuffers } from './PointCloudScene';

/**
 * Decode a points page — JSON or RUXP — into the typed arrays the scene wants.
 *
 * Pure, and separate from the scene, because it is where the two easy silent
 * corruptions live: reading columns positionally without checking `fields`
 * (which turns a Label cloud's label column into an x coordinate), and treating
 * 0..255 sRGB bytes as if they were 0..1 linear floats (which washes the whole
 * cloud out).
 *
 * Two transports, one destination. The RUXP path (#283,
 * `docs/gui/binary-points.md`) reaches `PageBuffers` with at most one table
 * lookup per point; the JSON path stays for old servers and for debuggability,
 * and the page-walking above and the GPU upload below are identical either way.
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

// ------------------------------------------------------------ RUXP (#283) ----

/**
 * A field that is present but not the shape this viewport can read.
 *
 * Returning `null` would be wrong here: `null` means "this cloud type has no
 * such attribute", which the caller treats as normal. A page that carries `xyz`
 * as anything other than `f32 x 3` is a server bug or a version mismatch, and
 * silently rendering nothing would hide it.
 */
function requireShape(
  page: RuxpPage,
  name: string,
  type: 'f32' | 'u8' | 'u32',
  components: number,
): boolean {
  const field = ruxpField(page, name);
  if (!field) return false;
  if (field.type !== type || field.components !== components) {
    throw new Error(
      `RUXP: field '${name}' is ${field.type}x${field.components}, expected ${type}x${components}`,
    );
  }
  return true;
}

/**
 * xyz triples straight off the wire, or `null` for a page with no positions.
 *
 * No copy and no loop: the `xyz` section *is* a `Float32Array` in the layout
 * three.js wants, which is the entire point of the format being planar.
 */
export function decodeBinaryPositions(page: RuxpPage): Float32Array | null {
  if (!requireShape(page, 'xyz', 'f32', 3)) return null;
  return page.view('xyz') as Float32Array;
}

/**
 * Linear rgb triples in 0..1, or `null` for a cloud with no colour.
 *
 * **This loop is the only per-point work left in the binary path**, and it was
 * kept only after the zero-work alternative was tried and rejected.
 *
 * The tempting version is to hand `PointCloudScene` the `Uint8Array` section
 * itself as a normalized `THREE.BufferAttribute(rgb, 3, true)` and let the GPU
 * do the work. It does not work, for two independent reasons:
 *
 * 1. `normalized: true` means exactly `r / 255`, and nothing else. three.js
 *    applies colour management to `THREE.Color` values and to textures
 *    (`texture.colorSpace`), but a vertex-colour *attribute* has no colour
 *    space to declare — the `color_vertex` shader chunk multiplies it in
 *    as-is, assuming it is already in the working (linear) space. Feeding it
 *    sRGB samples over 255 double-encodes, and the cloud comes out visibly
 *    washed out and flat. Same wrong answer as `r / 255` on the CPU, just
 *    later and harder to see.
 * 2. `PointCloudScene.writeColors` rewrites one `Float32Array` colour
 *    attribute from the retained sources every time the user toggles
 *    RGB ↔ label, and the label branch writes palette floats into the same
 *    attribute. A `u8` attribute would need a second attribute layout and a
 *    per-page branch through the whole scene for it.
 *
 * The honest zero-work fix is a custom shader that decodes sRGB on the GPU,
 * which means replacing `PointsMaterial` and owning that shader forever. It
 * buys one table lookup per point. Not worth it — revisit if profiling ever
 * says otherwise.
 *
 * So: the same 256-entry table the JSON path already uses, over a typed array
 * with no bounds surprises and no `NaN` to defend against — the bytes came from
 * a `Uint8Array`, so `clampByte` is not needed and is not called.
 */
export function decodeBinaryColors(page: RuxpPage): Float32Array | null {
  if (!requireShape(page, 'rgb', 'u8', 3)) return null;
  const rgb = page.view('rgb') as Uint8Array;
  const out = new Float32Array(rgb.length);
  for (let i = 0; i < rgb.length; i += 1) out[i] = SRGB_TO_LINEAR[rgb[i]];
  return out;
}

/**
 * Label values, or `null` when this page is not from a `Label` cloud.
 *
 * Zero-copy, like the positions: RUXP stores `label` as `u32 x 1`, which is
 * already the `Uint32Array` `PageBuffers` carries. `0` stays `0` — it means
 * unlabeled, and the palette lookup is what maps it (STANDARDS §3).
 */
export function decodeBinaryLabels(page: RuxpPage): Uint32Array | null {
  if (!requireShape(page, 'label', 'u32', 1)) return null;
  return page.view('label') as Uint32Array;
}

/** One fetched page, in whichever transport the server turned out to speak. */
export type StreamPage =
  | { format: 'binary'; page: RuxpPage }
  | { format: 'json'; page: CloudPointsPage };

/** Points in this page, whichever transport carried it. */
export function pageCount(page: StreamPage): number {
  return page.page.count;
}

/** Points in the whole cloud, whichever transport carried it. */
export function pageTotal(page: StreamPage): number {
  return page.page.total;
}

/**
 * Fold a geometry page and its optional sibling label page into one upload.
 *
 * The two pages are joined *positionally*: sibling clouds of one scan are
 * returned in storage order and are index-aligned, which is the only reason
 * this works without a join key (`docs/CONTRACTS.md`, STANDARDS §3.2).
 *
 * `null` means the page carries no geometry this viewport can render — a
 * `Normal` cloud, say. The caller skips it rather than uploading an empty page.
 */
export function toPageBuffers(
  geometry: StreamPage,
  labels: StreamPage | null,
): PageBuffers | null {
  const positions =
    geometry.format === 'binary'
      ? decodeBinaryPositions(geometry.page)
      : decodePositions(geometry.page);
  if (!positions) return null;

  const rgb =
    geometry.format === 'binary'
      ? decodeBinaryColors(geometry.page)
      : decodeColors(geometry.page);

  let labelValues: Uint32Array | null = null;
  if (labels) {
    labelValues =
      labels.format === 'binary' ? decodeBinaryLabels(labels.page) : decodeLabels(labels.page);
  }

  return { positions, rgb, labels: labelValues };
}
