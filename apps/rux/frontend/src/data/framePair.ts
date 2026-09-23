// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Frame-pair inspector model — URL-state parsing and view constants, without
 * React.
 *
 * The two frame IDs live in `?a=` and `?b=` URL search parameters, following
 * the same convention the frames page uses for `?frame=`. Each ID is parsed
 * independently so one invalid entry does not blank the other panel.
 */

import type { DescriptorMethod, FrameImageKind } from '../api/types';

export type { DescriptorMethod };

// -------------------------------------------------------- descriptor methods --

export const DESCRIPTOR_METHODS: readonly DescriptorMethod[] = ['orb', 'sift', 'akaze'];

/** Display labels for the descriptor-method selector. */
export const DESCRIPTOR_METHOD_LABELS: Record<DescriptorMethod, string> = {
  orb: 'ORB',
  sift: 'SIFT',
  akaze: 'AKAZE',
};

// ------------------------------------------------------------ image kinds --

export const IMAGE_KINDS: readonly FrameImageKind[] = [
  'color',
  'depth',
  'confidence',
  'segmentation',
];

/** Display labels for the image-kind tabs. */
export const IMAGE_KIND_LABELS: Record<FrameImageKind, string> = {
  color: 'Colour',
  depth: 'Depth',
  confidence: 'Confidence',
  segmentation: 'Seg',
};

/** Kinds that must be requested with `normalize=true` to be displayable. */
export const NORMALIZED_KINDS: ReadonlySet<FrameImageKind> = new Set([
  'depth',
  'confidence',
  'segmentation',
]);

// --------------------------------------------------------- URL-state parsing --

/**
 * Parse a frame id from a URL search parameter value.
 *
 * Returns `null` for anything that cannot be a frame id: absent, empty,
 * non-integer, or negative. Keeps each side of the pair independent — a null
 * on one side does not affect the other.
 */
export function parseFrameId(raw: string | null | undefined): number | null {
  if (raw === null || raw === undefined || raw === '') return null;
  // Only plain decimal digit strings (no scientific notation, no decimals).
  // `Number('1e3')` is 1000, but '1e3' in a URL was not written by a frame picker.
  if (!/^\d+$/.test(raw)) return null;
  const n = Number(raw);
  // isSafeInteger checks integer, finite, and within [-2^53+1, 2^53-1] in one call.
  if (!Number.isSafeInteger(n) || n < 0) return null;
  return n;
}
