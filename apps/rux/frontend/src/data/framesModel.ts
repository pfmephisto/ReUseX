// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The frame browser's filter and selection model, without React.
 *
 * Two decisions live here rather than in the component. The first is that the
 * filter is a *request*, not a predicate: `GET /frames?segmented=` narrows the
 * ids server-side, because the client-side alternative is one `/frames/{id}`
 * per frame to read one boolean and that route decodes the depth and
 * confidence blobs to answer it. So there is no `filterFrames(ids)` function
 * below — there is a function that turns the UI's filter into a query
 * parameter, and the server does the filtering.
 *
 * The second is that both the filter and the selection round-trip through the
 * URL, matching the viewport's existing `?cloud=` convention, so every state
 * this screen can be in is a link.
 */

import type { FrameImageKind, FrameInfo, FrameList } from '../api/types';

export type FrameFilter = 'all' | 'segmented' | 'unsegmented';

export const FRAME_FILTERS: readonly FrameFilter[] = ['all', 'segmented', 'unsegmented'];

/** Human labels for the filter control, in the order they are offered. */
export const FRAME_FILTER_LABELS: Record<FrameFilter, string> = {
  all: 'All',
  segmented: 'Segmented',
  unsegmented: 'Unsegmented',
};

/**
 * Read `?filter=` — anything unrecognised (or absent) means "all".
 *
 * A hand-edited URL must land on a working screen rather than an error: the
 * filter is a view preference, and there is no such thing as an invalid one.
 */
export function parseFrameFilter(raw: string | null | undefined): FrameFilter {
  return FRAME_FILTERS.includes(raw as FrameFilter) ? (raw as FrameFilter) : 'all';
}

/**
 * The `segmented` query value for a filter — `undefined` means "do not send it".
 *
 * `undefined` rather than omitting the key at the call site so this stays the
 * one place that knows the mapping, and so `buildQuery` drops it as it drops
 * every other undefined.
 */
export function segmentedParam(filter: FrameFilter): boolean | undefined {
  if (filter === 'segmented') return true;
  if (filter === 'unsegmented') return false;
  return undefined;
}

/**
 * Resolve `?frame=` against the ids actually on screen.
 *
 * Returns `null` rather than the raw number when the id is not in the list:
 * that is what happens on every filter change that excludes the selection, and
 * a detail pane fetching a frame the grid is not showing is a screen whose two
 * halves disagree. Falling back to "nothing selected" makes the grid the
 * authority.
 */
export function parseSelectedFrame(
  raw: string | null | undefined,
  ids: readonly number[],
): number | null {
  if (raw === null || raw === undefined || raw === '') return null;
  const id = Number(raw);
  if (!Number.isInteger(id)) return null;
  return ids.includes(id) ? id : null;
}

/**
 * Move the selection by `delta` positions within the visible ids.
 *
 * Clamps rather than wraps. Wrapping from the last frame to the first is a
 * jump of several hundred rows in a virtualised grid, which reads as the list
 * having been reloaded rather than as the cursor having moved.
 */
export function stepSelection(
  ids: readonly number[],
  current: number | null,
  delta: number,
): number | null {
  if (ids.length === 0) return null;
  if (current === null) return ids[0];

  const index = ids.indexOf(current);
  if (index < 0) return ids[0];

  const next = Math.min(ids.length - 1, Math.max(0, index + delta));
  return ids[next];
}

/**
 * The one-line count under the filter control.
 *
 * `total_count` and `segmented_count` describe the **whole scan** even when
 * `ids` is filtered — that is explicit in the contract — so the filtered size
 * comes from `ids.length` and never from the counts. Getting this backwards
 * would show "380 of 380" on a filter that is showing twelve frames.
 */
export function describeFrameCounts(list: FrameList, filter: FrameFilter): string {
  const shown = list.ids.length;
  const total = list.total_count;
  const segmented = list.segmented_count;

  if (filter === 'all') {
    return `${total} frames · ${segmented} segmented`;
  }
  const noun = filter === 'segmented' ? 'segmented' : 'unsegmented';
  return `${shown} ${noun} of ${total} frames`;
}

/**
 * A frame's capture time, or `null` when the scan did not record one.
 *
 * `-1` is the contract's "unknown", and it is the reason this exists: passing
 * it to `Date` yields 31 December 1969, a plausible-looking date that is
 * entirely fictional. A scan that carries no timestamps would show every frame
 * captured moments before the moon landing.
 */
export function formatFrameTimestamp(timestamp: number | undefined): string | null {
  if (timestamp === undefined || !Number.isFinite(timestamp) || timestamp <= 0) return null;
  // Epoch *seconds* on the wire; Date takes milliseconds.
  const date = new Date(timestamp * 1000);
  if (Number.isNaN(date.getTime())) return null;
  return date.toISOString().replace('T', ' ').replace(/\.\d+Z$/, 'Z');
}

/** One image a frame can offer, and whether this frame actually has it. */
export interface FrameImageSlot {
  kind: FrameImageKind;
  label: string;
  available: boolean;
  /**
   * True for every kind whose stored form is a 16-bit single-channel PNG. The
   * request must carry `normalize=true` or the browser paints near-black, and
   * the result then carries no metric scale and must be labelled as such.
   */
  normalized: boolean;
}

/**
 * The four image slots of a frame, in display order.
 *
 * Absent kinds are listed rather than hidden: "this frame has no confidence
 * image" is information about the capture, and a pane that silently shows
 * three panels for one frame and four for the next teaches the user nothing.
 *
 * `color` is always claimed available — `has_color` is not in the contract, and
 * a frame with no colour image answers 404 on the image route, which the
 * thumbnail's own error state already covers.
 */
export function frameImageSlots(frame: FrameInfo): FrameImageSlot[] {
  return [
    { kind: 'color', label: 'Colour', available: true, normalized: false },
    {
      kind: 'depth',
      label: 'Depth',
      available: frame.has_depth === true,
      normalized: true,
    },
    {
      kind: 'confidence',
      label: 'Confidence',
      available: frame.has_confidence === true,
      normalized: true,
    },
    {
      kind: 'segmentation',
      label: 'Segmentation',
      available: frame.has_segmentation,
      normalized: true,
    },
  ];
}

/**
 * A row-major 4x4 pose as four rows of four, or `null` if it is not one.
 *
 * The contract says row-major 4x4; a server that sends something else gets a
 * "no pose" pane rather than a grid of `undefined`.
 */
export function poseRows(pose: number[] | undefined): number[][] | null {
  if (!pose || pose.length !== 16) return null;
  return [pose.slice(0, 4), pose.slice(4, 8), pose.slice(8, 12), pose.slice(12, 16)];
}

/**
 * Translation column of a row-major 4x4 pose — the camera's world position.
 *
 * Row-major means the translation is the *last column* (elements 3, 7, 11), not
 * the last row. Reading the last row instead yields `[0, 0, 0]` for every
 * well-formed pose, which looks like a scan collapsed at the origin.
 */
export function poseTranslation(pose: number[] | undefined): [number, number, number] | null {
  if (!pose || pose.length !== 16) return null;
  return [pose[3], pose[7], pose[11]];
}
