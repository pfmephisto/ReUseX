// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The best-photo lookup as data: whether a part is linked to an instance, the
 * key a frames lookup is tagged with, and how a (possibly stale) lookup
 * resolves. Pure, so Kortlægning's evidence panel and edit dialog share one
 * rule: another key's photo or error is never shown as the current part's.
 */

import type { PartPhotos, SurveyPart, VisibleFrame } from '../api/types';

/**
 * A part counts as linked when it names a cloud and an instance id — and that
 * id is `>= 1`: label `0` means unlabeled (STANDARDS §3), so an `instance_id`
 * of 0 is not a real instance and both `/renders` and the frames lookup would
 * 400 on it.
 */
export function hasInstanceLink(part: SurveyPart | null): part is SurveyPart & { cloud: string; instance_id: number } {
  return !!part && !!part.cloud && part.instance_id !== null && part.instance_id >= 1;
}

/** Identifies which highlight a frame lookup's result belongs to. */
export function instanceKey(cloud: string, instanceId: number): string {
  return `${cloud}/${instanceId}`;
}

/**
 * A frame lookup's result, tagged with the highlight it was fetched for.
 *
 * `useAsync` keeps the previous `data` (and does not clear a previous
 * `error`) across a deps change until the new request settles — and `loading`
 * only flips to `true` inside the effect, so there is one render, right after
 * the selected part changes, where `loading` is still `false` and `data`
 * still holds the *old* part's result. Tagging the result with its own key
 * and comparing against the key computed fresh on every render (from props,
 * not from hook state) is what catches that render, not just the async race.
 */
export interface FrameLookup {
  key: string;
  frames: VisibleFrame[];
  /** True when the request for `key` itself failed (not just "zero frames"). */
  failed: boolean;
}

/**
 * Turns a (possibly stale) `FrameLookup` into the Foto tab's `photoFrameId`
 * / `photoFailed` input for `evidenceSources`. Exported and pure — kept
 * separate from `EvidencePanel` itself — so the "a different key's data (or
 * error) is never shown as the current part's" rule is unit-testable without
 * a DOM.
 *
 * `data` not matching `currentKey` (including `data` not having arrived yet)
 * is treated exactly like "still loading": `undefined`/not-failed. There is
 * deliberately no way to distinguish "loading" from "stale" in the output —
 * both must render as "Indlæser foto…", never as the previous part's photo
 * or error.
 */
export function resolvePhotoState(
  currentKey: string | null,
  data: FrameLookup | undefined,
): { photoFrameId: number | null | undefined; photoFailed: boolean } {
  if (!data || data.key !== currentKey) return { photoFrameId: undefined, photoFailed: false };
  if (data.failed) return { photoFrameId: undefined, photoFailed: true };
  return { photoFrameId: data.frames[0]?.frame_id ?? null, photoFailed: false };
}

// ---------------------------------------------------------------- table ----

/** Edge length asked of the server for a table-row thumbnail. */
export const ROW_THUMB_MAX_SIZE = 96;

/**
 * The table's "n fotos" text for a part. `undefined` photos (the batch has not
 * resolved, or the part has no instance) render as an empty string, never as
 * a guess.
 */
export function photoCountText(photos: PartPhotos | undefined): string {
  if (!photos) return '';
  return photos.count === 1 ? '1 foto' : `${photos.count} fotos`;
}

/**
 * The best frame to show as a row thumbnail: a part row its own best frame; a
 * type row the best frame of its first part (in part order) that has one.
 * `null` when there is none (or the batch has not resolved) — the row then
 * shows its quiet placeholder.
 */
export function rowThumbFrame(
  parts: readonly Pick<SurveyPart, 'code'>[],
  photos: Readonly<Record<string, PartPhotos>> | null,
): number | null {
  if (!photos) return null;
  for (const part of parts) {
    const id = photos[part.code]?.best_frame_id;
    if (id !== null && id !== undefined) return id;
  }
  return null;
}

// ------------------------------------------------------------ Fotos strip ----

/** Thumbnails shown in the Fotos row before collapsing the rest into `+n`. */
export const PHOTO_STRIP_MAX = 5;

/** Edge length asked of the server for a Fotos-strip thumbnail. */
export const PHOTO_STRIP_THUMB_SIZE = 160;

/** Splits a frame list into the thumbnails shown and the `+n` overflow count. */
export function photoStrip(
  frames: readonly VisibleFrame[],
  max: number = PHOTO_STRIP_MAX,
): { visible: VisibleFrame[]; overflow: number } {
  return { visible: frames.slice(0, max), overflow: Math.max(0, frames.length - max) };
}

/**
 * What the Fotos strip shows for a (possibly stale) lookup: the strip itself,
 * the `(n)` count in its label, or the line that stands in for it. A lookup
 * for another key counts as loading, never as this part's photos.
 */
export function photoStripModel(
  currentKey: string | null,
  data: FrameLookup | undefined,
): { strip: { visible: VisibleFrame[]; overflow: number } | null; count: number | null; message: string | null } {
  if (currentKey === null) {
    return { strip: null, count: null, message: 'Ingen fotos — bygningsdelen er ikke koblet til en instans.' };
  }
  if (!data || data.key !== currentKey) return { strip: null, count: null, message: 'Indlæser fotos…' };
  if (data.failed) return { strip: null, count: null, message: 'Fotos kunne ikke hentes.' };
  if (data.frames.length === 0) {
    return { strip: null, count: 0, message: 'Ingen fotos fundet for denne instans.' };
  }
  return { strip: photoStrip(data.frames), count: data.frames.length, message: null };
}

/**
 * What the table's photo batch depends on: the instance-backed parts, by code
 * and instance. The page re-fetches `GET /survey/photos` when this changes (a
 * sync adds parts, a delete removes one) and not on every edit of a quantity
 * or a note. Empty when no part is linked — nothing to fetch.
 */
export function photoBatchKey(types: readonly { parts: readonly SurveyPart[] }[]): string {
  const keys: string[] = [];
  for (const t of types) {
    for (const p of t.parts) {
      if (hasInstanceLink(p)) keys.push(`${p.code}=${instanceKey(p.cloud, p.instance_id)}`);
    }
  }
  return keys.sort().join(',');
}
