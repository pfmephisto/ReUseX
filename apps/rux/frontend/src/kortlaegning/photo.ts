// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The best-photo lookup as data: whether a part is linked to an instance, the
 * key a frames lookup is tagged with, and how a (possibly stale) lookup
 * resolves. Pure, so Kortlægning's evidence panel and edit dialog share one
 * rule: another key's photo or error is never shown as the current part's.
 */

import type { SurveyPart, VisibleFrame } from '../api/types';

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
