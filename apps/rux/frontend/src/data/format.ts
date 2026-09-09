// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Formatting for fields the contract marks optional.
 *
 * The rule this module enforces, in one place so it cannot be forgotten in one
 * column of one table: an absent value renders as an em-dash, never as
 * `undefined`, `null`, `NaN` or `0`. `area` and `source_instance_guid` are the
 * live examples — both are derived on read and both are genuinely absent for a
 * component with fewer than three boundary vertices or no instance
 * provenance — but the same applies to every `?` field in `api/types.ts`.
 *
 * The `0` case matters as much as the `undefined` one. A component with no
 * area is not a component with zero area, and printing `0.00 m²` states a
 * measurement that was never taken.
 */

/** The single glyph for "this project does not have that value". */
export const ABSENT = '—';

/** Grouped integers — the same formatter the dashboard uses for counts. */
const COUNT = new Intl.NumberFormat();

/**
 * Areas in m², to two decimals.
 *
 * Two decimals is a square centimetre, which is finer than any polygon derived
 * from a handheld depth scan deserves and is still coarse enough to read. A
 * non-finite value (a server bug, or a `NaN` that survived a bad polygon) is
 * treated as absent rather than printed.
 */
export function formatArea(area: number | undefined): string {
  if (area === undefined || !Number.isFinite(area)) return ABSENT;
  return `${area.toFixed(2)} m²`;
}

/** A count, grouped. `undefined` is absent; `0` is a real answer and is shown. */
export function formatCount(count: number | undefined): string {
  if (count === undefined || !Number.isFinite(count)) return ABSENT;
  return COUNT.format(count);
}

/**
 * Confidence as a percentage, or the words for the two special values.
 *
 * `-1` is the contract's "manually created", which is not a low confidence —
 * rendering it as `-100%` would rank hand-authored components below the worst
 * automatic ones in a sorted table.
 */
export function formatConfidence(confidence: number | undefined): string {
  if (confidence === undefined || !Number.isFinite(confidence)) return ABSENT;
  if (confidence < 0) return 'manual';
  return `${(confidence * 100).toFixed(0)}%`;
}

/** A string field, where the contract's "absent" is either missing or empty. */
export function formatText(value: string | null | undefined): string {
  const trimmed = value?.trim();
  return trimmed ? trimmed : ABSENT;
}

/**
 * A guid, shortened for a table cell but never invented.
 *
 * Shows the leading segment of a UUID, or the first eight characters of
 * anything else. Returns the em-dash for an absent guid rather than an
 * ellipsis, so a missing link and a truncated one never look alike.
 */
export function formatGuidShort(guid: string | null | undefined): string {
  const trimmed = guid?.trim();
  if (!trimmed) return ABSENT;
  const head = trimmed.split('-')[0];
  return head.length >= 8 ? head : trimmed.slice(0, 8);
}

/**
 * A parent-component id.
 *
 * `-1` means "no parent" and must not be printed: it is a sentinel, and it is
 * emphatically not a room. The API has no room field for a component, and
 * neither does this UI.
 */
export function formatParent(parentId: number | undefined): string {
  if (parentId === undefined || !Number.isFinite(parentId) || parentId < 0) return ABSENT;
  return String(parentId);
}

/** A pose or intrinsics number: fixed decimals so a matrix column lines up. */
export function formatFixed(value: number | undefined, decimals = 3): string {
  if (value === undefined || !Number.isFinite(value)) return ABSENT;
  return value.toFixed(decimals);
}
