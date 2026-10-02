// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * One resource value as the Kortlægning table shows and edits it. Pure, so
 * "blank → filled sends the value, an untouched blur sends nothing, emptying
 * sends null" is tested here; `ResourceCell` wires it to `useTextDraft`,
 * which runs exactly `draftCommit(draft, cellInputText(...), cellValidate(...))`.
 *
 * Wire format (spec §4.4): every value is a string or null. Numbers travel
 * with a `.` decimal; booleans as "true"/"false" (plan R10); dates ISO;
 * multiselect as a JSON array of option strings, e.g. `["concrete","steel"]`.
 */

import type { EnvironmentStatus, ResourceKey, Treatment } from '../api/types';
import { draftCommit, type DraftCommit, type DraftValidate } from '../app/textDraft';
import { ENV_LABEL, TREATMENT_LABEL, formatNumber, formatQuantityInput, parseDanishNumber } from './vocab';

/** The muted placeholder of a blank cell. */
export const BLANK = '—';

/**
 * Keys Phase 1 refuses to clear (`resource_keys.cpp` `clearable()`,
 * :125-129): an empty draft on one of these is a 400 ("'sys:quantity' cannot
 * be cleared"), so `cellValidate` rejects it client-side instead (R3-D1).
 * `ResourceCell`'s enum `<select>` reads this to leave out the blank option
 * for `sys:treatment`.
 */
export const NON_CLEARABLE: ReadonlySet<string> = new Set([
  'sys:name',
  'sys:quantity',
  'sys:unit',
  'sys:treatment',
  'sys:starred',
]);

const TRUTHY = new Set(['true', '1', 'ja', 'yes']);

export function isTrue(value: string | null): boolean {
  return value !== null && TRUTHY.has(value.trim().toLowerCase());
}

export function toggleValue(value: string | null): 'true' | 'false' {
  return isTrue(value) ? 'false' : 'true';
}

/** Danish labels for a leksikon `TriState` field's options (`PropertyType::TriState`, resource_keys.cpp:224-226). */
const YES_NO_UNKNOWN_LABEL: Record<string, string> = {
  yes: 'Ja',
  no: 'Nej',
  unknown: 'Ukendt',
};

/** An enum option's label: Danish for behandling, miljøstatus and yes/no/unknown, the option itself otherwise. */
export function optionLabel(key: ResourceKey, option: string): string {
  if (key.id === 'sys:treatment' && option in TREATMENT_LABEL) return TREATMENT_LABEL[option as Treatment];
  if (key.id === 'sys:environment' && option in ENV_LABEL) return ENV_LABEL[option as EnvironmentStatus];
  if (option in YES_NO_UNKNOWN_LABEL) return YES_NO_UNKNOWN_LABEL[option];
  return option;
}

/** The options a select lists: the key's, plus a stored value they no longer include. */
export function enumOptions(key: ResourceKey, value: string | null): string[] {
  const options = key.options ?? [];
  return value !== null && value !== '' && !options.includes(value) ? [...options, value] : [...options];
}

/** Parses a JSON array of strings, or null when `value` is not one (a plain string, malformed JSON, …). */
function parseStringArray(value: string): string[] | null {
  let parsed: unknown;
  try {
    parsed = JSON.parse(value);
  } catch {
    return null;
  }
  return Array.isArray(parsed) && parsed.every((v) => typeof v === 'string') ? (parsed as string[]) : null;
}

/** What a cell shows; `''` for a blank (the cell then draws `BLANK`, muted). */
export function cellDisplay(key: ResourceKey, value: string | null): string {
  if (value === null || value.trim() === '') return '';
  if (key.id === 'sys:starred') return isTrue(value) ? '★' : '';
  switch (key.data_type) {
    case 'number': {
      const n = Number(value);
      const shown = Number.isFinite(n) ? formatNumber(n, 3) : value;
      return key.unit ? `${shown} ${key.unit}` : shown;
    }
    case 'boolean':
      return isTrue(value) ? 'Ja' : 'Nej';
    case 'enum':
      return optionLabel(key, value);
    case 'multiselect': {
      // R3-A1: a leksikon EnumArray field — joined option labels, read-only.
      const items = parseStringArray(value);
      return items ? items.map((v) => optionLabel(key, v)).join(', ') : value;
    }
    default: {
      // R3-D7: a leksikon StringArray field (data_type 'text', editable:
      // false) stores its JSON array verbatim; show it joined, not raw JSON.
      const items = parseStringArray(value);
      return items ? items.join(', ') : value;
    }
  }
}

/** The text an editor starts from: a wire number in Danish input form; anything else as stored. */
export function cellInputText(key: ResourceKey, value: string | null): string {
  if (value === null) return '';
  if (key.data_type === 'number') {
    const n = Number(value);
    return value.trim() !== '' && Number.isFinite(n) ? formatQuantityInput(n) : value;
  }
  return value;
}

const ISO_DATE = /^\d{4}-\d{2}-\d{2}$/;

/** Parses a draft into the wire value to send: `null` clears, a failed parse is invalid. */
export function cellValidate(key: ResourceKey): DraftValidate<string | null> {
  return (draft) => {
    const t = draft.trim();
    if (t === '') return NON_CLEARABLE.has(key.id) ? null : { value: null };
    switch (key.data_type) {
      case 'number': {
        const n = parseDanishNumber(t);
        return n === null ? null : { value: String(n) };
      }
      case 'date':
        return ISO_DATE.test(t) ? { value: t } : null;
      case 'enum':
        return (key.options ?? []).includes(t) ? { value: t } : null;
      case 'boolean':
        return { value: isTrue(t) ? 'true' : 'false' };
      case 'multiselect':
        // R3-A1: no editor of its own yet — a draft never parses, so a blur
        // can never commit one.
        return null;
      default:
        return { value: t };
    }
  };
}

/** A cell's blur (or a select's change): what to send, if anything. */
export function cellCommit(key: ResourceKey, draft: string, value: string | null): DraftCommit<string | null> {
  return draftCommit(draft, cellInputText(key, value), cellValidate(key));
}
