// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * "Egne felter" on Skabeloner as data (R4-EF): rename, options and delete of
 * user columns. Validation reuses Kortlægning's "Tilføj kolonne" helpers, so
 * the two screens refuse the same names with the same copy.
 */

import { saveErrorMessage } from '../app/saveError';
import { COLUMN_KIND_LABEL, columnCreateConflict, columnDraftError, parseOptions } from '../kortlaegning/columnDraft';

const fold = (s: string) => s.trim().toLocaleLowerCase('da');

/**
 * Why `name` cannot replace the column's `ownLabel`, or null. The column's
 * own label is left out of the clash check, so keeping the name or changing
 * only its case is allowed.
 */
export function renameError(name: string, ownLabel: string, catalogueLabels: readonly string[]): string | null {
  const own = fold(ownLabel);
  const others = catalogueLabels.filter((l) => fold(l) !== own);
  return columnDraftError({ name, kind: 'text', optionsText: '' }, others);
}

/** The options as the editor shows them: one per line. */
export function optionsText(options: readonly string[] | undefined): string {
  return (options ?? []).join('\n');
}

/** The new option list, or null when it equals `current` (same values, same order). */
export function optionsChanged(text: string, current: readonly string[]): string[] | null {
  const next = parseOptions(text);
  const same = next.length === current.length && next.every((o, i) => o === current[i]);
  return same ? null : next;
}

/** Why the options cannot be saved, or null: a choice column needs at least one option. */
export function optionsError(text: string): string | null {
  return parseOptions(text).length === 0 ? 'Angiv mindst én valgmulighed.' : null;
}

/** The armed delete button's explanation: the server drops the column's stored values too. */
export function columnDeleteConfirm(name: string): string {
  return `Klik igen for at slette «${name}» og alle gemte værdier i feltet. Det kan ikke fortrydes.`;
}

/** A failed column write: name conflicts in Danish, everything else as a save error. */
export function columnErrorMessage(cause: unknown): string {
  return columnCreateConflict(cause) ?? saveErrorMessage(cause);
}

const KIND_LABEL: Record<string, string> = {
  ...COLUMN_KIND_LABEL,
  multiselect: 'Flervalg',
  url: 'Link',
};

export function columnKindLabel(type: string): string {
  return KIND_LABEL[type] ?? type;
}

/** Whether a column of this type has an option list to edit. */
export function hasOptions(type: string): boolean {
  return type === 'select' || type === 'multiselect';
}
