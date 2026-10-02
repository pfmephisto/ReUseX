// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The "Tilføj kolonne" dialog as data (spec §6.1): validate the draft before
 * any request, build the column-create body, and decide whether a seed
 * template is copied first (plan R11).
 *
 * `columnDraftError` covers only what can be checked client-side. A name
 * can still collide with a leksikon field name, or with a column that still
 * has stored values — the server rejects those with 409, and the dialog
 * (Task 8) shows that message the same way it shows this one: as a single
 * `error: string | null` it owns, set either from `columnDraftError` on
 * submit or from the caught 409's body. Nothing here needs to change to
 * make room for that — the two never compete for the same field.
 */

import type { ResourceColumnCreate, Template } from '../api/types';

export type ColumnKind = 'text' | 'number' | 'date' | 'boolean' | 'select';
export const COLUMN_KINDS: readonly ColumnKind[] = ['text', 'number', 'date', 'boolean', 'select'];
export const COLUMN_KIND_LABEL: Record<ColumnKind, string> = {
  text: 'Tekst',
  number: 'Tal',
  date: 'Dato',
  boolean: 'Ja/nej',
  select: 'Valgliste',
};

export interface ColumnDraft {
  name: string;
  kind: ColumnKind;
  /** Comma- or line-separated; used by `select` only. */
  optionsText: string;
}

export const EMPTY_COLUMN_DRAFT: ColumnDraft = { name: '', kind: 'text', optionsText: '' };

export function parseOptions(text: string): string[] {
  const seen = new Set<string>();
  const out: string[] = [];
  for (const raw of text.split(/[\n,]/)) {
    const option = raw.trim();
    const folded = option.toLocaleLowerCase('da');
    if (option === '' || seen.has(folded)) continue;
    seen.add(folded);
    out.push(option);
  }
  return out;
}

/** Why the draft cannot be submitted, or null. `existingLabels`: every catalogue key's label. */
export function columnDraftError(draft: ColumnDraft, existingLabels: readonly string[]): string | null {
  const name = draft.name.trim();
  if (name === '') return 'Angiv et navn.';
  const folded = name.toLocaleLowerCase('da');
  if (existingLabels.some((l) => l.trim().toLocaleLowerCase('da') === folded)) {
    return 'Der findes allerede et felt med det navn.';
  }
  if (draft.kind === 'select' && parseOptions(draft.optionsText).length === 0) {
    return 'Angiv mindst én valgmulighed.';
  }
  return null;
}

export function columnCreateBody(draft: ColumnDraft): ResourceColumnCreate {
  const body: ResourceColumnCreate = { name: draft.name.trim(), type: draft.kind };
  if (draft.kind === 'select') body.options = parseOptions(draft.optionsText);
  return body;
}

/** The dialog's note when the selected template is a seed, else null. */
export function seedNote(template: Template | null): string | null {
  return template?.seed ? `Kolonnen føjes til standardskabelonen »${template.name}«.` : null;
}

/** Whether to duplicate the template before appending: only a seed, only on request. */
export function duplicateFirst(template: Template | null, copyInstead: boolean): boolean {
  return copyInstead && template !== null && template.seed !== null;
}

/** The toast when the column was created but appending it to the template failed. */
export function columnPartialFailureMessage(name: string, detail: string): string {
  return `Kolonnen »${name}« er oprettet, men kunne ikke føjes til skabelonen: ${detail}`;
}
