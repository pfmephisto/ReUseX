// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * What a commit-on-blur text field sends when it loses focus. Pure, so the
 * "an untouched blur never commits" rule is testable without a DOM; the
 * `useTextDraft` hook is the only caller.
 */

/**
 * The value a text draft commits on blur, or null to send nothing: unchanged
 * after trimming (an untouched blur), or emptied when the field is required.
 */
export function textCommit(draft: string, current: string, required: boolean): string | null {
  const value = draft.trim();
  if (value === current.trim()) return null;
  if (required && value === '') return null;
  return value;
}

/** Parses a draft into the value to send, or null when the draft is invalid. */
export type DraftValidate<T> = (draft: string) => { value: T } | null;

export type DraftCommit<T> = { send: true; value: T } | { send: false; invalid: boolean };

/**
 * A validated field's blur. Text unchanged from `current` (after trimming)
 * sends nothing and is never invalid, even when `current` itself would not
 * parse; an invalid draft sends nothing and says so; a draft that parses to
 * the same value as `current` (e.g. `0` for an empty year) sends nothing.
 */
export function draftCommit<T>(draft: string, current: string, validate: DraftValidate<T>): DraftCommit<T> {
  if (draft.trim() === current.trim()) return { send: false, invalid: false };
  const parsed = validate(draft);
  if (parsed === null) return { send: false, invalid: true };
  const cur = validate(current);
  if (cur !== null && Object.is(cur.value, parsed.value)) return { send: false, invalid: false };
  return { send: true, value: parsed.value };
}
