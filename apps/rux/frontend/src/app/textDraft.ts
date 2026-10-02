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

/**
 * Parses a draft into the value to send, or null when the draft is invalid.
 * T must be a primitive (string, number, null, …): `draftCommit` compares the
 * parsed draft with the parsed stored value by `Object.is`, so two equal
 * objects would always count as a change.
 */
export type DraftValidate<T> = (draft: string) => { value: T } | null;

export type DraftCommit<T> = { send: true; value: T } | { send: false; invalid: boolean };

/**
 * A plain text field's blur (`textCommit`) as a `DraftCommit`: emptying a
 * required field is `invalid`, so the caller can say why it snapped back.
 */
export function textDraftCommit(draft: string, current: string, required: boolean): DraftCommit<string> {
  const value = textCommit(draft, current, required);
  if (value !== null) return { send: true, value };
  return { send: false, invalid: required && draft.trim() === '' && current.trim() !== '' };
}

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

/** How a field decides: plain text (`required` flag) or a validated draft. */
export type DraftMode<T> = boolean | { required?: boolean } | { validate: DraftValidate<T> };

/** A field's commit decision under `mode` — the one `useTextDraft` runs on blur. */
export function decideDraft<T>(draft: string, current: string, mode: DraftMode<T> = false): DraftCommit<T> {
  if (typeof mode === 'object' && 'validate' in mode) return draftCommit(draft, current, mode.validate);
  const required = typeof mode === 'object' ? mode.required === true : mode;
  // Plain text: T is string (useTextDraft's overloads guarantee it).
  return textDraftCommit(draft, current, required) as DraftCommit<T>;
}

/**
 * The commit owed when a field unmounts (a template switch, a row that
 * re-renders away) without a blur: a focused field's dirty draft would
 * otherwise be dropped. Nothing is owed when the blur already ran (not
 * focused) or Esc reverted the field.
 */
export function leaveCommit<T = string>(
  state: { focused: boolean; reverted: boolean; draft: string; current: string },
  mode: DraftMode<T> = false,
): DraftCommit<T> {
  if (!state.focused || state.reverted) return { send: false, invalid: false };
  return decideDraft(state.draft, state.current, mode);
}
