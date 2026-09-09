// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { StageParameter } from '../api/types';

/**
 * Form state for one stage's parameters.
 *
 * Text-typed fields are held as the raw string the user typed, not as a parsed
 * number, so a half-typed `-` or `0.` survives a re-render instead of snapping
 * back to the last valid value. Booleans are held as booleans because a
 * checkbox has no intermediate state to preserve.
 */
export type FieldValue = string | boolean;
export type FormState = Record<string, FieldValue>;

/** A parsed field, or why it could not be parsed. */
export type ParseResult =
  | { ok: true; value: number | boolean | string | number[] | undefined }
  | { ok: false; error: string };

/** Render a descriptor's default into the string a text input starts with. */
function defaultText(parameter: StageParameter): string {
  // A null default means "absent by default, with no neutral value". Inventing
  // a 0 or an empty-but-meaningful value here would change what the stage does.
  if (parameter.default === null) return '';
  return String(parameter.default);
}

/** Initial form state for a stage: every field showing the library default. */
export function initialFormState(parameters: StageParameter[]): FormState {
  const state: FormState = {};
  for (const parameter of parameters) {
    state[parameter.key] =
      parameter.type === 'boolean' ? parameter.default === true : defaultText(parameter);
  }
  return state;
}

/** True when the field still holds exactly what the server said it defaults to. */
export function isDefault(parameter: StageParameter, value: FieldValue): boolean {
  if (parameter.type === 'boolean') return value === (parameter.default === true);
  if (typeof value !== 'string') return false;

  const text = value.trim();
  const asDefault = defaultText(parameter);
  if (text === asDefault) return true;

  // `0.05` and `0.050` are the same default typed differently, and a form that
  // called the second one an edit would send a presence-sensitive key the user
  // never meant to pin.
  if (parameter.type === 'number' || parameter.type === 'integer') {
    if (text === '' || asDefault === '') return text === asDefault;
    const typed = Number(text);
    const original = Number(asDefault);
    return Number.isFinite(typed) && Number.isFinite(original) && typed === original;
  }
  return false;
}

/**
 * Parse one field.
 *
 * `undefined` means "leave this key out of the request entirely" — an empty
 * optional field, not a zero.
 */
export function parseField(parameter: StageParameter, value: FieldValue): ParseResult {
  if (parameter.type === 'boolean') return { ok: true, value: value === true };

  const text = typeof value === 'string' ? value.trim() : String(value);

  if (text === '') {
    // A required field cleared to blank is not the same as an optional one
    // left blank: the first would silently fall back to a default the user
    // just deleted, so it is refused instead.
    if (parameter.default === null) return { ok: true, value: undefined };
    return { ok: false, error: 'Cannot be empty — clear the whole form to use defaults' };
  }

  if (parameter.type === 'string') return { ok: true, value: text };

  if (parameter.type === 'integer_list') {
    const parts = text
      .split(',')
      .map((part) => part.trim())
      .filter((part) => part.length > 0);
    const values: number[] = [];
    for (const part of parts) {
      const parsed = Number(part);
      if (!Number.isInteger(parsed) || parsed < 0)
        return { ok: false, error: `"${part}" is not a non-negative whole number` };
      values.push(parsed);
    }
    return values.length === 0 ? { ok: true, value: undefined } : { ok: true, value: values };
  }

  const parsed = Number(text);
  if (!Number.isFinite(parsed)) return { ok: false, error: 'Not a number' };
  if (parameter.type === 'integer' && !Number.isInteger(parsed))
    return { ok: false, error: 'Must be a whole number' };
  if (parameter.minimum !== null && parsed < parameter.minimum)
    return { ok: false, error: `Must be at least ${parameter.minimum}` };
  if (parameter.maximum !== null && parsed > parameter.maximum)
    return { ok: false, error: `Must be at most ${parameter.maximum}` };

  return { ok: true, value: parsed };
}

/** Per-field errors, keyed by parameter. Empty when the form is submittable. */
export function formErrors(
  parameters: StageParameter[],
  state: FormState,
): Record<string, string> {
  const errors: Record<string, string> = {};
  for (const parameter of parameters) {
    const result = parseField(parameter, state[parameter.key] ?? '');
    if (!result.ok) errors[parameter.key] = result.error;
  }
  return errors;
}

/**
 * Build the `parameters` object for `POST /jobs`.
 *
 * Untouched fields are OMITTED rather than echoed back at their default. That
 * is not an optimisation — for `planes.plane_dist_threshold` and
 * `planes.min_inliers` the server treats the mere PRESENCE of the key as "pin
 * this, stop deriving it from measured noise" (#214). A form that helpfully
 * sent every field at its default would silently switch adaptive thresholding
 * off for every run started from the GUI, and the CLI and the GUI would
 * quietly disagree about what "default" means.
 *
 * Omitting the rest as well keeps `pipeline_log.parameters` a record of what
 * the user actually chose, which is what makes the history worth reading.
 *
 * @throws never — callers must check `formErrors` first; an unparseable field
 *         is skipped rather than sent as garbage.
 */
export function buildParameters(
  parameters: StageParameter[],
  state: FormState,
): Record<string, unknown> {
  const body: Record<string, unknown> = {};
  for (const parameter of parameters) {
    const value = state[parameter.key] ?? '';
    if (isDefault(parameter, value)) continue;

    const result = parseField(parameter, value);
    if (!result.ok || result.value === undefined) continue;
    body[parameter.key] = result.value;
  }
  return body;
}

/** How many fields differ from the library default. Drives the form's badge. */
export function changedCount(parameters: StageParameter[], state: FormState): number {
  return parameters.filter(
    (parameter) => !isDefault(parameter, state[parameter.key] ?? ''),
  ).length;
}
