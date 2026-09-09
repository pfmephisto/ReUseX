// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useId } from 'react';

import type { StageParameter } from '../api/types';
import { isDefault, type FormState } from '../pipeline/params';
import styles from './ParameterForm.module.css';

export interface ParameterFormProps {
  parameters: StageParameter[];
  state: FormState;
  errors: Record<string, string>;
  disabled?: boolean;
  onChange: (key: string, value: string | boolean) => void;
  onReset: () => void;
}

function placeholderFor(parameter: StageParameter): string | undefined {
  if (parameter.default !== null) return undefined;
  // The optional fields are the ones with no neutral value; say what leaving
  // them blank means rather than leaving an unexplained empty box.
  if (parameter.type === 'integer_list') return 'all labels';
  return 'whole cloud';
}

function rangeHint(parameter: StageParameter): string | null {
  if (parameter.minimum === null && parameter.maximum === null) return null;
  if (parameter.minimum !== null && parameter.maximum !== null)
    return `${parameter.minimum} – ${parameter.maximum}`;
  return parameter.minimum !== null ? `≥ ${parameter.minimum}` : `≤ ${parameter.maximum}`;
}

/**
 * The parameter form of one stage, built entirely from the server's
 * descriptors.
 *
 * Nothing about any individual knob is known here: no key, no default, no
 * bound. That is the point — the defaults live in the library option structs,
 * the server reads them off those structs, and this renders whatever it is
 * handed (STANDARDS §4). A knob added to `SegmentPlanesOptions` appears here
 * with no frontend change; one removed disappears, rather than lingering as a
 * field that silently does nothing.
 */
export function ParameterForm({
  parameters,
  state,
  errors,
  disabled,
  onChange,
  onReset,
}: ParameterFormProps) {
  const formId = useId();
  if (parameters.length === 0) return null;

  const changed = parameters.filter((p) => !isDefault(p, state[p.key] ?? '')).length;

  return (
    <div className={styles.form}>
      <div className={styles.formHead}>
        <span className={styles.formTitle}>Parameters</span>
        <span className={styles.changed}>
          {changed === 0 ? 'all defaults' : `${changed} changed`}
        </span>
        <button
          type="button"
          className={styles.reset}
          onClick={onReset}
          disabled={disabled || changed === 0}
        >
          Reset
        </button>
      </div>

      <div className={styles.grid}>
        {parameters.map((parameter) => {
          const id = `${formId}-${parameter.key}`;
          const error = errors[parameter.key];
          const range = rangeHint(parameter);
          const value = state[parameter.key];
          const pinned = parameter.presence_sensitive && !isDefault(parameter, value ?? '');

          return (
            <div key={parameter.key} className={styles.field}>
              <label className={styles.label} htmlFor={id}>
                {parameter.label}
                <span className={`${styles.key} mono`}>{parameter.key}</span>
              </label>

              {parameter.type === 'boolean' ? (
                <input
                  id={id}
                  className={styles.checkbox}
                  type="checkbox"
                  checked={value === true}
                  disabled={disabled}
                  onChange={(event) => onChange(parameter.key, event.target.checked)}
                  aria-describedby={`${id}-help`}
                />
              ) : (
                <input
                  id={id}
                  className={`${styles.input} ${error ? styles.invalid : ''} mono`}
                  type="text"
                  inputMode={
                    parameter.type === 'number' || parameter.type === 'integer'
                      ? 'decimal'
                      : 'text'
                  }
                  value={typeof value === 'string' ? value : ''}
                  placeholder={placeholderFor(parameter)}
                  disabled={disabled}
                  aria-invalid={error ? true : undefined}
                  aria-describedby={`${id}-help`}
                  onChange={(event) => onChange(parameter.key, event.target.value)}
                />
              )}

              <p className={styles.help} id={`${id}-help`}>
                {parameter.description}
                {range && <span className={`${styles.range} mono`}> {range}</span>}
              </p>

              {/* #214: the user needs to know that touching this field is not
                  just a value change — it switches adaptive derivation off. */}
              {pinned && (
                <p className={styles.pinned}>
                  Pinned — adaptive derivation is off for this parameter.
                </p>
              )}

              {error && <p className={styles.error}>{error}</p>}
            </div>
          );
        })}
      </div>
    </div>
  );
}
