// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * One resource value: formatted text, or — when `editing` — the editor its
 * `data_type` asks for (spec §6.1): a text/number/date field that commits on
 * blur (`useTextDraft` + `cellDraft`), a select for an enum, a tri-state
 * select (— / Ja / Nej) for a clearable boolean and a checkbox for ★ Vigtig.
 * Blank shows a muted `—` (plan R2: only the selected row edits). A select or
 * checkbox shows its new value at once, while its PATCH is in flight
 * (`shownChoice`), and judges the next change against it.
 * A multiselect key has no editor yet and always shows its joined labels
 * (R3-A1); an enum key Phase 1 refuses to clear offers no blank option
 * (R3-D1, `NON_CLEARABLE`).
 *
 * The editor swallows click/double-click so a click in a field never selects,
 * folds or opens the row underneath; Esc reverts and parks focus on `home`
 * (the table), Enter commits and does the same.
 */

import { useEffect, useRef, useState, type RefObject } from 'react';

import type { EnvironmentStatus, ResourceKey, Treatment } from '../../api/types';
import { TREATMENTS } from '../../api/types';
import { fieldKeys, useTextDraft } from '../../app/useTextDraft';
import {
  BLANK,
  NON_CLEARABLE,
  booleanChoice,
  cellDisplay,
  choiceCommit,
  cellInputText,
  cellValidate,
  enumOptions,
  isTrue,
  isTriStateBoolean,
  optionLabel,
  shownChoice,
  toggleValue,
  type PendingChoice,
} from '../../kortlaegning/cellDraft';
import { ENV_TONE } from '../../kortlaegning/vocab';
import { Pill } from '../Pill';
import styles from './ResourceCell.module.css';

export interface ResourceCellProps {
  resourceKey: ResourceKey;
  value: string | null;
  editing: boolean;
  /** May return the write's promise; a select/checkbox shows its choice until it settles. */
  onCommit: (value: string | null) => void | Promise<void>;
  /** A blur found a draft that does not parse; the field has snapped back. */
  onInvalid: (label: string) => void;
  /** Where focus goes after Esc/Enter in a field. */
  home: RefObject<HTMLElement | null>;
  /** `table`: compact, in a row; `field`: full width, in the detail panel. */
  variant?: 'table' | 'field';
}

function isTreatment(v: string): v is Treatment {
  return (TREATMENTS as readonly string[]).includes(v);
}

function CellText({ resourceKey: key, value }: { resourceKey: ResourceKey; value: string | null }) {
  const text = cellDisplay(key, value);
  if (text === '') return <span className={styles.blank}>{BLANK}</span>;
  if (key.id === 'sys:environment' && value !== null && value in ENV_TONE) {
    return <Pill tone={ENV_TONE[value as EnvironmentStatus]}>{text}</Pill>;
  }
  if (key.id === 'sys:treatment' && value !== null && isTreatment(value)) {
    return <Pill treatment={value}>{text}</Pill>;
  }
  return (
    <span className={key.data_type === 'number' ? `${styles.text} mono` : styles.text} title={text}>
      {text}
    </span>
  );
}

function DraftInput({ resourceKey: key, value, onCommit, onInvalid, home }: ResourceCellProps) {
  const draft = useTextDraft<string | null>(cellInputText(key, value), onCommit, {
    validate: cellValidate(key),
    onInvalid: () => onInvalid(key.label),
  });
  const keys = fieldKeys(draft, home);
  return (
    <input
      type={key.data_type === 'date' ? 'date' : 'text'}
      inputMode={key.data_type === 'number' ? 'decimal' : undefined}
      className={key.data_type === 'number' ? `${styles.input} mono` : styles.input}
      aria-label={key.label}
      placeholder={BLANK}
      {...draft.props}
      onKeyDown={(e) => {
        keys(e);
        // Enter committed by blurring; give the table its keys back.
        if (e.key === 'Enter' && e.defaultPrevented) home.current?.focus({ preventScroll: true });
      }}
    />
  );
}

/**
 * A select/checkbox's choice while its write is in flight. Each choice gets a
 * token; only the latest one's settling clears it, so a quick second choice
 * is not cleared by the first one's response.
 */
function usePendingChoice(onCommit: ResourceCellProps['onCommit']) {
  const [pending, setPending] = useState<PendingChoice | undefined>(undefined);
  const token = useRef(0);
  const mounted = useRef(true);
  useEffect(
    () => () => {
      mounted.current = false;
    },
    [],
  );
  function choose(value: string | null) {
    const mine = ++token.current;
    setPending({ value });
    void Promise.resolve(onCommit(value)).finally(() => {
      if (mounted.current && token.current === mine) setPending(undefined);
    });
  }
  return { pending, choose };
}

export function ResourceCell(props: ResourceCellProps) {
  const { resourceKey: key, value, editing, variant = 'table' } = props;
  const { pending, choose } = usePendingChoice(props.onCommit);
  // R3-A1: a multiselect key is read-only in Phase 3, selected row or not.
  if (!editing || key.data_type === 'multiselect') return <CellText resourceKey={key} value={value} />;
  const shown = shownChoice(value, pending);

  let editor;
  if (isTriStateBoolean(key)) {
    editor = (
      <select
        className={styles.select}
        aria-label={key.label}
        value={booleanChoice(shown)}
        onChange={(e) => {
          const c = choiceCommit(key, e.target.value, value, pending);
          if (c.send) choose(c.value);
        }}
      >
        <option value="">{BLANK}</option>
        <option value="true">Ja</option>
        <option value="false">Nej</option>
      </select>
    );
  } else if (key.data_type === 'boolean') {
    // sys:starred: never blank (NON_CLEARABLE), so a plain checkbox.
    editor = (
      <input
        type="checkbox"
        className={styles.checkbox}
        aria-label={key.label}
        checked={isTrue(shown)}
        onChange={() => choose(toggleValue(shown))}
      />
    );
  } else if (key.data_type === 'enum') {
    const blank = shown === null || shown === '';
    const clearable = !NON_CLEARABLE.has(key.id);
    editor = (
      <select
        className={styles.select}
        aria-label={key.label}
        value={shown ?? ''}
        onChange={(e) => {
          const c = choiceCommit(key, e.target.value, value, pending);
          if (c.send) choose(c.value);
        }}
      >
        {/* R3-D1: a key that cannot be cleared offers no blank choice; a
            blank stored value still shows as blank instead of option 1. */}
        {(clearable || blank) && (
          <option value="" disabled={!clearable}>
            {BLANK}
          </option>
        )}
        {enumOptions(key, shown).map((o) => (
          <option key={o} value={o}>
            {optionLabel(key, o)}
          </option>
        ))}
      </select>
    );
  } else {
    editor = <DraftInput {...props} />;
  }

  return (
    <div
      className={styles.editor}
      data-variant={variant}
      onClick={(e) => e.stopPropagation()}
      onDoubleClick={(e) => e.stopPropagation()}
    >
      {editor}
    </div>
  );
}
