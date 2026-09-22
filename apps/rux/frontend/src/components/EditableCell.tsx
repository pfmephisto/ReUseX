// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import type { PropertyDefinition } from '../api/types';
import styles from './EditableCell.module.css';

export interface EditableCellProps {
  value: string | undefined;
  colDef: PropertyDefinition;
  onSave: (value: string | null) => Promise<void>;
}

/**
 * One inline-editable property value.
 *
 * The behaviour splits on the property's type. `text`/`number`/`date` are
 * click-to-edit: the cell shows a value until clicked, then swaps to a focused
 * input that saves on Enter or blur and reverts on Escape. `boolean` and
 * `select` are always-visible controls that save immediately on change, because
 * a checkbox or a dropdown has nothing to "start editing".
 *
 * `value` is the source of truth. `draft` is local scratch that only exists
 * while editing; a `useEffect` re-reads `value` after every save so the cell
 * shows server truth the moment `onSave` resolves.
 */
export function EditableCell({ value, colDef, onSave }: EditableCellProps) {
  const [editing, setEditing] = useState(false);
  const [draft, setDraft] = useState(value ?? '');
  const inputRef = useRef<HTMLInputElement>(null);

  // Re-sync the draft to props whenever the stored value changes and we are not
  // mid-edit — this is what makes the cell controlled after an optimistic save.
  useEffect(() => {
    if (!editing) setDraft(value ?? '');
  }, [value, editing]);

  useEffect(() => {
    if (editing) inputRef.current?.focus();
  }, [editing]);

  const commit = async () => {
    setEditing(false);
    const trimmed = draft;
    const next = trimmed === '' ? null : trimmed;
    // Nothing changed — do not send an edit for a no-op.
    if ((value ?? '') === trimmed) return;
    await onSave(next);
  };

  const cancel = () => {
    setDraft(value ?? '');
    setEditing(false);
  };

  const onKeyDown = (event: React.KeyboardEvent<HTMLInputElement>) => {
    if (event.key === 'Enter') {
      event.preventDefault();
      void commit();
    } else if (event.key === 'Escape') {
      event.preventDefault();
      cancel();
    }
  };

  // ---- always-visible controls -------------------------------------------

  if (colDef.type === 'boolean') {
    return (
      <input
        type="checkbox"
        className={styles.checkbox}
        checked={value === 'true'}
        aria-label={colDef.name}
        onChange={(event) => void onSave(event.target.checked ? 'true' : 'false')}
      />
    );
  }

  if (colDef.type === 'select') {
    const options = colDef.options ?? [];
    return (
      <select
        className={styles.select}
        value={value ?? ''}
        aria-label={colDef.name}
        onChange={(event) => void onSave(event.target.value === '' ? null : event.target.value)}
      >
        <option value="">—</option>
        {options.map((opt) => (
          <option key={opt} value={opt}>
            {opt}
          </option>
        ))}
      </select>
    );
  }

  // ---- click-to-edit inputs ----------------------------------------------

  if (editing) {
    const inputType = colDef.type === 'number' ? 'number' : colDef.type === 'date' ? 'date' : 'text';
    return (
      <input
        ref={inputRef}
        className={styles.input}
        type={inputType}
        step={colDef.type === 'number' ? 'any' : undefined}
        value={draft}
        aria-label={colDef.name}
        onChange={(event) => setDraft(event.target.value)}
        onKeyDown={onKeyDown}
        onBlur={() => void commit()}
      />
    );
  }

  return (
    <button
      type="button"
      className={styles.display}
      onClick={() => setEditing(true)}
      title={value ?? ''}
    >
      {value !== undefined && value !== '' ? (
        <span className={styles.text}>{value}</span>
      ) : (
        <span className={styles.empty}>—</span>
      )}
    </button>
  );
}
