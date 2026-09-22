// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import type { PropertyDefinition } from '../api/types';
import { chipColorFor, SelectDropdown } from './SelectDropdown';
import { useTableNav } from './tableNav';
import styles from './EditableCell.module.css';

export interface EditableCellProps {
  value: string | undefined;
  colDef: PropertyDefinition;
  onSave: (value: string | null) => Promise<void>;
  /** Add an option to this column's definition (for the `select` picker). */
  onAddOption: (option: string) => Promise<void>;
  /** Row index in the data grid. */
  rowIndex: number;
  /** Data-column index (thumbnail / add-column columns excluded). */
  colIndex: number;
}

/**
 * One inline-editable property value, Notion's single-click-to-edit model.
 *
 * Read mode shows the value as plain text (or a coloured chip for `select`) and
 * is keyboard-reachable (`tabIndex=0`). The cell enters edit mode on a single
 * click, on `Enter`/`F2`, or when a printable character is typed while it is
 * focused — the last case seeding the input with that character. `boolean` has
 * no edit mode: a single click or `Enter` toggles it in place.
 *
 * Focus/edit ownership lives in `MaterialTable` via `TableNavContext`; this cell
 * reads `{focusedCell, editingCell}` to decide how to render and reports focus
 * back up. `value` is the source of truth; `draft` is scratch that exists only
 * while editing and is re-synced from `value` after each save.
 */
export function EditableCell({
  value,
  colDef,
  onSave,
  onAddOption,
  rowIndex,
  colIndex,
}: EditableCellProps) {
  const nav = useTableNav();
  const isFocused = nav.focusedCell?.row === rowIndex && nav.focusedCell.col === colIndex;
  const isEditing = nav.editingCell?.row === rowIndex && nav.editingCell.col === colIndex;

  const [draft, setDraft] = useState(value ?? '');
  const cellRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Re-sync the draft from props whenever the stored value changes and we are
  // not mid-edit — this is what makes the cell controlled after a save.
  useEffect(() => {
    if (!isEditing) setDraft(value ?? '');
  }, [value, isEditing]);

  // Entering edit mode: seed from a printable keypress if one was stashed, then
  // focus the input and put the caret at the end.
  useEffect(() => {
    if (!isEditing) return;
    const seed = nav.takeSeed();
    if (seed !== null) setDraft(seed);
    const input = inputRef.current;
    if (input) {
      input.focus();
      const end = input.value.length;
      input.setSelectionRange?.(end, end);
    }
    // Only run when edit mode turns on for this cell.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [isEditing]);

  // Keep the DOM focus on the cell div while it is the focused, non-editing
  // cell — so arrow keys keep flowing to the table handler.
  useEffect(() => {
    if (isFocused && !isEditing) cellRef.current?.focus();
  }, [isFocused, isEditing]);

  const commit = async () => {
    const trimmed = draft;
    const next = trimmed === '' ? null : trimmed;
    nav.exitEdit(true);
    if ((value ?? '') === trimmed) return; // no-op
    await onSave(next);
  };

  const cancel = () => {
    setDraft(value ?? '');
    nav.exitEdit(false);
  };

  // ---- boolean: no edit mode, toggles in place ---------------------------

  if (colDef.type === 'boolean') {
    const checked = value === 'true';
    const toggle = () => void onSave(checked ? 'false' : 'true');
    return (
      <div
        ref={cellRef}
        tabIndex={0}
        className={`${styles.cell} ${isFocused ? styles.focused : ''}`}
        onFocus={() => nav.setFocused(rowIndex, colIndex)}
        onClick={toggle}
        onKeyDown={(event) => {
          if (event.key === 'Enter' || event.key === 'F2' || event.key === ' ') {
            event.preventDefault();
            toggle();
          }
        }}
      >
        <input
          type="checkbox"
          className={styles.checkbox}
          checked={checked}
          aria-label={colDef.name}
          tabIndex={-1}
          onChange={toggle}
        />
      </div>
    );
  }

  // ---- select: custom dropdown, chip in read mode ------------------------

  if (colDef.type === 'select') {
    const chip =
      value !== undefined && value !== '' ? chipColorFor(value) : null;
    return (
      <div
        ref={cellRef}
        tabIndex={0}
        className={`${styles.cell} ${isFocused ? styles.focused : ''}`}
        onFocus={() => nav.setFocused(rowIndex, colIndex)}
        onClick={() => nav.startEdit(rowIndex, colIndex)}
        onKeyDown={(event) => {
          if (event.key === 'Enter' || event.key === 'F2') {
            event.preventDefault();
            nav.startEdit(rowIndex, colIndex);
          }
        }}
      >
        {chip ? (
          <span className={styles.chip} style={{ background: chip.bg, color: chip.text }}>
            {value}
          </span>
        ) : (
          <span className={styles.empty}>—</span>
        )}
        {isEditing && (
          <SelectDropdown
            value={value}
            colDef={colDef}
            onSave={onSave}
            onAddOption={onAddOption}
            onClose={() => nav.exitEdit(false)}
            anchorRef={cellRef}
          />
        )}
      </div>
    );
  }

  // ---- text / number / date: double-click to edit ------------------------

  const inputType =
    colDef.type === 'number' ? 'number' : colDef.type === 'date' ? 'date' : 'text';

  return (
    <div
      ref={cellRef}
      tabIndex={isEditing ? -1 : 0}
      className={`${styles.cell} ${isFocused ? styles.focused : ''}`}
      onFocus={() => nav.setFocused(rowIndex, colIndex)}
      onClick={() => {
        if (!isEditing) nav.startEdit(rowIndex, colIndex);
      }}
      onKeyDown={(event) => {
        if (isEditing) return; // input owns its own keys
        if (event.key === 'Enter' || event.key === 'F2') {
          event.preventDefault();
          nav.startEdit(rowIndex, colIndex);
        }
      }}
      title={!isEditing ? (value ?? '') : undefined}
    >
      {isEditing ? (
        <input
          ref={inputRef}
          className={styles.input}
          type={inputType}
          step={colDef.type === 'number' ? 'any' : undefined}
          value={draft}
          aria-label={colDef.name}
          onChange={(event) => setDraft(event.target.value)}
          onKeyDown={(event) => {
            if (event.key === 'Enter') {
              event.preventDefault();
              void commit();
            } else if (event.key === 'Escape') {
              event.preventDefault();
              cancel();
            }
            // Tab is handled by the table wrapper (save + move); let it bubble.
          }}
          onBlur={() => void commit()}
        />
      ) : value !== undefined && value !== '' ? (
        <span className={styles.text}>{value}</span>
      ) : (
        <span className={styles.empty}>—</span>
      )}
    </div>
  );
}
