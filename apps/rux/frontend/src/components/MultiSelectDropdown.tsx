// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useLayoutEffect, useRef, useState } from 'react';

import type { PropertyDefinition } from '../api/types';
import { chipColorFor } from './SelectDropdown';
import styles from './MultiSelectDropdown.module.css';

export interface MultiSelectDropdownProps {
  /** Current selected values. */
  values: string[];
  colDef: PropertyDefinition;
  onSave: (values: string[]) => void;
  /** Add the option to the column definition (calls `api.updatePropertyDefinition`). */
  onAddOption: (option: string) => Promise<void>;
  onClose: () => void;
  /** The panel positions itself below this element. */
  anchorRef: React.RefObject<HTMLElement | null>;
}

/**
 * A Notion-style multi-value tag picker for a `multiselect` column.
 *
 * Nearly identical to `SelectDropdown`, but the value strip shows several
 * removable pills (one per selected value) and picking an option TOGGLES it in
 * or out of the selected set while KEEPING the panel open — so several values
 * can be picked in one session. Already-selected options draw a leading ✓ in
 * the list. Escape or a click outside saves the current set and closes;
 * Backspace on an empty filter removes the last selected value.
 */
export function MultiSelectDropdown({
  values,
  colDef,
  onSave,
  onAddOption,
  onClose,
  anchorRef,
}: MultiSelectDropdownProps) {
  const [filter, setFilter] = useState('');
  const [selected, setSelected] = useState<string[]>(values);
  const [highlighted, setHighlighted] = useState(0);
  const panelRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Position below the anchor cell.
  const [pos, setPos] = useState({ top: 0, left: 0, minWidth: 240 });
  useLayoutEffect(() => {
    if (!anchorRef.current) return;
    const r = anchorRef.current.getBoundingClientRect();
    setPos({
      top: r.bottom + window.scrollY,
      left: r.left + window.scrollX,
      minWidth: Math.max(r.width, 240),
    });
  }, [anchorRef]);

  // Close on outside click, saving the current set.
  useEffect(() => {
    const handler = (e: MouseEvent) => {
      if (panelRef.current && !panelRef.current.contains(e.target as Node)) {
        onSave(selected);
        onClose();
      }
    };
    document.addEventListener('mousedown', handler);
    return () => document.removeEventListener('mousedown', handler);
  }, [selected, onSave, onClose]);

  // Escape saves and closes.
  useEffect(() => {
    const handler = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onSave(selected);
        onClose();
      }
    };
    document.addEventListener('keydown', handler);
    return () => document.removeEventListener('keydown', handler);
  }, [selected, onSave, onClose]);

  useEffect(() => {
    inputRef.current?.focus();
  }, []);

  const options = colDef.options ?? [];
  const filtered = options.filter((o) => o.toLowerCase().includes(filter.toLowerCase()));
  const showCreate =
    filter.trim() !== '' && !options.some((o) => o.toLowerCase() === filter.trim().toLowerCase());
  const allRows: Array<{ type: 'create' | 'option'; label: string }> = showCreate
    ? [{ type: 'create', label: filter.trim() }, ...filtered.map((o) => ({ type: 'option' as const, label: o }))]
    : filtered.map((o) => ({ type: 'option' as const, label: o }));

  const toggle = (val: string) => {
    setSelected((prev) => (prev.includes(val) ? prev.filter((v) => v !== val) : [...prev, val]));
    setFilter('');
  };

  const activate = (idx: number) => {
    const row = allRows[idx];
    if (!row) return;
    if (row.type === 'create') {
      void onAddOption(row.label).then(() => toggle(row.label));
    } else {
      toggle(row.label);
    }
    setHighlighted(0);
  };

  return (
    <div
      ref={panelRef}
      className={styles.panel}
      style={{ top: pos.top, left: pos.left, minWidth: pos.minWidth }}
      role="listbox"
      aria-label={colDef.name}
    >
      {/* Value strip: one removable pill per selected value + filter input. */}
      <div className={styles.valueStrip}>
        {selected.map((v) => {
          const c = chipColorFor(v);
          return (
            <span key={v} className={styles.pill} style={{ background: c.bg, color: c.text }}>
              <span className={styles.pillText}>{v}</span>
              <button
                type="button"
                className={styles.pillRemove}
                aria-label={`Remove ${v}`}
                onMouseDown={(e) => e.preventDefault()}
                onClick={() => toggle(v)}
              >
                ×
              </button>
            </span>
          );
        })}
        <input
          ref={inputRef}
          className={styles.search}
          placeholder={selected.length === 0 ? 'Search options…' : ''}
          value={filter}
          onChange={(e) => {
            setFilter(e.target.value);
            setHighlighted(0);
          }}
          aria-label="Filter or create option"
          onKeyDown={(e) => {
            e.stopPropagation();
            if (e.key === 'ArrowDown') {
              e.preventDefault();
              setHighlighted((h) => Math.min(h + 1, allRows.length - 1));
            } else if (e.key === 'ArrowUp') {
              e.preventDefault();
              setHighlighted((h) => Math.max(h - 1, 0));
            } else if (e.key === 'Enter') {
              e.preventDefault();
              activate(highlighted);
            } else if (e.key === 'Backspace' && !filter && selected.length > 0) {
              toggle(selected[selected.length - 1]);
            }
          }}
        />
      </div>

      {/* Option list. */}
      <div className={styles.listHeader}>SELECT OPTIONS</div>
      <div className={styles.options}>
        {allRows.map((row, i) => (
          <button
            key={`${row.type}:${row.label}`}
            type="button"
            className={`${styles.optionRow} ${i === highlighted ? styles.highlighted : ''}`}
            onMouseEnter={() => setHighlighted(i)}
            onClick={() => activate(i)}
          >
            {row.type === 'create' ? (
              <span className={styles.createLabel}>
                Create &quot;<span className={styles.createText}>{row.label}</span>&quot;
              </span>
            ) : (
              <>
                {selected.includes(row.label) && <span className={styles.check}>✓</span>}
                <span
                  className={styles.chip}
                  style={{
                    background: chipColorFor(row.label).bg,
                    color: chipColorFor(row.label).text,
                    marginLeft: selected.includes(row.label) ? 0 : '1.2em',
                  }}
                >
                  {row.label}
                </span>
              </>
            )}
          </button>
        ))}
        {allRows.length === 0 && <div className={styles.emptyNote}>No matching options</div>}
      </div>
    </div>
  );
}
