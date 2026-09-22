// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useLayoutEffect, useRef, useState } from 'react';

import type { PropertyDefinition } from '../api/types';
import styles from './SelectDropdown.module.css';

export interface SelectDropdownProps {
  value: string | undefined;
  colDef: PropertyDefinition;
  onSave: (value: string | null) => Promise<void>;
  /** Add the option to the column definition (calls `api.updatePropertyDefinition`). */
  onAddOption: (option: string) => Promise<void>;
  onClose: () => void;
  /** The panel positions itself below this element. */
  anchorRef: React.RefObject<HTMLElement | null>;
}

/**
 * Deterministic dark-theme chip palette. `hash(text) % CHIP_COLORS.length`
 * picks one, so a given option always draws with the same colour across every
 * cell — Notion's "coloured tag" affordance without a per-option colour store.
 */
export const CHIP_COLORS = [
  { bg: 'rgba(35, 131, 226, 0.15)', text: '#5bb0f0' }, // blue
  { bg: 'rgba(180, 65, 60, 0.15)', text: '#e07070' }, // red
  { bg: 'rgba(68, 131, 97, 0.15)', text: '#5cb87b' }, // green
  { bg: 'rgba(155, 93, 168, 0.15)', text: '#c07de0' }, // purple
  { bg: 'rgba(183, 133, 38, 0.15)', text: '#d4a843' }, // yellow
  { bg: 'rgba(90, 90, 90, 0.25)', text: '#b0b0b0' }, // gray
] as const;

/** Stable string hash (djb2), non-negative. */
export function chipColorFor(text: string): (typeof CHIP_COLORS)[number] {
  let hash = 5381;
  for (let i = 0; i < text.length; i += 1) {
    hash = (hash * 33) ^ text.charCodeAt(i);
  }
  return CHIP_COLORS[Math.abs(hash) % CHIP_COLORS.length];
}

/**
 * A Notion-style tag picker for a `select` column (spec §8.3).
 *
 * Two stacked sections. The top *value strip* shows the currently-selected
 * value as a removable pill (its `×` clears the value) sitting alongside a
 * filter input; the bottom *option list* draws one coloured-chip row per
 * matching option under a faint "SELECT AN OPTION OR CREATE ONE" header. When
 * the typed text matches none of the options a `Create "…"` row appears at the
 * top of the list, persisting the new option to the column definition before
 * storing it on the row. Keyboard: arrows move the highlight, Enter picks the
 * highlighted row (or creates), Escape closes without saving. The panel is
 * absolutely positioned below its anchor cell.
 */
export function SelectDropdown({
  value,
  colDef,
  onSave,
  onAddOption,
  onClose,
  anchorRef,
}: SelectDropdownProps) {
  const panelRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);
  const [query, setQuery] = useState('');
  const [highlight, setHighlight] = useState(0);
  const [pos, setPos] = useState<{ top: number; left: number; width: number } | null>(null);

  const options = colDef.options ?? [];
  const filtered = query
    ? options.filter((opt) => opt.toLowerCase().includes(query.toLowerCase()))
    : options;
  const exactMatch = options.some((opt) => opt.toLowerCase() === query.trim().toLowerCase());
  const canCreate = query.trim() !== '' && !exactMatch;

  // The list rows, in the same order the highlight indexes them: the optional
  // Create row is index 0, then the filtered options.
  const rowCount = (canCreate ? 1 : 0) + filtered.length;

  // Position the panel below the anchor, clamped into the viewport. Absolute
  // (not fixed) so it scrolls with the cell; done in a layout effect so the
  // first paint is already in the right place. Width: max(cell width, 240px).
  useLayoutEffect(() => {
    const anchor = anchorRef.current;
    if (!anchor) return;
    const rect = anchor.getBoundingClientRect();
    const width = Math.max(rect.width, 240);
    const left = Math.min(rect.left + window.scrollX, window.scrollX + window.innerWidth - width - 8);
    const top = rect.bottom + window.scrollY + 2;
    setPos({ top, left: Math.max(8, left), width });
  }, [anchorRef]);

  useEffect(() => {
    inputRef.current?.focus();
  }, []);

  // Close on any click outside the panel.
  useEffect(() => {
    const handler = (event: MouseEvent) => {
      if (panelRef.current && !panelRef.current.contains(event.target as Node)) {
        onClose();
      }
    };
    document.addEventListener('mousedown', handler);
    return () => document.removeEventListener('mousedown', handler);
  }, [onClose]);

  const select = async (option: string) => {
    if (option !== value) await onSave(option);
    onClose();
  };

  const clear = async () => {
    await onSave(null);
    onClose();
  };

  const create = async () => {
    const text = query.trim();
    if (text === '') return;
    await onAddOption(text);
    await onSave(text);
    onClose();
  };

  // Resolve the currently-highlighted list row and act on it.
  const activate = () => {
    if (canCreate && highlight === 0) {
      void create();
      return;
    }
    const optIndex = canCreate ? highlight - 1 : highlight;
    const opt = filtered[optIndex];
    if (opt !== undefined) void select(opt);
  };

  const onKeyDown = (event: React.KeyboardEvent) => {
    if (event.key === 'Escape') {
      event.preventDefault();
      onClose();
    } else if (event.key === 'ArrowDown') {
      event.preventDefault();
      setHighlight((h) => Math.min(h + 1, Math.max(0, rowCount - 1)));
    } else if (event.key === 'ArrowUp') {
      event.preventDefault();
      setHighlight((h) => Math.max(0, h - 1));
    } else if (event.key === 'Enter') {
      event.preventDefault();
      activate();
    }
  };

  const selectedChip = value !== undefined && value !== '' ? chipColorFor(value) : null;

  return (
    <div
      ref={panelRef}
      className={styles.panel}
      style={pos ? { top: pos.top, left: pos.left, width: pos.width } : { visibility: 'hidden' }}
      onKeyDown={onKeyDown}
      role="listbox"
      aria-label={colDef.name}
    >
      {/* Section 1 — value strip: selected pill (removable) + filter input. */}
      <div className={styles.valueStrip}>
        {selectedChip && (
          <span className={styles.pill} style={{ background: selectedChip.bg, color: selectedChip.text }}>
            <span className={styles.pillText}>{value}</span>
            <button
              type="button"
              className={styles.pillRemove}
              aria-label="Clear selection"
              onMouseDown={(event) => event.preventDefault()}
              onClick={() => void clear()}
            >
              ×
            </button>
          </span>
        )}
        <input
          ref={inputRef}
          className={styles.search}
          value={query}
          placeholder="Search options…"
          onChange={(event) => {
            setQuery(event.target.value);
            setHighlight(0);
          }}
          aria-label="Filter or create option"
        />
      </div>

      {/* Section 2 — option list. */}
      <div className={styles.listHeader}>SELECT AN OPTION OR CREATE ONE</div>
      <div className={styles.options}>
        {canCreate && (
          <button
            type="button"
            className={`${styles.optionRow} ${highlight === 0 ? styles.highlighted : ''}`}
            onMouseEnter={() => setHighlight(0)}
            onClick={() => void create()}
          >
            <span className={styles.createLabel}>Create</span>
            <span
              className={styles.chip}
              style={{
                background: chipColorFor(query.trim()).bg,
                color: chipColorFor(query.trim()).text,
              }}
            >
              {query.trim()}
            </span>
          </button>
        )}
        {filtered.map((opt, index) => {
          const rowIndex = canCreate ? index + 1 : index;
          const color = chipColorFor(opt);
          return (
            <button
              key={opt}
              type="button"
              role="option"
              aria-selected={opt === value}
              className={`${styles.optionRow} ${rowIndex === highlight ? styles.highlighted : ''}`}
              onMouseEnter={() => setHighlight(rowIndex)}
              onClick={() => void select(opt)}
            >
              <span className={styles.chip} style={{ background: color.bg, color: color.text }}>
                {opt}
              </span>
            </button>
          );
        })}
        {filtered.length === 0 && !canCreate && (
          <div className={styles.emptyNote}>No options</div>
        )}
      </div>
    </div>
  );
}
