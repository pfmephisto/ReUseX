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
 * A Notion-style tag picker for a `select` column.
 *
 * Opens as an absolutely-positioned panel anchored below the cell. The top
 * search input both filters the existing options and, when the typed text
 * matches none of them, offers to create a new one — which persists to the
 * column definition before the value is stored on the row. Keyboard: arrows
 * move the highlight, Enter picks the highlighted option (or creates), Escape
 * closes without saving.
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
  const [pos, setPos] = useState<{ top: number; left: number } | null>(null);

  const options = colDef.options ?? [];
  const filtered = query
    ? options.filter((opt) => opt.toLowerCase().includes(query.toLowerCase()))
    : options;
  const exactMatch = options.some((opt) => opt.toLowerCase() === query.trim().toLowerCase());
  const canCreate = query.trim() !== '' && !exactMatch;

  // Position the panel below the anchor, clamped into the viewport. Done in a
  // layout effect so the first paint is already in the right place.
  useLayoutEffect(() => {
    const anchor = anchorRef.current;
    if (!anchor) return;
    const rect = anchor.getBoundingClientRect();
    const panelWidth = Math.max(rect.width, 180);
    const left = Math.min(rect.left, window.innerWidth - panelWidth - 8);
    const top = Math.min(rect.bottom + 2, window.innerHeight - 8);
    setPos({ top, left: Math.max(8, left) });
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

  const create = async () => {
    const text = query.trim();
    if (text === '') return;
    await onAddOption(text);
    await onSave(text);
    onClose();
  };

  const onKeyDown = (event: React.KeyboardEvent) => {
    if (event.key === 'Escape') {
      event.preventDefault();
      onClose();
    } else if (event.key === 'ArrowDown') {
      event.preventDefault();
      setHighlight((h) => Math.min(h + 1, Math.max(0, filtered.length - 1)));
    } else if (event.key === 'ArrowUp') {
      event.preventDefault();
      setHighlight((h) => Math.max(0, h - 1));
    } else if (event.key === 'Enter') {
      event.preventDefault();
      if (filtered.length > 0 && filtered[highlight]) {
        void select(filtered[highlight]);
      } else if (canCreate) {
        void create();
      }
    }
  };

  return (
    <div
      ref={panelRef}
      className={styles.panel}
      style={pos ? { top: pos.top, left: pos.left } : { visibility: 'hidden' }}
      onKeyDown={onKeyDown}
      role="listbox"
      aria-label={colDef.name}
    >
      <input
        ref={inputRef}
        className={styles.search}
        value={query}
        placeholder="Search or create…"
        onChange={(event) => {
          setQuery(event.target.value);
          setHighlight(0);
        }}
        aria-label="Filter or create option"
      />
      <div className={styles.options}>
        {filtered.map((opt, index) => {
          const color = chipColorFor(opt);
          return (
            <button
              key={opt}
              type="button"
              role="option"
              aria-selected={opt === value}
              className={`${styles.option} ${index === highlight ? styles.highlighted : ''}`}
              onMouseEnter={() => setHighlight(index)}
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
      {canCreate && (
        <button type="button" className={styles.create} onClick={() => void create()}>
          Create <span className={styles.createText}>“{query.trim()}”</span>
        </button>
      )}
    </div>
  );
}
