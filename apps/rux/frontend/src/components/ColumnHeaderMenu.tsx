// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useLayoutEffect, useRef, useState } from 'react';

import type { PropertyDefinition, PropertyType } from '../api/types';

const PROPERTY_TYPES: PropertyType[] = ['text', 'number', 'date', 'boolean', 'select'];
const PROPERTY_TYPE_LABELS: Record<PropertyType, string> = {
  text: 'Text',
  number: 'Number',
  date: 'Date',
  boolean: 'Checkbox',
  select: 'Select',
};
import styles from './ColumnHeaderMenu.module.css';

export interface ColumnHeaderMenuProps {
  colDef: PropertyDefinition;
  position: { x: number; y: number };
  onClose: () => void;
  onRename: (name: string) => void;
  onTypeChange: (type: PropertyType) => void;
  onOptionsChange: (options: string[]) => void;
  onDelete: () => void;
  onMoveLeft: () => void;
  onMoveRight: () => void;
}

/** Split a comma-separated option list into trimmed, non-empty entries. */
function parseOptions(raw: string): string[] {
  return raw
    .split(',')
    .map((part) => part.trim())
    .filter((part) => part.length > 0);
}

/**
 * The per-column configuration panel, opened by a single left click on a header.
 *
 * A `position: fixed` card anchored below the header cell. A `useLayoutEffect` measures
 * it once it is mounted and nudges it back inside the viewport if it would spill
 * off the right or bottom edge — measured, not guessed, because the panel's
 * height depends on whether the column is a `select` (which grows an options
 * box). Dismisses on Escape or on a mousedown anywhere outside it, the usual
 * popover contract.
 */
export function ColumnHeaderMenu({
  colDef,
  position,
  onClose,
  onRename,
  onTypeChange,
  onOptionsChange,
  onDelete,
  onMoveLeft,
  onMoveRight,
}: ColumnHeaderMenuProps) {
  const cardRef = useRef<HTMLDivElement>(null);
  const [pos, setPos] = useState(position);

  // Keep the card inside the viewport. Runs after layout so the measured size
  // is real, and re-runs if the requested position changes.
  useLayoutEffect(() => {
    const card = cardRef.current;
    if (!card) return;
    const rect = card.getBoundingClientRect();
    const margin = 8;
    let x = position.x;
    let y = position.y;
    if (x + rect.width > window.innerWidth - margin) {
      x = Math.max(margin, window.innerWidth - rect.width - margin);
    }
    if (y + rect.height > window.innerHeight - margin) {
      y = Math.max(margin, window.innerHeight - rect.height - margin);
    }
    setPos({ x, y });
  }, [position]);

  // Dismiss on Escape or an outside mousedown.
  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      if (event.key === 'Escape') onClose();
    };
    const onDown = (event: MouseEvent) => {
      if (cardRef.current && !cardRef.current.contains(event.target as Node)) onClose();
    };
    document.addEventListener('keydown', onKey);
    document.addEventListener('mousedown', onDown);
    return () => {
      document.removeEventListener('keydown', onKey);
      document.removeEventListener('mousedown', onDown);
    };
  }, [onClose]);

  const commitRename = (event: React.FocusEvent<HTMLInputElement> | React.KeyboardEvent<HTMLInputElement>) => {
    const next = (event.target as HTMLInputElement).value.trim();
    if (next && next !== colDef.name) onRename(next);
  };

  return (
    <div
      ref={cardRef}
      className={styles.card}
      style={{ left: pos.x, top: pos.y }}
      role="menu"
      aria-label={`Column ${colDef.name}`}
    >
      <section className={styles.section}>
        <label className={styles.label} htmlFor="col-menu-rename">
          Name
        </label>
        <input
          id="col-menu-rename"
          className={styles.input}
          defaultValue={colDef.name}
          onBlur={commitRename}
          onKeyDown={(event) => {
            if (event.key === 'Enter') {
              event.preventDefault();
              commitRename(event);
            }
          }}
        />
      </section>

      <section className={styles.section}>
        <span className={styles.label}>Type</span>
        <div className={styles.types}>
          {PROPERTY_TYPES.map((type) => (
            <button
              key={type}
              type="button"
              className={`${styles.typeBtn} ${type === colDef.type ? styles.typeActive : ''}`}
              onClick={() => onTypeChange(type)}
            >
              {PROPERTY_TYPE_LABELS[type]}
            </button>
          ))}
        </div>
      </section>

      {colDef.type === 'select' && (
        <section className={styles.section}>
          <label className={styles.label} htmlFor="col-menu-options">
            Options (comma-separated)
          </label>
          <textarea
            id="col-menu-options"
            className={styles.textarea}
            defaultValue={(colDef.options ?? []).join(', ')}
            onBlur={(event) => onOptionsChange(parseOptions(event.target.value))}
          />
        </section>
      )}

      <section className={styles.actions}>
        <button type="button" className={styles.action} onClick={onMoveLeft}>
          Move left
        </button>
        <button type="button" className={styles.action} onClick={onMoveRight}>
          Move right
        </button>
        <button type="button" className={`${styles.action} ${styles.danger}`} onClick={onDelete}>
          Delete column
        </button>
      </section>
    </div>
  );
}
