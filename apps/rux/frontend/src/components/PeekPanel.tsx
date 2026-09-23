// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useMemo, useState } from 'react';

import { api } from '../api/client';
import type { PropertyDefinition, PropertyType } from '../api/types';
import { EditableCell } from './EditableCell';
import { TableNavContext, type CellCoord, type TableNav } from './tableNav';
import styles from './PeekPanel.module.css';

export interface PeekPanelProps {
  guid: string;
  columns: PropertyDefinition[];
  /** The material's property values, keyed by property name. */
  values: Record<string, string>;
  hasThumbnail: boolean;
  onClose: () => void;
  onSave: (propName: string, value: string | null) => Promise<void>;
  onAddOption: (colId: string, option: string) => Promise<void>;
  /** Add a new column (mirrors the table's `+` header button). */
  onAddColumn: () => void;
  /** Delete this passport; the caller reloads the table and closes the peek. */
  onDeleted: () => void;
}

/** Type-glyph shown in a peek property label, mirroring the header menu names. */
const PROPERTY_TYPE_ICON: Record<PropertyType, string> = {
  text: 'T',
  number: '#',
  date: '▦',
  boolean: '☑',
  select: '▾',
  multiselect: '▾',
};

/**
 * A side-drawer "page" for one material passport (spec §10.1).
 *
 * Slides in from the right over a dimming backdrop and lists the passport's
 * properties as label/value rows. Each value re-uses the table's `EditableCell`
 * — clicking a value opens that cell in edit mode immediately. Because
 * `EditableCell` reads its focus/edit state from a `TableNavContext`, the peek
 * supplies its own tiny nav whose single "editing cell" is the row the user
 * clicked (tracked as `peekEditingProp`); every property is row `col=0`, so the
 * property *index* is the row coordinate. Edits flow back through the same
 * `onSave` the table cells use, so the underlying row stays in sync.
 */
export function PeekPanel({
  guid,
  columns,
  values,
  hasThumbnail,
  onClose,
  onSave,
  onAddOption,
  onAddColumn,
  onDeleted,
}: PeekPanelProps) {
  const [peekEditingProp, setPeekEditingProp] = useState<string | null>(null);

  // Escape closes the peek.
  useEffect(() => {
    const handler = (event: KeyboardEvent) => {
      if (event.key === 'Escape') onClose();
    };
    document.addEventListener('keydown', handler);
    return () => document.removeEventListener('keydown', handler);
  }, [onClose]);

  const editingIndex = peekEditingProp
    ? columns.findIndex((c) => c.id === peekEditingProp)
    : -1;

  // A one-column, N-row nav: property i lives at {row: i, col: 0}. Only the
  // clicked property is "editing"; nothing is ever "focused" (the peek has no
  // spreadsheet cursor). The mutators translate coordinates back to a prop id.
  const nav: TableNav = useMemo(() => {
    const propAt = (row: number): string | null => columns[row]?.id ?? null;
    const editingCell: CellCoord | null = editingIndex >= 0 ? { row: editingIndex, col: 0 } : null;
    return {
      focusedCell: null,
      editingCell,
      colCount: 1,
      rowCount: columns.length,
      setFocused: () => {},
      startEdit: (row) => setPeekEditingProp(propAt(row)),
      exitEdit: () => setPeekEditingProp(null),
      clearFocus: () => setPeekEditingProp(null),
      takeSeed: () => null,
    };
  }, [columns, editingIndex]);

  const handleDelete = async () => {
    if (!window.confirm('Delete this material passport?')) return;
    await api.deleteMaterial(guid);
    onDeleted();
    onClose();
  };

  const thumbnailSrc = api.materialThumbnail(guid);

  return (
    <>
      <div className={styles.backdrop} onClick={onClose} aria-hidden="true" />
      <aside className={styles.panel} role="dialog" aria-label="Material passport">
        <header className={styles.header}>
          <button
            type="button"
            className={styles.close}
            onClick={onClose}
            aria-label="Close peek"
            title="Close"
          >
            ‹
          </button>
          <span className={styles.title} title={guid}>
            {guid}
          </span>
          <button
            type="button"
            className={styles.delete}
            onClick={() => void handleDelete()}
          >
            Delete
          </button>
        </header>

        <div className={styles.body}>
          <div className={styles.thumbnail}>
            {hasThumbnail ? (
              <img className={styles.thumbnailImg} src={thumbnailSrc} alt="" />
            ) : (
              <span className={styles.thumbnailPlaceholder} aria-hidden="true">
                <svg
                  viewBox="0 0 24 24"
                  width="40"
                  height="40"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                >
                  <path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z" />
                  <circle cx="12" cy="13" r="4" />
                </svg>
              </span>
            )}
          </div>

          <TableNavContext.Provider value={nav}>
            <div className={styles.props}>
              {columns.map((col, index) => (
                <div key={col.id} className={styles.propRow}>
                  <div className={styles.propLabel}>
                    <span className={styles.propIcon} aria-hidden="true">
                      {PROPERTY_TYPE_ICON[col.type]}
                    </span>
                    <span className={styles.propName}>{col.name}</span>
                  </div>
                  <div
                    className={styles.propValue}
                    onClick={() => setPeekEditingProp(col.id)}
                  >
                    <EditableCell
                      value={values[col.name]}
                      colDef={col}
                      onSave={(val) => onSave(col.name, val)}
                      onAddOption={(opt) => onAddOption(col.id, opt)}
                      rowIndex={index}
                      colIndex={0}
                    />
                  </div>
                </div>
              ))}

              <button type="button" className={styles.addColumn} onClick={onAddColumn}>
                + Add column
              </button>
            </div>
          </TableNavContext.Provider>
        </div>
      </aside>
    </>
  );
}
