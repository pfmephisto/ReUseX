// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { createContext, useContext } from 'react';

/** A cell coordinate in the data grid. `col` indexes data columns only. */
export interface CellCoord {
  row: number;
  col: number;
}

/**
 * Spreadsheet-style navigation state, shared from `MaterialTable` to every
 * `EditableCell` so a cell knows whether it is the focused or the editing one
 * without prop-drilling through TanStack's column definitions.
 *
 * `col` counts *data* columns only — the thumbnail and add-column columns are
 * not navigable, so they are excluded from the coordinate space entirely.
 */
export interface TableNav {
  focusedCell: CellCoord | null;
  editingCell: CellCoord | null;
  /** Number of data columns; used to clamp horizontal moves. */
  colCount: number;
  /** Number of rows; used to clamp vertical moves. */
  rowCount: number;
  setFocused: (row: number, col: number) => void;
  startEdit: (row: number, col: number, seed?: string) => void;
  /** Leave edit mode; `save` decides whether the in-flight draft is committed. */
  exitEdit: (save: boolean) => void;
  /** Consume-and-clear the seed character a printable keypress stashed. */
  takeSeed: () => string | null;
}

export const TableNavContext = createContext<TableNav | null>(null);

export function useTableNav(): TableNav {
  const ctx = useContext(TableNavContext);
  if (!ctx) throw new Error('useTableNav must be used inside a TableNavContext provider');
  return ctx;
}
