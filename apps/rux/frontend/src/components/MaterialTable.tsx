// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  flexRender,
  getCoreRowModel,
  useReactTable,
  type ColumnDef,
} from '@tanstack/react-table';

import { api } from '../api/client';
import type {
  MaterialDetail,
  MaterialInfo,
  PropertyType,
} from '../api/types';
import { useAsync } from '../app/useAsync';
import { describeWriteFailure, type WriteFailure } from '../data/writeState';
import { ColumnHeaderMenu } from './ColumnHeaderMenu';
import { EditableCell } from './EditableCell';
import { PeekPanel } from './PeekPanel';
import { TableNavContext, type CellCoord, type TableNav } from './tableNav';
import { ThumbnailCell } from './ThumbnailCell';
import { WriteBanner } from './WriteBanner';
import styles from './MaterialTable.module.css';

/**
 * Material passports as a Notion-style, full-width, inline-editable table.
 *
 * This replaces the old two-pane MaterialsPane (a list next to a detail
 * editor). The whole grid is editable in place: each cell edits one property,
 * each header right-click configures the column, and rows and columns are added
 * and removed from the table itself. TanStack Table supplies the headless model;
 * every visual decision is this component's CSS. A single click on a header
 * cell opens its column menu; a drag on the header's right edge resizes it.
 *
 * Writes are optimistic. A cell edit updates `details` immediately, sends a
 * sparse `PATCH /materials/{guid}`, and rolls back to the pre-edit map on
 * failure — the same 409-vs-503 contract the old pane relied on, surfaced
 * through `WriteBanner`.
 *
 * The property-definition schema (columns) and the create/delete/thumbnail
 * endpoints are stubbed in `materialTableStubs.ts` until the backend PR
 * (#413/#414/#415) merges; those stubs `console.warn` and are the single seam
 * to swap for real `api.*` calls.
 */
export function MaterialTable() {
  const materialsAsync = useAsync((signal) => api.materials(signal), []);
  const columnsAsync = useAsync((signal) => api.propertyDefinitions(signal), []);

  /** Per-row full detail (properties + has_thumbnail), fetched from the detail endpoint. */
  const [details, setDetails] = useState<Map<string, MaterialDetail>>(new Map());
  const [failure, setFailure] = useState<WriteFailure | null>(null);
  const [colMenu, setColMenu] = useState<{ id: string; x: number; y: number } | null>(null);

  // ---- sort / search -------------------------------------------------------
  const [sortConfig, setSortConfig] = useState<{ colName: string; dir: 'asc' | 'desc' } | null>(null);
  const [search, setSearch] = useState('');

  // ---- column widths (resize) / horizontal-scroll shadow ------------------
  const [colWidths, setColWidths] = useState<Record<string, number>>({});
  const scrollRef = useRef<HTMLDivElement>(null);
  const [isScrolledX, setIsScrolledX] = useState(false);

  // Seed local widths from the loaded column definitions, once per column.
  // useState can't take async data, so hydrate here — and only for columns the
  // user hasn't already resized this session (so a drag isn't clobbered).
  useEffect(() => {
    if (!columnsAsync.data) return;
    setColWidths((prev) => {
      const next = { ...prev };
      for (const col of columnsAsync.data!) {
        if (!(col.id in next)) {
          next[col.id] = col.width ?? 200;
        }
      }
      return next;
    });
  }, [columnsAsync.data]);

  // ---- row gutter / selection / peek --------------------------------------
  const [selectedRowIds, setSelectedRowIds] = useState<Set<string>>(new Set());
  const [hoveredRowId, setHoveredRowId] = useState<string | null>(null);
  // Peek panel target (Phase 2 wires the actual panel); the OPEN button sets it.
  const [peekGuid, setPeekGuid] = useState<string | null>(null);

  // ---- spreadsheet keyboard navigation -----------------------------------
  const [focusedCell, setFocusedCell] = useState<CellCoord | null>(null);
  const [editingCell, setEditingCell] = useState<CellCoord | null>(null);
  // A printable key pressed on a focused (non-editing) cell both starts edit
  // mode and seeds the input; the seed is stashed here for the cell to consume.
  const seedRef = useRef<string | null>(null);

  const dataCols = columnsAsync.data ?? [];

  // Filter by search, then sort by sortConfig. Both operate on property values
  // in the details map, which is populated after materialsAsync resolves.
  const sortedFilteredRows = useMemo(() => {
    let rows = materialsAsync.data ?? [];
    if (search.trim()) {
      const needle = search.trim().toLowerCase();
      rows = rows.filter((m) => {
        const props = details.get(m.guid)?.properties ?? {};
        return Object.values(props).some((v) => v.toLowerCase().includes(needle));
      });
    }
    if (sortConfig) {
      const { colName, dir } = sortConfig;
      rows = [...rows].sort((a, b) => {
        const aVal = details.get(a.guid)?.properties?.[colName] ?? '';
        const bVal = details.get(b.guid)?.properties?.[colName] ?? '';
        const cmp = aVal.localeCompare(bVal, undefined, { numeric: true, sensitivity: 'base' });
        return dir === 'asc' ? cmp : -cmp;
      });
    }
    return rows;
  }, [materialsAsync.data, details, search, sortConfig]);

  const rowCount = sortedFilteredRows.length;
  const colCount = dataCols.length;

  useEffect(() => {
    if (!materialsAsync.data) return;
    const controller = new AbortController();
    Promise.all(
      materialsAsync.data.map((m) =>
        api
          .material(m.guid, controller.signal)
          .then((d) => [m.guid, d] as const)
          .catch(() => [m.guid, { guid: m.guid } as MaterialDetail] as const),
      ),
    ).then((pairs) => setDetails(new Map(pairs)));
    return () => controller.abort();
  }, [materialsAsync.data]);

  // ---- cell save ----------------------------------------------------------

  const handleCellSave = useCallback(
    async (guid: string, propName: string, value: string | null) => {
      const prev = details.get(guid) ?? ({ guid } as MaterialDetail);
      const prevProps = prev.properties ?? {};
      const nextProps =
        value === null
          ? Object.fromEntries(Object.entries(prevProps).filter(([k]) => k !== propName))
          : { ...prevProps, [propName]: value };
      setDetails((d) => new Map(d).set(guid, { ...prev, properties: nextProps })); // optimistic
      setFailure(null);
      try {
        const updated = await api.patchMaterial(guid, { [propName]: value });
        setDetails((d) => new Map(d).set(guid, { ...prev, ...updated }));
      } catch (error) {
        setDetails((d) => new Map(d).set(guid, prev)); // rollback
        setFailure(
          describeWriteFailure(
            error instanceof Error ? error : new Error(String(error)),
            'this passport',
          ),
        );
      }
    },
    [details],
  );

  // ---- column resize ------------------------------------------------------

  const DEFAULT_COL_WIDTH = 200;

  const handleResizerMouseDown = useCallback(
    (e: React.MouseEvent, colId: string, startWidth: number) => {
      e.preventDefault();
      const startX = e.clientX;
      const MIN_WIDTH = 100;
      let currentWidth = startWidth;

      const onMouseMove = (ev: MouseEvent) => {
        currentWidth = Math.max(MIN_WIDTH, startWidth + (ev.clientX - startX));
        setColWidths((prev) => ({ ...prev, [colId]: currentWidth }));
      };

      const onMouseUp = () => {
        window.removeEventListener('mousemove', onMouseMove);
        window.removeEventListener('mouseup', onMouseUp);
        void api.updatePropertyDefinition(colId, { width: currentWidth });
      };

      window.addEventListener('mousemove', onMouseMove);
      window.addEventListener('mouseup', onMouseUp);
    },
    [],
  );

  // ---- column management --------------------------------------------------

  const handleAddColumn = useCallback(async () => {
    const order = columnsAsync.data?.length ?? 0;
    try {
      await api.createPropertyDefinition({ name: 'New column', type: 'text', sort_order: order });
      columnsAsync.reload();
    } catch (error) {
      setFailure(
        describeWriteFailure(
          error instanceof Error ? error : new Error(String(error)),
          'new column',
        ),
      );
    }
  }, [columnsAsync]);

  const handleRename = useCallback(
    async (id: string, name: string) => {
      await api.updatePropertyDefinition(id, { name });
      columnsAsync.reload();
    },
    [columnsAsync],
  );

  const handleTypeChange = useCallback(
    async (id: string, type: PropertyType) => {
      await api.updatePropertyDefinition(id, { type });
      columnsAsync.reload();
    },
    [columnsAsync],
  );

  const handleOptionsChange = useCallback(
    async (id: string, options: string[]) => {
      await api.updatePropertyDefinition(id, { options });
      columnsAsync.reload();
    },
    [columnsAsync],
  );

  const handleDeleteColumn = useCallback(
    async (id: string) => {
      await api.deletePropertyDefinition(id);
      columnsAsync.reload();
    },
    [columnsAsync],
  );

  const handleMoveColumn = useCallback(
    async (id: string, direction: 'left' | 'right') => {
      const cols = columnsAsync.data ?? [];
      const idx = cols.findIndex((c) => c.id === id);
      const target = direction === 'left' ? idx - 1 : idx + 1;
      if (target < 0 || target >= cols.length) return;
      await Promise.all([
        api.updatePropertyDefinition(id, { sort_order: cols[target].sort_order }),
        api.updatePropertyDefinition(cols[target].id, { sort_order: cols[idx].sort_order }),
      ]);
      columnsAsync.reload();
    },
    [columnsAsync],
  );

  // ---- row management -----------------------------------------------------

  const handleAddRow = useCallback(async () => {
    try {
      await api.createMaterial();
      materialsAsync.reload();
    } catch (error) {
      setFailure(
        describeWriteFailure(
          error instanceof Error ? error : new Error(String(error)),
          'new material',
        ),
      );
    }
  }, [materialsAsync]);

  const handleDeleteRow = useCallback(
    async (guid: string) => {
      if (!window.confirm('Delete this material passport?')) return;
      await api.deleteMaterial(guid);
      materialsAsync.reload();
    },
    [materialsAsync],
  );

  const handleInsertRowBelow = useCallback(async () => {
    // No positional-insert endpoint yet; append a new material like Add row.
    try {
      await api.createMaterial();
      materialsAsync.reload();
    } catch (error) {
      setFailure(
        describeWriteFailure(
          error instanceof Error ? error : new Error(String(error)),
          'new material',
        ),
      );
    }
  }, [materialsAsync]);

  // ---- row selection / gutter / peek --------------------------------------

  const toggleRowSelected = useCallback((guid: string) => {
    setSelectedRowIds((prev) => {
      const next = new Set(prev);
      if (next.has(guid)) next.delete(guid);
      else next.add(guid);
      return next;
    });
  }, []);

  const allGuids = materialsAsync.data?.map((m) => m.guid) ?? [];
  const allSelected = allGuids.length > 0 && allGuids.every((g) => selectedRowIds.has(g));
  const someSelected = selectedRowIds.size > 0 && !allSelected;

  const toggleSelectAll = useCallback(() => {
    setSelectedRowIds((prev) =>
      prev.size >= allGuids.length && allGuids.every((g) => prev.has(g))
        ? new Set()
        : new Set(allGuids),
    );
    // allGuids is derived per-render; the closure captures the current set.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [materialsAsync.data]);

  const handleDeleteSelected = useCallback(async () => {
    const guids = [...selectedRowIds];
    if (guids.length === 0) return;
    try {
      await Promise.all(guids.map((g) => api.deleteMaterial(g)));
      setSelectedRowIds(new Set());
      materialsAsync.reload();
    } catch (error) {
      setFailure(
        describeWriteFailure(
          error instanceof Error ? error : new Error(String(error)),
          'selected passports',
        ),
      );
    }
  }, [selectedRowIds, materialsAsync]);

  const handlePeekOpen = useCallback((guid: string) => {
    setPeekGuid(guid);
  }, []);

  // ---- navigation handlers ------------------------------------------------

  const setFocused = useCallback((row: number, col: number) => {
    setFocusedCell({ row, col });
  }, []);

  const startEdit = useCallback((row: number, col: number, seed?: string) => {
    if (seed !== undefined) seedRef.current = seed;
    setFocusedCell({ row, col });
    setEditingCell({ row, col });
  }, []);

  const exitEdit = useCallback((_save: boolean) => {
    // The cell owns the commit decision (via onBlur / Enter); here we only
    // leave edit mode. The cell stays SELECTED (focusedCell keeps its value).
    // `save` is part of the contract for symmetry and future use, but a text
    // cell has already committed by the time it calls this.
    setEditingCell(null);
  }, []);

  // SELECTED → IDLE: drop the cursor entirely.
  const clearFocus = useCallback(() => {
    setFocusedCell(null);
    setEditingCell(null);
  }, []);

  const takeSeed = useCallback(() => {
    const seed = seedRef.current;
    seedRef.current = null;
    return seed;
  }, []);

  const navValue: TableNav = {
    focusedCell,
    editingCell,
    rowCount,
    colCount,
    setFocused,
    startEdit,
    exitEdit,
    clearFocus,
    takeSeed,
  };

  // Table-level keyboard handler: arrows / Tab move the focused cell, Enter/F2
  // and printable characters start editing. Ignored while a cell is editing —
  // the input owns its keys then, except for Tab (save + move) handled below.
  const handleTableKeyDown = useCallback(
    (event: React.KeyboardEvent) => {
      const editing = editingCell;
      const focused = focusedCell;

      // Tab always moves, saving first if editing.
      if (event.key === 'Tab') {
        if (!focused) return;
        event.preventDefault();
        if (editing) setEditingCell(null);
        const flat = focused.row * colCount + focused.col;
        const next = event.shiftKey ? flat - 1 : flat + 1;
        const clamped = Math.max(0, Math.min(next, rowCount * colCount - 1));
        setFocusedCell({ row: Math.floor(clamped / colCount), col: clamped % colCount });
        return;
      }

      // Everything below is navigation only — never while editing.
      if (editing || !focused) return;

      const move = (dr: number, dc: number) => {
        event.preventDefault();
        setFocusedCell({
          row: Math.max(0, Math.min(focused.row + dr, rowCount - 1)),
          col: Math.max(0, Math.min(focused.col + dc, colCount - 1)),
        });
      };

      if (event.key === 'ArrowUp') move(-1, 0);
      else if (event.key === 'ArrowDown') move(1, 0);
      else if (event.key === 'ArrowLeft') move(0, -1);
      else if (event.key === 'ArrowRight') move(0, 1);
      else if (event.key === 'Escape') {
        // SELECTED → IDLE: drop the cursor.
        event.preventDefault();
        clearFocus();
      } else if (event.key === 'Backspace' || event.key === 'Delete') {
        // Clear the focused cell's value in place, no editor.
        event.preventDefault();
        const material = materialsAsync.data?.[focused.row];
        const col = dataCols[focused.col];
        if (material && col) void handleCellSave(material.guid, col.name, null);
      } else if (event.key === 'Enter' || event.key === 'F2') {
        event.preventDefault();
        setEditingCell({ ...focused });
      } else if (event.key.length === 1 && !event.ctrlKey && !event.metaKey && !event.altKey) {
        // A printable character starts edit mode seeded with that character.
        event.preventDefault();
        seedRef.current = event.key;
        setEditingCell({ ...focused });
      }
    },
    [
      editingCell,
      focusedCell,
      colCount,
      rowCount,
      clearFocus,
      materialsAsync.data,
      dataCols,
      handleCellSave,
    ],
  );

  // ---- TanStack columns ---------------------------------------------------

  const thumbnailCol: ColumnDef<MaterialInfo> = {
    id: '__thumbnail',
    size: 64,
    header: () => null,
    cell: ({ row }) => (
      <div className={styles.thumbWrap}>
        <ThumbnailCell
          guid={row.original.guid}
          hasThumbnail={details.get(row.original.guid)?.has_thumbnail ?? false}
          onUploaded={() => materialsAsync.reload()}
        />
        <button
          type="button"
          className={styles.openBtn}
          onClick={(e) => {
            e.stopPropagation();
            handlePeekOpen(row.original.guid);
          }}
        >
          Open
        </button>
      </div>
    ),
    enableResizing: false,
  };

  const dynamicCols: ColumnDef<MaterialInfo>[] = dataCols.map((col, colIndex) => ({
    id: col.id,
    header: col.name,
    cell: ({ row }) => (
      <EditableCell
        value={details.get(row.original.guid)?.properties?.[col.name]}
        colDef={col}
        onSave={(val) => handleCellSave(row.original.guid, col.name, val)}
        onAddOption={(opt) => handleOptionsChange(col.id, [...(col.options ?? []), opt])}
        rowIndex={row.index}
        colIndex={colIndex}
      />
    ),
  }));

  const addColumnCol: ColumnDef<MaterialInfo> = {
    id: '__add_col',
    size: 48,
    header: () => (
      <button className={styles.addColBtn} onClick={handleAddColumn} title="Add column">
        +
      </button>
    ),
    cell: () => null,
    enableResizing: false,
  };

  const allCols = [thumbnailCol, ...dynamicCols, addColumnCol];

  const table = useReactTable({
    data: sortedFilteredRows,
    columns: allCols,
    getCoreRowModel: getCoreRowModel(),
    columnResizeMode: 'onChange',
  });

  const menuColDef = colMenu
    ? (columnsAsync.data ?? []).find((c) => c.id === colMenu.id)
    : undefined;

  return (
    <TableNavContext.Provider value={navValue}>
    <div className={styles.root} data-peek={peekGuid ?? undefined}>
      {materialsAsync.error && <div className={styles.error}>Failed to load materials</div>}

      {failure && (
        <WriteBanner
          failure={failure}
          onRetry={undefined}
          onDismiss={() => setFailure(null)}
        />
      )}

      <div className={styles.toolbar}>
        <input
          type="search"
          className={styles.searchInput}
          placeholder="Search materials…"
          value={search}
          onChange={(e) => setSearch(e.target.value)}
          aria-label="Search materials"
        />
        {sortConfig && (
          <button
            type="button"
            className={styles.sortBadge}
            onClick={() => setSortConfig(null)}
            title="Clear sort"
          >
            {sortConfig.colName} {sortConfig.dir === 'asc' ? '↑' : '↓'} ×
          </button>
        )}
      </div>

      {selectedRowIds.size > 0 && (
        <div className={styles.bulkBar}>
          <span className={styles.bulkPill}>{selectedRowIds.size} selected</span>
          <button
            type="button"
            className={styles.bulkDelete}
            onClick={() => void handleDeleteSelected()}
          >
            Delete selected
          </button>
          <button
            type="button"
            className={styles.bulkClear}
            onClick={() => setSelectedRowIds(new Set())}
          >
            Clear
          </button>
        </div>
      )}

      <div
        ref={scrollRef}
        className={styles.scroll}
        onScroll={() => setIsScrolledX((scrollRef.current?.scrollLeft ?? 0) > 0)}
        tabIndex={-1}
        onKeyDown={handleTableKeyDown}
      >
        <table className={styles.table}>
          <thead>
            <tr>
              <th className={styles.gutterHead}>
                <input
                  type="checkbox"
                  className={styles.checkbox}
                  checked={allSelected}
                  ref={(el) => {
                    if (el) el.indeterminate = someSelected;
                  }}
                  aria-label="Select all rows"
                  onChange={toggleSelectAll}
                />
              </th>
              {table.getFlatHeaders().map((header) => {
                const colId = header.column.id;
                const isThumb = colId === '__thumbnail';
                const isAddCol = colId === '__add_col';
                const isDynamic = !isThumb && !isAddCol;
                const stickyClass = isThumb
                  ? `${styles.stickyCol}${isScrolledX ? ` ${styles.scrolled}` : ''}`
                  : '';
                const width = isDynamic
                  ? (colWidths[colId] ?? DEFAULT_COL_WIDTH)
                  : header.getSize() !== 150
                    ? header.getSize()
                    : undefined;
                return (
                  <th
                    key={header.id}
                    className={`${styles.headerCell} ${stickyClass}`.trim()}
                    style={{ width, position: 'relative' }}
                    onClick={
                      isDynamic
                        ? (e) => {
                            if ((e.target as HTMLElement).classList.contains(styles.resizer)) return;
                            const rect = e.currentTarget.getBoundingClientRect();
                            setColMenu({ id: colId, x: rect.left, y: rect.bottom });
                          }
                        : undefined
                    }
                  >
                    <span className={styles.headerLabel}>
                      {flexRender(header.column.columnDef.header, header.getContext())}
                    </span>
                    {isDynamic && (
                      <div
                        className={styles.resizer}
                        onMouseDown={(e) => {
                          e.stopPropagation();
                          handleResizerMouseDown(e, colId, colWidths[colId] ?? DEFAULT_COL_WIDTH);
                        }}
                      />
                    )}
                  </th>
                );
              })}
            </tr>
          </thead>
          <tbody>
            {table.getRowModel().rows.map((row) => {
              const guid = row.original.guid;
              const selected = selectedRowIds.has(guid);
              return (
                <tr
                  key={row.id}
                  className={styles.row}
                  data-hovered={hoveredRowId === guid ? 'true' : undefined}
                  data-selected={selected ? 'true' : undefined}
                  data-peeked={peekGuid === guid ? 'true' : undefined}
                  onMouseEnter={() => setHoveredRowId(guid)}
                  onMouseLeave={() =>
                    setHoveredRowId((cur) => (cur === guid ? null : cur))
                  }
                  onContextMenu={(e) => {
                    e.preventDefault();
                    void handleDeleteRow(guid);
                  }}
                >
                  <td className={styles.gutterCell}>
                    <div className={styles.gutter}>
                      <button
                        type="button"
                        className={styles.gutterInsert}
                        title="Insert row below"
                        onClick={() => void handleInsertRowBelow()}
                      >
                        +
                      </button>
                      <span className={styles.gutterDrag} aria-hidden="true">
                        ⠿
                      </span>
                      <input
                        type="checkbox"
                        className={styles.checkbox}
                        checked={selected}
                        aria-label="Select row"
                        onChange={() => toggleRowSelected(guid)}
                      />
                    </div>
                  </td>
                  {row.getVisibleCells().map((cell) => {
                    const colId = cell.column.id;
                    const isThumb = colId === '__thumbnail';
                    const isAddCol = colId === '__add_col';
                    const isDynamic = !isThumb && !isAddCol;
                    const stickyClass = isThumb
                      ? `${styles.stickyCol}${isScrolledX ? ` ${styles.scrolled}` : ''}`
                      : '';
                    return (
                      <td
                        key={cell.id}
                        className={stickyClass || undefined}
                        style={
                          isDynamic ? { width: colWidths[colId] ?? DEFAULT_COL_WIDTH } : undefined
                        }
                      >
                        {flexRender(cell.column.columnDef.cell, cell.getContext())}
                      </td>
                    );
                  })}
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>

      <button className={styles.addRow} onClick={handleAddRow}>
        <span className={styles.addRowIcon} aria-hidden="true">
          +
        </span>
        <span className={styles.addRowLabel}>New material</span>
      </button>

      {colMenu && menuColDef && (
        <ColumnHeaderMenu
          colDef={menuColDef}
          position={{ x: colMenu.x, y: colMenu.y }}
          sortDirection={sortConfig?.colName === menuColDef.name ? sortConfig.dir : null}
          onClose={() => setColMenu(null)}
          onRename={(name) => {
            void handleRename(colMenu.id, name);
            setColMenu(null);
          }}
          onTypeChange={(type) => {
            void handleTypeChange(colMenu.id, type);
            setColMenu(null);
          }}
          onOptionsChange={(opts) => {
            void handleOptionsChange(colMenu.id, opts);
            setColMenu(null);
          }}
          onDelete={() => {
            void handleDeleteColumn(colMenu.id);
            setColMenu(null);
          }}
          onMoveLeft={() => {
            void handleMoveColumn(colMenu.id, 'left');
            setColMenu(null);
          }}
          onMoveRight={() => {
            void handleMoveColumn(colMenu.id, 'right');
            setColMenu(null);
          }}
          onSortAsc={() => {
            setSortConfig({ colName: menuColDef.name, dir: 'asc' });
            setColMenu(null);
          }}
          onSortDesc={() => {
            setSortConfig({ colName: menuColDef.name, dir: 'desc' });
            setColMenu(null);
          }}
        />
      )}

      {peekGuid && (
        <PeekPanel
          guid={peekGuid}
          columns={columnsAsync.data ?? []}
          values={details.get(peekGuid)?.properties ?? {}}
          hasThumbnail={details.get(peekGuid)?.has_thumbnail ?? false}
          onClose={() => setPeekGuid(null)}
          onSave={(propName, value) => handleCellSave(peekGuid, propName, value)}
          onAddOption={(colId, opt) =>
            handleOptionsChange(colId, [
              ...((columnsAsync.data?.find((c) => c.id === colId)?.options) ?? []),
              opt,
            ])
          }
          onAddColumn={() => void handleAddColumn()}
          onDeleted={() => materialsAsync.reload()}
        />
      )}
    </div>
    </TableNavContext.Provider>
  );
}
