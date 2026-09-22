// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useState } from 'react';
import {
  flexRender,
  getCoreRowModel,
  useReactTable,
  type ColumnDef,
} from '@tanstack/react-table';

import { api } from '../api/client';
import type { MaterialInfo } from '../api/types';
import { useAsync } from '../app/useAsync';
import { describeWriteFailure, type WriteFailure } from '../data/writeState';
import { ColumnHeaderMenu } from './ColumnHeaderMenu';
import { EditableCell } from './EditableCell';
import { ThumbnailCell } from './ThumbnailCell';
import { WriteBanner } from './WriteBanner';
import { apiStubs, type PropertyType } from './materialTableStubs';
import styles from './MaterialTable.module.css';

/**
 * Material passports as a Notion-style, full-width, inline-editable table.
 *
 * This replaces the old two-pane MaterialsPane (a list next to a detail
 * editor). The whole grid is editable in place: each cell edits one property,
 * each header right-click configures the column, and rows and columns are added
 * and removed from the table itself. TanStack Table supplies the headless model;
 * every visual decision is this component's CSS.
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
  const columnsAsync = useAsync(apiStubs.propertyDefinitions, []);

  /** Per-row property maps, fetched from the detail endpoint. */
  const [details, setDetails] = useState<Map<string, Record<string, string>>>(new Map());
  const [failure, setFailure] = useState<WriteFailure | null>(null);
  const [colMenu, setColMenu] = useState<{ id: string; x: number; y: number } | null>(null);

  useEffect(() => {
    if (!materialsAsync.data) return;
    const controller = new AbortController();
    Promise.all(
      materialsAsync.data.map((m) =>
        api
          .material(m.guid, controller.signal)
          .then((d) => [m.guid, d.properties ?? {}] as const)
          .catch(() => [m.guid, {}] as const),
      ),
    ).then((pairs) => setDetails(new Map(pairs)));
    return () => controller.abort();
  }, [materialsAsync.data]);

  // ---- cell save ----------------------------------------------------------

  const handleCellSave = useCallback(
    async (guid: string, propName: string, value: string | null) => {
      const prev = details.get(guid) ?? {};
      const next =
        value === null
          ? Object.fromEntries(Object.entries(prev).filter(([k]) => k !== propName))
          : { ...prev, [propName]: value };
      setDetails((d) => new Map(d).set(guid, next)); // optimistic
      setFailure(null);
      try {
        const updated = await api.patchMaterial(guid, { [propName]: value });
        setDetails((d) => new Map(d).set(guid, updated.properties ?? {}));
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

  // ---- column management --------------------------------------------------

  const handleAddColumn = useCallback(async () => {
    const order = columnsAsync.data?.length ?? 0;
    await apiStubs.createPropertyDefinition({ name: 'New column', type: 'text', sort_order: order });
    columnsAsync.reload();
  }, [columnsAsync]);

  const handleRename = useCallback(
    async (id: string, name: string) => {
      await apiStubs.updatePropertyDefinition(id, { name });
      columnsAsync.reload();
    },
    [columnsAsync],
  );

  const handleTypeChange = useCallback(
    async (id: string, type: PropertyType) => {
      await apiStubs.updatePropertyDefinition(id, { type });
      columnsAsync.reload();
    },
    [columnsAsync],
  );

  const handleOptionsChange = useCallback(
    async (id: string, options: string[]) => {
      await apiStubs.updatePropertyDefinition(id, { options });
      columnsAsync.reload();
    },
    [columnsAsync],
  );

  const handleDeleteColumn = useCallback(
    async (id: string) => {
      await apiStubs.deletePropertyDefinition(id);
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
        apiStubs.updatePropertyDefinition(id, { sort_order: cols[target].sort_order }),
        apiStubs.updatePropertyDefinition(cols[target].id, { sort_order: cols[idx].sort_order }),
      ]);
      columnsAsync.reload();
    },
    [columnsAsync],
  );

  // ---- row management -----------------------------------------------------

  const handleAddRow = useCallback(async () => {
    await apiStubs.createMaterial();
    materialsAsync.reload();
  }, [materialsAsync]);

  const handleDeleteRow = useCallback(
    async (guid: string) => {
      if (!window.confirm('Delete this material passport?')) return;
      await apiStubs.deleteMaterial(guid);
      materialsAsync.reload();
    },
    [materialsAsync],
  );

  // ---- TanStack columns ---------------------------------------------------

  const thumbnailCol: ColumnDef<MaterialInfo> = {
    id: '__thumbnail',
    size: 64,
    header: () => null,
    cell: ({ row }) => (
      <ThumbnailCell
        guid={row.original.guid}
        // Stub; becomes row.original.has_thumbnail after the backend merges.
        hasThumbnail={false}
        onUploaded={() => materialsAsync.reload()}
      />
    ),
    enableResizing: false,
  };

  const dynamicCols: ColumnDef<MaterialInfo>[] = (columnsAsync.data ?? []).map((col) => ({
    id: col.id,
    header: col.name,
    cell: ({ row }) => (
      <EditableCell
        value={details.get(row.original.guid)?.[col.name]}
        colDef={col}
        onSave={(val) => handleCellSave(row.original.guid, col.name, val)}
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
    data: materialsAsync.data ?? [],
    columns: allCols,
    getCoreRowModel: getCoreRowModel(),
    columnResizeMode: 'onChange',
  });

  const menuColDef = colMenu
    ? (columnsAsync.data ?? []).find((c) => c.id === colMenu.id)
    : undefined;

  return (
    <div className={styles.root}>
      {materialsAsync.error && <div className={styles.error}>Failed to load materials</div>}

      {failure && (
        <WriteBanner
          failure={failure}
          onRetry={undefined}
          onDismiss={() => setFailure(null)}
        />
      )}

      <div className={styles.scroll}>
        <table className={styles.table}>
          <thead>
            <tr>
              {table.getFlatHeaders().map((header) => (
                <th
                  key={header.id}
                  style={{ width: header.getSize() !== 150 ? header.getSize() : undefined }}
                  onContextMenu={
                    header.column.id !== '__thumbnail' && header.column.id !== '__add_col'
                      ? (e) => {
                          e.preventDefault();
                          setColMenu({ id: header.column.id, x: e.clientX, y: e.clientY });
                        }
                      : undefined
                  }
                >
                  {flexRender(header.column.columnDef.header, header.getContext())}
                </th>
              ))}
            </tr>
          </thead>
          <tbody>
            {table.getRowModel().rows.map((row) => (
              <tr
                key={row.id}
                onContextMenu={(e) => {
                  e.preventDefault();
                  void handleDeleteRow(row.original.guid);
                }}
              >
                {row.getVisibleCells().map((cell) => (
                  <td key={cell.id}>{flexRender(cell.column.columnDef.cell, cell.getContext())}</td>
                ))}
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      <button className={styles.addRow} onClick={handleAddRow}>
        + Add material
      </button>

      {colMenu && menuColDef && (
        <ColumnHeaderMenu
          colDef={menuColDef}
          position={{ x: colMenu.x, y: colMenu.y }}
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
        />
      )}
    </div>
  );
}
