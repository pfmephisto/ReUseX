// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';
import type { RefObject } from 'react';

import { api } from '../../api/client';
import { useCanEdit } from '../../app/CaseRoleContext';
import type { PartPhotos, ResourceKey, SurveyType, Template } from '../../api/types';
import { EVIDENCE_LAST_KEY } from '../../kortlaegning/keys';
import {
  flattenRows,
  sameSelection,
  type EnvFilter,
  type Filters,
  type Row,
  type Selection,
  type Tab,
} from '../../kortlaegning/model';
import {
  cellModel,
  isManual,
  partDesignation,
  partRowLabel,
  type ResourceColumn,
  type ResourceIndex,
} from '../../kortlaegning/resources';
import { photoCountText, ROW_THUMB_MAX_SIZE, rowThumbFrame } from '../../kortlaegning/photo';
import { confidencePercent, formatQuantity, formatTonnes } from '../../kortlaegning/vocab';
import { ConfidenceBar } from '../ConfidenceBar';
import { Kbd } from '../Kbd';
import { Pill } from '../Pill';
import { ResourceCell } from './ResourceCell';
import { TypeMark } from './TypeMark';
import styles from './SurveyTable.module.css';

export interface SurveyTableProps {
  /** Already filtered by tab + filters. */
  types: SurveyType[];
  counts: Record<Tab, number>;
  tab: Tab;
  onTab: (t: Tab) => void;
  filters: Filters;
  onFilters: (f: Filters) => void;
  rooms: { id: number; name: string }[];
  open: ReadonlySet<number>;
  selection: Selection;
  /** Click. */
  onSelect: (s: Selection) => void;
  /** Chevron / click on the already-selected type. */
  onToggle: (typeId: number) => void;
  /** Double-click. */
  onOpenDialog: (s: Selection) => void;
  /** Panel keyboard. */
  onKeyDown: (e: React.KeyboardEvent) => void;
  tableRef: React.RefObject<HTMLDivElement | null>;
  /** Template-built columns (resolved_keys order, sys:name excluded — plan R4). */
  columns: ResourceColumn[];
  resources: ResourceIndex;
  /** The key catalogue: a part row finds its designation through it. */
  catalogue: ResourceKey[];
  templates: Template[];
  templateId: number | null;
  onTemplate: (id: number) => void;
  /** Returns the write's promise (a select/checkbox shows its choice until it settles). */
  onCellCommit: (code: string, keyId: string, value: string | null) => Promise<void>;
  onInvalid: (label: string) => void;
  onAddResource: () => void;
  onAddColumn: () => void;
  /**
   * Photo count + best frame per part code (`GET /survey/photos`); `null`
   * until it resolves — the rows render first and fill their thumbnails in.
   */
  photos: Readonly<Record<string, PartPhotos>> | null;
}

export const TABS: { id: Tab; label: string }[] = [
  { id: 'queue', label: 'Til gennemsyn' },
  { id: 'approved', label: 'Godkendt' },
  { id: 'all', label: 'Alle' },
  { id: 'rejected', label: 'Afvist' },
];

/** The table's empty row: an empty Afvist tab is not a filter miss. */
export function emptyRowText(tab: Tab, inTab: number): string {
  if (tab === 'rejected' && inTab === 0) return 'Ingen afviste typer.';
  return 'Ingen rækker matcher filtrene.';
}

/**
 * How long a click on the already-selected type row waits before it folds the
 * row, so the first click of a double-click (which opens the dialog) never
 * folds it as well.
 */
export const TOGGLE_DELAY_MS = 250;

/**
 * What a click on a type row does: the first click on an unselected row
 * selects it; the first click on the selected row folds it (after
 * `TOGGLE_DELAY_MS`); a double-click's later clicks do neither.
 */
export function typeRowClick(selected: boolean, detail: number): 'select' | 'toggle' | 'ignore' {
  if (detail > 1) return 'ignore';
  return selected ? 'toggle' : 'select';
}

/**
 * A row's small best-frame thumbnail. Without a frame (none found, or the
 * photo batch not in yet) it is a quiet placeholder of the same size, so the
 * row never shifts when the batch resolves.
 */
function RowThumb({ frameId }: { frameId: number | null }) {
  const [failed, setFailed] = useState<number | null>(null);
  if (frameId === null || failed === frameId) return <span className={styles.thumb} aria-hidden="true" />;
  return (
    <img
      className={styles.thumb}
      src={api.frameImageUrl(frameId, 'color', { maxSize: ROW_THUMB_MAX_SIZE })}
      alt=""
      loading="lazy"
      onError={() => setFailed(frameId)}
    />
  );
}

/**
 * One row's template cells. A type row shows the Mængde sum and its
 * type-scoped keys (through its carrier part); a part row shows every key.
 * Editors appear only in the selected row (plan R2).
 */
function Cells(props: {
  row: Row;
  type: SurveyType;
  columns: ResourceColumn[];
  resources: ResourceIndex;
  selected: boolean;
  onCellCommit: SurveyTableProps['onCellCommit'];
  onInvalid: SurveyTableProps['onInvalid'];
  home: RefObject<HTMLElement | null>;
}) {
  const { row, type, columns, resources, selected } = props;
  const canEdit = useCanEdit(); // a viewer's cells never open an editor
  return (
    <>
      {columns.map((column) => {
        const m = cellModel(row, type, column, resources);
        let content = null;
        if (m.kind === 'aggregate') {
          content = (
            <>
              {formatQuantity(type.quantity, type.unit)}{' '}
              <span className={styles.faint}>{formatTonnes(type.mass_t)}</span>
            </>
          );
        } else if (m.kind === 'value') {
          const target = m.target;
          content = (
            <ResourceCell
              resourceKey={column.key}
              value={m.value}
              editing={canEdit && selected && target !== null}
              onCommit={(v) => (target !== null ? props.onCellCommit(target, column.key.id, v) : undefined)}
              onInvalid={props.onInvalid}
              home={props.home}
            />
          );
        }
        return <td key={column.key.id}>{content}</td>;
      })}
    </>
  );
}

/**
 * The tabbed, filterable Kortlægning table: survey types grouped by tab,
 * expandable into their building-part rows. Purely presentational — every
 * piece of state (tab, filters, open set, selection) is a prop, and this
 * component only derives the flattened row list for render.
 */
export function SurveyTable(props: SurveyTableProps) {
  const canEdit = useCanEdit();
  const {
    types,
    counts,
    tab,
    onTab,
    filters,
    onFilters,
    rooms,
    open,
    selection,
    onSelect,
    onToggle,
    onOpenDialog,
    onKeyDown,
    tableRef,
    columns,
    resources,
    catalogue,
    templates,
    templateId,
    onTemplate,
    onCellCommit,
    onInvalid,
    onAddResource,
    onAddColumn,
    photos,
  } = props;

  const rows = flattenRows(types, open);
  const typeById = new Map(types.map((t) => [t.id, t]));

  // A pending fold from a click on the selected row; a double-click cancels it.
  const toggleTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const cancelToggle = () => {
    if (toggleTimer.current !== null) clearTimeout(toggleTimer.current);
    toggleTimer.current = null;
  };
  useEffect(() => cancelToggle, []);
  // A selection change (a part-row click, a key, the page) cancels a pending
  // fold, so a quick click on the type and then its part does not fold it.
  useEffect(() => cancelToggle(), [selection]);

  // Keep the selected row in view as selection moves by keyboard.
  useEffect(() => {
    const el = tableRef.current?.querySelector('[aria-selected="true"]');
    el?.scrollIntoView({ block: 'nearest' });
  }, [selection, tableRef]);

  return (
    <section className={styles.panel} onKeyDown={onKeyDown}>
      <div className={styles.tabs} role="tablist">
        {TABS.map(({ id, label }) => (
          <button
            key={id}
            type="button"
            role="tab"
            aria-selected={tab === id}
            className={styles.tab}
            onClick={() => onTab(id)}
          >
            {label} ({counts[id]})
          </button>
        ))}
      </div>

      <div className={styles.tools}>
        <label className={styles.picker}>
          <span className={styles.pickerLabel}>Skabelon</span>
          <select
            className={styles.select}
            aria-label="Skabelon"
            value={templateId === null ? '' : String(templateId)}
            onChange={(e) => onTemplate(Number(e.target.value))}
            disabled={templates.length === 0}
          >
            {templates.length === 0 && <option value="">Ingen skabeloner</option>}
            {templates.map((t) => (
              <option key={t.id} value={t.id}>
                {t.name}
              </option>
            ))}
          </select>
        </label>
        <input
          type="search"
          className={styles.search}
          placeholder="Søg bygningsdel…"
          aria-label="Søg"
          value={filters.search}
          onChange={(e) => onFilters({ ...filters, search: e.target.value })}
        />
        <select
          className={styles.select}
          aria-label="Rum"
          value={filters.roomId === null ? '' : String(filters.roomId)}
          onChange={(e) =>
            onFilters({
              ...filters,
              roomId: e.target.value === '' ? null : Number(e.target.value),
            })
          }
        >
          <option value="">Alle rum</option>
          {rooms.map((r) => (
            <option key={r.id} value={r.id}>
              {r.name}
            </option>
          ))}
        </select>
        <select
          className={styles.select}
          aria-label="Miljøstatus"
          value={filters.env ?? ''}
          onChange={(e) =>
            onFilters({
              ...filters,
              env: e.target.value === '' ? null : (e.target.value as EnvFilter),
            })
          }
        >
          <option value="">Al miljøstatus</option>
          <option value="ren">Ren</option>
          <option value="afventer">Afventer prøve</option>
          <option value="forurenet">Forurenet</option>
        </select>
        <label className={styles.starFilter}>
          <input
            type="checkbox"
            className={styles.checkbox}
            checked={filters.starred}
            onChange={(e) => onFilters({ ...filters, starred: e.target.checked })}
          />
          Kun vigtige ★
        </label>
        {canEdit && (
          <div className={styles.toolActions}>
            <button type="button" className={styles.btnGhost} onClick={onAddResource}>
              + Tilføj ressource
            </button>
            <button type="button" className={styles.btnGhost} onClick={onAddColumn} disabled={templateId === null}>
              + Tilføj kolonne
            </button>
          </div>
        )}
      </div>

      <div className={styles.keyBar} aria-label="Tastaturgenveje">
        <Kbd>↑</Kbd>
        <Kbd>↓</Kbd> naviger · <Kbd>→</Kbd>
        <Kbd>←</Kbd> fold ud/ind · <Kbd>Enter</Kbd> åbn redigering · <Kbd>G</Kbd> godkend ·{' '}
        <Kbd>A</Kbd> afvis · <Kbd>V</Kbd> vigtig · <Kbd>S</Kbd> segmentér · <Kbd>1</Kbd>–<Kbd>{EVIDENCE_LAST_KEY}</Kbd> evidens ·{' '}
        <Kbd>Esc</Kbd> tilbage
      </div>

      <div
        ref={tableRef}
        className={styles.wrap}
        tabIndex={0}
        aria-label="Kortlægningstabel — brug piletaster"
      >
        <table className={styles.table}>
          <thead>
            <tr>
              <th className={styles.stickyStart}>Betegnelse</th>
              {columns.map((c) => (
                <th key={c.key.id}>
                  <span className={styles.headLabel}>
                    {c.key.label}
                    {c.typeScoped && <TypeMark />}
                  </span>
                </th>
              ))}
              <th className={styles.stickyEnd}>Status</th>
            </tr>
          </thead>
          <tbody>
            {rows.length === 0 ? (
              <tr>
                <td className={styles.emptyRow} colSpan={columns.length + 2}>
                  {emptyRowText(tab, counts[tab])}
                </td>
              </tr>
            ) : (
              rows.map((row) => {
                const type = typeById.get(row.typeId);
                if (!type) return null;
                const selected = sameSelection(row, selection);

                if (row.kind === 'type') {
                  const isOpen = open.has(type.id);
                  return (
                    <tr
                      key={`type-${type.id}`}
                      className={styles.typeRow}
                      aria-selected={selected}
                      onClick={(e) => {
                        const action = typeRowClick(selected, e.detail);
                        if (action === 'select') {
                          cancelToggle();
                          onSelect({ typeId: type.id, partCode: null });
                        } else if (action === 'toggle') {
                          cancelToggle();
                          toggleTimer.current = setTimeout(() => {
                            toggleTimer.current = null;
                            onToggle(type.id);
                          }, TOGGLE_DELAY_MS);
                        } else {
                          cancelToggle();
                        }
                      }}
                      onDoubleClick={() => {
                        cancelToggle();
                        onOpenDialog({ typeId: type.id, partCode: null });
                      }}
                    >
                      <td className={styles.stickyStart}>
                        <div className={styles.nameCell}>
                          <button
                            type="button"
                            className={styles.chevron}
                            data-open={isOpen || undefined}
                            aria-expanded={isOpen}
                            aria-label={isOpen ? 'Fold ind' : 'Fold ud'}
                            onClick={(e) => {
                              e.stopPropagation();
                              onToggle(type.id);
                            }}
                          >
                            ▸
                          </button>
                          <RowThumb frameId={rowThumbFrame(type.parts, photos)} />
                          {type.starred && (
                            <span className={styles.star} role="img" aria-label="Vigtig" title="Vigtig">
                              ★
                            </span>
                          )}
                          <span className={styles.name}>{type.name}</span>
                          <span className={styles.faint}>
                            {type.parts.length === 1 ? '1 del' : `${type.parts.length} dele`}
                          </span>
                        </div>
                      </td>
                      <Cells
                        row={row}
                        type={type}
                        columns={columns}
                        resources={resources}
                        selected={selected}
                        onCellCommit={onCellCommit}
                        onInvalid={onInvalid}
                        home={tableRef}
                      />
                      <td className={styles.stickyEnd}>
                        {type.review_status === 'approved' ? (
                          <Pill tone="good">Godkendt ✓</Pill>
                        ) : type.review_status === 'rejected' ? (
                          <Pill tone="wait">Afvist</Pill>
                        ) : (
                          <ConfidenceBar percent={confidencePercent(type.confidence)} />
                        )}
                      </td>
                    </tr>
                  );
                }

                const part = type.parts.find((p) => p.code === row.partCode);
                if (!part) return null;
                return (
                  <tr
                    key={`part-${part.code}`}
                    className={styles.partRow}
                    aria-selected={selected}
                    onClick={() => {
                      cancelToggle();
                      onSelect({ typeId: type.id, partCode: part.code });
                    }}
                    onDoubleClick={() =>
                      onOpenDialog({ typeId: type.id, partCode: part.code })
                    }
                  >
                    <td className={`${styles.stickyStart} ${styles.partTd}`}>
                      <div className={styles.partCell}>
                        <RowThumb frameId={rowThumbFrame([part], photos)} />
                        {part.starred && (
                          <span className={styles.star} role="img" aria-label="Vigtig" title="Vigtig">
                            ★
                          </span>
                        )}
                        {partRowLabel(part, partDesignation(resources.get(part.code), catalogue))}
                        {isManual(part) && (
                          <Pill variant="outline" title="Tilføjet manuelt — ikke fra scanningen">
                            Manuel
                          </Pill>
                        )}
                        {part.note && (
                          <span
                            className={styles.noteMark}
                            role="img"
                            aria-label={`Har note: ${part.note}`}
                            title={part.note}
                          >
                            ✎
                          </span>
                        )}
                        {part.orphaned && (
                          <Pill
                            tone="warn"
                            title="Instansen findes ikke længere — gennemgå eller flyt delen"
                          >
                            forældet
                          </Pill>
                        )}
                      </div>
                    </td>
                    <Cells
                      row={row}
                      type={type}
                      columns={columns}
                      resources={resources}
                      selected={selected}
                      onCellCommit={onCellCommit}
                      onInvalid={onInvalid}
                      home={tableRef}
                    />
                    <td className={`${styles.stickyEnd} ${styles.faint}`}>{photoCountText(photos?.[part.code])}</td>
                  </tr>
                );
              })
            )}
          </tbody>
        </table>
      </div>
    </section>
  );
}
