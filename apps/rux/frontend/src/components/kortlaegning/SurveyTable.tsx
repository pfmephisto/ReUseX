// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef } from 'react';
import type { RefObject } from 'react';

import type { SurveyType, Template } from '../../api/types';
import {
  flattenRows,
  partLabel,
  sameSelection,
  type EnvFilter,
  type Filters,
  type Row,
  type Selection,
  type Tab,
} from '../../kortlaegning/model';
import { cellModel, isManual, type ResourceColumn, type ResourceIndex } from '../../kortlaegning/resources';
import { confidencePercent, formatQuantity, formatTonnes } from '../../kortlaegning/vocab';
import { ConfidenceBar } from '../ConfidenceBar';
import { Kbd } from '../Kbd';
import { Pill } from '../Pill';
import { ResourceCell } from './ResourceCell';
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
  templates: Template[];
  templateId: number | null;
  onTemplate: (id: number) => void;
  onCellCommit: (code: string, keyId: string, value: string | null) => void;
  onInvalid: (label: string) => void;
  onAddResource: () => void;
  onAddColumn: () => void;
}

const TABS: { id: Tab; label: string }[] = [
  { id: 'queue', label: 'Til gennemsyn' },
  { id: 'approved', label: 'Godkendt' },
  { id: 'all', label: 'Alle' },
];

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
              editing={selected && target !== null}
              onCommit={(v) => {
                if (target !== null) props.onCellCommit(target, column.key.id, v);
              }}
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
    templates,
    templateId,
    onTemplate,
    onCellCommit,
    onInvalid,
    onAddResource,
    onAddColumn,
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
        <div className={styles.toolActions}>
          <button type="button" className={styles.btnGhost} onClick={onAddResource}>
            + Tilføj ressource
          </button>
          <button type="button" className={styles.btnGhost} onClick={onAddColumn} disabled={templateId === null}>
            + Tilføj kolonne
          </button>
        </div>
      </div>

      <div className={styles.keyBar} aria-label="Tastaturgenveje">
        <Kbd>↑</Kbd>
        <Kbd>↓</Kbd> naviger · <Kbd>→</Kbd>
        <Kbd>←</Kbd> fold ud/ind · <Kbd>Enter</Kbd> åbn redigering · <Kbd>G</Kbd> godkend ·{' '}
        <Kbd>A</Kbd> afvis · <Kbd>V</Kbd> vigtig · <Kbd>1</Kbd>–<Kbd>4</Kbd> evidens ·{' '}
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
              <th>Betegnelse</th>
              {columns.map((c) => (
                <th key={c.key.id}>
                  <span className={styles.headLabel}>
                    {c.key.label}
                    {c.typeScoped && (
                      <span className={styles.typeMark} title="Gælder alle dele af typen">
                        type
                      </span>
                    )}
                  </span>
                </th>
              ))}
              <th>Status</th>
            </tr>
          </thead>
          <tbody>
            {rows.length === 0 ? (
              <tr>
                <td className={styles.emptyRow} colSpan={columns.length + 2}>
                  Ingen rækker matcher filtrene.
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
                      <td>
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
                      <td>
                        {type.review_status === 'approved' ? (
                          <Pill tone="good">Godkendt ✓</Pill>
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
                    <td className={styles.partTd}>
                      <div className={styles.partCell}>
                        {part.starred && (
                          <span className={styles.star} role="img" aria-label="Vigtig" title="Vigtig">
                            ★
                          </span>
                        )}
                        {partLabel(part)}
                        {isManual(part) && (
                          <Pill tone="accent" title="Tilføjet manuelt — ikke fra scanningen">
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
                    <td className={styles.faint}>—</td>
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
