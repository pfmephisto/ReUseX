// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect } from 'react';

import type { SurveyType } from '../../api/types';
import {
  flattenRows,
  sameSelection,
  type EnvFilter,
  type Filters,
  type Selection,
  type Tab,
} from '../../kortlaegning/model';
import {
  confidencePercent,
  ENV_LABEL,
  ENV_TONE,
  formatQuantity,
  formatTonnes,
  TREATMENT_LABEL,
} from '../../kortlaegning/vocab';
import { ConfidenceBar } from '../ConfidenceBar';
import { Kbd } from '../Kbd';
import { Pill } from '../Pill';
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
  /** Table-wrap keyboard. */
  onKeyDown: (e: React.KeyboardEvent) => void;
  tableRef: React.RefObject<HTMLDivElement | null>;
}

const TABS: { id: Tab; label: string }[] = [
  { id: 'queue', label: 'Til gennemsyn' },
  { id: 'approved', label: 'Godkendt' },
  { id: 'all', label: 'Alle' },
];

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
  } = props;

  const rows = flattenRows(types, open);
  const typeById = new Map(types.map((t) => [t.id, t]));

  // Keep the selected row in view as selection moves by keyboard.
  useEffect(() => {
    const el = tableRef.current?.querySelector('[aria-selected="true"]');
    el?.scrollIntoView({ block: 'nearest' });
  }, [selection, tableRef]);

  return (
    <section className={styles.panel}>
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
        onKeyDown={onKeyDown}
      >
        <table className={styles.table}>
          <thead>
            <tr>
              <th>Betegnelse</th>
              <th>Mængde</th>
              <th>EAK</th>
              <th>BIM7AA</th>
              <th>Behandling</th>
              <th>Miljø</th>
              <th>Status</th>
            </tr>
          </thead>
          <tbody>
            {rows.length === 0 ? (
              <tr>
                <td className={styles.emptyRow} colSpan={7}>
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
                      onClick={() => onSelect({ typeId: type.id, partCode: null })}
                      onDoubleClick={() => onOpenDialog({ typeId: type.id, partCode: null })}
                    >
                      <td className={styles.nameCell}>
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
                          <span className={styles.star} aria-hidden="true">
                            ★
                          </span>
                        )}
                        <span className={styles.name}>{type.name}</span>
                        <span className={styles.faint}>{type.parts.length} dele</span>
                      </td>
                      <td>
                        {formatQuantity(type.quantity, type.unit)}{' '}
                        <span className={styles.faint}>{formatTonnes(type.mass_t)}</span>
                      </td>
                      <td className="mono">{type.eak_code}</td>
                      <td>{type.bim7aa_code}</td>
                      <td>
                        <Pill treatment={type.treatment}>{TREATMENT_LABEL[type.treatment]}</Pill>
                      </td>
                      <td>
                        <Pill tone={ENV_TONE[type.environment_status]}>
                          {ENV_LABEL[type.environment_status]}
                        </Pill>
                      </td>
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
                    onClick={() => onSelect({ typeId: type.id, partCode: part.code })}
                    onDoubleClick={() =>
                      onOpenDialog({ typeId: type.id, partCode: part.code })
                    }
                  >
                    <td className={styles.partCell}>
                      {part.code} · {part.room_name}
                      {part.orphaned && (
                        <Pill
                          tone="warn"
                          title="Instansen findes ikke længere — gennemgå eller flyt delen"
                        >
                          forældet
                        </Pill>
                      )}
                    </td>
                    <td>{formatQuantity(part.quantity, type.unit)}</td>
                    <td className="mono">{type.eak_code}</td>
                    <td></td>
                    <td></td>
                    <td></td>
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
