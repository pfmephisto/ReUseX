// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { PropertyDefinition } from '../api/types';
import type {
  BooleanFilter,
  ColumnFilterValue,
  DateFilter,
  NumberFilter,
  SelectFilter,
} from './columnFilterHelpers';
import styles from './MaterialTable.module.css';

interface ColumnFilterCellProps {
  colDef: PropertyDefinition;
  value: ColumnFilterValue | undefined;
  onChange: (val: ColumnFilterValue) => void;
}

/** Type-aware filter control rendered in the filter row below column headers. */
export function ColumnFilterCell({ colDef, value, onChange }: ColumnFilterCellProps) {
  // Prevent header click from opening the column menu when interacting with filters.
  const stopProp = (e: React.SyntheticEvent) => e.stopPropagation();

  if (colDef.type === 'text' || colDef.type === 'url') {
    return (
      <input
        type="search"
        className={styles.filterInput}
        placeholder="Contains…"
        value={(value as string | undefined) ?? ''}
        onChange={(e) => onChange(e.target.value)}
        aria-label={`Filter ${colDef.name}`}
        onClick={stopProp}
      />
    );
  }

  if (colDef.type === 'number') {
    const nf = (value as NumberFilter | undefined) ?? {};
    return (
      <div className={styles.filterRange} onClick={stopProp}>
        <input
          type="number"
          className={styles.filterInput}
          placeholder="Min"
          value={nf.min ?? ''}
          onChange={(e) =>
            onChange({
              ...nf,
              min: e.target.value === '' ? undefined : parseFloat(e.target.value),
            })
          }
          aria-label={`Minimum for ${colDef.name}`}
        />
        <input
          type="number"
          className={styles.filterInput}
          placeholder="Max"
          value={nf.max ?? ''}
          onChange={(e) =>
            onChange({
              ...nf,
              max: e.target.value === '' ? undefined : parseFloat(e.target.value),
            })
          }
          aria-label={`Maximum for ${colDef.name}`}
        />
      </div>
    );
  }

  if (colDef.type === 'date') {
    const df = (value as DateFilter | undefined) ?? {};
    return (
      <div className={styles.filterRange} onClick={stopProp}>
        <input
          type="date"
          className={styles.filterInput}
          value={df.from ?? ''}
          onChange={(e) => onChange({ ...df, from: e.target.value || undefined })}
          aria-label={`From date for ${colDef.name}`}
        />
        <input
          type="date"
          className={styles.filterInput}
          value={df.to ?? ''}
          onChange={(e) => onChange({ ...df, to: e.target.value || undefined })}
          aria-label={`To date for ${colDef.name}`}
        />
      </div>
    );
  }

  if (colDef.type === 'boolean') {
    const bf = (value as BooleanFilter | undefined) ?? null;
    return (
      <div className={styles.filterBoolGroup} onClick={stopProp}>
        {([null, 'true', 'false'] as BooleanFilter[]).map((opt) => (
          <button
            key={String(opt)}
            type="button"
            className={`${styles.filterBoolBtn}${bf === opt ? ` ${styles.filterBoolActive}` : ''}`}
            onClick={() => onChange(opt)}
            aria-pressed={bf === opt}
          >
            {opt === null ? 'Any' : opt === 'true' ? 'Yes' : 'No'}
          </button>
        ))}
      </div>
    );
  }

  if (colDef.type === 'select' || colDef.type === 'multiselect') {
    const sf = (value as SelectFilter | undefined) ?? [];
    const options = colDef.options ?? [];
    if (options.length === 0) return null;
    return (
      <div className={styles.filterCheckList} onClick={stopProp}>
        {options.map((opt) => (
          <label key={opt} className={styles.filterCheckLabel}>
            <input
              type="checkbox"
              checked={sf.includes(opt)}
              onChange={(e) => {
                const next = e.target.checked ? [...sf, opt] : sf.filter((v) => v !== opt);
                onChange(next);
              }}
            />
            <span>{opt}</span>
          </label>
        ))}
      </div>
    );
  }

  return null;
}
