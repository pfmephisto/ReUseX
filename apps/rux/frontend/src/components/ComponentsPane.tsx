// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useMemo } from 'react';

import { api } from '../api/client';
import type { ComponentDetail, ComponentInfo } from '../api/types';
import { useAsync } from '../app/useAsync';
import {
  ABSENT,
  formatArea,
  formatConfidence,
  formatCount,
  formatGuidShort,
  formatParent,
  formatText,
} from '../data/format';
import { DataTable, type Column } from './DataTable';
import { EmptyState } from './EmptyState';
import { ErrorBanner } from './ErrorBanner';
import { Spinner } from './Spinner';
import styles from './ComponentsPane.module.css';

/**
 * The building-component inventory.
 *
 * Read-only. Components are written by the reconstruction stages, and a GUI
 * that let a user retype a wall's type without re-running anything would be
 * offering to make the table disagree with the geometry it summarises.
 *
 * Two of the columns — `area` and `source_instance_guid` — are derived by the
 * server on read and are genuinely optional: a component with fewer than three
 * boundary vertices has no area, and one that was created by hand has no source
 * instance. Both render as an em-dash when absent, never as `NaN` or the empty
 * string, which is the whole reason `data/format.ts` exists as a tested module.
 *
 * There is deliberately **no room column.** ReUseX does not associate a
 * building component with a room; `parent_id` is a parent *component*. Labelling
 * it "room" would be a lie this table then displays several hundred times.
 */
export interface ComponentsPaneProps {
  /** Selected component name, carried in the URL by the page. */
  selected: string | null;
  onSelect: (name: string | null) => void;
  /** Type filter, also in the URL. */
  type: string | null;
  onTypeChange: (type: string | null) => void;
}

const COLUMNS: Column<ComponentInfo>[] = [
  { key: 'name', header: 'Component', render: (row) => row.name },
  { key: 'type', header: 'Type', render: (row) => formatText(row.type) },
  { key: 'area', header: 'Area', numeric: true, render: (row) => formatArea(row.area) },
  {
    key: 'vertex_count',
    header: 'Vertices',
    numeric: true,
    render: (row) => formatCount(row.vertex_count),
  },
  {
    key: 'confidence',
    header: 'Confidence',
    numeric: true,
    render: (row) => formatConfidence(row.confidence),
  },
  {
    key: 'source_instance_guid',
    header: 'From instance',
    render: (row) => (
      <span title={row.source_instance_guid ?? undefined}>
        {formatGuidShort(row.source_instance_guid)}
      </span>
    ),
  },
];

export function ComponentsPane({
  selected,
  onSelect,
  type,
  onTypeChange,
}: ComponentsPaneProps) {
  // The type filter is served by the API (`GET /components?type=`), so it is a
  // dependency of the request rather than a client-side predicate — the same
  // shape as the frame browser's segmented filter.
  const components = useAsync(
    (signal) => api.components(type ?? undefined, signal),
    [type],
  );

  // Offered types come from an unfiltered read, so choosing "Wall" does not
  // reduce the menu to just "Wall" and strand the user there.
  const all = useAsync((signal) => api.components(undefined, signal), []);
  const types = useMemo(() => {
    const seen = new Set((all.data ?? []).map((row) => row.type).filter(Boolean));
    return [...seen].sort();
  }, [all.data]);

  if (components.error) {
    return (
      <ErrorBanner
        error={components.error}
        onRetry={components.reload}
        context="building components"
      />
    );
  }
  if (!components.data) return <Spinner label="Reading components…" />;

  return (
    <div className={styles.pane}>
      <div className={styles.list}>
        <div className={styles.toolbar}>
          <label className={styles.filterLabel} htmlFor="component-type">
            Type
          </label>
          <select
            id="component-type"
            className={styles.select}
            value={type ?? ''}
            onChange={(event) => onTypeChange(event.target.value || null)}
          >
            <option value="">All types</option>
            {types.map((option) => (
              <option key={option} value={option}>
                {option}
              </option>
            ))}
          </select>
          <span className={styles.count}>
            {formatCount(components.data.length)} shown
          </span>
        </div>

        <DataTable
          columns={COLUMNS}
          rows={components.data}
          rowKey={(row) => row.name}
          onRowClick={(row) => onSelect(row.name)}
          empty={
            <EmptyState
              title={type ? `No ${type} components` : 'No building components'}
              detail="Components appear once the reconstruction stages have classified surfaces."
            />
          }
        />
      </div>

      {selected !== null && (
        <ComponentDetailPane name={selected} onClose={() => onSelect(null)} />
      )}
    </div>
  );
}

function ComponentDetailPane({ name, onClose }: { name: string; onClose: () => void }) {
  const detail = useAsync((signal) => api.component(name, signal), [name]);

  return (
    <aside className={styles.detail} aria-label={`Component ${name}`}>
      <header className={styles.detailHead}>
        <h3 className={styles.detailTitle}>{name}</h3>
        <button type="button" className={styles.close} onClick={onClose}>
          Close
        </button>
      </header>
      {detail.error ? (
        <ErrorBanner
          error={detail.error}
          onRetry={detail.reload}
          context={`component ${name}`}
        />
      ) : !detail.data ? (
        <Spinner label="Reading component…" />
      ) : (
        <ComponentFacts detail={detail.data} />
      )}
    </aside>
  );
}

function ComponentFacts({ detail }: { detail: ComponentDetail }) {
  const plane = detail.plane;

  return (
    <dl className={styles.facts}>
      <Fact label="Type" value={formatText(detail.type)} />
      <Fact label="GUID" value={formatText(detail.guid)} mono />
      <Fact label="Area" value={formatArea(detail.area)} />
      <Fact label="Vertices" value={formatCount(detail.vertex_count)} />
      <Fact label="Confidence" value={formatConfidence(detail.confidence)} />
      {/* Parent *component*. Not a room — see the note on this file. */}
      <Fact label="Parent" value={formatParent(detail.parent_id)} />
      <Fact
        label="From instance"
        value={formatText(detail.source_instance_guid)}
        mono
      />
      <Fact
        label="Plane"
        value={
          plane && plane.length === 4
            ? plane.map((value) => value.toFixed(4)).join(', ')
            : ABSENT
        }
        mono
      />
      <Fact label="Metadata" value={formatText(detail.metadata)} mono />
      <Fact label="Notes" value={formatText(detail.notes)} />
    </dl>
  );
}

function Fact({ label, value, mono }: { label: string; value: string; mono?: boolean }) {
  return (
    <div className={styles.fact}>
      <dt className={styles.factLabel}>{label}</dt>
      <dd className={`${styles.factValue} ${mono ? 'mono' : ''}`}>{value}</dd>
    </div>
  );
}
