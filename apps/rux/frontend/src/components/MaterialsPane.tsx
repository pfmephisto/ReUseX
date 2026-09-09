// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useMemo, useState } from 'react';

import { api } from '../api/client';
import type { MaterialInfo } from '../api/types';
import { useAsync } from '../app/useAsync';
import { formatCount, formatGuidShort, formatText } from '../data/format';
import {
  type PropertyRow,
  applyPropertyPatch,
  diffProperties,
  duplicateRowKeys,
  isEmptyPatch,
  propertyIssues,
  propertyRows,
  rowsToDraft,
} from '../data/patch';
import { describeWriteFailure, type WriteFailure } from '../data/writeState';
import { DataTable, type Column } from './DataTable';
import { EmptyState } from './EmptyState';
import { ErrorBanner } from './ErrorBanner';
import { Spinner } from './Spinner';
import { WriteBanner } from './WriteBanner';
import styles from './MaterialsPane.module.css';

/**
 * Material passports, with a property editor.
 *
 * The editor sends `PATCH /materials/{guid}` with only the properties that
 * changed. That sparseness is not an optimisation: a passport imported from
 * MaterialEPAS carries fields this form never renders, and a PUT of the whole
 * property set would delete every one of them the first time anyone corrected a
 * typo. Deleting is expressed as `null`, which is why "clear" is a distinct
 * action from emptying the value box — `""` stores an empty property, `null`
 * removes it, and the server does exactly what it is told.
 */
export interface MaterialsPaneProps {
  selected: string | null;
  onSelect: (guid: string | null) => void;
}

const COLUMNS: Column<MaterialInfo>[] = [
  {
    key: 'guid',
    header: 'GUID',
    render: (row) => <span title={row.guid}>{formatGuidShort(row.guid)}</span>,
  },
  { key: 'id', header: 'Id', render: (row) => formatText(row.id) },
  {
    key: 'property_count',
    header: 'Properties',
    numeric: true,
    render: (row) => formatCount(row.property_count),
  },
  { key: 'version_number', header: 'Version', render: (row) => formatText(row.version_number) },
  { key: 'created_at', header: 'Created', render: (row) => formatText(row.created_at) },
];

export function MaterialsPane({ selected, onSelect }: MaterialsPaneProps) {
  const materials = useAsync((signal) => api.materials(signal), []);

  if (materials.error) {
    return (
      <ErrorBanner
        error={materials.error}
        onRetry={materials.reload}
        context="material passports"
      />
    );
  }
  if (!materials.data) return <Spinner label="Reading passports…" />;

  return (
    <div className={styles.pane}>
      <div className={styles.list}>
        <DataTable
          columns={COLUMNS}
          rows={materials.data}
          rowKey={(row) => row.guid}
          onRowClick={(row) => onSelect(row.guid)}
          empty={
            <EmptyState
              title="No material passports"
              detail="`rux import materialepas` brings passports in; `rux create materials` derives them from instances."
            />
          }
        />
      </div>
      {selected !== null && (
        <MaterialEditor
          guid={selected}
          onClose={() => onSelect(null)}
          onSaved={materials.reload}
        />
      )}
    </div>
  );
}

interface MaterialEditorProps {
  guid: string;
  onClose: () => void;
  /** The list shows `property_count`, so it goes stale on every save. */
  onSaved: () => void;
}

function MaterialEditor({ guid, onClose, onSaved }: MaterialEditorProps) {
  const material = useAsync((signal) => api.material(guid, signal), [guid]);

  /** Server truth, advanced optimistically and rolled back on failure. */
  const [stored, setStored] = useState<Record<string, string>>({});
  const [rows, setRows] = useState<PropertyRow[]>([]);
  const [loadedGuid, setLoadedGuid] = useState<string | null>(null);
  const [saving, setSaving] = useState(false);
  const [failure, setFailure] = useState<WriteFailure | null>(null);

  // Seeded as a render-time fold rather than an effect, so the first paint
  // already has the form populated instead of flashing empty rows.
  const properties = material.data?.properties;
  if (material.data && loadedGuid !== guid) {
    const next = properties ?? {};
    setStored(next);
    setRows(propertyRows(next));
    setLoadedGuid(guid);
    setFailure(null);
  }

  const draft = useMemo(() => rowsToDraft(rows), [rows]);
  const duplicates = useMemo(() => duplicateRowKeys(rows), [rows]);
  const issues = useMemo(() => propertyIssues(draft), [draft]);
  const patch = useMemo(() => diffProperties(stored, draft), [stored, draft]);
  const dirty = !isEmptyPatch(patch);
  const blocked = duplicates.length > 0 || issues.length > 0;

  const save = useCallback(async () => {
    if (!dirty || blocked || saving) return;

    const rollback = stored;
    // Optimistic: apply the same semantics the server will, so the pane does
    // not flicker between two truths while the request is in flight.
    setStored(applyPropertyPatch(stored, patch));
    setSaving(true);
    setFailure(null);

    try {
      const updated = await api.patchMaterial(guid, patch);
      const settled = updated.properties ?? {};
      setStored(settled);
      setRows(propertyRows(settled));
      onSaved();
    } catch (error) {
      // Nothing was written on 409 or 503 — both are documented as leaving the
      // project untouched — so putting the pre-patch map back is exact, not a
      // guess. The user's rows are deliberately left as they typed them.
      setStored(rollback);
      setFailure(
        describeWriteFailure(
          error instanceof Error ? error : new Error(String(error)),
          'this passport',
        ),
      );
    } finally {
      setSaving(false);
    }
  }, [dirty, blocked, saving, stored, patch, guid, onSaved]);

  const reset = useCallback(() => {
    setRows(propertyRows(stored));
    setFailure(null);
  }, [stored]);

  return (
    <aside className={styles.detail} aria-label={`Material passport ${guid}`}>
      <header className={styles.detailHead}>
        <h3 className={styles.detailTitle}>Passport</h3>
        <button type="button" className={styles.close} onClick={onClose}>
          Close
        </button>
      </header>
      <p className={`${styles.guid} mono`}>{guid}</p>

      {material.error ? (
        <ErrorBanner
          error={material.error}
          onRetry={material.reload}
          context="this passport"
        />
      ) : !material.data ? (
        <Spinner label="Reading passport…" />
      ) : (
        <>
          {failure && (
            <WriteBanner
              failure={failure}
              onRetry={failure.retryable ? save : undefined}
              onDismiss={() => setFailure(null)}
            />
          )}

          <div className={styles.rows}>
            {rows.length === 0 && (
              <p className={styles.note}>This passport stores no properties yet.</p>
            )}
            {rows.map((row, index) => (
              <div
                key={row.id}
                className={`${styles.row} ${duplicates.includes(row.key.trim()) ? styles.rowInvalid : ''}`}
              >
                <input
                  className={`${styles.key} ${row.added ? '' : styles.readonly}`}
                  value={row.key}
                  readOnly={!row.added}
                  aria-label={row.added ? 'Property name' : `Property ${row.key}`}
                  placeholder="Property name"
                  onChange={(event) =>
                    setRows((current) =>
                      current.map((item, at) =>
                        at === index ? { ...item, key: event.target.value } : item,
                      ),
                    )
                  }
                />
                <input
                  className={styles.value}
                  value={row.value}
                  aria-label={`Value of ${row.key || 'the new property'}`}
                  placeholder="Value"
                  onChange={(event) =>
                    setRows((current) =>
                      current.map((item, at) =>
                        at === index ? { ...item, value: event.target.value } : item,
                      ),
                    )
                  }
                />
                <button
                  type="button"
                  className={styles.clear}
                  // Removing the row is what sends `null`; emptying the value
                  // box would store an empty property instead.
                  title="Remove this property (sends null)"
                  onClick={() =>
                    setRows((current) => current.filter((_, at) => at !== index))
                  }
                >
                  Clear
                </button>
              </div>
            ))}
          </div>

          {duplicates.length > 0 && (
            <p className={styles.invalid}>
              Two rows share the name {duplicates.map((key) => `"${key}"`).join(', ')} — one
              would silently overwrite the other.
            </p>
          )}
          {issues.map((issue, index) => (
            // Issues are positional findings about a blank name; there is no id.
            // eslint-disable-next-line react/no-array-index-key
            <p key={index} className={styles.invalid}>
              {issue.message}
            </p>
          ))}

          <div className={styles.actions}>
            <button
              type="button"
              className={styles.add}
              onClick={() =>
                setRows((current) => [
                  ...current,
                  // A counter would collide after a remove; the timestamp is
                  // only ever compared for equality, never read.
                  { id: `new:${Date.now()}:${current.length}`, key: '', value: '', added: true },
                ])
              }
            >
              Add property
            </button>
            <button
              type="button"
              className={styles.reset}
              disabled={!dirty || saving}
              onClick={reset}
            >
              Revert
            </button>
            <button
              type="button"
              className={styles.save}
              disabled={!dirty || blocked || saving}
              onClick={save}
            >
              {saving ? 'Saving…' : 'Save'}
            </button>
          </div>

          <p className={styles.note}>
            {dirty
              ? `${formatCount(Object.keys(patch).length)} changed ${
                  Object.keys(patch).length === 1 ? 'property' : 'properties'
                } will be sent. Everything else is left untouched.`
              : 'No changes. Only edited properties are ever sent.'}
          </p>
        </>
      )}
    </aside>
  );
}
