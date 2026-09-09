// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useMemo, useState } from 'react';

import { api } from '../api/client';
import type { CloudInfo } from '../api/types';
import { useAsync } from '../app/useAsync';
import { formatCount } from '../data/format';
import {
  UNRENAMABLE_CLOUD,
  applyLabelPatch,
  diffLabels,
  isEmptyPatch,
  isLegendEditable,
  labelIssues,
} from '../data/patch';
import { describeWriteFailure, type WriteFailure } from '../data/writeState';
import { labelColorIndex, readLabelPalette } from '../viewport/labelColors';
import { EmptyState } from './EmptyState';
import { ErrorBanner } from './ErrorBanner';
import { Spinner } from './Spinner';
import { WriteBanner } from './WriteBanner';
import styles from './LabelsPane.module.css';

/**
 * The label legend editor.
 *
 * Renames existing classes — it cannot create them, and the server enforces
 * that: an id not already in the legend is a 400, because a name against an id
 * no point carries reads as data loss rather than as the typo it is. The form
 * therefore has no "add" affordance at all, which is the honest shape for an
 * operation the backing store does not support.
 *
 * ## Why `instances` is disabled *and* handled
 *
 * `create instances` writes names of the form `SM<class>-<id> (<n>p)`, and both
 * the v10 schema migration and `io/export_scene.cpp` parse the semantic class
 * back out of that string. It is a record wearing a caption's clothes. The
 * server answers 409, and the fields here are disabled — but the 409 is still
 * handled, because a future cloud could join that list and a UI whose only
 * defence is a hardcoded name would silently offer a corrupting edit.
 *
 * Swatches come from `viewport/labelColors.ts`, the same module the renderer
 * and `LabelLegend` read, so the colour beside a name here is by construction
 * the colour of those points in the viewport.
 */
export interface LabelsPaneProps {
  /** Selected Label cloud, carried in the URL by the page. */
  cloud: string | null;
  onCloudChange: (cloud: string | null) => void;
}

export function LabelsPane({ cloud, onCloudChange }: LabelsPaneProps) {
  const clouds = useAsync((signal) => api.clouds(signal), []);

  const labelClouds = useMemo(
    () => (clouds.data ?? []).filter((entry: CloudInfo) => entry.type === 'Label'),
    [clouds.data],
  );

  const active =
    cloud && labelClouds.some((entry) => entry.name === cloud)
      ? cloud
      : (labelClouds[0]?.name ?? null);

  if (clouds.error) {
    return (
      <ErrorBanner error={clouds.error} onRetry={clouds.reload} context="the cloud list" />
    );
  }
  if (!clouds.data) return <Spinner label="Reading clouds…" />;

  if (labelClouds.length === 0) {
    return (
      <EmptyState
        title="No label clouds"
        detail="`rux create planes`, `rooms` or `instances` produce the Label clouds whose legends are edited here."
      />
    );
  }

  return (
    <div className={styles.pane}>
      <div className={styles.toolbar}>
        <label className={styles.filterLabel} htmlFor="label-cloud">
          Cloud
        </label>
        <select
          id="label-cloud"
          className={styles.select}
          value={active ?? ''}
          onChange={(event) => onCloudChange(event.target.value || null)}
        >
          {labelClouds.map((entry) => (
            <option key={entry.name} value={entry.name}>
              {entry.name} ({formatCount(entry.point_count)} points)
            </option>
          ))}
        </select>
      </div>

      {active && <LegendEditor key={active} cloud={active} />}
    </div>
  );
}

function LegendEditor({ cloud }: { cloud: string }) {
  const legend = useAsync((signal) => api.cloudLabels(cloud, signal), [cloud]);

  /** Server truth, advanced optimistically and rolled back on failure. */
  const [stored, setStored] = useState<Record<string, string>>({});
  const [draft, setDraft] = useState<Record<string, string>>({});
  const [loaded, setLoaded] = useState(false);
  const [saving, setSaving] = useState(false);
  const [failure, setFailure] = useState<WriteFailure | null>(null);

  const palette = useMemo(() => readLabelPalette(), []);
  const editable = isLegendEditable(cloud);

  if (legend.data && !loaded) {
    setStored(legend.data);
    setDraft(legend.data);
    setLoaded(true);
  }

  const patch = useMemo(() => diffLabels(stored, draft), [stored, draft]);
  const issues = useMemo(() => labelIssues(stored, draft), [stored, draft]);
  const dirty = !isEmptyPatch(patch);

  const save = useCallback(async () => {
    if (!dirty || issues.length > 0 || saving || !editable) return;

    const rollback = stored;
    setStored(applyLabelPatch(stored, patch));
    setSaving(true);
    setFailure(null);

    try {
      const updated = await api.patchCloudLabels(cloud, patch);
      setStored(updated);
      setDraft(updated);
    } catch (error) {
      // 409 and 503 both leave the legend exactly as it was — the server
      // validates the whole patch before touching storage, precisely because
      // `save_label_definitions` replaces the cloud's map wholesale. So the
      // rollback restores the true state rather than approximating it.
      setStored(rollback);
      setFailure(
        describeWriteFailure(
          error instanceof Error ? error : new Error(String(error)),
          `the '${cloud}' legend`,
        ),
      );
    } finally {
      setSaving(false);
    }
  }, [dirty, issues.length, saving, editable, stored, patch, cloud]);

  const entries = useMemo(
    () =>
      Object.keys(stored)
        .map((id) => ({ id, numeric: Number(id) }))
        .filter((entry) => Number.isFinite(entry.numeric) && entry.numeric >= 1)
        .sort((a, b) => a.numeric - b.numeric),
    [stored],
  );

  if (legend.error) {
    return (
      <ErrorBanner
        error={legend.error}
        onRetry={legend.reload}
        context={`the '${cloud}' legend`}
      />
    );
  }
  if (!legend.data) return <Spinner label="Reading the legend…" />;

  if (entries.length === 0) {
    return (
      <EmptyState
        title={`'${cloud}' has no label definitions`}
        detail="A Label cloud carries names only once a stage has written them. Label 0 is unlabeled and never has a name."
      />
    );
  }

  return (
    <div className={styles.editor}>
      {!editable && (
        <p className={styles.refusal}>
          The <span className="mono">{UNRENAMABLE_CLOUD}</span> cloud's names encode the
          semantic class and instance id (<span className="mono">SM&lt;class&gt;-&lt;id&gt;
          (&lt;n&gt;p)</span>) and are parsed back by the schema migration and by the
          scene export. Renaming one would corrupt a record while looking like a caption
          edit, so the server refuses it and these fields are read-only.
        </p>
      )}

      {failure && (
        <WriteBanner
          failure={failure}
          onRetry={failure.retryable ? save : undefined}
          onDismiss={() => setFailure(null)}
        />
      )}

      <div className={styles.rows}>
        {entries.map((entry) => {
          const slot = labelColorIndex(entry.numeric, palette.colors.length);
          const changed = stored[entry.id] !== draft[entry.id];
          return (
            <div key={entry.id} className={styles.row}>
              <span
                className={styles.swatch}
                style={{ background: slot < 0 ? palette.unlabeled : palette.colors[slot] }}
                aria-hidden="true"
              />
              <span className={`${styles.id} mono`}>{entry.id}</span>
              <input
                className={`${styles.name} ${changed ? styles.changed : ''}`}
                value={draft[entry.id] ?? ''}
                readOnly={!editable}
                aria-label={`Name of label ${entry.id}`}
                onChange={(event) =>
                  setDraft((current) => ({ ...current, [entry.id]: event.target.value }))
                }
              />
            </div>
          );
        })}
      </div>

      {issues.map((issue) => (
        <p key={issue.id} className={styles.invalid}>
          Label {issue.id}: {issue.message}
        </p>
      ))}

      {editable && (
        <div className={styles.actions}>
          <button
            type="button"
            className={styles.reset}
            disabled={!dirty || saving}
            onClick={() => {
              setDraft(stored);
              setFailure(null);
            }}
          >
            Revert
          </button>
          <button
            type="button"
            className={styles.save}
            disabled={!dirty || issues.length > 0 || saving}
            onClick={save}
          >
            {saving ? 'Saving…' : 'Save'}
          </button>
        </div>
      )}

      <p className={styles.note}>
        {dirty
          ? `${formatCount(Object.keys(patch).length)} renamed. Only those ids are sent; every other class is left alone.`
          : 'Classes can be renamed, not created — an id that is not already in this legend is refused.'}
      </p>
    </div>
  );
}
