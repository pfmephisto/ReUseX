// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState } from 'react';
import { Link } from 'react-router-dom';

import { ApiRequestError, describeFailure } from '../../api/client';
import type { Template } from '../../api/types';
import { SKABELONER_PATH } from '../../app/links';
import {
  csvCountLine,
  csvDownloadErrorMessage,
  csvFilename,
  DELIMITER_OPTIONS,
  downloadState,
  ENCODING_OPTIONS,
  HEADER_OPTIONS,
  readCsvOptions,
  type CsvOptions,
} from '../../rapport/csvOptions';
import { TemplateSelect } from './TemplateSelect';
import styles from './DataExportPanel.module.css';

export interface DataExportPanelProps {
  templates: Template[];
  selectedId: number | null;
  onSelect: (id: number | null) => void;
  onCsvChange: (patch: Partial<CsvOptions>) => void;
  /** A CSV-option write is queued or in flight (R12). */
  writing: boolean;
  csvUrl: (id: number) => string;
  /** A download failed: a non-OK response, or the fetch itself. Shown as the page's toast. */
  onDownloadError: (message: string) => void;
}

/** Saves `blob` as `filename` via a throwaway, revoked object URL. */
function saveBlob(blob: Blob, filename: string): void {
  const url = URL.createObjectURL(blob);
  try {
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    a.remove();
  } finally {
    URL.revokeObjectURL(url);
  }
}

/**
 * Data-eksport (spec §6.3): pick a template, set its CSV options (saved back
 * to the template), download the backend-built CSV. A plain `<a href
 * download>` would save a non-OK response's JSON error body as if it were
 * the file, so this fetches it instead: on success it saves a Blob under the
 * server's `Content-Disposition` filename, on failure it reports the error
 * through `onDownloadError` and saves nothing.
 */
export function DataExportPanel({
  templates,
  selectedId,
  onSelect,
  onCsvChange,
  writing,
  csvUrl,
  onDownloadError,
}: DataExportPanelProps) {
  const t = templates.find((x) => x.id === selectedId) ?? null;
  const opts = readCsvOptions(t?.csv);
  const [downloading, setDownloading] = useState(false);
  const dl = downloadState(t, writing);

  const download = async () => {
    if (!t) return;
    setDownloading(true);
    try {
      const url = csvUrl(t.id);
      const response = await fetch(url);
      if (!response.ok) {
        throw new ApiRequestError(response.status, await describeFailure(response), url);
      }
      const blob = await response.blob();
      saveBlob(blob, csvFilename(response.headers.get('Content-Disposition')));
    } catch (cause) {
      onDownloadError(csvDownloadErrorMessage(cause));
    } finally {
      setDownloading(false);
    }
  };

  return (
    <section className={styles.panel} aria-labelledby="rapport-dataeksport">
      <h3 id="rapport-dataeksport" className={styles.heading}>
        Data-eksport
      </h3>
      {templates.length === 0 ? (
        <p className={styles.muted}>
          Der er ingen skabeloner i projektet. Opret eller gendan dem under{' '}
          <Link className={styles.crossLink} to={SKABELONER_PATH}>
            Skabeloner
          </Link>
          .
        </p>
      ) : (
        <>
          <div className={styles.grid}>
            <TemplateSelect
              id="dataeksport-template"
              label="Skabelon"
              templates={templates}
              value={selectedId}
              onChange={onSelect}
            />
            <Choice
              label="Skilletegn"
              value={opts.delimiter}
              options={DELIMITER_OPTIONS}
              disabled={!t}
              onChange={(v) => onCsvChange({ delimiter: v })}
            />
            <Choice
              label="Tegnsæt"
              value={opts.encoding}
              options={ENCODING_OPTIONS}
              disabled={!t}
              onChange={(v) => onCsvChange({ encoding: v })}
            />
            <Choice
              label="Kolonneoverskrift"
              value={opts.header}
              options={HEADER_OPTIONS}
              disabled={!t}
              onChange={(v) => onCsvChange({ header: v })}
            />
          </div>
          <div className={styles.row}>
            <button type="button" className={styles.btnPrimary} disabled={!dl.enabled || downloading} onClick={download}>
              {downloading ? 'Henter…' : 'Download CSV'}
            </button>
            <span className={styles.muted} role="status">
              {dl.reason ?? (t ? csvCountLine(t.resolved_keys.length) : '')}
            </span>
            <Link className={styles.crossLink} to={SKABELONER_PATH}>
              Redigér skabeloner
            </Link>
          </div>
        </>
      )}
    </section>
  );
}

function Choice<T extends string>({
  label,
  value,
  options,
  disabled,
  onChange,
}: {
  label: string;
  value: T;
  options: readonly { value: T; label: string }[];
  disabled: boolean;
  onChange: (v: T) => void;
}) {
  return (
    <label className={styles.field}>
      <span className={styles.fieldLabel}>{label}</span>
      <select className={styles.input} value={value} disabled={disabled} onChange={(e) => onChange(e.target.value as T)}>
        {options.map((o) => (
          <option key={o.value} value={o.value}>
            {o.label}
          </option>
        ))}
      </select>
    </label>
  );
}
