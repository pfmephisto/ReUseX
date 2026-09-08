// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { PipelineLogEntry } from '../api/types';
import { EmptyState } from './EmptyState';
import styles from './PipelineLogList.module.css';

export interface PipelineLogListProps {
  entries: PipelineLogEntry[];
}

/** Text label per status. The colour is a second channel, never the only one. */
const STATUS_LABEL: Record<PipelineLogEntry['status'], string> = {
  running: 'Running',
  success: 'Success',
  failed: 'Failed',
};

/**
 * Parse the stored timestamp for arithmetic only.
 *
 * `pipeline_log.started_at` defaults to sqlite's `datetime('now')`, which is UTC
 * written as `YYYY-MM-DD HH:MM:SS` with no zone marker — and the contract
 * declares the field as a bare string, so a client cannot assume more than that.
 * `Date.parse` would read it as *local* time. That does not matter for a
 * difference between two such stamps, which is all this is used for, as long as
 * both are pinned to the same offset; appending `Z` does that and also stops a
 * DST boundary between the two from inventing an hour of runtime.
 *
 * The displayed value stays the raw stored string. Re-rendering it in local time
 * would silently shift every entry by the viewer's offset and claim a precision
 * the contract does not grant.
 */
function epochMs(stored: string): number | null {
  const normalized = /^\d{4}-\d{2}-\d{2}[ T]\d{2}:\d{2}:\d{2}$/.test(stored)
    ? `${stored.replace(' ', 'T')}Z`
    : stored;
  const parsed = Date.parse(normalized);
  return Number.isNaN(parsed) ? null : parsed;
}

function formatDuration(entry: PipelineLogEntry): string | null {
  if (!entry.finished_at) return null; // empty means "still running"
  const from = epochMs(entry.started_at);
  const to = epochMs(entry.finished_at);
  if (from === null || to === null || to < from) return null;

  const seconds = Math.round((to - from) / 1000);
  if (seconds < 60) return `${seconds}s`;
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m ${seconds % 60}s`;
  return `${Math.floor(minutes / 60)}h ${minutes % 60}m`;
}

/**
 * Was this run started from the GUI?
 *
 * The contract says a `POST /jobs` run stores its `job_id` inside the stage
 * parameters, which is how durable history is joined back to a job after a
 * server restart. `parameters` is an opaque *string* though — written by
 * whatever ran the stage, including a hand-run `rux` — so it is parsed
 * defensively. An unparseable blob means "no marker", never a crash: a
 * malformed parameters column must not take out the whole history panel.
 */
function jobIdOf(parameters?: string): string | null {
  if (!parameters) return null;
  try {
    const parsed: unknown = JSON.parse(parameters);
    if (typeof parsed !== 'object' || parsed === null) return null;
    const jobId = (parsed as Record<string, unknown>).job_id;
    return typeof jobId === 'string' && jobId.length > 0 ? jobId : null;
  } catch {
    return null;
  }
}

export function PipelineLogList({ entries }: PipelineLogListProps) {
  if (entries.length === 0) {
    return (
      <EmptyState
        title="No stages have been run yet"
        detail="Every `rux` stage that touches this project records itself here, whether it was started from the GUI or the command line."
      />
    );
  }

  return (
    <ol className={styles.list}>
      {entries.map((entry) => {
        const duration = formatDuration(entry);
        const jobId = jobIdOf(entry.parameters);

        return (
          <li key={entry.id} className={`${styles.entry} ${styles[entry.status]}`}>
            <div className={styles.head}>
              <span className={styles.stage}>{entry.stage}</span>
              <span className={styles.status}>{STATUS_LABEL[entry.status]}</span>
              {jobId && (
                <span className={styles.viaGui} title={`Job ${jobId}`}>
                  via GUI
                </span>
              )}
              <span className={`${styles.time} mono`}>{entry.started_at}</span>
              {duration && <span className={`${styles.duration} mono`}>{duration}</span>}
            </div>
            {entry.error_msg && <p className={styles.error}>{entry.error_msg}</p>}
          </li>
        );
      })}
    </ol>
  );
}
