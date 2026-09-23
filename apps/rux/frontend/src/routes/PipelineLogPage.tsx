// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useMemo, useState } from 'react';

import type { PipelineLogEntry } from '../api/types';
import { api } from '../api/client';
import { isTerminal } from '../api/types';
import { useAsync } from '../app/useAsync';
import { useJobs } from '../app/JobsContext';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import {
  applyHistoryFilter,
  historyRows,
  stagesInHistory,
} from '../pipeline/history';
import styles from './PipelineLogPage.module.css';

const STATUS_LABEL = {
  running: 'Running',
  success: 'Success',
  failed: 'Failed',
} as const;

/** Status filter chips, in the order they read as a run's life-cycle. */
const STATUSES: PipelineLogEntry['status'][] = ['running', 'success', 'failed'];

/**
 * The pipeline log (#449).
 *
 * Split out of the runner so that "what has happened to this project" is its own
 * surface, free of the run/cancel controls. The log is durable and also records
 * command-line runs — see `PipelinePage` for why it is not derived from `/jobs`.
 *
 * On top of the per-stage buttons the runner used to carry, this view adds a
 * status filter and a free-text search. All three narrow the same 200-entry
 * window the server returns; nothing is refetched when a filter changes, so
 * typing in the search box is instant and never races the live reload.
 */
export function PipelineLogPage() {
  const { jobs } = useJobs();
  const [stageFilter, setStageFilter] = useState<string | null>(null);
  const [statusFilter, setStatusFilter] = useState<PipelineLogEntry['status'] | null>(null);
  const [query, setQuery] = useState('');

  const log = useAsync((signal) => api.pipelineLog(200, signal), []);

  // A finished job always adds a history row, so the log is refreshed once per
  // terminal transition. Keyed on the set of terminal job ids rather than on
  // `jobs`, which changes on every progress tick and would otherwise refetch a
  // few times a second. Mirrors `PipelinePage`.
  const terminalKey = useMemo(
    () =>
      jobs
        .filter((job) => isTerminal(job.status))
        .map((job) => job.id)
        .sort()
        .join(','),
    [jobs],
  );
  const { reload: reloadLog } = log;
  useEffect(() => {
    if (terminalKey === '') return;
    reloadLog();
  }, [terminalKey, reloadLog]);

  const rows = useMemo(() => historyRows(log.data ?? []), [log.data]);
  const stageNames = useMemo(() => stagesInHistory(rows), [rows]);
  const visibleRows = useMemo(
    () => applyHistoryFilter(rows, { stage: stageFilter, status: statusFilter, query }),
    [rows, stageFilter, statusFilter, query],
  );

  const filtered = visibleRows.length !== rows.length;

  return (
    <div className={styles.page}>
      <div className={styles.head}>
        <h2 className={styles.heading}>Pipeline log</h2>
        {rows.length > 0 && (
          <span className={styles.count}>
            {filtered ? `${visibleRows.length} of ${rows.length}` : `${rows.length}`} entries
          </span>
        )}
      </div>

      {rows.length > 0 && (
        <div className={styles.controls}>
          <input
            type="search"
            className={styles.search}
            placeholder="Search stage, status, parameters or error…"
            aria-label="Search the pipeline log"
            value={query}
            onChange={(event) => setQuery(event.target.value)}
          />

          <div className={styles.filterGroups}>
            <div className={styles.filterGroup}>
              <span className={styles.filterLabel}>Status</span>
              <div className={styles.filters}>
                <button
                  type="button"
                  className={`${styles.filter} ${statusFilter === null ? styles.filterOn : ''}`}
                  onClick={() => setStatusFilter(null)}
                  aria-pressed={statusFilter === null}
                >
                  All
                </button>
                {STATUSES.map((status) => (
                  <button
                    key={status}
                    type="button"
                    className={`${styles.filter} ${statusFilter === status ? styles.filterOn : ''}`}
                    onClick={() => setStatusFilter(status)}
                    aria-pressed={statusFilter === status}
                  >
                    {STATUS_LABEL[status]}
                  </button>
                ))}
              </div>
            </div>

            {stageNames.length > 1 && (
              <div className={styles.filterGroup}>
                <span className={styles.filterLabel}>Stage</span>
                <div className={styles.filters}>
                  <button
                    type="button"
                    className={`${styles.filter} ${stageFilter === null ? styles.filterOn : ''}`}
                    onClick={() => setStageFilter(null)}
                    aria-pressed={stageFilter === null}
                  >
                    All
                  </button>
                  {stageNames.map((name) => (
                    <button
                      key={name}
                      type="button"
                      className={`${styles.filter} ${stageFilter === name ? styles.filterOn : ''}`}
                      onClick={() => setStageFilter(name)}
                      aria-pressed={stageFilter === name}
                    >
                      {name}
                    </button>
                  ))}
                </div>
              </div>
            )}
          </div>
        </div>
      )}

      {log.loading && !log.data && <Spinner label="Loading the pipeline log" />}
      {log.error && (
        <ErrorBanner error={log.error} onRetry={log.reload} context="the pipeline log" />
      )}

      {log.data && rows.length === 0 && (
        <EmptyState
          title="Nothing has been run against this project yet"
          detail="Run a stage from the Pipeline page, or from the command line — every run records itself here either way."
        />
      )}

      {log.data && rows.length > 0 && visibleRows.length === 0 && (
        <EmptyState
          title="No entries match the current filters"
          detail="Clear the search or pick a different status or stage to see more of the log."
        />
      )}

      {visibleRows.length > 0 && (
        <ol className={styles.timeline}>
          {visibleRows.map((row) => {
            const parameterKeys = Object.keys(row.parameters);
            return (
              <li key={row.entry.id} className={`${styles.row} ${styles[row.entry.status]}`}>
                <div className={styles.rowHead}>
                  <span className={styles.rowStage}>{row.entry.stage}</span>
                  <span className={styles.rowStatus}>{STATUS_LABEL[row.entry.status]}</span>
                  {row.jobId && (
                    <span className={styles.viaGui} title={`Job ${row.jobId}`}>
                      job <span className="mono">{row.jobId.slice(0, 8)}</span>
                    </span>
                  )}
                  <span className={`${styles.rowTime} mono`}>{row.entry.started_at}</span>
                  <span className={`${styles.rowDuration} mono`}>
                    {row.duration ?? (row.entry.status === 'running' ? '…' : '—')}
                  </span>
                </div>

                {parameterKeys.length > 0 && (
                  <p className={`${styles.rowParams} mono`}>
                    {parameterKeys
                      .map((key) => `${key}=${JSON.stringify(row.parameters[key])}`)
                      .join('  ')}
                  </p>
                )}

                {row.entry.error_msg && <p className={styles.rowError}>{row.entry.error_msg}</p>}
              </li>
            );
          })}
        </ol>
      )}
    </div>
  );
}
