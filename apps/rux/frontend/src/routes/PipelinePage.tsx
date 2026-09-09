// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useState } from 'react';

import { ApiRequestError, api } from '../api/client';
import { isTerminal } from '../api/types';
import { useAsync } from '../app/useAsync';
import { useJobs } from '../app/JobsContext';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { StageCard } from '../components/StageCard';
import { filterByStage, historyRows, stagesInHistory } from '../pipeline/history';
import { buildStageCards } from '../pipeline/stageModel';
import styles from './PipelinePage.module.css';

const STATUS_LABEL = {
  running: 'Running',
  success: 'Success',
  failed: 'Failed',
} as const;

/**
 * The pipeline runner (#305).
 *
 * Two panes that answer two different questions: "what can I run now" (the
 * cards, driven by `/stages`) and "what has been run" (the timeline, driven by
 * `pipeline_log`). The second is deliberately not derived from the first —
 * `/jobs` only knows what this server process started, and it forgets on
 * restart, while `pipeline_log` is durable and also records runs made from the
 * command line. A history view built from jobs would quietly hide half of what
 * happened to the project.
 */
export function PipelinePage() {
  const { jobs } = useJobs();
  const [submitErrors, setSubmitErrors] = useState<Record<string, string>>({});
  const [stageFilter, setStageFilter] = useState<string | null>(null);

  const catalogue = useAsync((signal) => api.stages(signal), []);
  const log = useAsync((signal) => api.pipelineLog(200, signal), []);

  // A finished job can unblock the next stage and always adds a history row,
  // so both panes are refreshed once per terminal transition. Keyed on the set
  // of terminal job ids rather than on `jobs`, which changes on every progress
  // tick and would otherwise refetch the log a few times a second.
  const terminalKey = useMemo(
    () =>
      jobs
        .filter((job) => isTerminal(job.status))
        .map((job) => job.id)
        .sort()
        .join(','),
    [jobs],
  );
  const { reload: reloadCatalogue } = catalogue;
  const { reload: reloadLog } = log;
  useEffect(() => {
    if (terminalKey === '') return;
    reloadCatalogue();
    reloadLog();
  }, [terminalKey, reloadCatalogue, reloadLog]);

  const cards = useMemo(
    () => buildStageCards(catalogue.data ?? [], jobs),
    [catalogue.data, jobs],
  );

  const run = useCallback(
    (stage: string, parameters: Record<string, unknown>) => {
      setSubmitErrors((previous) => ({ ...previous, [stage]: '' }));
      api
        .submitJob({ stage, parameters })
        .then(() => {
          // Nothing to store: the accepted job arrives over the WebSocket, and
          // treating this response as truth would race the `job.started` event
          // that may already have overtaken it.
        })
        .catch((cause: unknown) => {
          const message =
            cause instanceof ApiRequestError
              ? `${cause.message} (HTTP ${cause.status})`
              : cause instanceof Error
                ? cause.message
                : String(cause);
          setSubmitErrors((previous) => ({ ...previous, [stage]: message }));
        });
    },
    [],
  );

  const cancel = useCallback((jobId: string) => {
    // Idempotent by contract, and the real outcome arrives as `job.finished` —
    // so the response is not asserted on here. Whether the stage honoured the
    // request is the server's call to report, not this button's to assume.
    void api.cancelJob(jobId).catch(() => undefined);
  }, []);

  const rows = useMemo(() => historyRows(log.data ?? []), [log.data]);
  const visibleRows = useMemo(() => filterByStage(rows, stageFilter), [rows, stageFilter]);
  const stageNames = useMemo(() => stagesInHistory(rows), [rows]);

  return (
    <div className={styles.page}>
      <section className={styles.section}>
        <h2 className={styles.heading}>Stages</h2>
        {catalogue.loading && !catalogue.data && <Spinner label="Loading the stage catalogue" />}
        {catalogue.error && (
          <ErrorBanner
            error={catalogue.error}
            onRetry={catalogue.reload}
            context="the stage catalogue"
          />
        )}
        {catalogue.data && (
          <div className={styles.cards}>
            {cards.map((card) => (
              <StageCard
                key={card.stage}
                card={card}
                submitError={submitErrors[card.stage] || undefined}
                onRun={run}
                onCancel={cancel}
              />
            ))}
          </div>
        )}
      </section>

      <section className={styles.section}>
        <div className={styles.historyHead}>
          <h2 className={styles.heading}>History</h2>
          {stageNames.length > 1 && (
            <div className={styles.filters}>
              <button
                type="button"
                className={`${styles.filter} ${stageFilter === null ? styles.filterOn : ''}`}
                onClick={() => setStageFilter(null)}
              >
                All
              </button>
              {stageNames.map((name) => (
                <button
                  key={name}
                  type="button"
                  className={`${styles.filter} ${stageFilter === name ? styles.filterOn : ''}`}
                  onClick={() => setStageFilter(name)}
                >
                  {name}
                </button>
              ))}
            </div>
          )}
        </div>

        {log.loading && !log.data && <Spinner label="Loading the pipeline log" />}
        {log.error && (
          <ErrorBanner error={log.error} onRetry={log.reload} context="the pipeline log" />
        )}

        {log.data && visibleRows.length === 0 && (
          <EmptyState
            title="Nothing has been run against this project yet"
            detail="Run a stage above, or from the command line — every run records itself here either way."
          />
        )}

        {visibleRows.length > 0 && (
          <ol className={styles.timeline}>
            {visibleRows.map((row) => {
              const parameterKeys = Object.keys(row.parameters);
              return (
                <li
                  key={row.entry.id}
                  className={`${styles.row} ${styles[row.entry.status]}`}
                >
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

                  {row.entry.error_msg && (
                    <p className={styles.rowError}>{row.entry.error_msg}</p>
                  )}
                </li>
              );
            })}
          </ol>
        )}
      </section>
    </div>
  );
}
