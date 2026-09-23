// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useState } from 'react';

import { Link } from 'react-router-dom';

import { ApiRequestError, api } from '../api/client';
import { isTerminal } from '../api/types';
import { useAsync } from '../app/useAsync';
import { useJobs } from '../app/JobsContext';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { StageCard } from '../components/StageCard';
import { buildStageCards } from '../pipeline/stageModel';
import styles from './PipelinePage.module.css';

/**
 * The pipeline runner (#305, split from the log in #449).
 *
 * This surface answers only "what can I run now" — the stage cards, driven by
 * `/stages` and `POST /jobs`. "What has been run" is a separate page
 * (`/pipeline/log`, `PipelineLogPage`): the two were combined originally, but a
 * log with its own search and filters is a reading surface, not a control
 * panel, and mixing the two crowded both. The history is deliberately not
 * derived from the running jobs — see `PipelineLogPage` for why.
 */
export function PipelinePage() {
  const { jobs } = useJobs();
  const [submitErrors, setSubmitErrors] = useState<Record<string, string>>({});

  const catalogue = useAsync((signal) => api.stages(signal), []);

  // A finished job can unblock the next stage, so the catalogue is refreshed
  // once per terminal transition. Keyed on the set of terminal job ids rather
  // than on `jobs`, which changes on every progress tick and would otherwise
  // refetch a few times a second.
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
  useEffect(() => {
    if (terminalKey === '') return;
    reloadCatalogue();
  }, [terminalKey, reloadCatalogue]);

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

  return (
    <div className={styles.page}>
      <section className={styles.section}>
        <div className={styles.head}>
          <h2 className={styles.heading}>Stages</h2>
          <Link className={styles.logLink} to="/pipeline/log">
            View log
          </Link>
        </div>
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
    </div>
  );
}
