// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import { useJobs } from '../app/JobsContext';
import { isTerminal, type Job } from '../api/types';
import { StageProgress } from './StageProgress';
import styles from './JobToaster.module.css';

/** How long a finished job's toast stays up before it retires itself. */
const TERMINAL_DISMISS_MS = 8000;

/**
 * Live job feed, bottom-right.
 *
 * Phase 2's only job UI: there is no runner yet, so nothing here submits or
 * cancels anything. What it does prove end to end is the whole event path —
 * observer → server → `seq`-ordered reducer → render — which is what Phase 3
 * builds the stage cards on top of.
 *
 * Running jobs stay pinned. A job that reaches a terminal status is held
 * briefly and then dropped, except a failure, which stays until dismissed: an
 * error message that vanishes after eight seconds is an error message the user
 * cannot act on.
 */
export function JobToaster() {
  const { jobs } = useJobs();
  const [dismissed, setDismissed] = useState<Record<string, true>>({});
  const timers = useRef(new Map<string, ReturnType<typeof setTimeout>>());

  useEffect(() => {
    const handles = timers.current;
    for (const job of jobs) {
      if (!isTerminal(job.status)) continue;
      if (job.status === 'failed') continue; // sticky until dismissed
      if (handles.has(job.id) || dismissed[job.id]) continue;
      handles.set(
        job.id,
        setTimeout(() => {
          handles.delete(job.id);
          setDismissed((current) => ({ ...current, [job.id]: true }));
        }, TERMINAL_DISMISS_MS),
      );
    }
  }, [jobs, dismissed]);

  useEffect(() => {
    const handles = timers.current;
    return () => {
      for (const handle of handles.values()) clearTimeout(handle);
      handles.clear();
    };
  }, []);

  const visible = jobs.filter((job) => !dismissed[job.id]);
  if (visible.length === 0) return null;

  return (
    <div className={styles.stack} role="status" aria-live="polite">
      {visible.map((job) => (
        <JobToast
          key={job.id}
          job={job}
          onDismiss={() => setDismissed((current) => ({ ...current, [job.id]: true }))}
        />
      ))}
    </div>
  );
}

function JobToast({ job, onDismiss }: { job: Job; onDismiss: () => void }) {
  return (
    <article className={`${styles.toast} ${styles[job.status]}`}>
      <header className={styles.head}>
        <span className={styles.stage}>{job.stage}</span>
        <span className={styles.status}>{job.status}</span>
        <button className={styles.close} onClick={onDismiss} aria-label="Dismiss">
          ×
        </button>
      </header>

      {job.status === 'running' && job.progress && <StageProgress progress={job.progress} />}

      {job.error && <p className={styles.error}>{job.error}</p>}

      {job.cancel_requested && !isTerminal(job.status) && (
        <p className={styles.note}>Cancellation requested — waiting for the stage to notice.</p>
      )}
    </article>
  );
}
