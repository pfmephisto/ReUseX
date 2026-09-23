// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { PipelineLogEntry } from '../api/types';
import { formatDuration, jobIdOf } from '../pipeline/history';
import { EmptyState } from './EmptyState';
import styles from './PipelineLogList.module.css';

export interface PipelineLogListProps {
  entries: PipelineLogEntry[];
  /** Strip badges and secondary fields; used in the overview sidebar. */
  compact?: boolean;
}

/** Text label per status. The colour is a second channel, never the only one. */
const STATUS_LABEL: Record<PipelineLogEntry['status'], string> = {
  running: 'Running',
  success: 'Success',
  failed: 'Failed',
};

export function PipelineLogList({ entries, compact = false }: PipelineLogListProps) {
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
              {!compact && jobId && (
                <span className={styles.viaGui} title={`Job ${jobId}`}>
                  via GUI
                </span>
              )}
              <span className={`${styles.time} mono`}>{entry.started_at}</span>
              {!compact && duration && (
                <span className={`${styles.duration} mono`}>{duration}</span>
              )}
            </div>
            {!compact && entry.error_msg && <p className={styles.error}>{entry.error_msg}</p>}
          </li>
        );
      })}
    </ol>
  );
}
