// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { JobProgress } from '../api/types';
import styles from './StageProgress.module.css';

export interface StageProgressProps {
  progress: JobProgress;
}

/**
 * One phase of a running stage, as a bar.
 *
 * The contract's `total: 0` is not "zero work" — it means the phase never
 * declared a work count, and `fraction` is null for exactly that reason. So the
 * indeterminate case gets a sweeping bar and no number at all. Synthesising a
 * percentage there (0%, or worse, a guess that creeps toward 100) would be
 * inventing progress the server explicitly declined to claim, and this UI is
 * watched during twenty-minute stages where a wrong ETA is expensive.
 *
 * `stage_label` is displayed and `stage` is not: the contract says match on the
 * token, never on the label, and nothing here matches on anything.
 */
export function StageProgress({ progress }: StageProgressProps) {
  const determinate = progress.total > 0;

  // `fraction` is optional even when `total > 0`, and the contract defines it as
  // current/total — so recomputing it is reading the contract, not guessing.
  // Clamped because a phase that overshoots its own estimate must not overflow
  // the track.
  const raw = progress.fraction ?? (determinate ? progress.current / progress.total : null);
  const fraction = raw === null ? null : Math.min(1, Math.max(0, raw));
  const percent = fraction === null ? null : Math.round(fraction * 100);

  return (
    <div className={styles.wrap}>
      <div className={styles.head}>
        <span className={styles.label}>{progress.stage_label}</span>
        {determinate ? (
          <span className={`${styles.count} mono`}>
            {progress.current}/{progress.total}
            {percent !== null && <span className={styles.percent}> {percent}%</span>}
          </span>
        ) : (
          <span className={styles.count}>working…</span>
        )}
      </div>

      <div
        className={styles.track}
        role="progressbar"
        aria-label={progress.stage_label}
        aria-valuemin={determinate ? 0 : undefined}
        aria-valuemax={determinate ? progress.total : undefined}
        aria-valuenow={determinate ? progress.current : undefined}
      >
        {determinate ? (
          // The only inline style in this component: a measured value, not a
          // design decision, so there is no token that could express it.
          <span className={styles.fill} style={{ width: `${(fraction ?? 0) * 100}%` }} />
        ) : (
          <span className={`${styles.fill} ${styles.sweep}`} />
        )}
      </div>
    </div>
  );
}
