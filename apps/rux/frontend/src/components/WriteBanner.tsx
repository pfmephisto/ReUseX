// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { WriteFailure } from '../data/writeState';
import styles from './WriteBanner.module.css';

export interface WriteBannerProps {
  failure: WriteFailure;
  /** Offered only when re-sending the identical request could work. */
  onRetry?: () => void;
  onDismiss: () => void;
}

/**
 * Why an edit did not take.
 *
 * Separate from `ErrorBanner` because the two answer different questions.
 * `ErrorBanner` says "this could not be loaded" and can always offer a retry;
 * this says "this was not saved", and whether a retry is even *possible*
 * depends on which of two similar-looking statuses came back — 409 (a stage
 * holds the writer lock for minutes, or the edit is refused outright) versus
 * 503 (the lock was busy for an instant). The classification lives in
 * `data/writeState.ts`; this only renders it.
 *
 * The retry button appears only when retrying is the answer. A button that
 * cannot work is worse than no button: it invites the user to keep pressing it
 * while a stage runs.
 */
export function WriteBanner({ failure, onRetry, onDismiss }: WriteBannerProps) {
  return (
    <div className={styles.banner} role="alert" data-kind={failure.kind}>
      <div className={styles.body}>
        <span className={styles.title}>{failure.title}</span>
        <p className={styles.detail}>{failure.detail}</p>
        {failure.serverMessage && failure.serverMessage !== failure.detail && (
          <p className={styles.server}>{failure.serverMessage}</p>
        )}
      </div>
      <div className={styles.actions}>
        {failure.retryable && onRetry && (
          <button type="button" className={styles.retry} onClick={onRetry}>
            Save again
          </button>
        )}
        <button type="button" className={styles.dismiss} onClick={onDismiss}>
          Dismiss
        </button>
      </div>
    </div>
  );
}
