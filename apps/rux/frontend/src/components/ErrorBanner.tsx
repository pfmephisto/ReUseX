// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { explainLoadError, RETRY_LABEL } from '../app/errorCopy';
import styles from './ErrorBanner.module.css';

export interface ErrorBannerProps {
  error: Error;
  onRetry?: () => void;
  /**
   * What was being loaded, as a Danish definite noun phrase — "projektoversigten",
   * "prøverne". Used in both sentences; defaults to "dataene".
   */
  context?: string;
}

/** A failed load, said so the user can act on it (copy: `app/errorCopy.ts`). */
export function ErrorBanner({ error, onRetry, context }: ErrorBannerProps) {
  const { heading, message, retryIsTheAnswer } = explainLoadError(error, context);

  return (
    <div className={styles.banner} role="alert">
      <div className={styles.body}>
        <span className={styles.heading}>{heading}</span>
        <p className={styles.message}>{message}</p>
      </div>
      {onRetry && (
        <button
          type="button"
          className={`${styles.retry} ${retryIsTheAnswer ? styles.primary : ''}`}
          onClick={onRetry}
        >
          {RETRY_LABEL}
        </button>
      )}
    </div>
  );
}
