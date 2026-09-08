// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { ApiRequestError } from '../api/client';
import styles from './ErrorBanner.module.css';

export interface ErrorBannerProps {
  error: Error;
  onRetry?: () => void;
  /** What was being loaded, e.g. "the project summary". Used in the sentence. */
  context?: string;
}

interface Explanation {
  /** What happened, in the user's terms. */
  message: string;
  /** Whether retrying is the thing to do about it. */
  retryIsTheAnswer: boolean;
}

/**
 * Turn a failure into something the user can act on.
 *
 * The three statuses the contract singles out mean genuinely different things
 * and want different reactions, so they are not collapsed into one "request
 * failed": 503 is transient and wants the button, 501 is a missing feature and
 * retrying will never help, 404 says the data is simply not in this project.
 */
function explain(error: Error, context?: string): Explanation {
  const subject = context ?? 'this data';

  if (error instanceof ApiRequestError) {
    if (error.isRetryable) {
      return {
        message: `The project database was busy — a running stage held the write lock while ${subject} was requested. Nothing is wrong; try again in a moment.`,
        retryIsTheAnswer: true,
      };
    }
    if (error.isNotImplemented) {
      return {
        message: `This server build does not implement ${subject} yet.`,
        retryIsTheAnswer: false,
      };
    }
    if (error.isNotFound) {
      return {
        message: `Not in this project: ${subject} does not exist in the open .rux file.`,
        retryIsTheAnswer: false,
      };
    }
  }

  // Falls through for a network failure too, where `message` is the fetch
  // rejection text. Never the stack: a stack tells the user nothing and buries
  // the one line that does.
  return { message: error.message, retryIsTheAnswer: true };
}

export function ErrorBanner({ error, onRetry, context }: ErrorBannerProps) {
  const { message, retryIsTheAnswer } = explain(error, context);

  return (
    <div className={styles.banner} role="alert">
      <div className={styles.body}>
        <span className={styles.heading}>Could not load {context ?? 'this data'}</span>
        <p className={styles.message}>{message}</p>
      </div>
      {onRetry && (
        <button
          type="button"
          className={`${styles.retry} ${retryIsTheAnswer ? styles.primary : ''}`}
          onClick={onRetry}
        >
          Retry
        </button>
      )}
    </div>
  );
}
