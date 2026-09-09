// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * What to tell the user when an edit did not take.
 *
 * `ErrorBanner` explains a failed *read*, where 503 is the only interesting
 * status and "try again" is always reasonable advice. A failed *write* has two
 * statuses that look alike and are not:
 *
 *  - **409** — a pipeline job holds the project's writer lock for the whole of
 *    a stage, which is minutes, or the server refuses this edit outright.
 *    Retrying now cannot work, and offering a retry button is advice that will
 *    fail in front of the user.
 *  - **503** — the lock was busy for a moment and the request lost the race.
 *    Nothing was written; sending exactly the same request again is the fix.
 *
 * Collapsing them into "the edit failed, retry" is the thing this module
 * exists to prevent. Nothing was written in either case, which is what makes
 * the optimistic rollback in the panes safe.
 */

import { ApiRequestError } from '../api/client';

export type WriteFailureKind =
  | 'conflict'
  | 'busy'
  | 'rejected'
  | 'missing'
  | 'unsupported'
  | 'unknown';

export interface WriteFailure {
  kind: WriteFailureKind;
  /** Headline, in the user's terms. */
  title: string;
  /** What happened and what to do about it. Always mentions that nothing was written. */
  detail: string;
  /** Whether retrying the identical request now is the thing to do. */
  retryable: boolean;
  /** The server's own wording, kept verbatim for the cases where it is specific. */
  serverMessage?: string;
}

/**
 * Classify a write failure.
 *
 * `subject` names what was being edited ("this label legend", "this
 * passport") and is used in the sentence, the same way `ErrorBanner` uses its
 * `context`.
 *
 * The server's message is carried through rather than replaced wherever it is
 * more specific than anything this function could invent — the `instances`
 * refusal, for instance, explains that those names encode the semantic class,
 * which no generic 409 wording would.
 */
export function describeWriteFailure(error: Error, subject: string): WriteFailure {
  if (error instanceof ApiRequestError) {
    if (error.isConflict) {
      return {
        kind: 'conflict',
        title: 'Not saved — the project is being written to',
        detail: `A pipeline stage holds the project's writer lock, so ${subject} was not changed. Wait for the running stage to finish, then save again. Retrying now will be refused the same way.`,
        retryable: false,
        serverMessage: error.message,
      };
    }
    if (error.isRetryable) {
      return {
        kind: 'busy',
        title: 'Not saved — the writer lock was busy',
        detail: `The lock could not be taken in time and nothing was written to ${subject}. This one is transient: send it again.`,
        retryable: true,
        serverMessage: error.message,
      };
    }
    if (error.status === 400) {
      return {
        kind: 'rejected',
        title: 'Not saved — the server rejected the edit',
        detail: error.message,
        retryable: false,
        serverMessage: error.message,
      };
    }
    if (error.isNotFound) {
      return {
        kind: 'missing',
        title: 'Not saved — it is no longer in this project',
        detail: `${subject} was not found. It may have been replaced by a stage that re-ran while this screen was open; reload to see the current state.`,
        retryable: false,
        serverMessage: error.message,
      };
    }
    if (error.status === 415) {
      // Unreachable through `patchJson`, which always sets the header. Kept
      // because a proxy that strips Content-Type would otherwise surface as an
      // unexplained failure, and the cause is worth naming when it happens.
      return {
        kind: 'unsupported',
        title: 'Not saved — the request was not accepted as JSON',
        detail: `The server requires 'Content-Type: application/json' on an edit and did not see it. Something between this page and ${subject} is rewriting the request.`,
        retryable: false,
        serverMessage: error.message,
      };
    }
  }

  return {
    kind: 'unknown',
    title: 'Not saved',
    detail: error.message,
    retryable: true,
  };
}

/**
 * Whether a 409 is worth refetching after.
 *
 * A job-lock conflict leaves the resource exactly as it was, so the pane's
 * data is still current and reloading it would only cost a request. A refusal
 * ("this cloud cannot be renamed") likewise changes nothing. So: never — which
 * is worth stating as a function rather than as an absence, because the
 * instinct on a 409 elsewhere is to refetch and merge.
 */
export function shouldReloadAfter(failure: WriteFailure): boolean {
  return failure.kind === 'missing';
}
