// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useRef, useState } from 'react';

import { ApiRequestError } from '../api/client';
import { useCanEdit } from './CaseRoleContext';
import type { SerialQueue } from './serialQueue';
import { chainFor } from './writeChain';

export interface MutationQueueOptions {
  /** Any failure that has no `onUnprocessable` handler (shown as a toast). */
  onError: (cause: unknown) => void;
  /** After every mutation, success or not — e.g. re-read the sidebar badges. */
  onSettled?: () => void;
  /**
   * Which chain the writes join: 'app' (the default), the app-wide chain the
   * case screens' first loads wait for (R11), or 'page', a chain of this page's
   * own for writes that change no survey state and may run long (Rapport).
   */
  scope?: 'app' | 'page';
  /**
   * Writes that are not to the case (your own API tokens): not refused for a
   * viewer of the open case.
   */
  ignoreCaseRole?: boolean;
}

export interface MutationQueue {
  /** True while any mutation is queued or in flight. Gates buttons only. */
  busy: boolean;
  /**
   * Queues `run`. The returned promise settles (never rejects) once the
   * mutation has finished, its error handling included; callers may ignore it.
   */
  mutate: (run: () => Promise<void>, onUnprocessable?: (cause: ApiRequestError) => void) => Promise<void>;
}

/**
 * A page's writes on one serial chain — the app-wide `appWriteChain` unless
 * `scope: 'page'`. Nothing is dropped: a field commit made while another request is in flight waits its
 * turn. `busy` counts queued requests too, so two overlapping requests cannot
 * clear it early.
 */
export function useMutationQueue(options: MutationQueueOptions): MutationQueue {
  const queueRef = useRef<SerialQueue | null>(null);
  if (queueRef.current === null) {
    queueRef.current = chainFor(options.scope);
  }
  const optionsRef = useRef(options);
  optionsRef.current = options;
  // A viewer's write is refused here, before a request the server would
  // answer 403 anyway (most edit controls are hidden for them too).
  const allowed = useCanEdit();
  const allowedRef = useRef(allowed);
  allowedRef.current = allowed;
  const [inFlight, setInFlight] = useState(0);

  const mutate = useCallback(
    (run: () => Promise<void>, onUnprocessable?: (cause: ApiRequestError) => void): Promise<void> => {
      setInFlight((n) => n + 1);
      return queueRef.current!.enqueue(async () => {
        try {
          if (!allowedRef.current && !optionsRef.current.ignoreCaseRole)
            throw new ApiRequestError(403, 'Du har kun læseadgang til denne sag.', '');
          await run();
        } catch (cause) {
          if (cause instanceof ApiRequestError && cause.isUnprocessable && onUnprocessable) {
            onUnprocessable(cause);
          } else {
            optionsRef.current.onError(cause);
          }
        } finally {
          setInFlight((n) => n - 1);
          optionsRef.current.onSettled?.();
        }
      });
    },
    [],
  );

  return { busy: inFlight > 0, mutate };
}
