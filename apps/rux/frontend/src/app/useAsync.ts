// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useRef, useState } from 'react';

export interface AsyncResult<T> {
  data?: T;
  error?: Error;
  loading: boolean;
  /** Re-run the request. Also used to retry after a 503. */
  reload: () => void;
}

/**
 * Run an abortable request and expose loading/error/data.
 *
 * Deliberately tiny: this frontend has no server-state library, and a
 * dependency that owns caching, retries and invalidation would be a much larger
 * commitment than a Phase-2 dashboard justifies. It aborts on unmount and on a
 * dependency change, so a slow `/project` cannot land after the user has
 * navigated away and overwrite fresher state.
 */
export function useAsync<T>(
  run: (signal: AbortSignal) => Promise<T>,
  deps: readonly unknown[],
): AsyncResult<T> {
  const [data, setData] = useState<T | undefined>(undefined);
  const [error, setError] = useState<Error | undefined>(undefined);
  const [loading, setLoading] = useState(true);
  const [nonce, setNonce] = useState(0);

  // `run` is typically an inline closure, so it changes identity every render.
  // Holding it in a ref keeps it out of the effect's dependency list, which is
  // driven by the caller-supplied `deps` instead.
  const runRef = useRef(run);
  runRef.current = run;

  useEffect(() => {
    const controller = new AbortController();
    let live = true;
    setLoading(true);

    runRef
      .current(controller.signal)
      .then((value) => {
        if (!live) return;
        setData(value);
        setError(undefined);
      })
      .catch((cause: unknown) => {
        if (!live || controller.signal.aborted) return;
        setError(cause instanceof Error ? cause : new Error(String(cause)));
      })
      .finally(() => {
        if (live) setLoading(false);
      });

    return () => {
      live = false;
      controller.abort();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [...deps, nonce]);

  const reload = useCallback(() => setNonce((value) => value + 1), []);

  return { data, error, loading, reload };
}
