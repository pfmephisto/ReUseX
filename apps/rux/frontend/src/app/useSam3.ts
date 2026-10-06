// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useRef, useState } from 'react';

import { ApiRequestError, api } from '../api/client';
import type { Sam3ModelStatus } from '../api/types';
import { runWithProvisioning, sam3View, type Sam3View } from '../data/sam3Provisioning';

const sleep = (ms: number) => new Promise<void>((resolve) => setTimeout(resolve, ms));

export interface Sam3Handle {
  /** Last status seen; null before the first probe answers. */
  status: Sam3ModelStatus | null;
  /** False when the server has no managed model provider (501): hide the chip. */
  available: boolean;
  view: Sam3View;
  /**
   * Run a segment request through the managed-model flow: waits out
   * preparation (polling every 2 s) and busy-database 503s, then retries.
   * Rejects with `SegmentRunError` (show its message) or `SegmentCancelled`
   * (ignore: the component went away).
   */
  run: <T>(request: () => Promise<T>) => Promise<T>;
}

/**
 * The managed SAM3 model's status for one screen, plus the runner every
 * segment call goes through (`data/sam3Provisioning.ts`). Probes the status
 * once on mount — the probe never starts a download; the first run does.
 */
export function useSam3(): Sam3Handle {
  const [status, setStatus] = useState<Sam3ModelStatus | null>(null);
  const [available, setAvailable] = useState(true);
  const mounted = useRef(true);

  useEffect(() => {
    mounted.current = true;
    const controller = new AbortController();
    api
      .sam3Status(undefined, controller.signal)
      .then((s) => setStatus(s))
      .catch((e: unknown) => {
        if (e instanceof ApiRequestError && e.isNotImplemented) setAvailable(false);
      });
    return () => {
      mounted.current = false;
      controller.abort();
    };
  }, []);

  const run = useCallback(async <T,>(request: () => Promise<T>): Promise<T> => {
    const result = await runWithProvisioning(request, {
      status: () => api.sam3Status().catch(() => null),
      sleep,
      onStatus: (s) => {
        if (mounted.current) setStatus(s);
      },
      isCancelled: () => !mounted.current,
    });
    // A run succeeded, so the model is loadable now; refresh the chip.
    if (mounted.current) api.sam3Status().then((s) => mounted.current && setStatus(s), () => undefined);
    return result;
  }, []);

  return { status, available, view: sam3View(status), run };
}
