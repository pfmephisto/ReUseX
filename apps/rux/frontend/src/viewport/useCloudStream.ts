// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import { ApiRequestError, api } from '../api/client';
import type { CloudPointsPage } from '../api/types';
import { decodeColors, decodeLabels, decodePositions } from './decode';
import { DEFAULT_PAGE_SIZE, clampPageSize, loadFraction, planPages } from './pagination';
import type { PageBuffers } from './PointCloudScene';

export interface CloudStreamState {
  /** Points handed to `onPage` so far. */
  loaded: number;
  /** Total points in the cloud; undefined until the first page comes back. */
  total?: number;
  /** `loaded / total`, or null while the total is unknown. */
  fraction: number | null;
  done: boolean;
  error?: Error;
}

export interface CloudStreamOptions {
  /** Geometry cloud to stream. Nothing is fetched while this is undefined. */
  cloud?: string;
  /**
   * A `Label` cloud to zip against the geometry, or null for none.
   *
   * The contract guarantees sibling clouds of one scan are returned in storage
   * order and are index-aligned, so the same `offset`/`limit` window can be
   * pulled from both and joined positionally. That guarantee is the only reason
   * this works without a join key.
   */
  labelCloud?: string | null;
  pageSize?: number;
  /** Called once per decoded page, in page order. */
  onPage: (buffers: PageBuffers) => void;
  /** Called once when every page has been applied. */
  onComplete?: () => void;
}

/** How many times a page is retried when the server answers 503. */
const RETRY_LIMIT = 3;

/**
 * Walk a cloud page by page, handing each decoded page to `onPage`.
 *
 * Pages are fetched **sequentially**, not in parallel. `rux gui` opens a fresh
 * `ProjectDB` per request and materialises the whole cloud to serve one page
 * (see the NOTE in `apps/rux/src/gui/api.cpp`), so concurrent requests buy
 * nothing and make `SQLITE_BUSY` more likely while a job holds the writer. What
 * the user actually wants here is the *first* page fast, which sequential
 * already gives — the cloud builds up in front of them instead of appearing all
 * at once at the end.
 *
 * A 503 is retried with backoff rather than surfaced: it means a running job
 * momentarily held the database, which is a normal condition in this app and
 * not something to make the user re-click.
 */
export function useCloudStream(options: CloudStreamOptions): CloudStreamState {
  const { cloud, labelCloud = null, pageSize = DEFAULT_PAGE_SIZE } = options;

  const [state, setState] = useState<CloudStreamState>({
    loaded: 0,
    fraction: null,
    done: false,
  });

  // Callbacks are inline closures at the call site; a ref keeps them out of the
  // effect's dependencies so a parent re-render cannot restart the download.
  const onPageRef = useRef(options.onPage);
  onPageRef.current = options.onPage;
  const onCompleteRef = useRef(options.onComplete);
  onCompleteRef.current = options.onComplete;

  useEffect(() => {
    if (!cloud) {
      setState({ loaded: 0, fraction: null, done: true });
      return;
    }

    const controller = new AbortController();
    let cancelled = false;
    const limit = clampPageSize(pageSize);

    const fetchPage = async (name: string, offset: number, want: number) => {
      for (let attempt = 0; ; attempt += 1) {
        try {
          return await api.cloudPoints(
            name,
            { offset, limit: want, format: 'json' },
            controller.signal,
          );
        } catch (cause) {
          if (cancelled) throw cause;
          const retryable = cause instanceof ApiRequestError && cause.isRetryable;
          if (!retryable || attempt >= RETRY_LIMIT) throw cause;
          await sleep(150 * 2 ** attempt);
        }
      }
    };

    const applyPage = (geometry: CloudPointsPage, labels: CloudPointsPage | null) => {
      const positions = decodePositions(geometry);
      if (!positions) return 0;
      onPageRef.current({
        positions,
        rgb: decodeColors(geometry),
        labels: labels ? decodeLabels(labels) : null,
      });
      return geometry.count;
    };

    const run = async () => {
      setState({ loaded: 0, fraction: null, done: false });

      // The first page doubles as the "how big is this cloud" probe, so the
      // common small-cloud case costs one request rather than two.
      const first = await fetchPage(cloud, 0, limit);
      if (cancelled) return;

      const total = first.total;
      const firstLabels = labelCloud ? await fetchPage(labelCloud, 0, limit) : null;
      if (cancelled) return;

      let loaded = applyPage(first, firstLabels);
      setState({ loaded, total, fraction: loadFraction(loaded, total), done: false });

      const remaining = planPages(total, limit).slice(1);
      for (const plan of remaining) {
        if (cancelled) return;
        const page = await fetchPage(cloud, plan.offset, plan.limit);
        const labelPage = labelCloud
          ? await fetchPage(labelCloud, plan.offset, plan.limit)
          : null;
        if (cancelled) return;
        loaded += applyPage(page, labelPage);
        setState({ loaded, total, fraction: loadFraction(loaded, total), done: false });
      }

      if (cancelled) return;
      setState({ loaded, total, fraction: loadFraction(loaded, total), done: true });
      onCompleteRef.current?.();
    };

    run().catch((cause: unknown) => {
      if (cancelled || controller.signal.aborted) return;
      setState((current) => ({
        ...current,
        done: true,
        error: cause instanceof Error ? cause : new Error(String(cause)),
      }));
    });

    return () => {
      cancelled = true;
      controller.abort();
    };
  }, [cloud, labelCloud, pageSize]);

  return state;
}

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}
