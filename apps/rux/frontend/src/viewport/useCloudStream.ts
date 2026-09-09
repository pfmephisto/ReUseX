// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import { ApiRequestError, api, type RuxApiClient } from '../api/client';
import { isRuxp, parseRuxp } from './binaryPoints';
import { pageCount, pageTotal, toPageBuffers, type StreamPage } from './decode';
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

/** The two point-page routes a stream uses. Narrow so a test can fake it. */
export type PointsClient = Pick<RuxApiClient, 'cloudPoints' | 'cloudPointsBinary'>;

export interface PageFetcherOptions {
  /** Defaults to the app client. */
  client?: PointsClient;
  signal?: AbortSignal;
  /** True once the stream was torn down; stops the retry loop rethrowing forever. */
  isCancelled?: () => boolean;
  /** Injected so a test does not sit on a real timer. */
  delay?: (ms: number) => Promise<void>;
}

/** Fetch one page of a named cloud, in the best format the server supports. */
export type FetchPage = (name: string, offset: number, limit: number) => Promise<StreamPage>;

/**
 * A page fetcher with **sticky** format negotiation, for the life of one stream.
 *
 * `format=binary` is tried first (`docs/gui/binary-points.md`). A server that
 * predates #283 answers 501, and one that ignores the parameter altogether
 * answers 200 with a JSON body — both mean "this server has no RUXP", so the
 * fetcher latches to JSON and every later page in the stream goes there
 * directly. Retrying binary per page would cost a wasted round trip on each of
 * the two hundred pages a large scan takes.
 *
 * A body that *does* start with the RUXP magic but then fails to parse is a
 * different animal: the server claims to speak the format and does not. That
 * propagates to `state.error` and reaches the user, because silently falling
 * back would turn a server bug into a permanent, invisible slow path.
 *
 * A 503 is retried with backoff rather than surfaced: it means a running job
 * momentarily held the database, which is a normal condition in this app and
 * not something to make the user re-click.
 *
 * Exported separately from the hook so the negotiation can be tested without a
 * DOM, a canvas or a React renderer.
 */
export function createPageFetcher(options: PageFetcherOptions = {}): FetchPage {
  const client = options.client ?? api;
  const { signal, isCancelled = () => false, delay = sleep } = options;

  // The latch. Starts optimistic; only ever moves binary -> json.
  let binarySupported = true;

  const fetchOnce = async (name: string, offset: number, limit: number): Promise<StreamPage> => {
    if (binarySupported) {
      let buffer: ArrayBuffer;
      try {
        buffer = await client.cloudPointsBinary(name, { offset, limit }, signal);
      } catch (cause) {
        // 503 is transient and must reach the retry loop unchanged; only a
        // "this server cannot do binary" answer flips the latch.
        if (!(cause instanceof ApiRequestError) || !cause.isNotImplemented) throw cause;
        binarySupported = false;
        return fetchOnce(name, offset, limit);
      }
      if (!isRuxp(buffer)) {
        binarySupported = false;
        return fetchOnce(name, offset, limit);
      }
      return { format: 'binary', page: parseRuxp(buffer) };
    }
    return { format: 'json', page: await client.cloudPoints(name, { offset, limit }, signal) };
  };

  return async (name, offset, limit) => {
    for (let attempt = 0; ; attempt += 1) {
      try {
        return await fetchOnce(name, offset, limit);
      } catch (cause) {
        if (isCancelled()) throw cause;
        const retryable = cause instanceof ApiRequestError && cause.isRetryable;
        if (!retryable || attempt >= RETRY_LIMIT) throw cause;
        await delay(150 * 2 ** attempt);
      }
    }
  };
}

/**
 * Walk a cloud page by page, handing each decoded page to `onPage`.
 *
 * Pages are fetched **sequentially**, not in parallel — but no longer because
 * the server cannot afford otherwise. Since #283 a page reads only the blob
 * bytes that page needs, so peak server memory is `O(page)` and a parallel
 * fetch would be affordable. Sequential stays for two reasons that outlast that
 * change: what the user actually wants is the *first* page fast, and sequential
 * already gives it — the cloud builds up in front of them instead of appearing
 * all at once at the end — and `rux gui` opens a fresh `ProjectDB` per request,
 * so concurrent readers only widen the window in which a running job's writer
 * turns one of them into a `SQLITE_BUSY`.
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

    // One fetcher per stream, so the format latch is per stream too: a reload
    // against a restarted server tries binary again.
    const fetchPage = createPageFetcher({
      signal: controller.signal,
      isCancelled: () => cancelled,
    });

    const applyPage = (geometry: StreamPage, labels: StreamPage | null) => {
      const buffers = toPageBuffers(geometry, labels);
      if (!buffers) return 0;
      onPageRef.current(buffers);
      return pageCount(geometry);
    };

    const run = async () => {
      setState({ loaded: 0, fraction: null, done: false });

      // The first page doubles as the "how big is this cloud" probe, so the
      // common small-cloud case costs one request rather than two.
      const first = await fetchPage(cloud, 0, limit);
      if (cancelled) return;

      const total = pageTotal(first);
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
