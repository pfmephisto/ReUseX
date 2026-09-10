// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import { ApiRequestError, api, type CloudPointsQuery, type RuxApiClient } from '../api/client';
import { isRuxp, parseRuxp } from './binaryPoints';
import { pageCount, pageIsLod, pageTotal, toPageBuffers, type StreamPage } from './decode';
import {
  DEFAULT_OVERVIEW_POINTS,
  DEFAULT_PAGE_SIZE,
  clampOverviewPoints,
  clampPageSize,
  loadFraction,
  planPages,
} from './pagination';
import type { PageBuffers } from './PointCloudScene';

export interface CloudStreamState {
  /** Points handed to `onPage` as full-resolution pages so far. */
  loaded: number;
  /**
   * Points in the coarse overview, once it has been applied.
   *
   * Deliberately not folded into `loaded`: the overview is a view of the whole
   * cloud, so adding it to a count that is compared against `total` would
   * report progress the stream has not made.
   */
  overview?: number;
  /** Total points in the cloud; undefined until the first page comes back. */
  total?: number;
  /** `loaded / total`, or null while the total is unknown. */
  fraction: number | null;
  done: boolean;
  error?: Error;
}

/** Which of the two things a page is, when it reaches `onPage`. */
export interface PageKind {
  /**
   * True for the coarse whole-scene page fetched before paging starts. The
   * caller must keep it in its own scene layer: it covers the same volume the
   * full-resolution pages are about to cover, and it is dropped once they have.
   */
  overview: boolean;
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
   * this works without a join key — and it is why the overview pass names the
   * geometry cloud as the label cloud's `lodSource`, so the two subsamples
   * pick the same point indices instead of two unrelated ones.
   */
  labelCloud?: string | null;
  pageSize?: number;
  /**
   * Budget for the coarse whole-scene overview fetched first (#320). `0`
   * disables it and restores the plain prefix-paging behaviour.
   */
  overviewPoints?: number;
  /** Called once per decoded page, in page order. */
  onPage: (buffers: PageBuffers, kind: PageKind) => void;
  /**
   * Called once the full-resolution stream has covered everything the overview
   * showed, so the caller can drop the overview layer. Not called when there
   * was no overview.
   */
  onOverviewSuperseded?: () => void;
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

/**
 * Fetch one page of a named cloud, in the best format the server supports.
 *
 * The query is passed as an object rather than positional `offset`/`limit`
 * because the endpoint now has two mutually exclusive modes — a window
 * (`offset`/`limit`) or a whole-cloud LOD (`maxPoints`) — and a positional
 * signature would have to encode "not this one" as a pair of `undefined`s.
 */
export type FetchPage = (name: string, query: CloudPointsQuery) => Promise<StreamPage>;

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

  const fetchOnce = async (name: string, query: CloudPointsQuery): Promise<StreamPage> => {
    if (binarySupported) {
      let buffer: ArrayBuffer;
      try {
        buffer = await client.cloudPointsBinary(name, query, signal);
      } catch (cause) {
        // 503 is transient and must reach the retry loop unchanged; only a
        // "this server cannot do binary" answer flips the latch.
        if (!(cause instanceof ApiRequestError) || !cause.isNotImplemented) throw cause;
        binarySupported = false;
        return fetchOnce(name, query);
      }
      if (!isRuxp(buffer)) {
        binarySupported = false;
        return fetchOnce(name, query);
      }
      return { format: 'binary', page: parseRuxp(buffer) };
    }
    return { format: 'json', page: await client.cloudPoints(name, query, signal) };
  };

  return async (name, query) => {
    for (let attempt = 0; ; attempt += 1) {
      try {
        return await fetchOnce(name, query);
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
 * Walk a cloud, coarsely first and then page by page, handing each decoded page
 * to `onPage`.
 *
 * ## The overview pass (#320)
 *
 * The first request is not the first page — it is `max_points`, a voxel
 * subsample of the **whole** cloud. A viewport does not want the first 100 000
 * points of a 10-million-point scan; it wants all ten million of them, coarsely,
 * and then finer. One request now puts the entire scan on screen, correctly
 * framed, before any of the ~100 paging requests behind it have returned.
 *
 * It doubles as the "how big is this cloud" probe, so it costs no extra round
 * trip. Three answers are possible and all three are handled:
 *
 * - **A LOD page** (`lod` set): the scan is bigger than the budget. Apply it as
 *   the overview, then page the cloud properly and drop the overview at the end.
 * - **The whole cloud** (`lod` clear, `count === total`): it fit the budget, so
 *   this *is* the cloud. Apply it as an ordinary page and stop — one request
 *   for a small cloud, where the old code always made at least one more.
 * - **A prefix** (`lod` clear, `count < total`): a server that predates
 *   `max_points` ignored it. Fall through to plain paging from offset 0. One
 *   wasted request against an old server, which is the same trade the RUXP
 *   fallback already makes.
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
  const {
    cloud,
    labelCloud = null,
    pageSize = DEFAULT_PAGE_SIZE,
    overviewPoints = DEFAULT_OVERVIEW_POINTS,
  } = options;

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
  const onSupersededRef = useRef(options.onOverviewSuperseded);
  onSupersededRef.current = options.onOverviewSuperseded;

  useEffect(() => {
    if (!cloud) {
      setState({ loaded: 0, fraction: null, done: true });
      return;
    }

    const controller = new AbortController();
    let cancelled = false;
    const limit = clampPageSize(pageSize);
    const overviewBudget = clampOverviewPoints(overviewPoints);

    // One fetcher per stream, so the format latch is per stream too: a reload
    // against a restarted server tries binary again.
    const fetchPage = createPageFetcher({
      signal: controller.signal,
      isCancelled: () => cancelled,
    });

    const applyPage = (
      geometry: StreamPage,
      labels: StreamPage | null,
      kind: PageKind = { overview: false },
    ) => {
      const buffers = toPageBuffers(geometry, labels);
      if (!buffers) return 0;
      onPageRef.current(buffers, kind);
      return pageCount(geometry);
    };

    /** Geometry plus its label sibling, over the same window or selection. */
    const fetchBoth = async (query: CloudPointsQuery) => {
      const geometry = await fetchPage(cloud, query);
      if (cancelled || !labelCloud) return { geometry, labels: null };
      // `lodSource` is what keeps the two index-aligned: without it the label
      // cloud has no positions of its own to voxelise, and a second,
      // independent selection would zip the wrong labels onto the points.
      const labels = await fetchPage(labelCloud, {
        ...query,
        ...(query.maxPoints === undefined ? {} : { lodSource: cloud }),
      });
      return { geometry, labels };
    };

    const run = async () => {
      setState({ loaded: 0, fraction: null, done: false });

      let total: number | undefined;
      let overview = 0;

      if (overviewBudget > 0) {
        const first = await fetchBoth({ maxPoints: overviewBudget });
        if (cancelled) return;
        total = pageTotal(first.geometry);

        if (!pageIsLod(first.geometry) && pageCount(first.geometry) >= total) {
          // The whole cloud fit the budget, so it is already on screen at full
          // resolution. Paging it again would refetch the same points.
          const loaded = applyPage(first.geometry, first.labels);
          setState({ loaded, total, fraction: loadFraction(loaded, total), done: true });
          onCompleteRef.current?.();
          return;
        }

        if (pageIsLod(first.geometry)) {
          overview = applyPage(first.geometry, first.labels, { overview: true });
          setState({ loaded: 0, overview, total, fraction: loadFraction(0, total), done: false });
        }
        // Otherwise the server ignored `max_points`; fall through and page.
      }

      let loaded = 0;
      let applied = 0;

      /** One full-resolution page. False means the stream was torn down. */
      const pageAt = async (offset: number, pageLimit: number) => {
        const page = await fetchBoth({ offset, limit: pageLimit });
        if (cancelled) return false;
        if (total === undefined) total = pageTotal(page.geometry);
        loaded += applyPage(page.geometry, page.labels);
        applied += 1;
        setState({ loaded, overview, total, fraction: loadFraction(loaded, total), done: false });
        return true;
      };

      // Without a usable overview the first page is the size probe, exactly as
      // it was before #320 — so a `overviewPoints: 0` stream behaves
      // identically to the old one, request for request.
      let firstPlan = 0;
      if (total === undefined) {
        if (!(await pageAt(0, limit))) return;
        firstPlan = 1;
      }

      for (const plan of planPages(total ?? 0, limit).slice(firstPlan)) {
        if (cancelled) return;
        if (!(await pageAt(plan.offset, plan.limit))) return;
      }

      if (cancelled) return;
      if (overview > 0 && applied > 0) onSupersededRef.current?.();
      setState({
        loaded,
        overview,
        total,
        fraction: loadFraction(loaded, total),
        done: true,
      });
      onCompleteRef.current?.();
    };

    // `overviewPoints` is a number, so a caller passing a literal cannot
    // restart the stream by re-rendering — same reason the callbacks are refs.
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
  }, [cloud, labelCloud, pageSize, overviewPoints]);

  return state;
}

function sleep(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}
