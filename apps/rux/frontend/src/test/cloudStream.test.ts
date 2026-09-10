// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Format negotiation for a cloud stream (#283).
 *
 * `useCloudStream` itself is a React hook: testing it would need a renderer and
 * a DOM, and this suite runs in `environment: 'node'` with neither. The part
 * worth testing is not the hook but the decision inside it — *which* transport
 * a page comes over — so `createPageFetcher` is exported separately and is what
 * is exercised here. Nothing below needs a canvas, a clock or a socket: the
 * client is a stub object satisfying `PointsClient`, and the retry backoff is
 * injected.
 *
 * The property being pinned is that the fallback is **sticky**. Getting it
 * wrong is not a correctness bug — the JSON path decodes fine — it is a
 * performance bug that only shows up against an old server and only as "the
 * viewport feels slow", which is exactly the kind of thing that survives for a
 * year unnoticed. So the assertions are on *call counts*, not just on results.
 */

import { describe, expect, it } from 'vitest';
import { ApiRequestError } from '../api/client';
import type { CloudPointsPage } from '../api/types';
import { pageIsLod } from '../viewport/decode';
import { createPageFetcher, type PointsClient } from '../viewport/useCloudStream';
import { CLOUD_POINTS_PAGE } from './fixtures';
import { XYZRGB_THREE_POINTS, buildRuxpPage } from './ruxpPage';

interface Recorded {
  name: string;
  offset?: number;
  limit?: number;
  maxPoints?: number;
  lodSource?: string;
}

interface Stub {
  client: PointsClient;
  binaryCalls: Recorded[];
  jsonCalls: Recorded[];
}

/**
 * A `PointsClient` whose binary route answers however the test says.
 *
 * `binary` is a function of the call index so a test can make the *second*
 * request behave differently from the first — the only way to show that the
 * latch, once flipped, stays flipped.
 */
function stubClient(binary: (call: number) => ArrayBuffer | Error): Stub {
  const binaryCalls: Recorded[] = [];
  const jsonCalls: Recorded[] = [];
  const client: PointsClient = {
    cloudPointsBinary(name, options = {}) {
      binaryCalls.push({ name, ...options });
      const answer = binary(binaryCalls.length - 1);
      return answer instanceof Error ? Promise.reject(answer) : Promise.resolve(answer);
    },
    cloudPoints(name, options = {}): Promise<CloudPointsPage> {
      jsonCalls.push({ name, ...options });
      return Promise.resolve(CLOUD_POINTS_PAGE);
    },
  };
  return { client, binaryCalls, jsonCalls };
}

const RUXP_PAGE = () => buildRuxpPage(XYZRGB_THREE_POINTS);

/** What an old `rux gui` sends back for `format=binary`. */
const notImplemented = () =>
  new ApiRequestError(501, 'binary point transport is not implemented', '/api/v1/clouds/c/points');

/** What a server holding the database for a running job sends back. */
const locked = () => new ApiRequestError(503, 'database is locked', '/api/v1/clouds/c/points');

/** A JSON body served with a 200 — a server that ignored `format` entirely. */
function jsonBody(): ArrayBuffer {
  return new TextEncoder().encode(JSON.stringify(CLOUD_POINTS_PAGE)).buffer as ArrayBuffer;
}

describe('createPageFetcher — the happy path', () => {
  it('asks for binary first and parses what comes back', async () => {
    const { client, binaryCalls, jsonCalls } = stubClient(RUXP_PAGE);
    const fetchPage = createPageFetcher({ client });

    const result = await fetchPage('cloud', { offset: 0, limit: 100 });
    expect(result.format).toBe('binary');
    if (result.format !== 'binary') throw new Error('unreachable');
    expect(result.page.count).toBe(3);
    expect(result.page.total).toBe(9);
    expect(Array.from(result.page.view('xyz')!)).toEqual([
      1, 2, 3, -4.5, 0, 0.25, 1024, -0.5, 65536,
    ]);

    // JSON is never touched when binary works.
    expect(jsonCalls).toHaveLength(0);
    expect(binaryCalls).toEqual([{ name: 'cloud', offset: 0, limit: 100 }]);
  });

  it('passes offset and limit through unchanged', async () => {
    const { client, binaryCalls } = stubClient(RUXP_PAGE);
    const fetchPage = createPageFetcher({ client });
    await fetchPage('planes', { offset: 400_000, limit: 100_000 });
    expect(binaryCalls[0]).toEqual({ name: 'planes', offset: 400_000, limit: 100_000 });
  });

  it('passes an overview query through unchanged, including lodSource', async () => {
    // The fetcher is transport negotiation and nothing else — it must not have
    // an opinion about which of the endpoint's two modes it is carrying (#320).
    const { client, binaryCalls } = stubClient(RUXP_PAGE);
    const fetchPage = createPageFetcher({ client });
    await fetchPage('labels', { maxPoints: 200_000, lodSource: 'cloud' });
    expect(binaryCalls[0]).toEqual({ name: 'labels', maxPoints: 200_000, lodSource: 'cloud' });
  });

  it('reports the LOD flag on a page that carries it', async () => {
    const { client } = stubClient(() => buildRuxpPage({ ...XYZRGB_THREE_POINTS, flags: 1 }));
    const result = await createPageFetcher({ client })('cloud', { maxPoints: 3 });
    expect(result.format).toBe('binary');
    expect(pageIsLod(result)).toBe(true);
  });

  it('reads a JSON overview answer as LOD too', async () => {
    // The two transports say the same thing differently, and the caller must
    // not have to know which one it got.
    const { client } = stubClient(() => notImplemented());
    const result = await createPageFetcher({ client })('cloud', { maxPoints: 3 });
    expect(result.format).toBe('json');
    // The stub's JSON fixture carries no `lod` field — an old server that
    // ignored `max_points` and answered with an ordinary prefix.
    expect(pageIsLod(result)).toBe(false);
    expect(pageIsLod({ format: 'json', page: { ...CLOUD_POINTS_PAGE, lod: true } })).toBe(true);
  });
});

describe('createPageFetcher — falling back to JSON', () => {
  it('falls back on 501 and never asks for binary again', async () => {
    const { client, binaryCalls, jsonCalls } = stubClient(() => notImplemented());
    const fetchPage = createPageFetcher({ client });

    for (const offset of [0, 100, 200]) {
      const result = await fetchPage('cloud', { offset, limit: 100 });
      expect(result.format).toBe('json');
    }

    // One wasted round trip for the whole stream, not one per page. A 200-page
    // scan against an old server is what makes the difference visible.
    expect(binaryCalls).toHaveLength(1);
    expect(jsonCalls.map((call) => call.offset)).toEqual([0, 100, 200]);
  });

  it('falls back when a 200 body is not RUXP at all', async () => {
    // A server (or a proxy) that ignored `format=binary` and sent JSON with a
    // 200. Indistinguishable from success at the HTTP layer; the magic check is
    // the only thing that catches it.
    const { client, binaryCalls, jsonCalls } = stubClient(jsonBody);
    const fetchPage = createPageFetcher({ client });

    expect((await fetchPage('cloud', { offset: 0, limit: 100 })).format).toBe('json');
    expect((await fetchPage('cloud', { offset: 100, limit: 100 })).format).toBe('json');
    expect(binaryCalls).toHaveLength(1);
    expect(jsonCalls).toHaveLength(2);
  });

  it('re-serves the page it fell back on, rather than dropping it', async () => {
    const { client, jsonCalls } = stubClient(() => notImplemented());
    const fetchPage = createPageFetcher({ client });
    const result = await fetchPage('cloud', { offset: 700, limit: 50 });
    // The offset that failed over binary is the offset refetched over JSON —
    // losing it would leave a hole in the cloud, silently.
    expect(jsonCalls[0]).toEqual({ name: 'cloud', offset: 700, limit: 50 });
    expect(result.format).toBe('json');
    if (result.format !== 'json') throw new Error('unreachable');
    expect(result.page).toBe(CLOUD_POINTS_PAGE);
  });

  it('shares one latch across the geometry and label clouds of a stream', async () => {
    const { client, binaryCalls } = stubClient(() => notImplemented());
    const fetchPage = createPageFetcher({ client });
    await fetchPage('cloud', { offset: 0, limit: 100 });
    await fetchPage('labels', { offset: 0, limit: 100 });
    // The two clouds are served by the same server; discovering it twice would
    // be one wasted request per page, not per stream.
    expect(binaryCalls).toHaveLength(1);
  });

  it('gives a fresh fetcher a fresh latch, so a restarted server is retried', async () => {
    const { client, binaryCalls } = stubClient(() => notImplemented());
    await createPageFetcher({ client })('cloud', { offset: 0, limit: 100 });
    await createPageFetcher({ client })('cloud', { offset: 0, limit: 100 });
    expect(binaryCalls).toHaveLength(2);
  });
});

describe('createPageFetcher — errors that are not a fallback', () => {
  it('surfaces a malformed RUXP body instead of quietly using JSON', async () => {
    // The server claims to speak the format and does not. Falling back would
    // turn a server bug into a permanent, invisible slow path.
    const truncated = () => {
      const full = RUXP_PAGE();
      const out = new ArrayBuffer(full.byteLength - 1);
      new Uint8Array(out).set(new Uint8Array(full, 0, out.byteLength));
      return out;
    };
    const { client, jsonCalls } = stubClient(truncated);
    const fetchPage = createPageFetcher({ client, delay: async () => {} });

    await expect(fetchPage('cloud', { offset: 0, limit: 100 })).rejects.toThrow(/RUXP/);
    expect(jsonCalls).toHaveLength(0);
  });

  it('retries a 503 rather than surfacing it, then succeeds', async () => {
    // A running job momentarily held the writer. Normal in this app, and not
    // something to make the user re-click.
    const waits: number[] = [];
    const { client, binaryCalls } = stubClient((call) => (call < 2 ? locked() : RUXP_PAGE()));
    const fetchPage = createPageFetcher({
      client,
      delay: async (ms) => {
        waits.push(ms);
      },
    });

    const result = await fetchPage('cloud', { offset: 0, limit: 100 });
    expect(result.format).toBe('binary');
    expect(binaryCalls).toHaveLength(3);
    expect(waits).toEqual([150, 300]); // exponential, not a busy loop
  });

  it('gives up on a 503 that will not clear', async () => {
    const { client, binaryCalls } = stubClient(() => locked());
    const fetchPage = createPageFetcher({ client, delay: async () => {} });
    await expect(fetchPage('cloud', { offset: 0, limit: 100 })).rejects.toThrow(/locked/);
    // Four attempts: the first plus RETRY_LIMIT retries.
    expect(binaryCalls).toHaveLength(4);
  });

  it('does not retry, and does not fall back, on a 404', async () => {
    const missing = new ApiRequestError(404, 'no such cloud', '/api/v1/clouds/nope/points');
    const { client, binaryCalls, jsonCalls } = stubClient(() => missing);
    const fetchPage = createPageFetcher({ client, delay: async () => {} });
    await expect(fetchPage('nope', { offset: 0, limit: 100 })).rejects.toBe(missing);
    expect(binaryCalls).toHaveLength(1);
    expect(jsonCalls).toHaveLength(0);
  });

  it('stops retrying once the stream is torn down', async () => {
    // The unmount case: a cancelled stream must not sit in a backoff loop
    // holding a reference to a scene that is already disposed.
    const { client, binaryCalls } = stubClient(() => locked());
    const fetchPage = createPageFetcher({
      client,
      isCancelled: () => true,
      delay: async () => {},
    });
    await expect(fetchPage('cloud', { offset: 0, limit: 100 })).rejects.toThrow(/locked/);
    expect(binaryCalls).toHaveLength(1);
  });
});
