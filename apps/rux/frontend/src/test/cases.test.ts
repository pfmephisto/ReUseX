// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { CasesClient, type CasesFetch } from '../api/cases';
import { RuxApiClient, caseBaseUrl } from '../api/client';
import {
  CASES_PATH,
  caseBasename,
  caseHref,
  formatBytes,
  isCaseId,
  LAST_CASE_KEY,
  legacyRedirectTarget,
  nameFromFile,
  parseCaseLocation,
  readLastCase,
  uploadChunks,
  writeLastCase,
} from '../app/cases';

describe('case locations (S2)', () => {
  it('tells the list, a case and an old path apart', () => {
    expect(parseCaseLocation('/sager')).toEqual({ kind: 'list' });
    expect(parseCaseLocation('/sager/')).toEqual({ kind: 'list' });
    expect(parseCaseLocation('/sager/kontor')).toEqual({ kind: 'case', cid: 'kontor', rest: '/' });
    expect(parseCaseLocation('/sager/kontor/')).toEqual({ kind: 'case', cid: 'kontor', rest: '/' });
    expect(parseCaseLocation('/sager/kontor-2/kortlaegning')).toEqual({
      kind: 'case',
      cid: 'kontor-2',
      rest: '/kortlaegning',
    });
    expect(parseCaseLocation('/sager/a/pipeline/log')).toEqual({ kind: 'case', cid: 'a', rest: '/pipeline/log' });
    expect(parseCaseLocation('/')).toEqual({ kind: 'legacy' });
    expect(parseCaseLocation('/kortlaegning')).toEqual({ kind: 'legacy' });
    expect(parseCaseLocation('/sagerx')).toEqual({ kind: 'legacy' });
  });

  it('sends anything that cannot be a server-made id to the list', () => {
    for (const p of ['/sager/..', '/sager/%2e%2e/x', '/sager/Kontor', '/sager/a%2Fb', '/sager/%E0%A4%A']) {
      expect(parseCaseLocation(p), p).toEqual({ kind: 'list' });
    }
    expect(isCaseId('kontor-2')).toBe(true);
    expect(isCaseId('-x')).toBe(false);
    expect(isCaseId('a'.repeat(65))).toBe(false);
  });

  it('builds the basename and hrefs of a case', () => {
    expect(CASES_PATH).toBe('/sager');
    expect(caseBasename('kontor')).toBe('/sager/kontor');
    expect(caseHref('kontor')).toBe('/sager/kontor');
    expect(caseHref('kontor', '/kortlaegning')).toBe('/sager/kontor/kortlaegning');
    expect(caseHref('kontor', 'viewport')).toBe('/sager/kontor/viewport');
  });

  it('forwards an old path to the last-used case, or to the list', () => {
    const ids = ['a', 'b'];
    expect(legacyRedirectTarget('/kortlaegning', '?type=3', 'b', ids)).toBe('/sager/b/kortlaegning?type=3');
    expect(legacyRedirectTarget('/', '', 'a', ids)).toBe('/sager/a');
    // The last case is gone (deleted, or another server): the list.
    expect(legacyRedirectTarget('/kortlaegning', '', 'gone', ids)).toBe('/sager');
    expect(legacyRedirectTarget('/kortlaegning', '', null, ids)).toBe('/sager');
  });

  it('remembers the last case, and survives broken storage', () => {
    const map = new Map<string, string>();
    const storage = { getItem: (k: string) => map.get(k) ?? null, setItem: (k: string, v: string) => void map.set(k, v) };
    expect(readLastCase(storage)).toBeNull();
    writeLastCase('kontor', storage);
    expect(map.get(LAST_CASE_KEY)).toBe('kontor');
    expect(readLastCase(storage)).toBe('kontor');
    map.set(LAST_CASE_KEY, '../evil');
    expect(readLastCase(storage)).toBeNull();
    const broken = {
      getItem: () => {
        throw new Error('blocked');
      },
      setItem: () => {
        throw new Error('blocked');
      },
    };
    expect(readLastCase(broken)).toBeNull();
    expect(() => writeLastCase('x', broken)).not.toThrow();
  });
});

describe('uploads (S2)', () => {
  it('splits a file into server-sized chunks, resuming where the server is', () => {
    expect(uploadChunks(10, 4)).toEqual([
      { start: 0, end: 4 },
      { start: 4, end: 8 },
      { start: 8, end: 10 },
    ]);
    expect(uploadChunks(10, 4, 8)).toEqual([{ start: 8, end: 10 }]);
    expect(uploadChunks(8, 4, 8)).toEqual([]);
    expect(() => uploadChunks(10, 0)).toThrow(RangeError);
  });

  it('suggests a case name from the file name', () => {
    expect(nameFromFile('NewOffice.rux')).toBe('NewOffice');
    expect(nameFromFile('scan.RUX')).toBe('scan');
    expect(nameFromFile('notes.txt')).toBe('notes.txt');
  });

  it('formats sizes in Danish units', () => {
    expect(formatBytes(512)).toBe('512 B');
    expect(formatBytes(1_234_567)).toBe('1,2 MB');
    expect(formatBytes(992_546_816)).toBe('993 MB');
    expect(formatBytes(-1)).toBe('—');
  });

  it('uploads chunk by chunk, then completes the case', async () => {
    const calls: { url: string; method?: string; size?: number }[] = [];
    const fetchLike: CasesFetch = (url, init) => {
      calls.push({
        url,
        method: init?.method,
        size: init?.body instanceof Blob ? init.body.size : undefined,
      });
      const json = (body: unknown, status = 200) =>
        Promise.resolve(new Response(JSON.stringify(body), { status, headers: { 'Content-Type': 'application/json' } }));
      if (url.endsWith('/uploads')) return json({ id: 'u1', name: 'Ny', size: 10, received: 0, chunk_bytes: 4 }, 201);
      if (url.endsWith('/complete')) return json({ id: 'ny', name: 'Ny' }, 201);
      return json({ id: 'u1', received: 0 });
    };
    const client = new CasesClient({ fetch: fetchLike });
    const progress: number[] = [];
    const file = new Blob([new Uint8Array(10)]);
    const created = await client.uploadFile(file, 'Ny', (p) => progress.push(p.sent));
    expect(created.id).toBe('ny');
    expect(calls.map((c) => `${c.method} ${c.url}`)).toEqual([
      'POST /api/v1/uploads',
      'PUT /api/v1/uploads/u1?offset=0',
      'PUT /api/v1/uploads/u1?offset=4',
      'PUT /api/v1/uploads/u1?offset=8',
      'POST /api/v1/uploads/u1/complete',
    ]);
    expect(calls.slice(1, 4).map((c) => c.size)).toEqual([4, 4, 2]);
    expect(progress).toEqual([0, 4, 8, 10]);
  });

  it('abandons the upload on the server when a chunk fails', async () => {
    const calls: string[] = [];
    const fetchLike: CasesFetch = (url, init) => {
      calls.push(`${init?.method} ${url}`);
      if (url.endsWith('/uploads'))
        return Promise.resolve(
          new Response(JSON.stringify({ id: 'u1', name: 'x', size: 4, received: 0, chunk_bytes: 4 }), { status: 201 }),
        );
      if (init?.method === 'PUT')
        return Promise.resolve(new Response(JSON.stringify({ error: 'disk full' }), { status: 507 }));
      return Promise.resolve(new Response(null, { status: 204 }));
    };
    const client = new CasesClient({ fetch: fetchLike });
    await expect(client.uploadFile(new Blob([new Uint8Array(4)]), 'x')).rejects.toMatchObject({ status: 507 });
    expect(calls).toContain('DELETE /api/v1/uploads/u1');
  });

  it('creates, renames and deletes a case on the server-level routes', async () => {
    const calls: string[] = [];
    const fetchLike: CasesFetch = (url, init) => {
      calls.push(`${init?.method} ${url} ${String(init?.body ?? '')}`);
      return Promise.resolve(
        init?.method === 'DELETE' ? new Response(null, { status: 204 }) : new Response('{}', { status: 200 }),
      );
    };
    const client = new CasesClient({ fetch: fetchLike });
    await client.list();
    await client.create('Ny sag');
    await client.update('ny-sag', { name: 'Omdøbt' });
    await client.remove('ny-sag');
    expect(calls).toEqual([
      'GET /api/v1/cases ',
      'POST /api/v1/cases {"name":"Ny sag"}',
      'PATCH /api/v1/cases/ny-sag {"name":"Omdøbt"}',
      'DELETE /api/v1/cases/ny-sag ',
    ]);
  });
});

describe('case-scoped API client (S2)', () => {
  it('roots project routes at the case and server routes at /api/v1', async () => {
    const urls: string[] = [];
    const client = new RuxApiClient({
      fetch: (url) => {
        urls.push(url);
        return Promise.resolve(new Response('{"endpoints":[]}', { status: 200 }));
      },
    });
    client.selectCase('kontor');
    expect(caseBaseUrl('kontor')).toBe('/api/v1/cases/kontor');
    expect(client.url('/clouds')).toBe('/api/v1/cases/kontor/clouds');
    expect(client.renderUrl({ view: 'plan' })).toBe('/api/v1/cases/kontor/renders?view=plan');
    expect(client.eventsUrl({ protocol: 'http:', host: 'h:1' })).toBe('ws://h:1/api/v1/cases/kontor/events');
    await client.health();
    await client.sam3Status();
    await client.endpoints();
    expect(urls).toEqual([
      '/api/v1/cases/kontor/health',
      '/api/v1/models/sam3/status',
      '/api/v1/endpoints',
    ]);
  });
});
