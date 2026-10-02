// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Resource / template client methods: paths, verbs, bodies and envelope
 * unwrapping, against the Phase 1 contract (docs/gui/openapi.yaml §resources,
 * §templates).
 */

import { describe, expect, it } from 'vitest';

import { RuxApiClient, type FetchLike } from '../api/client';

function client(payload: unknown, status = 200) {
  const calls: { url: string; method?: string; body?: string; headers?: Record<string, string> }[] = [];
  const fetchLike: FetchLike = (url, init) => {
    calls.push({ url, method: init?.method, body: init?.body as string | undefined, headers: init?.headers });
    return Promise.resolve(
      new Response(status === 204 ? null : JSON.stringify(payload), {
        status,
        headers: { 'Content-Type': 'application/json' },
      }),
    );
  };
  return { calls, api: new RuxApiClient({ baseUrl: '/api/v1', fetch: fetchLike }) };
}

describe('resources client', () => {
  it('reads the key catalogue as a bare array (GET /resources/keys)', async () => {
    const key = {
      id: 'sys:eak',
      label: 'EAK',
      category: 'Kortlægning',
      scope: 'type',
      data_type: 'text',
      unit: null,
      options: [],
      editable: true,
    };
    const { calls, api } = client([key]);
    expect(await api.resourceKeys()).toEqual([key]);
    expect(calls[0]).toMatchObject({ url: '/api/v1/resources/keys', method: 'GET' });
  });

  it('reads resources with and without a template', async () => {
    const { calls, api } = client({ resources: [{ code: 'RX-001', type_id: 2, manual: true, values: {} }] });
    expect(await api.resources()).toHaveLength(1);
    await api.resources(7);
    expect(calls.map((c) => c.url)).toEqual(['/api/v1/resources', '/api/v1/resources?template=7']);
  });

  it('patches values sparsely, null meaning clear', async () => {
    const { calls, api } = client({
      resource: { code: 'RX-001', type_id: 2, manual: true, values: {} },
      siblings: [],
    });
    const out = await api.patchResource('RX-001', { 'sys:note': 'ok', 'col:3': null });
    expect(calls[0]).toMatchObject({ url: '/api/v1/resources/RX-001', method: 'PATCH' });
    expect(calls[0].headers?.['Content-Type']).toBe('application/json');
    expect(JSON.parse(calls[0].body!)).toEqual({ values: { 'sys:note': 'ok', 'col:3': null } });
    expect(out.siblings).toEqual([]);
  });

  it('url-encodes the resource code', async () => {
    const { calls, api } = client({
      resource: { code: 'a/b', type_id: 1, manual: true, values: {} },
      siblings: [],
    });
    await api.patchResource('a/b', {});
    expect(calls[0].url).toBe('/api/v1/resources/a%2Fb');
  });

  it('creates a manual resource and deletes one expecting 204', async () => {
    const { calls, api } = client({ code: 'RX-019', type_id: 2, manual: true, values: {} }, 201);
    expect((await api.createResource({ type_id: 2, name: 'Ekstra søjle' })).code).toBe('RX-019');
    expect(JSON.parse(calls[0].body!)).toEqual({ type_id: 2, name: 'Ekstra søjle' });
    const del = client(null, 204);
    await del.api.deleteResource('RX-019');
    expect(del.calls[0]).toMatchObject({ url: '/api/v1/resources/RX-019', method: 'DELETE' });
  });

  it('manages resource columns on the renamed path', async () => {
    const { calls, api } = client([]);
    await api.resourceColumns();
    await api.createResourceColumn({ name: 'Stand', type: 'select', options: ['God', 'Dårlig'] });
    await api.updateResourceColumn('4', { options: ['God'] });
    expect(calls.map((c) => `${c.method} ${c.url}`)).toEqual([
      'GET /api/v1/resources/columns',
      'POST /api/v1/resources/columns',
      'PATCH /api/v1/resources/columns/4',
    ]);
    const del = client(null, 204);
    await del.api.deleteResourceColumn('4');
    expect(del.calls[0]).toMatchObject({ url: '/api/v1/resources/columns/4', method: 'DELETE' });
  });
});

describe('templates client', () => {
  const tpl = {
    id: 3,
    name: 'Hurtig genbrugsscreening',
    members: [{ key: 'sys:name' }],
    csv: {},
    seed: 'screening',
    resolved_keys: ['sys:name'],
    missing: [],
    created_at: '',
    updated_at: '',
  };

  it('lists templates out of their envelope', async () => {
    const { calls, api } = client({ templates: [tpl] });
    expect(await api.templates()).toEqual([tpl]);
    expect(calls[0].url).toBe('/api/v1/templates');
  });

  it('duplicates and patches a template', async () => {
    const { calls, api } = client(tpl);
    await api.duplicateTemplate(3);
    await api.patchTemplate(3, { members: [{ key: 'sys:name' }, { key: 'col:4' }] });
    expect(calls.map((c) => `${c.method} ${c.url}`)).toEqual([
      'POST /api/v1/templates/3/duplicate',
      'PATCH /api/v1/templates/3',
    ]);
    expect(JSON.parse(calls[1].body!)).toEqual({ members: [{ key: 'sys:name' }, { key: 'col:4' }] });
  });
});
