// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/** Contract tests for the template CRUD calls and the report-generate body (spec §5.5, §6.3). */

import { describe, expect, it } from 'vitest';
import { ApiRequestError, RuxApiClient, type FetchLike } from '../api/client';

interface RecordedRequest {
  url: string;
  method?: string;
  headers?: Record<string, string>;
  body?: string;
}

function clientFor(payload: unknown, status = 200) {
  const calls: RecordedRequest[] = [];
  const fetchLike: FetchLike = (url, options) => {
    calls.push({ url, ...(options as Omit<RecordedRequest, 'url'>) });
    const body = status === 204 ? null : JSON.stringify(payload);
    return Promise.resolve(
      new Response(body, { status, headers: { 'Content-Type': 'application/json' } }),
    );
  };
  return { calls, api: new RuxApiClient({ baseUrl: '/api/v1', fetch: fetchLike }) };
}

const TEMPLATE = {
  id: 7,
  name: 'Ny skabelon',
  members: [],
  csv: {},
  seed: null,
  resolved_keys: [],
  missing: [],
  created_at: '2026-10-02T10:00:00Z',
  updated_at: '2026-10-02T10:00:00Z',
};

describe('template CRUD', () => {
  it('creates with a JSON POST', async () => {
    const { calls, api } = clientFor(TEMPLATE, 201);
    const t = await api.createTemplate({ name: 'Ny skabelon', members: [] });
    expect(t.id).toBe(7);
    expect(calls[0].url).toBe('/api/v1/templates');
    expect(calls[0].method).toBe('POST');
    expect(calls[0].headers?.['Content-Type']).toBe('application/json');
    expect(JSON.parse(calls[0].body!)).toEqual({ name: 'Ny skabelon', members: [] });
  });

  it('patches only the given fields', async () => {
    const { calls, api } = clientFor({ ...TEMPLATE, members: [{ category: 'Egne felter' }] });
    await api.patchTemplate(7, { members: [{ category: 'Egne felter' }] });
    expect(calls[0].url).toBe('/api/v1/templates/7');
    expect(calls[0].method).toBe('PATCH');
    expect(JSON.parse(calls[0].body!)).toEqual({ members: [{ category: 'Egne felter' }] });
  });

  it('deletes and accepts a 204', async () => {
    const { calls, api } = clientFor(null, 204);
    await expect(api.deleteTemplate(7)).resolves.toBeUndefined();
    expect(calls[0].url).toBe('/api/v1/templates/7');
    expect(calls[0].method).toBe('DELETE');
  });

  it('maps a duplicate-name 409 to ApiRequestError', async () => {
    const { api } = clientFor({ error: 'template name already exists' }, 409);
    const err = await api.patchTemplate(7, { name: 'Materialepas (fuld)' }).catch((e) => e);
    expect(err).toBeInstanceOf(ApiRequestError);
    expect((err as ApiRequestError).status).toBe(409);
  });

  it('restores seeds with a JSON POST and ignores the body', async () => {
    const { calls, api } = clientFor({ templates: [] });
    await expect(api.restoreSeedTemplates()).resolves.toBeUndefined();
    expect(calls[0].url).toBe('/api/v1/templates/restore-seeds');
    expect(calls[0].method).toBe('POST');
  });

  it('builds the resources CSV URL for one template', () => {
    const { api } = clientFor(null);
    expect(api.resourcesExportCsvUrl(7)).toBe('/api/v1/resources/export.csv?template=7');
  });
});

describe('report generation', () => {
  it('sends no template when none is chosen', async () => {
    const { calls, api } = clientFor({ id: 1 }, 201);
    await api.generateReport(null);
    expect(JSON.parse(calls[0].body!)).toEqual({});
  });

  it('sends the chosen template id', async () => {
    const { calls, api } = clientFor({ id: 1 }, 201);
    await api.generateReport(7);
    expect(calls[0].url).toBe('/api/v1/reports/ressourcekortlaegning');
    expect(JSON.parse(calls[0].body!)).toEqual({ resource_template_id: 7 });
  });
});
