// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Survey / sample client methods: paths, verbs, bodies, and the 422 gate
 * surfacing as a typed error. Payloads mirror docs/gui/openapi.yaml; they are
 * hand-written because the endpoints are new (re-record into fixtures.ts once
 * a real server serves them).
 */

import { describe, expect, it } from 'vitest';

import { ApiRequestError, RuxApiClient, type FetchLike } from '../api/client';
import { TREATMENTS } from '../api/types';

function client(payload: unknown, status = 200) {
  const calls: { url: string; method?: string; body?: string }[] = [];
  const fetchLike: FetchLike = (url, init) => {
    calls.push({ url, method: init?.method, body: init?.body as string | undefined });
    return Promise.resolve(
      new Response(status === 204 ? null : JSON.stringify(payload), {
        status,
        headers: { 'Content-Type': 'application/json' },
      }),
    );
  };
  return { calls, api: new RuxApiClient({ baseUrl: '/api/v1', fetch: fetchLike }) };
}

describe('survey client', () => {
  it('lists treatments in waste-hierarchy order', () => {
    expect([...TREATMENTS]).toEqual(['bevaring', 'genbrug', 'genanvendelse', 'nyttiggoerelse', 'bortskaffelse']);
  });

  it('reads the survey, summary and fractions', async () => {
    const { calls, api } = client({ types: [], counts: { queue: 0, approved: 0, rejected: 0, all: 0 } });
    await api.survey();
    await api.surveySummary();
    await api.surveyFractions();
    expect(calls.map((c) => c.url)).toEqual(['/api/v1/survey', '/api/v1/survey/summary', '/api/v1/survey/fractions']);
  });

  it('patches a type sparsely with a JSON body', async () => {
    const { calls, api } = client({ id: 4 });
    await api.patchSurveyType(4, { review_status: 'approved', mass_t: null });
    expect(calls[0]).toMatchObject({ url: '/api/v1/survey/types/4', method: 'PATCH' });
    expect(JSON.parse(calls[0].body!)).toEqual({ review_status: 'approved', mass_t: null });
  });

  it('deletes a type and reads what went with it', async () => {
    const { calls, api } = client({ parts_deleted: 3, instances_dismissed: 2 });
    const r = await api.deleteSurveyType(7);
    expect(calls[0]).toMatchObject({ url: '/api/v1/survey/types/7', method: 'DELETE' });
    expect(r).toEqual({ parts_deleted: 3, instances_dismissed: 2 });
  });

  it('reads the photo batch in one request', async () => {
    const payload = { parts: { 'RX-001': { count: 6, best_frame_id: 12 } } };
    const { calls, api } = client(payload);
    const r = await api.surveyPhotos();
    expect(calls[0]).toMatchObject({ url: '/api/v1/survey/photos' });
    expect(calls[0].method ?? 'GET').toBe('GET');
    expect(r).toEqual(payload);
  });

  it('reads the panoramas near an instance, url-encoding the cloud', async () => {
    const payload = {
      point: [0, 0, 2],
      cloud: 'my instances',
      instance_id: 3,
      max_distance: 15,
      panoramas: [{ panorama_id: 5, node_id: 9, distance: 4, u: 0.7, v: 0.4, heading: 'resected' }],
      total: 1,
    };
    const { calls, api } = client(payload);
    const r = await api.instancePanoramas('my instances', 3);
    expect(calls[0].url).toBe('/api/v1/instances/my%20instances/3/panoramas');
    expect(r.panoramas[0].panorama_id).toBe(5);
  });

  it('url-encodes part codes', async () => {
    const { calls, api } = client({ code: 'RX-001' });
    await api.patchSurveyPart('RX-001', { starred: true });
    expect(calls[0].url).toBe('/api/v1/survey/parts/RX-001');
  });

  it('surfaces the sample gate as a 422 ApiRequestError', async () => {
    const { api } = client({ error: 'cannot be approved while sample(s) P-01 await a lab answer' }, 422);
    const err = await api.patchSurveyType(1, { review_status: 'approved' }).catch((e: unknown) => e);
    expect(err).toBeInstanceOf(ApiRequestError);
    expect((err as ApiRequestError).isUnprocessable).toBe(true);
    expect((err as ApiRequestError).message).toContain('P-01');
  });

  it('manages samples: list, create, patch, links, delete', async () => {
    const { calls, api } = client({ samples: [] });
    expect(await api.samples()).toEqual([]);
    await api.createSample({ title: 'PCB i fugemasse', type_ids: [6] });
    await api.patchSample(2, { stage: 'svar', result: 'ren' });
    await api.setSampleLinks(2, [6, 11]);
    expect(calls.map((c) => `${c.method} ${c.url}`)).toEqual([
      'GET /api/v1/samples',
      'POST /api/v1/samples',
      'PATCH /api/v1/samples/2',
      'PUT /api/v1/samples/2/links',
    ]);
    expect(JSON.parse(calls[3].body!)).toEqual({ type_ids: [6, 11] });
  });

  it('deletes a sample expecting 204', async () => {
    const { calls, api } = client(null, 204);
    await api.deleteSample(3);
    expect(calls[0]).toMatchObject({ url: '/api/v1/samples/3', method: 'DELETE' });
  });

  it('creates a sample with only Miljø’s fields', async () => {
    const { calls, api } = client({ id: 4, code: 'P-04' }, 201);
    await api.createSample({ title: 'Asbest i fugemasse', what: 'Fuge mod nord', type_ids: [6] });
    expect(calls[0]).toMatchObject({ url: '/api/v1/samples', method: 'POST' });
    expect(JSON.parse(calls[0].body!)).toEqual({
      title: 'Asbest i fugemasse',
      what: 'Fuge mod nord',
      type_ids: [6],
    });
  });

  it('builds render URLs with a comma layer list', () => {
    const { api } = client({});
    expect(api.renderUrl({ view: 'plan', highlight_instance: 12, layers: ['cloud', 'rooms'] })).toBe(
      '/api/v1/renders?view=plan&highlight_instance=12&layers=cloud%2Crooms',
    );
  });
});
