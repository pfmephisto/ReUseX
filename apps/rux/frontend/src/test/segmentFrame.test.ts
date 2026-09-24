// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Contract tests for `RuxApiClient.segmentFrame`.
 *
 * Pinned against the `POST /frames/{id}/segment` shape in
 * `docs/gui/openapi.yaml` (#409).  Same stub-transport pattern as
 * `client.test.ts`: no network, no server.
 */

import { describe, expect, it } from 'vitest';
import { ApiRequestError, RuxApiClient, type FetchLike } from '../api/client';

const SEGMENT_RESULT = {
  frame_id: 42,
  labeled_pixels: 50_000,
  saved: true,
  labels: { '1': 'wall', '2': 'floor' },
};

interface RecordedRequest {
  url: string;
  method?: string;
  headers?: Record<string, string>;
  body?: string;
}

function stubFetch(payload: unknown, init: { status?: number; statusText?: string } = {}) {
  const calls: RecordedRequest[] = [];
  const fetchLike: FetchLike = (url, options) => {
    calls.push({ url, ...options });
    const body = typeof payload === 'string' ? payload : JSON.stringify(payload);
    return Promise.resolve(
      new Response(body, {
        status: init.status ?? 200,
        statusText: init.statusText ?? 'OK',
        headers: { 'Content-Type': 'application/json' },
      }),
    );
  };
  return { calls, fetchLike };
}

function clientFor(payload: unknown, init?: { status?: number; statusText?: string }) {
  const { calls, fetchLike } = stubFetch(payload, init);
  return { calls, api: new RuxApiClient({ baseUrl: '/api/v1', fetch: fetchLike }) };
}

describe('segmentFrame', () => {
  it('POSTs to /frames/{id}/segment with JSON body and returns the result', async () => {
    const { api, calls } = clientFor(SEGMENT_RESULT);
    const result = await api.segmentFrame(42, {
      model_path: '/models/sam3',
      prompts: [{ text: 'wall', boxes: [['pos', [0, 0, 100, 100]]] }],
      confidence: 0.6,
      save: true,
    });

    expect(calls).toHaveLength(1);
    expect(calls[0].url).toBe('/api/v1/frames/42/segment');
    expect(calls[0].method).toBe('POST');
    expect(calls[0].headers?.['Content-Type']).toBe('application/json');

    const body = JSON.parse(calls[0].body ?? '{}') as Record<string, unknown>;
    expect(body['model_path']).toBe('/models/sam3');
    expect(body['confidence']).toBe(0.6);
    expect(body['save']).toBe(true);
    const prompts = body['prompts'] as Array<{ text: string; boxes: unknown[] }>;
    expect(prompts[0].text).toBe('wall');
    expect(prompts[0].boxes[0]).toEqual(['pos', [0, 0, 100, 100]]);

    expect(result.frame_id).toBe(42);
    expect(result.labeled_pixels).toBe(50_000);
    expect(result.saved).toBe(true);
    expect(result.labels['1']).toBe('wall');
    expect(result.labels['2']).toBe('floor');
  });

  it('sends only model_path when no optional fields are provided', async () => {
    const { api, calls } = clientFor(SEGMENT_RESULT);
    await api.segmentFrame(7, { model_path: '/path/to/model' });

    const body = JSON.parse(calls[0].body ?? '{}') as Record<string, unknown>;
    expect(body['model_path']).toBe('/path/to/model');
    expect(body['prompts']).toBeUndefined();
    expect(body['confidence']).toBeUndefined();
    expect(body['save']).toBeUndefined();
  });

  it('maps 503 to ApiRequestError (no segmenter registered)', async () => {
    const { api } = clientFor(
      { error: 'no SAM3 segmenter registered' },
      { status: 503, statusText: 'Service Unavailable' },
    );
    const err = (await api
      .segmentFrame(1, { model_path: '/x' })
      .catch((e: unknown) => e)) as ApiRequestError;

    expect(err).toBeInstanceOf(ApiRequestError);
    expect(err.status).toBe(503);
    expect(err.isRetryable).toBe(true);
    expect(err.message).toBe('no SAM3 segmenter registered');
  });

  it('maps 409 to isConflict (pipeline job holds the write lock)', async () => {
    const { api } = clientFor(
      { error: 'pipeline job running' },
      { status: 409, statusText: 'Conflict' },
    );
    const err = (await api
      .segmentFrame(1, { model_path: '/x' })
      .catch((e: unknown) => e)) as ApiRequestError;

    expect(err).toBeInstanceOf(ApiRequestError);
    expect(err.status).toBe(409);
    expect(err.isConflict).toBe(true);
    expect(err.isRetryable).toBe(false);
  });

  it('maps 404 to isNotFound (frame does not exist)', async () => {
    const { api } = clientFor(
      { error: 'frame 99 not found' },
      { status: 404, statusText: 'Not Found' },
    );
    const err = (await api
      .segmentFrame(99, { model_path: '/x' })
      .catch((e: unknown) => e)) as ApiRequestError;

    expect(err).toBeInstanceOf(ApiRequestError);
    expect(err.isNotFound).toBe(true);
  });

  it('maps 400 to a plain ApiRequestError for bad model_path', async () => {
    const { api } = clientFor(
      { error: 'invalid model_path' },
      { status: 400, statusText: 'Bad Request' },
    );
    const err = (await api
      .segmentFrame(1, { model_path: '' })
      .catch((e: unknown) => e)) as ApiRequestError;

    expect(err).toBeInstanceOf(ApiRequestError);
    expect(err.status).toBe(400);
    expect(err.isNotFound).toBe(false);
    expect(err.isConflict).toBe(false);
    expect(err.isRetryable).toBe(false);
  });
});
