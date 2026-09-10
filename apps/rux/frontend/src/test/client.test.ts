// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Contract tests for `RuxApiClient`.
 *
 * The transport is injected, so nothing here needs a server, a socket or a
 * network: a stub `fetch` records the request it was handed and replies with a
 * payload recorded from a real `rux gui` (see `fixtures.ts`). What is being
 * pinned is the seam between the client and the contract — envelope unwrapping,
 * URL construction, the mutating-route content type, and the error mapping.
 */

import { describe, expect, it } from 'vitest';
import { ApiRequestError, RuxApiClient, type FetchLike } from '../api/client';
import {
  CLOUDS,
  CLOUD_POINTS_PAGE,
  ERROR_NOT_FOUND,
  ERROR_NOT_IMPLEMENTED,
  HEALTH,
  JOBS,
  PIPELINE_LOG,
  PROJECT_SUMMARY,
  STAGES,
  STAGES_EMPTY_PROJECT,
  STAGE_VALIDATION_BLOCKED,
} from './fixtures';

interface RecordedRequest {
  url: string;
  method?: string;
  headers?: Record<string, string>;
  body?: string;
  signal?: AbortSignal;
}

/** A stub transport that always answers `payload`, recording what it was asked. */
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

describe('envelope unwrapping', () => {
  it('returns the Health body as-is', async () => {
    const { api } = clientFor(HEALTH);
    const health = await api.health();
    expect(health.status).toBe('ok');
    expect(health.api_version).toBe('1.0.0');
    expect(health.project.schema_version).toBe(11);
  });

  it('unwraps {clouds: [...]} into CloudInfo[]', async () => {
    const { api } = clientFor(CLOUDS);
    const clouds = await api.clouds();
    // The recorded corridor scan carries exactly these six clouds after
    // `create clouds` + `create planes`.
    expect(clouds).toHaveLength(6);
    expect(clouds.map((cloud) => cloud.name)).toEqual([
      'cloud',
      'normals',
      'labels',
      'planes',
      'plane_centroids',
      'plane_normals',
    ]);
    const cloud = clouds[0];
    expect(cloud.type).toBe('PointXYZRGB');
    expect(cloud.point_count).toBe(10500);
    expect(cloud.organized).toBe(false);
  });

  it('returns the ProjectSummary body as-is, projects included', async () => {
    const { api } = clientFor(PROJECT_SUMMARY);
    const summary = await api.projectSummary();
    expect(summary.path).toBe('e2e.rux');
    expect(summary.schema_version).toBe(11);
    expect(summary.clouds).toHaveLength(6);
    expect(summary.meshes).toEqual([]);
    expect(summary.sensor_frames.total_count).toBe(10);
    expect(summary.projects[0].name).toBe('ReUseX test fixture: NewOffice corridor');
  });

  it('unwraps {projects: [...]}', async () => {
    const { api } = clientFor({ projects: PROJECT_SUMMARY.projects });
    const projects = await api.projects();
    expect(projects).toHaveLength(1);
    expect(projects[0].id).toBe('default');
  });

  it('unwraps {meshes: [...]}', async () => {
    const { api } = clientFor({ meshes: [{ name: 'mesh', vertex_count: 8, face_count: 12 }] });
    const meshes = await api.meshes();
    expect(meshes).toHaveLength(1);
    expect(meshes[0].vertex_count).toBe(8);
  });

  it('unwraps {materials: [...]}', async () => {
    const { api } = clientFor({ materials: [{ guid: 'abc', property_count: 3 }] });
    const materials = await api.materials();
    expect(materials).toEqual([{ guid: 'abc', property_count: 3 }]);
  });

  it('unwraps {components: [...]}', async () => {
    const { api } = clientFor({ components: [{ name: 'wall_0', guid: 'g0', type: 'Wall' }] });
    const components = await api.components();
    expect(components).toHaveLength(1);
    expect(components[0].type).toBe('Wall');
  });

  it('unwraps {stages: [...]}', async () => {
    const { api } = clientFor(STAGES);
    const stages = await api.stages();
    expect(stages).toHaveLength(5);
    expect(stages.map((stage) => stage.stage)).toEqual([
      'clouds',
      'planes',
      'rooms',
      'instances',
      'mesh',
    ]);
    // The recorded project has been through clouds -> planes -> rooms, so every
    // contract is satisfied; `mesh` is still the one stage with no runner.
    const mesh = stages[4];
    expect(mesh.ready).toBe(true);
    expect(mesh.runnable).toBe(false);
    expect(mesh.parameters).toEqual([]);
  });

  it('carries the parameter schema and the blocked reason', async () => {
    const { api } = clientFor(STAGES_EMPTY_PROJECT);
    const stages = await api.stages();

    const planes = stages.find((stage) => stage.stage === 'planes');
    expect(planes?.ready).toBe(false);
    expect(planes?.blockers[0]).toContain("requires cloud 'cloud'");
    // The structured half of the same finding: what is missing, and the
    // ordered commands that would produce it (#295).
    expect(planes?.issues.map((issue) => issue.artifact)).toEqual(['cloud', 'normals']);
    expect(planes?.issues[0].commands).toContain('rux create clouds');
    // Blocked is not the same as unknown: the knobs are described either way.
    expect(planes?.parameters.length).toBeGreaterThan(0);
  });

  it('fetches one stage\'s validation', async () => {
    const { api, calls } = clientFor(STAGE_VALIDATION_BLOCKED);
    const stage = await api.stageValidation('planes');
    expect(calls[0].url).toBe('/api/v1/stages/planes/validation');
    expect(stage.stage).toBe('planes');
    expect(stage.ready).toBe(false);
  });

  it('unwraps {jobs: [...]}', async () => {
    const { api } = clientFor(JOBS);
    const jobs = await api.jobs();
    expect(jobs).toHaveLength(1);
    expect(jobs[0].status).toBe('succeeded');
    expect(jobs[0].progress?.current).toBe(10500);
  });

  it('unwraps the pipeline log under `entries`, not `log`', async () => {
    const { api } = clientFor(PIPELINE_LOG);
    const entries = await api.pipelineLog();
    expect(entries).toHaveLength(3);
    expect(entries.map((entry) => entry.id)).toEqual([3, 2, 1]);
    expect(entries[0].stage).toBe('segment_rooms');
    expect(entries[0].status).toBe('success');
  });

  it('unwraps instances out of an envelope that also carries `cloud`', async () => {
    const { api, calls } = clientFor({
      cloud: 'instances',
      instances: [{ instance_id: 1, guid: 'g1', semantic_class: 3, point_count: 42 }],
    });
    const instances = await api.instances('instances');
    expect(instances).toHaveLength(1);
    expect(instances[0].instance_id).toBe(1);
    expect(calls[0].url).toBe('/api/v1/instances/instances');
  });

  it('unwraps {endpoints: [...]}', async () => {
    const { api } = clientFor({
      endpoints: [{ method: 'GET', path: '/health', summary: 'Liveness' }],
    });
    const endpoints = await api.endpoints();
    expect(endpoints[0].path).toBe('/health');
  });
});

describe('URL construction', () => {
  it('serialises offset/limit/format for a points page', async () => {
    const { api, calls } = clientFor(CLOUD_POINTS_PAGE);
    await api.cloudPoints('cloud', { offset: 100, limit: 50 });
    expect(calls[0].url).toBe('/api/v1/clouds/cloud/points?offset=100&limit=50&format=json');
  });

  it('honours an explicit format rather than overriding it', async () => {
    const { api, calls } = clientFor(CLOUD_POINTS_PAGE);
    await api.cloudPoints('cloud', { offset: 0, limit: 4, format: 'binary' });
    expect(calls[0].url).toBe('/api/v1/clouds/cloud/points?offset=0&limit=4&format=binary');
  });

  it('serialises max_points and lod_source for a LOD read', async () => {
    // The wire names are snake_case; the TypeScript surface is camelCase. The
    // translation is the client's job and nothing above it should know either
    // spelling (#320).
    const { api, calls } = clientFor(CLOUD_POINTS_PAGE);
    await api.cloudPoints('labels', { maxPoints: 200_000, lodSource: 'cloud' });
    expect(calls[0].url).toBe(
      '/api/v1/clouds/labels/points?max_points=200000&lod_source=cloud&format=json',
    );
  });

  it('refuses max_points together with a window, at the call site', async () => {
    // The server answers 400 for this, but a caller that wrote it has a bug in
    // its own logic, not a bad server. Failing here names the mistake where it
    // was made instead of a round trip later.
    const { api } = clientFor(CLOUD_POINTS_PAGE);
    expect(() => api.cloudPoints('cloud', { maxPoints: 1000, offset: 0 })).toThrow(TypeError);
    expect(() => api.cloudPoints('cloud', { maxPoints: 1000, limit: 10 })).toThrow(
      /cannot be combined/,
    );
    await expect(api.cloudPointsBinary('cloud', { maxPoints: 1000, offset: 0 })).rejects.toThrow(
      TypeError,
    );
  });

  it('omits undefined query values instead of serialising the string "undefined"', async () => {
    const { api, calls } = clientFor(CLOUD_POINTS_PAGE);
    await api.cloudPoints('cloud', {});
    expect(calls[0].url).toBe('/api/v1/clouds/cloud/points?format=json');
    expect(calls[0].url).not.toContain('undefined');
  });

  it('drops an undefined filter entirely, leaving no query string at all', async () => {
    const { api, calls } = clientFor(PIPELINE_LOG);
    await api.pipelineLog();
    expect(calls[0].url).toBe('/api/v1/pipeline-log');

    const withLimit = clientFor(PIPELINE_LOG);
    await withLimit.api.pipelineLog(25);
    expect(withLimit.calls[0].url).toBe('/api/v1/pipeline-log?limit=25');
  });

  it('drops an undefined component type filter', async () => {
    const { api, calls } = clientFor({ components: [] });
    await api.components();
    expect(calls[0].url).toBe('/api/v1/components');

    const typed = clientFor({ components: [] });
    await typed.api.components('Wall');
    expect(typed.calls[0].url).toBe('/api/v1/components?type=Wall');
  });

  it('does not produce a double slash from a base URL with a trailing slash', async () => {
    const { calls, fetchLike } = stubFetch(CLOUDS);
    const api = new RuxApiClient({ baseUrl: '/api/v1/', fetch: fetchLike });
    await api.clouds();
    expect(calls[0].url).toBe('/api/v1/clouds');
    expect(calls[0].url).not.toContain('//');
  });

  it('percent-encodes a cloud name that needs escaping', async () => {
    const { api, calls } = clientFor(CLOUD_POINTS_PAGE);
    await api.cloudPoints('odd name/with#chars', { offset: 0, limit: 4 });
    expect(calls[0].url).toBe(
      '/api/v1/clouds/odd%20name%2Fwith%23chars/points?offset=0&limit=4&format=json',
    );
  });

  it('builds image and blob URLs without fetching them', () => {
    const { api } = clientFor(HEALTH);
    expect(api.frameImageUrl(7)).toBe('/api/v1/frames/7/image?kind=color');
    expect(api.frameImageUrl(7, 'depth')).toBe('/api/v1/frames/7/image?kind=depth');
    expect(api.meshDataUrl('mesh a')).toBe('/api/v1/meshes/mesh%20a/data');
    expect(api.meshTextureUrl('m', 'tex 1.png')).toBe(
      '/api/v1/meshes/m/textures/tex%201.png',
    );
    expect(api.panoramaImageUrl(3)).toBe('/api/v1/panoramas/3/image');
  });
});

describe('mutating routes', () => {
  it('POSTs a job with the JSON content type the server requires', async () => {
    const { api, calls } = clientFor(JOBS.jobs[0]);
    const job = await api.submitJob({ stage: 'planes', parameters: { radius: 0.5 } });

    expect(calls[0].url).toBe('/api/v1/jobs');
    expect(calls[0].method).toBe('POST');
    // Without this header `rux gui` answers 415 — it is what a forged
    // "simple request" cannot set.
    expect(calls[0].headers?.['Content-Type']).toBe('application/json');
    expect(JSON.parse(calls[0].body ?? '')).toEqual({
      stage: 'planes',
      parameters: { radius: 0.5 },
    });
    expect(job.id).toBe(JOBS.jobs[0].id);
  });

  it('POSTs a cancellation to /jobs/{id}/cancel', async () => {
    const { api, calls } = clientFor(JOBS.jobs[0]);
    await api.cancelJob('dfbc481b-b2f7-4c68-aa29-3cfd9bde0611');
    expect(calls[0].url).toBe('/api/v1/jobs/dfbc481b-b2f7-4c68-aa29-3cfd9bde0611/cancel');
    expect(calls[0].method).toBe('POST');
    expect(calls[0].headers?.['Content-Type']).toBe('application/json');
    expect(calls[0].body).toBe('{}');
  });

  it('percent-encodes a job id on the cancel route', async () => {
    const { api, calls } = clientFor(JOBS.jobs[0]);
    await api.cancelJob('a/b');
    expect(calls[0].url).toBe('/api/v1/jobs/a%2Fb/cancel');
  });
});

describe('error mapping', () => {
  it('maps a 404 body to ApiRequestError.isNotFound with the server message', async () => {
    const { api } = clientFor(ERROR_NOT_FOUND, { status: 404, statusText: 'Not Found' });
    const failure = await api.cloud('nope').catch((error: unknown) => error);

    expect(failure).toBeInstanceOf(ApiRequestError);
    const error = failure as ApiRequestError;
    expect(error.status).toBe(404);
    expect(error.isNotFound).toBe(true);
    expect(error.isNotImplemented).toBe(false);
    expect(error.isRetryable).toBe(false);
    // The server's own wording, not a client-invented one.
    expect(error.message).toBe("no such cloud 'nope'");
    expect(error.url).toBe('/api/v1/clouds/nope');
  });

  it('maps a 501 body to isNotImplemented', async () => {
    const { api } = clientFor(ERROR_NOT_IMPLEMENTED, {
      status: 501,
      statusText: 'Not Implemented',
    });
    const error = (await api
      .cloudPoints('cloud', { format: 'binary' })
      .catch((failure: unknown) => failure)) as ApiRequestError;

    expect(error).toBeInstanceOf(ApiRequestError);
    expect(error.status).toBe(501);
    expect(error.isNotImplemented).toBe(true);
    expect(error.isNotFound).toBe(false);
    expect(error.message).toBe(ERROR_NOT_IMPLEMENTED.error);
  });

  it('maps a 503 to isRetryable — the database was momentarily locked', async () => {
    const { api } = clientFor(
      { error: 'project database is locked by a running job', status: 503 },
      { status: 503, statusText: 'Service Unavailable' },
    );
    const error = (await api.clouds().catch((failure: unknown) => failure)) as ApiRequestError;

    expect(error.status).toBe(503);
    expect(error.isRetryable).toBe(true);
    expect(error.message).toBe('project database is locked by a running job');
  });

  it('falls back to statusText when a failure body is not JSON', async () => {
    // A wedged server (or a proxy) can answer HTML. `describeFailure` must not
    // throw a SyntaxError while it is building the throw — the caller has to
    // see an ApiRequestError carrying the status either way.
    const { api } = clientFor('<html><body>502 Bad Gateway</body></html>', {
      status: 502,
      statusText: 'Bad Gateway',
    });
    const failure = await api.clouds().catch((error: unknown) => error);

    expect(failure).toBeInstanceOf(ApiRequestError);
    expect((failure as ApiRequestError).status).toBe(502);
    expect((failure as ApiRequestError).message).toBe('Bad Gateway');
    expect((failure as Error).name).not.toBe('SyntaxError');
  });

  it('falls back to the status code when there is neither JSON nor statusText', async () => {
    const { api } = clientFor('', { status: 500, statusText: '' });
    const error = (await api.clouds().catch((failure: unknown) => failure)) as ApiRequestError;
    expect(error).toBeInstanceOf(ApiRequestError);
    expect(error.message).toBe('HTTP 500');
  });

  it('ignores an empty `error` string and falls back rather than throwing blank', async () => {
    const { api } = clientFor({ error: '', status: 404 }, { status: 404, statusText: 'Not Found' });
    const error = (await api.clouds().catch((failure: unknown) => failure)) as ApiRequestError;
    expect(error.message).toBe('Not Found');
  });

  it('surfaces a failure from a mutating route too', async () => {
    const { api } = clientFor(
      { error: 'stage is not runnable', status: 409 },
      { status: 409, statusText: 'Conflict' },
    );
    const error = (await api
      .submitJob({ stage: 'mesh' })
      .catch((failure: unknown) => failure)) as ApiRequestError;
    expect(error).toBeInstanceOf(ApiRequestError);
    expect(error.status).toBe(409);
    expect(error.message).toBe('stage is not runnable');
  });
});

describe('eventsUrl', () => {
  it('derives ws:// from an http origin', () => {
    const { api } = clientFor(HEALTH);
    expect(api.eventsUrl({ protocol: 'http:', host: 'localhost:8420' })).toBe(
      'ws://localhost:8420/api/v1/events',
    );
  });

  it('derives wss:// from an https origin', () => {
    const { api } = clientFor(HEALTH);
    expect(api.eventsUrl({ protocol: 'https:', host: 'gui.example.org' })).toBe(
      'wss://gui.example.org/api/v1/events',
    );
  });

  it('takes only the path from an absolute base URL', () => {
    const { fetchLike } = stubFetch(HEALTH);
    const api = new RuxApiClient({ baseUrl: 'http://elsewhere:9000/api/v1', fetch: fetchLike });
    expect(api.eventsUrl({ protocol: 'http:', host: 'localhost:8420' })).toBe(
      'ws://localhost:8420/api/v1/events',
    );
  });
});
