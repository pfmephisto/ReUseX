// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The one HTTP client for the ReUseX GUI API (`docs/gui/openapi.yaml`, v1.0.0).
 *
 * Deliberately a single module: every assumption this frontend makes about the
 * contract lives here, so contract polish (#285) and the binary point transport
 * (#283) each have exactly one file to change.
 *
 * Two rules from `docs/gui/README.md` are baked in and must not be worked
 * around in calling code:
 *
 *  1. **Same-origin only.** The default base URL is a relative `/api/v1`. In
 *     development, Vite's `server.proxy` forwards it to `rux gui`, so the
 *     browser never makes a cross-origin request. `rux gui` cannot answer a
 *     CORS preflight (Crow 1.3 replies to `OPTIONS` before it has parsed the
 *     request headers), so any JSON-bodied cross-origin call would fail.
 *  2. **Mutating routes carry `Content-Type: application/json`.** The server
 *     requires it — it is what a forged "simple request" cannot set.
 */

import type {
  CloudInfo,
  CloudPointsPage,
  ComponentDetail,
  ComponentInfo,
  EndpointInfo,
  FrameInfo,
  FrameImageKind,
  FrameList,
  Health,
  InstanceInfo,
  Job,
  JobRequest,
  MaterialDetail,
  MaterialInfo,
  MeshInfo,
  PanoramaInfo,
  PipelineLogEntry,
  ProjectInfo,
  ProjectSummary,
  StageInfo,
  TextureInfo,
} from './types';

/** Default base path. Relative on purpose — see rule 1 above. */
export const DEFAULT_BASE_URL = '/api/v1';

/**
 * A non-2xx response, carrying the status so a caller can branch on it.
 *
 * The statuses worth branching on, per the contract: `404` (no such resource),
 * `501` (`format=binary`, not implemented until #283), `503` (the project
 * database was momentarily locked by a running job — retry), `403` (origin not
 * allowlisted) and `415` (mutating route without the JSON content type).
 */
export class ApiRequestError extends Error {
  readonly status: number;
  readonly url: string;

  constructor(status: number, message: string, url: string) {
    super(message);
    this.name = 'ApiRequestError';
    this.status = status;
    this.url = url;
  }

  /** The resource does not exist. */
  get isNotFound(): boolean {
    return this.status === 404;
  }

  /** The server understands the request but has not implemented it yet. */
  get isNotImplemented(): boolean {
    return this.status === 501;
  }

  /** Transient: the database was locked by a running job. Safe to retry. */
  get isRetryable(): boolean {
    return this.status === 503;
  }
}

/** Injectable `fetch`, so tests need no network and no global patching. */
export type FetchLike = (
  input: string,
  init?: { method?: string; headers?: Record<string, string>; body?: string; signal?: AbortSignal },
) => Promise<Response>;

export interface ClientOptions {
  /** Base URL including the version prefix. Defaults to `/api/v1`. */
  baseUrl?: string;
  /** Override the transport. Defaults to the global `fetch`. */
  fetch?: FetchLike;
}

/** Query parameters, with `undefined` entries dropped rather than serialised. */
type Query = Record<string, string | number | boolean | undefined>;

function buildQuery(query?: Query): string {
  if (!query) return '';
  const params = new URLSearchParams();
  for (const [key, value] of Object.entries(query)) {
    if (value !== undefined) params.set(key, String(value));
  }
  const encoded = params.toString();
  return encoded ? `?${encoded}` : '';
}

/**
 * Pull a human-readable reason out of a failed response.
 *
 * The contract says every non-2xx body is an `Error` object with a non-empty
 * `error`. A server that is wedged badly enough may not manage that, so this
 * degrades to the status text rather than throwing while building a throw.
 */
async function describeFailure(response: Response): Promise<string> {
  try {
    const body = (await response.json()) as { error?: unknown };
    if (typeof body?.error === 'string' && body.error.length > 0) return body.error;
  } catch {
    /* not JSON, or truncated — fall through */
  }
  return response.statusText || `HTTP ${response.status}`;
}

export class RuxApiClient {
  private readonly baseUrl: string;
  private readonly doFetch: FetchLike;

  constructor(options: ClientOptions = {}) {
    // Trailing slashes would produce `//clouds`, which the router does not match.
    this.baseUrl = (options.baseUrl ?? DEFAULT_BASE_URL).replace(/\/+$/, '');
    this.doFetch = options.fetch ?? ((input, init) => fetch(input, init));
  }

  /** Absolute (or root-relative) URL for a contract path. Public for <img src>. */
  url(path: string, query?: Query): string {
    return `${this.baseUrl}${path}${buildQuery(query)}`;
  }

  private async requestJson<T>(path: string, query?: Query, signal?: AbortSignal): Promise<T> {
    const url = this.url(path, query);
    const response = await this.doFetch(url, { method: 'GET', signal });
    if (!response.ok) {
      throw new ApiRequestError(response.status, await describeFailure(response), url);
    }
    return (await response.json()) as T;
  }

  private async postJson<T>(path: string, body?: unknown, signal?: AbortSignal): Promise<T> {
    const url = this.url(path);
    const response = await this.doFetch(url, {
      method: 'POST',
      // Required by the server on every mutating route — see rule 2 above.
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body ?? {}),
      signal,
    });
    if (!response.ok) {
      throw new ApiRequestError(response.status, await describeFailure(response), url);
    }
    return (await response.json()) as T;
  }

  // ------------------------------------------------------------- meta ----

  health(signal?: AbortSignal): Promise<Health> {
    return this.requestJson<Health>('/health', undefined, signal);
  }

  async endpoints(signal?: AbortSignal): Promise<EndpointInfo[]> {
    const body = await this.requestJson<{ endpoints: EndpointInfo[] }>(
      '/endpoints',
      undefined,
      signal,
    );
    return body.endpoints;
  }

  // ---------------------------------------------------------- project ----

  projectSummary(signal?: AbortSignal): Promise<ProjectSummary> {
    return this.requestJson<ProjectSummary>('/project', undefined, signal);
  }

  async projects(signal?: AbortSignal): Promise<ProjectInfo[]> {
    const body = await this.requestJson<{ projects: ProjectInfo[] }>(
      '/projects',
      undefined,
      signal,
    );
    return body.projects;
  }

  // ----------------------------------------------------------- clouds ----

  async clouds(signal?: AbortSignal): Promise<CloudInfo[]> {
    const body = await this.requestJson<{ clouds: CloudInfo[] }>('/clouds', undefined, signal);
    return body.clouds;
  }

  cloud(name: string, signal?: AbortSignal): Promise<CloudInfo> {
    return this.requestJson<CloudInfo>(`/clouds/${encodeURIComponent(name)}`, undefined, signal);
  }

  /**
   * One page of points.
   *
   * `format` is the forward-compatibility seam for #283: the JSON shape will
   * not change when `binary` arrives, so the page-walking logic above this can
   * stay put and only the decode step is replaced. Asking for `binary` today
   * raises an `ApiRequestError` with `isNotImplemented`.
   *
   * It is sent explicitly even when the caller omits it. The contract's default
   * is `json` today, but this client decodes the JSON shape unconditionally —
   * so the request must name the format it is prepared to parse rather than
   * inherit whatever a future server decides to default to.
   */
  cloudPoints(
    name: string,
    options: { offset?: number; limit?: number; format?: 'json' | 'binary' } = {},
    signal?: AbortSignal,
  ): Promise<CloudPointsPage> {
    return this.requestJson<CloudPointsPage>(
      `/clouds/${encodeURIComponent(name)}/points`,
      { offset: options.offset, limit: options.limit, format: options.format ?? 'json' },
      signal,
    );
  }

  // ----------------------------------------------------------- meshes ----

  async meshes(signal?: AbortSignal): Promise<MeshInfo[]> {
    const body = await this.requestJson<{ meshes: MeshInfo[] }>('/meshes', undefined, signal);
    return body.meshes;
  }

  mesh(name: string, signal?: AbortSignal): Promise<MeshInfo> {
    return this.requestJson<MeshInfo>(`/meshes/${encodeURIComponent(name)}`, undefined, signal);
  }

  /** URL of the raw geometry blob, in the mesh's stored format. */
  meshDataUrl(name: string): string {
    return this.url(`/meshes/${encodeURIComponent(name)}/data`);
  }

  async meshTextures(name: string, signal?: AbortSignal): Promise<TextureInfo[]> {
    const body = await this.requestJson<{ textures: TextureInfo[] }>(
      `/meshes/${encodeURIComponent(name)}/textures`,
      undefined,
      signal,
    );
    return body.textures;
  }

  meshTextureUrl(name: string, texture: string): string {
    return this.url(
      `/meshes/${encodeURIComponent(name)}/textures/${encodeURIComponent(texture)}`,
    );
  }

  // ----------------------------------------------------------- frames ----

  frames(signal?: AbortSignal): Promise<FrameList> {
    return this.requestJson<FrameList>('/frames', undefined, signal);
  }

  frame(id: number, signal?: AbortSignal): Promise<FrameInfo> {
    return this.requestJson<FrameInfo>(`/frames/${id}`, undefined, signal);
  }

  /** URL of one of a frame's images, in its **stored** orientation. */
  frameImageUrl(id: number, kind: FrameImageKind = 'color'): string {
    return this.url(`/frames/${id}/image`, { kind });
  }

  // -------------------------------------------------------- panoramas ----

  async panoramas(signal?: AbortSignal): Promise<PanoramaInfo[]> {
    const body = await this.requestJson<{ panoramas: PanoramaInfo[] }>(
      '/panoramas',
      undefined,
      signal,
    );
    return body.panoramas;
  }

  panorama(id: number, signal?: AbortSignal): Promise<PanoramaInfo> {
    return this.requestJson<PanoramaInfo>(`/panoramas/${id}`, undefined, signal);
  }

  panoramaImageUrl(id: number): string {
    return this.url(`/panoramas/${id}/image`);
  }

  // ------------------------------------------------------- components ----

  async components(type?: string, signal?: AbortSignal): Promise<ComponentInfo[]> {
    const body = await this.requestJson<{ components: ComponentInfo[] }>(
      '/components',
      { type },
      signal,
    );
    return body.components;
  }

  component(name: string, signal?: AbortSignal): Promise<ComponentDetail> {
    return this.requestJson<ComponentDetail>(
      `/components/${encodeURIComponent(name)}`,
      undefined,
      signal,
    );
  }

  // -------------------------------------------------------- materials ----

  async materials(signal?: AbortSignal): Promise<MaterialInfo[]> {
    const body = await this.requestJson<{ materials: MaterialInfo[] }>(
      '/materials',
      undefined,
      signal,
    );
    return body.materials;
  }

  material(guid: string, signal?: AbortSignal): Promise<MaterialDetail> {
    return this.requestJson<MaterialDetail>(
      `/materials/${encodeURIComponent(guid)}`,
      undefined,
      signal,
    );
  }

  // -------------------------------------------------------- instances ----

  async instances(cloud: string, signal?: AbortSignal): Promise<InstanceInfo[]> {
    const body = await this.requestJson<{ cloud: string; instances: InstanceInfo[] }>(
      `/instances/${encodeURIComponent(cloud)}`,
      undefined,
      signal,
    );
    return body.instances;
  }

  // --------------------------------------------------------- pipeline ----

  async stages(signal?: AbortSignal): Promise<StageInfo[]> {
    const body = await this.requestJson<{ stages: StageInfo[] }>('/stages', undefined, signal);
    return body.stages;
  }

  async pipelineLog(limit?: number, signal?: AbortSignal): Promise<PipelineLogEntry[]> {
    const body = await this.requestJson<{ entries: PipelineLogEntry[] }>(
      '/pipeline-log',
      { limit },
      signal,
    );
    return body.entries;
  }

  // ------------------------------------------------------------- jobs ----

  async jobs(signal?: AbortSignal): Promise<Job[]> {
    const body = await this.requestJson<{ jobs: Job[] }>('/jobs', undefined, signal);
    return body.jobs;
  }

  job(id: string, signal?: AbortSignal): Promise<Job> {
    return this.requestJson<Job>(`/jobs/${encodeURIComponent(id)}`, undefined, signal);
  }

  /**
   * Submit a stage run.
   *
   * The returned `status` is a snapshot at response time and is **not** to be
   * asserted on — an idle server may already have started, or even failed, the
   * job. Drive the UI from `/events` or from a poll.
   */
  submitJob(request: JobRequest, signal?: AbortSignal): Promise<Job> {
    return this.postJson<Job>('/jobs', request, signal);
  }

  /** Request cancellation. Idempotent; the outcome arrives as `job.finished`. */
  cancelJob(id: string, signal?: AbortSignal): Promise<Job> {
    return this.postJson<Job>(`/jobs/${encodeURIComponent(id)}/cancel`, {}, signal);
  }

  // -------------------------------------------------------- websocket ----

  /**
   * WebSocket URL for the event channel, derived from the API base.
   *
   * Resolved against `location` so a relative base (the normal case) produces
   * an absolute `ws://`/`wss://` URL — `new WebSocket()` requires one.
   */
  eventsUrl(origin?: { protocol: string; host: string }): string {
    const loc = origin ?? (typeof location !== 'undefined' ? location : undefined);
    if (!loc) return `${this.baseUrl}/events`;
    const scheme = loc.protocol === 'https:' ? 'wss:' : 'ws:';
    const path = this.baseUrl.startsWith('http')
      ? new URL(this.baseUrl).pathname
      : this.baseUrl;
    return `${scheme}//${loc.host}${path}/events`;
  }
}

/** The client the app uses. Same-origin, default base path. */
export const api = new RuxApiClient();
