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
  GsplatInfo,
  Health,
  InstanceInfo,
  Job,
  JobRequest,
  LabelLegend,
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
 * `501` (`format=binary` against a server that predates #283 — fall back to
 * JSON), `503` (the project database was momentarily locked by a running job —
 * retry), `403` (origin not allowlisted) and `415` (mutating route without the
 * JSON content type).
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

  /**
   * The edit conflicts with the project's state and was **not** applied.
   *
   * Deliberately distinct from {@link isRetryable}: 503 means "the writer lock
   * was busy for a moment, send it again", while 409 means either a pipeline
   * job holds the lock for the duration of a stage — retrying now will fail
   * again — or the server refuses this edit outright and always will. Telling
   * the user to "try again" for a 409 is advice that cannot work.
   */
  get isConflict(): boolean {
    return this.status === 409;
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
 * How much of a cloud to ask for, in one of the endpoint's two modes.
 *
 * `offset`/`limit` take a **window** of the cloud in storage order.
 * `maxPoints` takes a spatially representative view of **all** of it (#320) —
 * a different question, so the server refuses the two together rather than
 * inventing a meaning for an offset into a set the client cannot enumerate.
 * That mutual exclusion is enforced here too, one layer earlier, so a caller
 * gets a `TypeError` at the call site instead of a 400 over the wire.
 */
export interface CloudPointsQuery {
  offset?: number;
  limit?: number;
  /**
   * Budget for a level-of-detail read of the whole cloud. The answer holds at
   * most this many points, and — because the underlying grid is dyadic — may
   * hold as few as roughly a quarter of them.
   */
  maxPoints?: number;
  /**
   * Cloud whose positions drive the selection, for a sibling that has none of
   * its own (a `Label` cloud). The two must be index-aligned; the resulting
   * pages then describe the same points and can still be zipped positionally.
   */
  lodSource?: string;
}

function pointsQuery(options: CloudPointsQuery): Query {
  if (options.maxPoints !== undefined && (options.offset !== undefined || options.limit !== undefined)) {
    throw new TypeError('maxPoints cannot be combined with offset or limit');
  }
  return {
    offset: options.offset,
    limit: options.limit,
    max_points: options.maxPoints,
    lod_source: options.lodSource,
  };
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

  /**
   * A sparse edit against an existing resource.
   *
   * Same shape as {@link postJson}, and in particular the same
   * `Content-Type: application/json` — the server's CSRF gate keys on "is this
   * method mutating", not on POST, so a PATCH without the header is a 415 too.
   */
  private async patchJson<T>(path: string, body: unknown, signal?: AbortSignal): Promise<T> {
    const url = this.url(path);
    const response = await this.doFetch(url, {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
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
   * One page of points, as JSON rows.
   *
   * `format` is sent explicitly even when the caller omits it. The contract's
   * default is `json` today, but this method decodes the JSON shape
   * unconditionally — so the request must name the format it is prepared to
   * parse rather than inherit whatever a future server decides to default to.
   * For the binary transport use {@link cloudPointsBinary}, which is a
   * different *return type*, not a different argument.
   */
  cloudPoints(
    name: string,
    options: CloudPointsQuery & { format?: 'json' | 'binary' } = {},
    signal?: AbortSignal,
  ): Promise<CloudPointsPage> {
    return this.requestJson<CloudPointsPage>(
      `/clouds/${encodeURIComponent(name)}/points`,
      { ...pointsQuery(options), format: options.format ?? 'json' },
      signal,
    );
  }

  /**
   * One page of points as a RUXP v1 body (`docs/gui/binary-points.md`, #283).
   *
   * Returns the raw `ArrayBuffer` rather than a parsed page: parsing belongs to
   * the viewport (`viewport/binaryPoints.ts`), and keeping the two apart is what
   * lets this client stay the one place that knows about HTTP.
   *
   * A server that predates the format answers **501**, which arrives as an
   * `ApiRequestError` with `isNotImplemented` — the signal a caller uses to
   * fall back to {@link cloudPoints} for the rest of its stream. Note that this
   * client does **not** read the `X-Ruxp-*` response headers: they are a `curl`
   * convenience, and the spec is explicit that the body header is the contract.
   */
  async cloudPointsBinary(
    name: string,
    options: CloudPointsQuery = {},
    signal?: AbortSignal,
  ): Promise<ArrayBuffer> {
    const url = this.url(`/clouds/${encodeURIComponent(name)}/points`, {
      ...pointsQuery(options),
      format: 'binary',
    });
    const response = await this.doFetch(url, { method: 'GET', signal });
    if (!response.ok) {
      // The failure body is still JSON on this route — the contract's `Error`
      // object — so the shared reason extraction applies unchanged.
      throw new ApiRequestError(response.status, await describeFailure(response), url);
    }
    return response.arrayBuffer();
  }

  /**
   * The label legend of one `Label` cloud.
   *
   * The same map `GET /clouds/{name}` embeds; this route exists so an editor
   * can re-read just the legend after a write instead of the whole record.
   */
  async cloudLabels(name: string, signal?: AbortSignal): Promise<Record<string, string>> {
    const body = await this.requestJson<LabelLegend>(
      `/clouds/${encodeURIComponent(name)}/labels`,
      undefined,
      signal,
    );
    return body.labels;
  }

  /**
   * Rename label classes. **Sparse** — send only the ids that changed.
   *
   * Every id must already exist in the legend and every name must be non-empty
   * (both 400), and the `instances` cloud is refused with 409 because its names
   * are `SM<class>-<id> (<n>p)` records the pipeline parses back, not captions.
   * Returns the full legend after the rename, not just the edited ids.
   */
  async patchCloudLabels(
    name: string,
    labels: Record<string, string>,
    signal?: AbortSignal,
  ): Promise<Record<string, string>> {
    const body = await this.patchJson<LabelLegend>(
      `/clouds/${encodeURIComponent(name)}/labels`,
      { labels },
      signal,
    );
    return body.labels;
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

  // ---------------------------------------------------------- gsplats ----

  /**
   * Gaussian splats stored in this project.
   *
   * An empty array is the normal answer for a project nobody has run
   * `rux create gsplat` on — not a `404`, and not an error. Unpaged by
   * contract: a project holds a handful of splats, not a number that grows
   * with the size of the scan.
   */
  async gsplats(signal?: AbortSignal): Promise<GsplatInfo[]> {
    const body = await this.requestJson<{ gsplats: GsplatInfo[] }>(
      '/gsplats',
      undefined,
      signal,
    );
    return body.gsplats;
  }

  gsplat(name: string, signal?: AbortSignal): Promise<GsplatInfo> {
    return this.requestJson<GsplatInfo>(
      `/gsplats/${encodeURIComponent(name)}`,
      undefined,
      signal,
    );
  }

  /**
   * URL of one splat's PLY blob, for the renderer to fetch itself.
   *
   * A URL rather than an `ArrayBuffer`: the splat loader streams and parses the
   * file on a worker, and routing hundreds of megabytes through this client
   * first would buffer the whole thing twice for no gain. The same reasoning as
   * {@link meshDataUrl}.
   */
  gsplatDataUrl(name: string): string {
    return this.url(`/gsplats/${encodeURIComponent(name)}/data`);
  }

  // ----------------------------------------------------------- frames ----

  /**
   * Frame ids, optionally narrowed to those that do or do not carry a mask.
   *
   * `segmented` is a **server-side** filter and must be used as one: the
   * client-side alternative is `GET /frames/{id}` per frame just to read one
   * boolean, and that route decodes the depth and confidence blobs to answer.
   *
   * `total_count` / `segmented_count` always describe the whole scan, never the
   * filtered result — use `ids.length` for the size of the filtered set.
   */
  frames(
    options: { segmented?: boolean } = {},
    signal?: AbortSignal,
  ): Promise<FrameList> {
    return this.requestJson<FrameList>('/frames', { segmented: options.segmented }, signal);
  }

  frame(id: number, signal?: AbortSignal): Promise<FrameInfo> {
    return this.requestJson<FrameInfo>(`/frames/${id}`, undefined, signal);
  }

  /**
   * URL of one of a frame's images, in its **stored** orientation.
   *
   * `max_size` downscales the longest edge — what makes a grid of hundreds of
   * thumbnails affordable. `normalize` asks for a *displayable* rendering
   * rather than the stored measurement, and is required for `depth`,
   * `confidence` and `segmentation`: those are 16-bit single-channel PNGs, and
   * a browser decoding a 3 m room stored in millimetres (3000 of 65535) paints
   * near-black. The normalised image carries **no metric scale**.
   *
   * The server reports the mapped range in `X-Image-Range-Min`/`-Max`, which an
   * `<img>` cannot read. That is deliberate on both sides: they are a `curl`
   * convenience, so nothing here is built on them.
   */
  frameImageUrl(
    id: number,
    kind: FrameImageKind = 'color',
    options: { maxSize?: number; normalize?: boolean } = {},
  ): string {
    return this.url(`/frames/${id}/image`, {
      kind,
      max_size: options.maxSize,
      // Sent only when true: the contract's default is already false, and an
      // explicit `normalize=false` on every colour thumbnail is noise in a log.
      normalize: options.normalize ? true : undefined,
    });
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

  /**
   * URL of the equirectangular JPEG, in its **stored** orientation.
   *
   * `maxSize` downscales the longest edge, and a picker must use it: a stored
   * equirect is routinely 8192x4096 and several megabytes, so a strip of
   * twenty 96-pixel thumbnails is otherwise tens of megabytes.
   */
  panoramaImageUrl(id: number, options: { maxSize?: number } = {}): string {
    return this.url(`/panoramas/${id}/image`, { max_size: options.maxSize });
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

  /**
   * Add, change or clear passport properties. **Sparse** — only what changed.
   *
   * A `null` value deletes the property; a string sets it. Anything not named
   * is left alone, including the MaterialEPAS fields this GUI does not model.
   * Returns the passport as `GET` would render it after the edit.
   */
  patchMaterial(
    guid: string,
    properties: Record<string, string | null>,
    signal?: AbortSignal,
  ): Promise<MaterialDetail> {
    return this.patchJson<MaterialDetail>(
      `/materials/${encodeURIComponent(guid)}`,
      { properties },
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

  /**
   * Re-check one stage's input contract.
   *
   * Returns the same record `/stages` does for that stage, so a card can be
   * substituted in place after a run finishes rather than refetching the whole
   * catalogue to learn that `clouds` succeeding unblocked `planes`.
   */
  stageValidation(stage: string, signal?: AbortSignal): Promise<StageInfo> {
    return this.requestJson<StageInfo>(
      `/stages/${encodeURIComponent(stage)}/validation`,
      undefined,
      signal,
    );
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
