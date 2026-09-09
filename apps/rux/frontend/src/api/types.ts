// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Wire types for the ReUseX GUI API.
 *
 * Hand-written mirror of the `components.schemas` section of
 * `docs/gui/openapi.yaml` (contract version 1.0.0). These are the *wire* shapes
 * — no convenience fields, no renaming — so that a diff against the spec is a
 * literal one. Anything derived belongs in the component that derives it.
 *
 * Optional (`?`) here means "the spec does not list it under `required`".
 */

/** `Error` — the body of every non-2xx response. */
export interface ApiError {
  error: string;
  status?: number;
}

/** `Health` — liveness probe and version handshake. */
export interface Health {
  status: 'ok';
  api_version: string;
  version: string;
  implementation?: 'rux-gui' | 'ruxd';
  project: {
    name: string;
    open: boolean;
    schema_version?: number;
  };
}

/** `EndpointInfo` — one row of the server's self-described route table. */
export interface EndpointInfo {
  method: string;
  path: string;
  summary: string;
  binary?: boolean;
}

/** `ProjectInfo` — a project metadata record. */
export interface ProjectInfo {
  id: string;
  name: string;
  building_address?: string;
  /** 0 means "not set". */
  year_of_construction?: number;
  survey_date?: string;
  survey_organisation?: string;
  notes?: string;
}

export type CloudType = 'PointXYZRGB' | 'Normal' | 'Label' | 'PointXYZ';

/** `CloudInfo` — one named point cloud. */
export interface CloudInfo {
  name: string;
  type: CloudType;
  point_count: number;
  width: number;
  height: number;
  organized: boolean;
  /**
   * Label id → name, for `Label` clouds only. Key `"0"` is never present:
   * 0 means unlabeled (STANDARDS §3).
   */
  labels?: Record<string, string>;
}

/** `CloudPointsPage` — one page of point rows. */
export interface CloudPointsPage {
  name: string;
  type: string;
  offset: number;
  /** Points in *this* page. */
  count: number;
  /** Points in the whole cloud. */
  total: number;
  /** Field names, in the order they appear in each row of `points`. */
  fields: string[];
  points: number[][];
}

/** `MeshInfo` — one stored mesh. */
export interface MeshInfo {
  name: string;
  format?: 'ply' | 'obj';
  vertex_count: number;
  face_count: number;
  stage?: string;
  parameters?: string;
  created_at?: string;
  texture_count?: number;
}

/** `TextureInfo` — texture metadata (never the image bytes). */
export interface TextureInfo {
  tex_name: string;
  format: string;
  width: number;
  height: number;
}

/** `FrameList` — sensor frame ids plus aggregate counts. */
export interface FrameList {
  ids: number[];
  total_count: number;
  segmented_count: number;
  width?: number;
  height?: number;
}

/** `Intrinsics` — pinhole intrinsics of one sensor. */
export interface Intrinsics {
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  width: number;
  height: number;
  /** Row-major 4x4 camera-to-base transform. */
  local_transform?: number[];
}

/** `FrameInfo` — one sensor frame. */
export interface FrameInfo {
  id: number;
  /** Epoch seconds; -1 when unknown. */
  timestamp?: number;
  /** Row-major 4x4 world pose. */
  pose: number[];
  intrinsics?: Intrinsics;
  has_depth?: boolean;
  has_confidence?: boolean;
  has_segmentation: boolean;
}

export type FrameImageKind = 'color' | 'depth' | 'confidence' | 'segmentation';

/** `PanoramaInfo` — one 360 panorama with its pose provenance. */
export interface PanoramaInfo {
  id: number;
  filename: string;
  timestamp?: number;
  /** Matched sensor frame, -1 if unmatched. */
  node_id: number;
  has_pose: boolean;
  pose?: number[];
  pose_source: 'timestamp' | 'aligned';
  align_inliers?: number;
  /** Angular RMS of inlier bearings, degrees. -1 if unaligned. */
  align_rms?: number;
}

/** `ComponentInfo` — one building component. */
export interface ComponentInfo {
  name: string;
  guid: string;
  type: string;
  /**
   * Parent **component**, -1 if none. Not a room: ReUseX does not associate a
   * component with a room, and this must not be presented as if it did.
   */
  parent_id?: number;
  /** -1 means manually created. */
  confidence?: number;
  vertex_count?: number;
  /**
   * Boundary area in m², derived on read (Newell) rather than stored. Absent
   * when the boundary has fewer than three vertices — so it is genuinely
   * optional and must never be rendered as `NaN`.
   */
  area?: number;
  /**
   * The segmentation instance this component came from (#211), lifted out of
   * the opaque `metadata` JSON. Absent for a manually created component.
   */
  source_instance_guid?: string;
}

/** `ComponentDetail` — a component plus its boundary polygon. */
export interface ComponentDetail extends ComponentInfo {
  /** Hessian normal form [a, b, c, d]. */
  plane?: number[];
  /** Boundary vertices as [x, y, z] triples. */
  vertices?: number[][];
  metadata?: string;
  notes?: string;
}

/**
 * `LabelLegend` — label id → display name, for one `Label` cloud.
 *
 * Both the body of `GET /clouds/{name}/labels` and the body of the `PATCH`,
 * which is why it is one type: the patch is a *sparse* legend, carrying only
 * the ids that changed. `"0"` never appears — 0 means unlabeled.
 */
export interface LabelLegend {
  labels: Record<string, string>;
}

/**
 * `MaterialPatch` — the body of `PATCH /materials/{guid}`.
 *
 * Sparse: only the properties that changed. A `null` value **deletes** the
 * property; a string sets it. Omitting a property leaves it untouched, which
 * is the whole reason this is not a PUT — passports carry MaterialEPAS fields
 * no GUI form models, and a PUT would silently drop them.
 */
export interface MaterialPatch {
  properties: Record<string, string | null>;
}

/** `MaterialInfo` — a material passport, listing shape. */
export interface MaterialInfo {
  id?: string;
  guid: string;
  property_count?: number;
  created_at?: string;
  version_number?: string;
}

/** `MaterialDetail` — a passport with its stored properties. */
export interface MaterialDetail extends MaterialInfo {
  linked_node_id?: number;
  properties?: Record<string, string>;
}

/** `InstanceInfo` — one instance row of an instance-label cloud. */
export interface InstanceInfo {
  /** Label value in the instance cloud (>= 1). */
  instance_id: number;
  guid: string;
  semantic_class: number;
  point_count: number;
  /** Linked material passport, null when unlinked. */
  material_guid?: string | null;
}

/** `ValidationIssue` — one finding of a stage's input-contract check. */
export interface ValidationIssue {
  /** Machine-readable check id, e.g. `missing_stage_input`. */
  check: string;
  message: string;
  severity: 'warning' | 'error';
  /** Terminal-formatted resolution, possibly multi-line. Empty when none. */
  hint: string;
  /** The named cloud/table this is about; empty when it is about nothing named. */
  artifact: string;
  /** The same resolution as `hint`, in order, as separate commands. */
  commands: string[];
}

export type ParameterType = 'number' | 'integer' | 'boolean' | 'string' | 'integer_list';

/** `StageParameter` — one knob of a runnable stage, as the server describes it. */
export interface StageParameter {
  /** The key to send inside `JobRequest.parameters`. */
  key: string;
  type: ParameterType;
  label: string;
  description: string;
  /** Null means the parameter is absent by default and has no neutral value. */
  default: number | boolean | string | null;
  minimum: number | null;
  maximum: number | null;
  /** True when sending the key at all changes behaviour, whatever its value. */
  presence_sensitive: boolean;
}

/** `StageInfo` — one entry of the stage catalogue. */
export interface StageInfo {
  stage: string;
  /** What this stage writes into `pipeline_log.stage`; empty when no runner. */
  log_name: string;
  summary: string;
  command: string;
  runnable: boolean;
  cancellable: boolean;
  ready: boolean;
  /** Artifacts this stage writes. */
  outputs: string[];
  /** Printable summary of the error-severity `issues`. */
  blockers: string[];
  issues: ValidationIssue[];
  /** Empty for a stage with no runner. */
  parameters: StageParameter[];
}

/** `PipelineLogEntry` — one durable stage-execution record. */
export interface PipelineLogEntry {
  id: number;
  stage: string;
  status: 'running' | 'success' | 'failed';
  started_at: string;
  /** Empty while still running. */
  finished_at?: string;
  /** Stage parameters JSON, as stored. Carries `job_id` for GUI-started runs. */
  parameters?: string;
  error_msg?: string;
}

export type JobStatus = 'queued' | 'running' | 'succeeded' | 'failed' | 'cancelled';

/** `JobProgress` — live progress of the phase a stage is currently in. */
export interface JobProgress {
  /** Machine-readable `core::Stage` token, e.g. `region_growing`. Match on this. */
  stage: string;
  /** Display form of the same phase. Never match on this. */
  stage_label: string;
  current: number;
  /** 0 means indeterminate — render a spinner, not a bar. */
  total: number;
  fraction?: number | null;
}

/** `Job` — one submitted stage run. */
export interface Job {
  id: string;
  project: string;
  stage: string;
  status: JobStatus;
  parameters?: Record<string, unknown>;
  error?: string;
  submitted_at: string;
  started_at?: string;
  finished_at?: string;
  cancel_requested?: boolean;
  progress?: JobProgress;
}

/** `JobRequest` — the body of `POST /jobs`. */
export interface JobRequest {
  project?: string;
  stage: string;
  parameters?: Record<string, unknown>;
}

/** `ProjectSummary` — the dashboard payload, in one request. */
export interface ProjectSummary {
  /** File name only — the server never discloses its filesystem layout. */
  path: string;
  schema_version: number;
  projects: ProjectInfo[];
  clouds: CloudInfo[];
  meshes: MeshInfo[];
  sensor_frames: {
    total_count: number;
    segmented_count: number;
    width?: number;
    height?: number;
  };
  panoramic_images: {
    total_count: number;
    /** Panoramas linked to a sensor frame. */
    matched_count: number;
  };
  components: {
    total_count: number;
    count_by_type: Record<string, number>;
  };
  materials: MaterialInfo[];
}

// ------------------------------------------------------------- websocket ----
// docs/gui/websocket-events.md + docs/gui/events.schema.json.

/** Server → client message types. Treat an unknown `type` as ignorable. */
export type EventType =
  | 'hello'
  | 'job.submitted'
  | 'job.started'
  | 'job.progress'
  | 'job.finished'
  | 'error';

/** The `hello` handshake, sent once immediately on connect. */
export interface HelloEvent {
  type: 'hello';
  timestamp: string;
  api_version: string;
  implementation: string;
  project: string;
  /** Every currently known job, most recent first. */
  jobs: Job[];
}

/** A job lifecycle/progress event. Order these by `seq`, never by arrival. */
export interface JobEvent {
  type: 'job.submitted' | 'job.started' | 'job.progress' | 'job.finished';
  /** Assigned under the server's job lock, at the moment state changed. */
  seq: number;
  timestamp: string;
  project: string;
  job: Job;
}

/** The server rejected a client message. */
export interface ErrorEvent {
  type: 'error';
  timestamp: string;
  error: string;
}

export type ServerEvent = HelloEvent | JobEvent | ErrorEvent | { type: string };

/** Narrowing helper: does this envelope carry a `Job` and a `seq`? */
export function isJobEvent(event: ServerEvent): event is JobEvent {
  return (
    typeof (event as JobEvent).seq === 'number' &&
    typeof (event as JobEvent).job === 'object' &&
    (event as JobEvent).job !== null &&
    typeof (event as JobEvent).job.id === 'string'
  );
}

/** Narrowing helper for the connect handshake. */
export function isHelloEvent(event: ServerEvent): event is HelloEvent {
  return event.type === 'hello';
}

/** True once a job can no longer change state. */
export function isTerminal(status: JobStatus): boolean {
  return status === 'succeeded' || status === 'failed' || status === 'cancelled';
}
