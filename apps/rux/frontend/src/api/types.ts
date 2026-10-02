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
   * Present for `Label` clouds only. Classifies the cloud as structural
   * segmentation (`"geometry"`: planes, rooms) or object-class annotation
   * (`"semantic"`: instances, annotation-derived). Server-authoritative —
   * the client must not re-derive this from the name.
   */
  label_kind?: 'geometry' | 'semantic';
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
  /**
   * Present only on a `max_points` request (#320). True when the rows are a
   * voxel-subsampled view of the whole cloud rather than a window of it;
   * false when the cloud already fit the budget and came back complete.
   */
  lod?: boolean;
  /** Voxel edge the subsample settled on, present only when `lod` is true. */
  voxel_size?: number;
}

/** `TileInfo` — one spatial tile's AABB and point count (#395). */
export interface TileInfo {
  id: number;
  count: number;
  min: [number, number, number];
  max: [number, number, number];
}

/**
 * `CloudTileIndex` — the spatial tile index of one cloud (#395).
 *
 * Present only for clouds stored in morton_10bit_bitrev order. Tile `k`
 * contains the points where `(sort_key & (K-1)) == k`, where
 * `sort_key = reverse_bits30(morton(x,y,z))` and `K = tile_count`. The server
 * selects them with an O(N) scan at serve time.
 */
export interface CloudTileIndex {
  name: string;
  tile_count: number;
  tile_bits: number;
  point_count: number;
  tiles: TileInfo[];
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

/**
 * `GsplatInfo` — one Gaussian splat stored in the project (#322).
 *
 * Produced by `rux create gsplat` (or brought in with `rux import gsplat`) and
 * stored in the `.rux` like a mesh, so this is metadata about a project row,
 * never a filesystem path.
 */
export interface GsplatInfo {
  name: string;
  /** Storage format of the blob. `ply` is the INRIA 3DGS layout. */
  format: 'ply';
  gaussian_count: number;
  /** 0 means view-independent colour, not "missing". */
  sh_degree: number;
  byte_size: number;
  stage?: string;
  parameters?: string;
  created_at?: string;
}

/** `TextureInfo` — texture metadata (never the image bytes). */
export interface TextureInfo {
  tex_name: string;
  format: string;
  width: number;
  height: number;
}

/** `ScanGroup` — one import session's frames within a `FrameList` (#462). */
export interface ScanGroup {
  scan_id: number;
  /** Raw path passed to `rux import`; take the final segment for display. */
  source_path: string;
  imported_at: string;
  /** Filtered frame ids belonging to this scan, in ascending order. */
  ids: number[];
}

/** `FrameList` — sensor frame ids plus aggregate counts. */
export interface FrameList {
  ids: number[];
  total_count: number;
  segmented_count: number;
  width?: number;
  height?: number;
  /**
   * Per-scan breakdown of the ids above (#462).
   * Absent for projects imported before schema v17.
   */
  scans?: ScanGroup[];
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
  /**
   * Row-major 4x4 world pose, verbatim as stored. A frame with no stored pose
   * reads back as identity — check `has_pose` before trusting it.
   */
  pose: number[];
  /** Whether `pose` is a usable stored pose rather than the identity fallback. */
  has_pose?: boolean;
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
  /** True once `rux align 360` has resected this panorama. */
  has_pose: boolean;
  /**
   * Row-major 4x4 camera-to-world pose from content-based alignment —
   * **identity until `rux align 360` has run**. Check `has_pose` first.
   */
  pose?: number[];
  pose_source: 'timestamp' | 'aligned';
  align_inliers?: number;
  /** Angular RMS of inlier bearings, degrees. -1 if unaligned. */
  align_rms?: number;
  /** Whether `frame_pose` is present. */
  has_frame_pose?: boolean;
  /**
   * Row-major 4x4 pose of the timestamp-matched sensor frame (`node_id`),
   * derived on read.
   *
   * A separate field from `pose` on purpose: a borrowed frame pose carries the
   * panorama's mounting offset and the timestamp-match error, and is a
   * different claim from a resected one.
   */
  frame_pose?: number[];
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
  /** Whether a thumbnail image is stored for this passport. */
  has_thumbnail?: boolean;
}

/** The kind of a user-defined material editor column. */
export type PropertyType =
  | 'text'
  | 'number'
  | 'date'
  | 'boolean'
  | 'select'
  | 'multiselect'
  | 'url';

/**
 * `PropertyDefinition` — a user-defined column in the material editor.
 *
 * `options` is populated only for the `select`/`multiselect` types.
 */
export interface PropertyDefinition {
  id: string;
  name: string;
  type: PropertyType;
  options?: string[];
  sort_order: number;
  width?: number;
}

/** `MaterialCreate` — the (optional) body of `POST /materials`. */
export interface MaterialCreate {
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

/**
 * `VisibleFrame` — one entry from `/frames/visibility` or
 * `/instances/{cloud}/{id}/frames`.
 *
 * `centrality` is the sort key (ascending): 0 at the principal point
 * (most central), ~1 at a corner.  `score` is the higher-is-better
 * complement (`1 - centrality`).
 */
export interface VisibleFrame {
  frame_id: number;
  centrality: number;
  score: number;
  depth: number;
  u: number;
  v: number;
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

// ----------------------------------------------------------- pose graph ----

/** One node in the pose graph: a sensor frame with a world pose. */
export interface PoseGraphNode {
  /** DB node_id (sensor frame id). */
  id: number;
  /** 4×4 world transform, column-major float64. */
  pose: number[];
}

/** Edge type produced by the optimizer. */
export type PoseGraphEdgeType = 'odometry' | 'loop_closure' | 'panorama';

/** One directed edge with its post-solve residual. */
export interface PoseGraphEdge {
  from: number;
  to: number;
  type: PoseGraphEdgeType;
  /** 0.5 × whitened squared residual after convergence (GTSAM). Low = satisfied. */
  residual: number;
  /** 1/σ² translational information weight at build time; absent if not extractable. */
  weight?: number;
}

/** Full pose graph response from `GET /api/v1/posegraph`. */
export interface PoseGraph {
  nodes: PoseGraphNode[];
  edges: PoseGraphEdge[];
}

/** Body of `POST /api/v1/posegraph/edges`. */
export interface PoseGraphEdgeCreate {
  from: number;
  to: number;
  type?: PoseGraphEdgeType;
  weight?: number;
}

/** Response of `DELETE /api/v1/posegraph/edges/{from}/{to}`. */
export interface PoseGraphEdgeDeleteResult {
  deleted: number;
  from: number;
  to: number;
  type: PoseGraphEdgeType | null;
}

/** Body of `POST /api/v1/posegraph/icp` (#465). */
export interface IcpRefineRequest {
  from: number;
  to: number;
}

/**
 * Response of `POST /api/v1/posegraph/icp` (#465).
 *
 * `relative_pose` is a 16-element row-major 4×4 matrix T_to⁻¹ @ T_delta @ T_from
 * (maps a point from the "from" camera frame into the "to" camera frame).
 */
export interface IcpRefineResult {
  from: number;
  to: number;
  /** Row-major 4×4 relative pose (16 elements). */
  relative_pose: number[];
  /** RMS correspondence error after ICP (metres). Low = good alignment. */
  fitness: number;
  /** Fraction of source points within 5 cm of target after alignment. */
  inlier_fraction: number;
  /** True when ICP reached its convergence criterion. */
  converged: boolean;
}

// ------------------------------------------------- frame visibility (#453) ----

/**
 * `VisibleFrame` — one frame in which a queried point projects inside the image.
 *
 * Mirror of `components.schemas.VisibleFrame` in `docs/gui/openapi.yaml`.
 */
export interface VisibleFrame {
  /** DB node_id of the sensor frame. */
  frame_id: number;
  /** Normalised distance from principal point: 0 = dead centre, ~1 = corner. Sort key. */
  centrality: number;
  /** Higher-is-better complement: `1 - centrality`. */
  score: number;
  /** Point depth in the camera optical frame, metres (> 0). */
  depth: number;
  /** Projected pixel column (0 = left edge). */
  u: number;
  /** Projected pixel row (0 = top edge). */
  v: number;
}

/**
 * `FrameVisibilityList` — frames that see a world point, most central first.
 *
 * Mirror of `components.schemas.FrameVisibilityList`. The `count` is how many
 * `frames` carries (bounded by `limit`); `total` is the full visible count.
 */
export interface FrameVisibilityList {
  /** The queried world point [x, y, z] (a centroid for the instance variant). */
  point: [number, number, number];
  frames: VisibleFrame[];
  /** Number of frames in this response. */
  count: number;
  /** Total visible frames, before the `limit` cap. */
  total: number;
}

// ------------------------------------------------- frame-pair inspection ----

/**
 * Feature-descriptor algorithm for pairwise frame matching.
 *
 * Backend implementation reference: `tools/gt_curator/opencv_features.py`.
 * Used both in the `POST /frames/{a}/descriptor-match/{b}` request body and
 * in the result, so both sides of the wire speak the same string.
 */
export type DescriptorMethod = 'orb' | 'sift' | 'akaze';

/**
 * `DescriptorMatchResult` — response of `POST /frames/{a}/descriptor-match/{b}`.
 *
 * **Backend endpoint not yet implemented as of #446.** The GUI stub will
 * receive a 404 and surface it gracefully. The schema below is the contract
 * the implementation must satisfy:
 *
 *   POST /api/v1/frames/{frame_a}/descriptor-match/{frame_b}
 *   Body:    { method: DescriptorMethod }
 *   Success: DescriptorMatchResult (200 OK)
 *   Error:   { error: string }   (400 bad frame id, 404 frame not found,
 *                                 422 no depth for backprojection, 500 server)
 *
 * Matching pipeline (mirrors opencv_features.py):
 *  1. Load stored colour images for both frames.
 *  2. Extract keypoints and descriptors with the requested detector.
 *  3. BF-match + Lowe ratio test (threshold 0.85).
 *  4. Back-project matched 2-D points into 3-D using the stored depth image
 *     and sensor intrinsics; discard pairs where either depth is invalid.
 *  5. RANSAC rigid-body estimation (threshold 0.10 m, 500 iterations).
 *  6. Return keypoints, inlier mask, relative transform, and RMS error.
 */
export interface DescriptorMatchResult {
  frame_a: number;
  frame_b: number;
  method: DescriptorMethod;
  /** RANSAC-surviving inlier count. */
  n_inliers: number;
  /** RMS of inlier reprojection errors in metres; null when fewer than 3 inliers. */
  rms_m: number | null;
  /** All matched pixel coordinates in frame A (Lowe-filtered, including outliers). */
  keypoints_a: [number, number][];
  /** Matched pixel coordinates in frame B, index-aligned with keypoints_a. */
  keypoints_b: [number, number][];
  /** True for RANSAC inliers (index-aligned with keypoints_a/b). */
  inlier_mask: boolean[];
  /**
   * Row-major 4×4 rigid transform T_AB that maps 3-D points from frame B's
   * optical coordinate frame into frame A's — the loop-edge convention:
   * `p_A ≈ T_AB @ p_B`.  Null when RANSAC failed or fewer than 3 depth pairs.
   */
  transform: number[] | null;
  /** Non-null only when the server could not compute matches. */
  error: string | null;
}

// ------------------------------------------------------------ segment (#409) ----

/**
 * Wire tuple for one SAM3 bounding-box hint.
 *
 * Shape: `["pos" | "neg", [x1, y1, x2, y2]]` in image pixel coordinates.
 * The polarity marks whether the enclosed region is a positive example
 * (include) or a negative example (exclude).
 */
export type FrameSegmentBox = ['pos' | 'neg', [number, number, number, number]];

/** One SAM3 text + optional box prompt. */
export interface FrameSegmentPrompt {
  /** Open-vocabulary class name, e.g. `"wall"`. Required by the contract. */
  text: string;
  /** Bounding-box hints, each tagged with a polarity. */
  boxes?: FrameSegmentBox[];
  /** Per-prompt threshold override; negative (absent) → use top-level confidence. */
  confidence?: number;
}

/** Body of `POST /frames/{id}/segment`. */
export interface FrameSegmentRequest {
  /** Server-side filesystem path to a TRT engine directory or `.onnx` file. */
  model_path: string;
  /** Empty / absent ⟹ use the model's built-in default class list. */
  prompts?: FrameSegmentPrompt[];
  /** Global detection threshold [0, 1]. Default 0.5. */
  confidence?: number;
  /** Write the label map back to the project. Default true. */
  save?: boolean;
  /**
   * Use CUDA/TensorRT inference. Absent ⟹ server-wide default
   * (`--segment-cuda` / `--no-segment-cuda`). Pass `false` on CPU-only
   * hosts to route inference through the ONNX CPU backend (#467).
   */
  use_cuda?: boolean;
}

/** Response of `POST /frames/{id}/segment`. */
export interface FrameSegmentResult {
  frame_id: number;
  labeled_pixels: number;
  saved: boolean;
  /**
   * Label id → class name. Populated from `prompts`; empty when the model's
   * built-in default list was used.
   */
  labels: Record<string, string>;
}

// -------------------------------------------------------- export-templates ----

/**
 * `ExportTemplate` — a named, saved CSV export column selection (#459).
 *
 * Mirror of `components.schemas.ExportTemplate` in `docs/gui/openapi.yaml`.
 * `config.columns` holds the ordered column names; absent or empty means all.
 */
export interface ExportTemplate {
  id: number;
  name: string;
  config: { columns?: string[] };
  created_at: string;
  updated_at: string;
}

// ------------------------------------------------------- panorama segment ----

/** Body of `POST /panoramas/{id}/segment`. */
export interface PanoramaSegmentRequest {
  /** Server-side filesystem path to a TRT engine directory or `.onnx` file. */
  model_path: string;
  /** Text-only prompts; empty / absent ⟹ model's built-in default class list. */
  prompts?: Array<{ text: string; confidence?: number }>;
  /** Global detection threshold [0, 1]. Default 0.5. */
  confidence?: number;
  /** Number of equator tiles around the sphere. Default 8. */
  n_yaw?: number;
  /** Per-tile horizontal FOV in degrees. Default 90. */
  fov_deg?: number;
  /** Write the equirect label map back to the project. Default true. */
  save?: boolean;
  /**
   * Use CUDA/TensorRT inference. Absent ⟹ server-wide default
   * (`--segment-cuda` / `--no-segment-cuda`). Pass `false` on CPU-only
   * hosts to route inference through the ONNX CPU backend (#467).
   */
  use_cuda?: boolean;
}

/** Response of `POST /panoramas/{id}/segment`. */
export interface PanoramaSegmentResult {
  pano_id: number;
  labeled_pixels: number;
  saved: boolean;
  /** Label id → class name. Empty when the model's built-in list was used. */
  labels: Record<string, string>;
}

// ----------------------------------------------------------------- survey ----

/** `Treatment` — waste-hierarchy step (affaldshierarki), best first. */
export type Treatment =
  | 'bevaring'
  | 'genbrug'
  | 'genanvendelse'
  | 'nyttiggoerelse'
  | 'bortskaffelse';

/** `Treatment` values in waste-hierarchy order, best first. */
export const TREATMENTS = [
  'bevaring',
  'genbrug',
  'genanvendelse',
  'nyttiggoerelse',
  'bortskaffelse',
] as const satisfies readonly Treatment[];

/** `ReviewStatus` — a survey type's place in the review workflow. */
export type ReviewStatus = 'queue' | 'approved' | 'rejected';

/**
 * `EnvironmentStatus` — derived from a survey type's linked samples, never
 * stored. `afventer` blocks approval.
 */
export type EnvironmentStatus = 'ren_screening' | 'afventer' | 'forurenet' | 'ren_proevesvar';

/** Stage of an environmental sample's lab workflow. */
export type SampleStage = 'planlagt' | 'udtaget' | 'sendt' | 'svar';

/** Lab result of an environmental sample, once answered. */
export type SampleResult = 'ren' | 'forurenet';

/** `SurveyPart` — one bygningsdel (building part) filed under a survey type. */
export interface SurveyPart {
  code: string;
  type_id: number;
  cloud: string | null;
  instance_id: number | null;
  room_id: number | null;
  room_name: string;
  quantity: number;
  starred: boolean;
  note: string;
  material_guid: string | null;
  instance_guid: string | null;
  orphaned: boolean;
}

/** `SurveyType` — one Kortlægning group row, with its parts. */
export interface SurveyType {
  id: number;
  name: string;
  eak_code: string;
  eak_name: string;
  bim7aa_code: string;
  unit: string;
  treatment: Treatment;
  review_status: ReviewStatus;
  confidence: number | null;
  mass_t: number | null;
  note: string;
  starred: boolean;
  semantic_class: number;
  environment_status: EnvironmentStatus;
  sample_ids: number[];
  /** Sum of the parts' quantities. */
  quantity: number;
  parts: SurveyPart[];
  created_at: string;
  updated_at: string;
}

/** `SurveyCounts` — review-workflow counts. `all` is queue + approved. */
export interface SurveyCounts {
  queue: number;
  approved: number;
  rejected: number;
  all: number;
}

/** Body of `GET /survey`. */
export interface Survey {
  types: SurveyType[];
  counts: SurveyCounts;
}

/** `SurveySummary` — KPIs for Overblik and the Kortlægning coverage notice. */
export interface SurveySummary {
  counts: SurveyCounts;
  /** Tonnes per treatment over non-rejected types. */
  circularity: Record<Treatment, number>;
  total_mass_t: number;
  /** (bevaring + genbrug) / total. */
  reuse_share: number | null;
  pending_samples: number;
  /** Non-rejected types whose miljøstatus is forurenet. */
  contaminated_types: number;
  unlabeled_points: number | null;
  /** Share (0..1) of instance-cloud points with a label; null without an instance cloud. */
  classified_share: number | null;
  rooms_without_parts: string[];
}

/** One row of `SurveyFractions.fractions`. */
export interface SurveyFraction {
  eak_code: string;
  name: string;
  treatment: Treatment;
  mass_t: number;
  /** Tonnes from forurenet types; never merged with clean tonnes. */
  contaminated: boolean;
}

/** One row of `SurveyFractions.blocking`. */
export interface SurveyBlockingType {
  type_id: number;
  name: string;
  eak_code: string;
  treatment: Treatment;
  mass_t: number | null;
  /**
   * `sample`: awaiting a sample (blocks even when approved); `review`: not
   * approved yet; `mass`: approved but tonnage unknown. Precedence
   * sample > review > mass.
   */
  reason: 'review' | 'sample' | 'mass';
}

/**
 * `SurveyFractions` — approved tonnes per EAK code, treatment and
 * contamination for waste reporting. `bevaring` never counts.
 */
export interface SurveyFractions {
  fractions: SurveyFraction[];
  total_t: number;
  /** Length of `blocking`. */
  blocking_types: number;
  /** Non-rejected types that keep the report from being sent, in type-id order. */
  blocking: SurveyBlockingType[];
  /** True when `blocking` is empty. */
  ready: boolean;
}

/** `Sample` — one environmental sample, with its linked survey types. */
export interface Sample {
  id: number;
  code: string;
  title: string;
  what: string;
  stage: SampleStage;
  result: SampleResult | null;
  type_ids: number[];
  created_at: string;
  updated_at: string;
}

/** Body of `POST /survey/sync` — counts of what sync_survey changed. */
export interface SurveySyncReport {
  types_created: number;
  parts_created: number;
  parts_existing: number;
  /** Instance rows sync considered; 0 means the instance cloud holds none. */
  instances_seen: number;
  /** Rows sync wrote first for an instance cloud from before schema v10. */
  instances_backfilled: number;
  rooms_assigned: boolean;
  parts_orphaned: number;
  orphaned_codes: string[];
  /** Instance links put back for instance-backed parts that lost theirs (a `create instances` re-run cascade-deletes them). */
  links_restored?: number;
}

/** Body of `POST /survey/types`. */
export interface SurveyTypeCreate {
  name: string;
  eak_code?: string;
  bim7aa_code?: string;
  unit?: string;
  treatment?: Treatment;
}

/**
 * `SurveyTypePatch` — sparse update for `PATCH /survey/types/{id}`; only
 * present fields change. `review_status: 'approved'` is refused (422) while
 * the type's derived `environment_status` is `afventer`.
 */
export interface SurveyTypePatch {
  name?: string;
  eak_code?: string;
  bim7aa_code?: string;
  unit?: string;
  note?: string;
  treatment?: Treatment;
  review_status?: ReviewStatus;
  confidence?: number | null;
  mass_t?: number | null;
  starred?: boolean;
  /** Redistributes across the type's existing parts (core::set_type_quantity). */
  quantity?: number;
}

/** `SurveyPartPatch` — sparse update for `PATCH /survey/parts/{code}`. */
export interface SurveyPartPatch {
  /** Re-files the part under a different survey type. */
  type_id?: number;
  quantity?: number;
  starred?: boolean;
  note?: string;
  room_name?: string;
}

/** Body of `POST /samples`. */
export interface SampleCreate {
  title: string;
  what?: string;
  type_ids?: number[];
}

/**
 * `SamplePatch` — sparse update for `PATCH /samples/{id}`. `result` can only
 * be set once `stage` is `svar`; `result: null` clears it.
 */
export interface SamplePatch {
  title?: string;
  what?: string;
  stage?: SampleStage;
  result?: SampleResult | null;
}

// ------------------------------------------------- resources & templates ----

/** Where a key's write lands (spec §4.3): `type` changes every part of the type. */
export type ResourceKeyScope = 'type' | 'part';

/**
 * Which editor a key gets in Kortlægning (spec §6.1). `multiselect` is a
 * leksikon `EnumArray` field (R3-A1): Phase 3 shows its joined values
 * read-only, with no editor of its own yet.
 */
export type ResourceDataType = 'text' | 'number' | 'enum' | 'multiselect' | 'boolean' | 'date';

/**
 * `ResourceKey` — one row of `GET /resources/keys`. `id` is `lex:<guid>`
 * (leksikon), `col:<id>` (user column) or `sys:<name>` (built-in); the UI
 * shows `label`, never the id.
 */
export interface ResourceKey {
  id: string;
  label: string;
  category: string;
  scope: ResourceKeyScope;
  data_type: ResourceDataType;
  unit: string | null;
  /** Choices of an `enum` or `multiselect` key, wire values (e.g. `genbrug`). */
  options: string[];
  /** False for derived keys (`sys:environment`): a write is a 400. */
  editable: boolean;
}

/** `Resource` — one survey part's values by key id. `null` = no value. */
export interface Resource {
  code: string;
  type_id: number;
  /** Added by hand (no instance); only these can be deleted. */
  manual: boolean;
  values: Record<string, string | null>;
}

/** Response of `PATCH /resources/{code}`: the resource, plus the type's other parts after a type-scoped write. */
export interface ResourcePatchResult {
  resource: Resource;
  siblings: Resource[];
}

/** Body of `POST /resources` — a manual part under an existing type. */
export interface ResourceCreate {
  type_id: number;
  name?: string;
}

/** Body of `POST /resources/columns` (a `PropertyDefinition` without its id). */
export interface ResourceColumnCreate {
  name: string;
  type: PropertyType;
  options?: string[];
  sort_order?: number;
  width?: number;
}

/** A template member: a whole category, or one key (spec §5.1). */
export type TemplateMember = { category: string } | { key: string };

/** The tag of a seeded template (spec §5.3). */
export type TemplateSeed = 'materialepas' | 'screening';

/** CSV export options saved on a template (Phase 4 edits them). */
export interface TemplateCsv {
  delimiter?: string;
  encoding?: string;
  header?: 'label' | 'key';
}

/** `Template` — one row of `GET /templates`, resolved against the live catalogue. */
export interface Template {
  id: number;
  name: string;
  members: TemplateMember[];
  csv: TemplateCsv;
  seed: TemplateSeed | null;
  /** Ordered key ids the members resolve to now (spec §5.2). */
  resolved_keys: string[];
  /** Members whose key or category no longer exists. */
  missing: TemplateMember[];
  created_at: string;
  updated_at: string;
}

/** Body of `POST /templates` (name required) or `PATCH /templates/{id}` (sparse). */
export interface TemplatePatch {
  name?: string;
  members?: TemplateMember[];
  csv?: TemplateCsv;
}

/** Camera placement for `GET /renders`. */
export type RenderView = 'plan' | 'top' | 'front' | 'orbit';

/** Query parameters of `GET /renders`. */
export interface RenderQuery {
  view?: RenderView;
  /** Which of 8 viewpoints on the orbit ring to render. Only used with `view: 'orbit'`. */
  orbit_index?: number;
  /** Layers to draw, in draw order. */
  layers?: string[];
  /** Instance id to paint; every other point is dimmed. */
  highlight_instance?: number;
  /** Named Label cloud supplying the instance id per point. Defaults to `instances`. */
  highlight_cloud?: string;
  width?: number;
  height?: number;
}

// --------------------------------------------------------------- reports ----

/** `ReportPdfVersion` — metadata for one stored Ressourcekortlægning PDF. */
export interface ReportPdfVersion {
  /** Stable numeric id; use in `GET /reports/ressourcekortlaegning/{id}`. */
  id: number;
  /** Generation time, UTC, as sqlite stores it ("2026-08-09 10:05:00"). Parse with `parseServerUtc`. */
  created_at: string;
  /** Human-readable label stored with the PDF. */
  label: string;
  /** Size of the PDF blob in bytes. */
  size_bytes: number;
  /** 1-based, generation order: the "v3" a user sees. */
  version: number;
  /** Types that blocked the report at generation; 0 = complete; null before schema v23. */
  blocking_types: number | null;
}

/**
 * Parses a server timestamp into a `Date`. Two shapes are accepted:
 * - sqlite's zone-less UTC (`"YYYY-MM-DD HH:MM:SS"`, seconds optional), as
 *   `ReportPdfVersion.created_at` is stored;
 * - ISO 8601 with an explicit zone (`"…T10:05:00Z"`, `"…T12:05:00+02:00"`,
 *   optional fraction).
 *
 * `new Date(s)` must never be used on these strings: browsers parse a
 * space-separated, zone-less timestamp as **local** time, not UTC, which
 * silently shifts the displayed instant by the viewer's offset. So both shapes
 * go through `Date.UTC`, with an ISO offset applied by hand. Returns `null`
 * for zone-less ISO and for anything else that does not match.
 */
export function parseServerUtc(value: string): Date | null {
  const match =
    /^(\d{4})-(\d{2})-(\d{2})(?: (\d{2}):(\d{2})(?::(\d{2}))?|T(\d{2}):(\d{2}):(\d{2})(?:\.(\d+))?(Z|([+-])(\d{2}):(\d{2})))$/.exec(
      value.trim(),
    );
  if (!match) return null;
  const [, year, month, day, sh, smin, ss, ih, imin, is, frac, zone, sign, oh, om] = match;
  const iso = zone !== undefined;
  const f = {
    y: Number(year),
    mo: Number(month) - 1,
    d: Number(day),
    h: Number(iso ? ih : sh),
    mi: Number(iso ? imin : smin),
    s: Number(iso ? is : (ss ?? 0)),
  };
  // Date.UTC rolls an impossible field over (2026-02-30 -> 2026-03-02), so
  // only accept the instant when every field comes back as written.
  const local = new Date(Date.UTC(f.y, f.mo, f.d, f.h, f.mi, f.s));
  if (
    local.getUTCFullYear() !== f.y ||
    local.getUTCMonth() !== f.mo ||
    local.getUTCDate() !== f.d ||
    local.getUTCHours() !== f.h ||
    local.getUTCMinutes() !== f.mi ||
    local.getUTCSeconds() !== f.s
  ) {
    return null;
  }
  let offsetMin = 0;
  if (iso && zone !== 'Z') {
    const [hh, mm] = [Number(oh), Number(om)];
    if (hh > 23 || mm > 59) return null;
    offsetMin = (sign === '-' ? -1 : 1) * (hh * 60 + mm);
  }
  const ms = local.getTime() + (frac ? Math.round(Number(`0.${frac}`) * 1000) : 0) - offsetMin * 60_000;
  return Number.isNaN(ms) ? null : new Date(ms);
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
