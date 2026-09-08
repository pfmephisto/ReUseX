// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Recorded server payloads, verbatim.
 *
 * **How these were recorded.** `rux gui` was started on the
 * `tests/fixtures/scans/office_corridor.rux` corridor scan (schema v11) after
 * `rux create clouds -g 0.02` and `rux create planes`, and every response below
 * is the body that server actually returned on 2026-09-08, against contract
 * version 1.0.0 (`docs/gui/openapi.yaml`, `docs/gui/websocket-events.md`).
 * `WS_EVENTS` is the frame sequence one `/api/v1/events` client received while
 * a `planes` job was submitted and ran to completion.
 *
 * **They are recordings, not test data.** If the contract changes, re-record
 * them against a real server — do not hand-edit them to make a test pass. A
 * hand-edited fixture asserts what we wish the server did, which is exactly the
 * failure mode a contract test exists to catch.
 *
 * They live in one `.ts` module rather than a directory of `.json` files
 * because JSON cannot carry a comment: each `.json` would need its own
 * `.license` sidecar for REUSE compliance, whereas this needs one SPDX header.
 * The other half of the reason is the type annotations below — they are
 * load-bearing. Each export is annotated with its wire type from
 * `src/api/types.ts`, so a drift between the hand-written types and the
 * recorded payloads is a compile error, not a silently passing test.
 */

import type {
  ApiError,
  CloudInfo,
  CloudPointsPage,
  HelloEvent,
  Health,
  Job,
  JobEvent,
  PipelineLogEntry,
  ProjectSummary,
  StageInfo,
} from '../api/types';

/** `GET /api/v1/health` */
export const HEALTH: Health = {
  api_version: '1.0.0',
  implementation: 'rux-gui',
  project: {
    name: 'e2e.rux',
    open: true,
    schema_version: 11,
  },
  status: 'ok',
  version: '0.0.5',
};

/** `GET /api/v1/project` */
export const PROJECT_SUMMARY: ProjectSummary = {
  clouds: [
    {
      height: 1,
      name: 'cloud',
      organized: false,
      point_count: 10500,
      type: 'PointXYZRGB',
      width: 10500,
    },
    {
      height: 1,
      name: 'normals',
      organized: false,
      point_count: 10500,
      type: 'Normal',
      width: 10500,
    },
    {
      height: 1,
      name: 'labels',
      organized: false,
      point_count: 10500,
      type: 'Label',
      width: 10500,
    },
    {
      height: 1,
      name: 'planes',
      organized: false,
      point_count: 10500,
      type: 'Label',
      width: 10500,
    },
    {
      height: 1,
      name: 'plane_centroids',
      organized: false,
      point_count: 2,
      type: 'PointXYZ',
      width: 2,
    },
    {
      height: 1,
      name: 'plane_normals',
      organized: false,
      point_count: 2,
      type: 'Normal',
      width: 2,
    },
  ],
  components: {
    count_by_type: {},
    total_count: 0,
  },
  materials: [],
  meshes: [],
  panoramic_images: {
    matched_count: 0,
    total_count: 0,
  },
  path: 'e2e.rux',
  projects: [
    {
      building_address: 'Link Arkitektur A/S, Nordre Fasanvej 108B, 2000 Frederiksberg, Denmark',
      id: 'default',
      name: 'ReUseX test fixture: NewOffice corridor',
      notes:
        'Ten consecutive sensor frames (node_id 1995-2004) trimmed from the NewOffice iOS-LiDAR capture. GPL-3.0-or-later. See tests/fixtures/scans/README.md.',
      survey_date: '',
      survey_organisation: 'Povl Filip Sonne-Frederiksen',
      year_of_construction: 0,
    },
  ],
  schema_version: 11,
  sensor_frames: {
    height: 960,
    segmented_count: 0,
    total_count: 10,
    width: 720,
  },
};

/** `GET /api/v1/clouds` */
export const CLOUDS: { clouds: CloudInfo[] } = {
  clouds: [
    {
      height: 1,
      name: 'cloud',
      organized: false,
      point_count: 10500,
      type: 'PointXYZRGB',
      width: 10500,
    },
    {
      height: 1,
      name: 'normals',
      organized: false,
      point_count: 10500,
      type: 'Normal',
      width: 10500,
    },
    {
      height: 1,
      name: 'labels',
      organized: false,
      point_count: 10500,
      type: 'Label',
      width: 10500,
    },
    {
      height: 1,
      name: 'planes',
      organized: false,
      point_count: 10500,
      type: 'Label',
      width: 10500,
    },
    {
      height: 1,
      name: 'plane_centroids',
      organized: false,
      point_count: 2,
      type: 'PointXYZ',
      width: 2,
    },
    {
      height: 1,
      name: 'plane_normals',
      organized: false,
      point_count: 2,
      type: 'Normal',
      width: 2,
    },
  ],
};

/** `GET /api/v1/stages` */
export const STAGES: { stages: StageInfo[] } = {
  stages: [
    {
      blockers: [],
      cancellable: false,
      ready: true,
      runnable: true,
      stage: 'clouds',
    },
    {
      blockers: [],
      cancellable: true,
      ready: true,
      runnable: true,
      stage: 'planes',
    },
    {
      blockers: [],
      cancellable: true,
      ready: true,
      runnable: true,
      stage: 'rooms',
    },
    {
      blockers: [],
      cancellable: true,
      ready: true,
      runnable: true,
      stage: 'instances',
    },
    {
      blockers: [
        "missing_stage_input: stage 'mesh' requires cloud 'rooms' which is not present in the project",
      ],
      cancellable: false,
      ready: false,
      runnable: false,
      stage: 'mesh',
    },
  ],
};

/** `GET /api/v1/pipeline-log` */
export const PIPELINE_LOG: { entries: PipelineLogEntry[] } = {
  entries: [
    {
      error_msg: '',
      finished_at: '2026-09-08 14:34:06',
      id: 3,
      parameters: '{"job_id":"dfbc481b-b2f7-4c68-aa29-3cfd9bde0611"}',
      stage: 'segment_planes',
      started_at: '2026-09-08 14:34:05',
      status: 'success',
    },
    {
      error_msg: '',
      finished_at: '2026-09-08 14:33:07',
      id: 2,
      parameters:
        '{"adaptive":true,"angle_threshold":25,"interval_0":16,"interval_factor":1.5,"noise_seed":42,"radius":0.5}',
      stage: 'segment_planes',
      started_at: '2026-09-08 14:33:06',
      status: 'success',
    },
    {
      error_msg: '',
      finished_at: '2026-09-08 14:32:47',
      id: 1,
      parameters:
        '{"confidence_threshold":2,"max_distance":4,"min_distance":0,"resolution":0.02,"sampling_factor":4}',
      stage: 'cloud_reconstruction',
      started_at: '2026-09-08 14:32:45',
      status: 'success',
    },
  ],
};

/** `GET /api/v1/jobs`, after the recorded `planes` run finished. */
export const JOBS: { jobs: Job[] } = {
  jobs: [
    {
      cancel_requested: false,
      error: '',
      finished_at: '2026-09-08T14:34:06Z',
      id: 'dfbc481b-b2f7-4c68-aa29-3cfd9bde0611',
      parameters: {},
      progress: {
        current: 10500,
        fraction: 1.0,
        stage: 'region_growing',
        stage_label: 'Region Growing',
        total: 10500,
      },
      project: 'e2e.rux',
      stage: 'planes',
      started_at: '2026-09-08T14:34:05Z',
      status: 'succeeded',
      submitted_at: '2026-09-08T14:34:05Z',
    },
  ],
};

/** `GET /api/v1/clouds/cloud/points?offset=0&limit=4&format=json` */
export const CLOUD_POINTS_PAGE: CloudPointsPage = {
  count: 4,
  fields: ['x', 'y', 'z', 'r', 'g', 'b'],
  name: 'cloud',
  offset: 0,
  points: [
    [21.4287052154541, 10.152006149291992, -1.3002054691314697, 126, 126, 118],
    [21.0346622467041, 9.239747047424316, -1.28462815284729, 103, 87, 51],
    [21.072128295898438, 9.254780769348145, -1.280574083328247, 65, 52, 20],
    [21.015592575073242, 9.275043487548828, -1.2889971733093262, 55, 45, 10],
  ],
  total: 10500,
  type: 'PointXYZRGB',
};

/** `GET /api/v1/clouds/planes/points?offset=0&limit=4&format=json` */
export const LABEL_POINTS_PAGE: CloudPointsPage = {
  count: 4,
  fields: ['label'],
  name: 'planes',
  offset: 0,
  points: [[2], [0], [0], [0]],
  total: 10500,
  type: 'Label',
};

/** `GET /api/v1/clouds/nope/points` → 404. */
export const ERROR_NOT_FOUND: ApiError = {
  error: "no such cloud 'nope'",
  status: 404,
};

/** `GET /api/v1/clouds/cloud/points?format=binary` → 501. */
export const ERROR_NOT_IMPLEMENTED: ApiError = {
  error:
    'binary point transport is not implemented yet; it is Phase 2/5 of issue #265. Use format=json.',
  status: 501,
};

/**
 * Every frame one `/api/v1/events` client received across a `planes` run.
 *
 * A tuple rather than an array so a test can index a specific frame and still
 * get a narrowed type: `[0]` is the `hello` handshake, `[1..5]` are the job
 * events with `seq` 1..5.
 */
export const WS_EVENTS: [HelloEvent, JobEvent, JobEvent, JobEvent, JobEvent, JobEvent] = [
  {
    api_version: '1.0.0',
    implementation: 'rux-gui',
    jobs: [],
    project: 'e2e.rux',
    timestamp: '2026-09-08T14:34:05Z',
    type: 'hello',
  },
  {
    job: {
      cancel_requested: false,
      error: '',
      finished_at: '',
      id: 'dfbc481b-b2f7-4c68-aa29-3cfd9bde0611',
      parameters: {},
      progress: {
        current: 0,
        fraction: null,
        stage: 'idle',
        stage_label: 'Idle',
        total: 0,
      },
      project: 'e2e.rux',
      stage: 'planes',
      started_at: '',
      status: 'queued',
      submitted_at: '2026-09-08T14:34:05Z',
    },
    project: 'e2e.rux',
    seq: 1,
    timestamp: '2026-09-08T14:34:05Z',
    type: 'job.submitted',
  },
  {
    job: {
      cancel_requested: false,
      error: '',
      finished_at: '',
      id: 'dfbc481b-b2f7-4c68-aa29-3cfd9bde0611',
      parameters: {},
      progress: {
        current: 0,
        fraction: null,
        stage: 'idle',
        stage_label: 'Idle',
        total: 0,
      },
      project: 'e2e.rux',
      stage: 'planes',
      started_at: '2026-09-08T14:34:05Z',
      status: 'running',
      submitted_at: '2026-09-08T14:34:05Z',
    },
    project: 'e2e.rux',
    seq: 2,
    timestamp: '2026-09-08T14:34:05Z',
    type: 'job.started',
  },
  {
    job: {
      cancel_requested: false,
      error: '',
      finished_at: '',
      id: 'dfbc481b-b2f7-4c68-aa29-3cfd9bde0611',
      parameters: {},
      progress: {
        current: 0,
        fraction: 0,
        stage: 'region_growing',
        stage_label: 'Region Growing',
        total: 10500,
      },
      project: 'e2e.rux',
      stage: 'planes',
      started_at: '2026-09-08T14:34:05Z',
      status: 'running',
      submitted_at: '2026-09-08T14:34:05Z',
    },
    project: 'e2e.rux',
    seq: 3,
    timestamp: '2026-09-08T14:34:06Z',
    type: 'job.progress',
  },
  {
    job: {
      cancel_requested: false,
      error: '',
      finished_at: '',
      id: 'dfbc481b-b2f7-4c68-aa29-3cfd9bde0611',
      parameters: {},
      progress: {
        current: 10500,
        fraction: 1,
        stage: 'region_growing',
        stage_label: 'Region Growing',
        total: 10500,
      },
      project: 'e2e.rux',
      stage: 'planes',
      started_at: '2026-09-08T14:34:05Z',
      status: 'running',
      submitted_at: '2026-09-08T14:34:05Z',
    },
    project: 'e2e.rux',
    seq: 4,
    timestamp: '2026-09-08T14:34:06Z',
    type: 'job.progress',
  },
  {
    job: {
      cancel_requested: false,
      error: '',
      finished_at: '2026-09-08T14:34:06Z',
      id: 'dfbc481b-b2f7-4c68-aa29-3cfd9bde0611',
      parameters: {},
      progress: {
        current: 10500,
        fraction: 1,
        stage: 'region_growing',
        stage_label: 'Region Growing',
        total: 10500,
      },
      project: 'e2e.rux',
      stage: 'planes',
      started_at: '2026-09-08T14:34:05Z',
      status: 'succeeded',
      submitted_at: '2026-09-08T14:34:05Z',
    },
    project: 'e2e.rux',
    seq: 5,
    timestamp: '2026-09-08T14:34:06Z',
    type: 'job.finished',
  },
];

/** The job id every recorded job event carries. */
export const RECORDED_JOB_ID = 'dfbc481b-b2f7-4c68-aa29-3cfd9bde0611';
