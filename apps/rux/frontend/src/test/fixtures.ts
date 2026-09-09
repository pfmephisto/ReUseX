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
 * `STAGES`, `STAGES_EMPTY_PROJECT`, `STAGE_VALIDATION_BLOCKED` and
 * `PIPELINE_LOG` were re-recorded on 2026-09-09 for the Phase 3 contract
 * additions (#305), from a fresh copy of the same scan driven clouds ->
 * planes -> rooms through `POST /jobs`. They therefore come from a different
 * server run than `JOBS`/`WS_EVENTS`, whose shape this change did not touch —
 * so `PIPELINE_LOG_JOB_ID`, not `RECORDED_JOB_ID`, is the id that appears in
 * the log entries below.
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
  ComponentInfo,
  FrameInfo,
  FrameList,
  HelloEvent,
  Health,
  Job,
  JobEvent,
  LabelLegend,
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

/** `GET /api/v1/stages`, on a project that has been through clouds+planes+rooms. */
export const STAGES: { stages: StageInfo[] } = {
  stages: [
    {
      blockers: [],
      cancellable: false,
      command: 'rux create clouds',
      issues: [],
      log_name: 'cloud_reconstruction',
      outputs: [
        'cloud',
        'normals',
      ],
      parameters: [
        {
          default: 0.05,
          description: 'Voxel grid resolution used to downsample the fused cloud.',
          key: 'resolution',
          label: 'Voxel size [m]',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 0.0,
          description: 'Depth samples closer than this are discarded.',
          key: 'min_distance',
          label: 'Minimum depth [m]',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 4.0,
          description: 'Depth samples farther than this are discarded.',
          key: 'max_distance',
          label: 'Maximum depth [m]',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 4,
          description: 'Keep every Nth pixel of each depth frame.',
          key: 'sampling_factor',
          label: 'Pixel subsampling',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: 2,
          description: 'Discard depth samples below this confidence level.',
          key: 'confidence_threshold',
          label: 'Minimum confidence',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'integer',
        },
      ],
      ready: true,
      runnable: true,
      stage: 'clouds',
      summary: 'back-project depth frames into a fused cloud',
    },
    {
      blockers: [],
      cancellable: true,
      command: 'rux create planes',
      issues: [],
      log_name: 'segment_planes',
      outputs: [
        'planes',
        'plane_centroids',
        'plane_normals',
      ],
      parameters: [
        {
          default: 25.0,
          description: 'Maximum normal deviation for a point to join a plane.',
          key: 'angle_threshold',
          label: 'Angle threshold [deg]',
          maximum: 365.0,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 0.07,
          description: 'Maximum point-to-plane distance. Sending this key at all pins the threshold and switches off adaptive derivation for it.',
          key: 'plane_dist_threshold',
          label: 'Distance threshold [m]',
          maximum: 1.0,
          minimum: 0.0,
          presence_sensitive: true,
          type: 'number',
        },
        {
          default: 1000,
          description: 'Smallest accepted plane, in points. Sending this key at all pins it and switches off adaptive derivation for it.',
          key: 'min_inliers',
          label: 'Minimum cluster size',
          maximum: 1000000.0,
          minimum: 3.0,
          presence_sensitive: true,
          type: 'integer',
        },
        {
          default: 0.5,
          description: 'Neighbourhood radius used while growing a plane.',
          key: 'radius',
          label: 'Region-growing radius [m]',
          maximum: 5.0,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 16.0,
          description: 'Initial interval between plane refits.',
          key: 'interval_0',
          label: 'Initial refit interval',
          maximum: 10000.0,
          minimum: 1.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 1.5,
          description: 'Multiplier applied to the refit interval after each refit.',
          key: 'interval_factor',
          label: 'Refit interval factor',
          maximum: 10.0,
          minimum: 1.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: true,
          description: 'Derive the distance threshold (~3 sigma) and the minimum cluster size from measured cloud noise. Explicit values still win per parameter.',
          key: 'adaptive',
          label: 'Adaptive thresholds',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'boolean',
        },
        {
          default: 42,
          description: 'Deterministic seed for the noise estimator (STANDARDS §6).',
          key: 'noise_seed',
          label: 'Noise-estimator seed',
          maximum: null,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: null,
          description: 'Filter expression restricting which points are segmented. Empty means the whole cloud.',
          key: 'filter',
          label: 'Point filter',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'string',
        },
      ],
      ready: true,
      runnable: true,
      stage: 'planes',
      summary: 'detect planar surfaces',
    },
    {
      blockers: [],
      cancellable: true,
      command: 'rux create rooms',
      issues: [],
      log_name: 'segment_rooms',
      outputs: [
        'rooms',
      ],
      parameters: [
        {
          default: 0.5,
          description: 'Spatial discretisation used to build the plane graph.',
          key: 'grid_size',
          label: 'Grid size [m]',
          maximum: 10.0,
          minimum: 0.01,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 1.0,
          description: 'Higher values produce more, smaller rooms.',
          key: 'resolution',
          label: 'Leiden resolution',
          maximum: 10.0,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 0.01,
          description: 'Randomness of the refinement phase; lower is more deterministic.',
          key: 'beta',
          label: 'Leiden beta',
          maximum: 1.0,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 100,
          description: 'Finite bound on the Leiden iteration count.',
          key: 'max_iter',
          label: 'Maximum iterations',
          maximum: 1000.0,
          minimum: -1.0,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: 5,
          description: 'Neighbours polled per point when propagating room labels off the sampled subset.',
          key: 'propagate_k',
          label: 'Propagation neighbours',
          maximum: null,
          minimum: 1.0,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: 0.5,
          description: 'Points with no room label inside this radius stay unlabeled.',
          key: 'propagate_max_radius',
          label: 'Propagation radius [m]',
          maximum: 10.0,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: null,
          description: 'Filter expression restricting which points are partitioned. Empty means the whole cloud.',
          key: 'filter',
          label: 'Point filter',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'string',
        },
      ],
      ready: true,
      runnable: true,
      stage: 'rooms',
      summary: 'partition the plane graph into rooms',
    },
    {
      blockers: [],
      cancellable: true,
      command: 'rux create instances',
      issues: [],
      log_name: 'segment_instances',
      outputs: [
        'instances',
      ],
      parameters: [
        {
          default: 'labels',
          description: 'Name of the semantic-label cloud to split into instances.',
          key: 'semantic_cloud',
          label: 'Semantic cloud',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'string',
        },
        {
          default: 'instances',
          description: 'Name of the instance-label cloud to write.',
          key: 'output_cloud',
          label: 'Output cloud',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'string',
        },
        {
          default: 0.5,
          description: 'Euclidean distance threshold separating two instances.',
          key: 'cluster_tolerance',
          label: 'Cluster tolerance [m]',
          maximum: 5.0,
          minimum: 0.01,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 50,
          description: 'Smallest accepted instance, in points.',
          key: 'min_cluster_size',
          label: 'Minimum instance size',
          maximum: 100000.0,
          minimum: 1.0,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: 1000000,
          description: 'Largest accepted instance, in points.',
          key: 'max_cluster_size',
          label: 'Maximum instance size',
          maximum: 10000000.0,
          minimum: 10.0,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: null,
          description: 'Restrict clustering to these semantic labels. Empty means every label above 0.',
          key: 'labels',
          label: 'Semantic labels',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'integer_list',
        },
      ],
      ready: true,
      runnable: true,
      stage: 'instances',
      summary: 'split semantic labels into spatial instances',
    },
    {
      blockers: [],
      cancellable: false,
      command: 'rux create mesh',
      issues: [],
      log_name: '',
      outputs: [
        'mesh',
      ],
      parameters: [],
      ready: true,
      runnable: false,
      stage: 'mesh',
      summary: 'solidify the cell complex into a room-partitioned mesh',
    },
  ],
};

/**
 * `GET /api/v1/stages` on a freshly created, empty project.
 *
 * Recorded because the interesting half of a stage card only exists here: an
 * unready stage, its missing artifacts, and the derived resolution commands
 * (#295) the card turns into "run X first".
 */
export const STAGES_EMPTY_PROJECT: { stages: StageInfo[] } = {
  stages: [
    {
      blockers: [
        'missing_stage_input: stage \'clouds\' requires stored sensor frames (run \'rux import\' first)',
      ],
      cancellable: false,
      command: 'rux create clouds',
      issues: [
        {
          artifact: 'sensor_frames',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
          ],
          hint: 'Run \'rux import rtabmap <scan.db>\' to produce \'sensor_frames\'',
          message: 'stage \'clouds\' requires stored sensor frames (run \'rux import\' first)',
          severity: 'error',
        },
      ],
      log_name: 'cloud_reconstruction',
      outputs: [
        'cloud',
        'normals',
      ],
      parameters: [
        {
          default: 0.05,
          description: 'Voxel grid resolution used to downsample the fused cloud.',
          key: 'resolution',
          label: 'Voxel size [m]',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 0.0,
          description: 'Depth samples closer than this are discarded.',
          key: 'min_distance',
          label: 'Minimum depth [m]',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 4.0,
          description: 'Depth samples farther than this are discarded.',
          key: 'max_distance',
          label: 'Maximum depth [m]',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 4,
          description: 'Keep every Nth pixel of each depth frame.',
          key: 'sampling_factor',
          label: 'Pixel subsampling',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: 2,
          description: 'Discard depth samples below this confidence level.',
          key: 'confidence_threshold',
          label: 'Minimum confidence',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'integer',
        },
      ],
      ready: false,
      runnable: true,
      stage: 'clouds',
      summary: 'back-project depth frames into a fused cloud',
    },
    {
      blockers: [
        'missing_stage_input: stage \'planes\' requires cloud \'cloud\' which is not present in the project',
        'missing_stage_input: stage \'planes\' requires cloud \'normals\' which is not present in the project',
      ],
      cancellable: true,
      command: 'rux create planes',
      issues: [
        {
          artifact: 'cloud',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds',
          message: 'stage \'planes\' requires cloud \'cloud\' which is not present in the project',
          severity: 'error',
        },
        {
          artifact: 'normals',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds',
          message: 'stage \'planes\' requires cloud \'normals\' which is not present in the project',
          severity: 'error',
        },
      ],
      log_name: 'segment_planes',
      outputs: [
        'planes',
        'plane_centroids',
        'plane_normals',
      ],
      parameters: [
        {
          default: 25.0,
          description: 'Maximum normal deviation for a point to join a plane.',
          key: 'angle_threshold',
          label: 'Angle threshold [deg]',
          maximum: 365.0,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 0.07,
          description: 'Maximum point-to-plane distance. Sending this key at all pins the threshold and switches off adaptive derivation for it.',
          key: 'plane_dist_threshold',
          label: 'Distance threshold [m]',
          maximum: 1.0,
          minimum: 0.0,
          presence_sensitive: true,
          type: 'number',
        },
        {
          default: 1000,
          description: 'Smallest accepted plane, in points. Sending this key at all pins it and switches off adaptive derivation for it.',
          key: 'min_inliers',
          label: 'Minimum cluster size',
          maximum: 1000000.0,
          minimum: 3.0,
          presence_sensitive: true,
          type: 'integer',
        },
        {
          default: 0.5,
          description: 'Neighbourhood radius used while growing a plane.',
          key: 'radius',
          label: 'Region-growing radius [m]',
          maximum: 5.0,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 16.0,
          description: 'Initial interval between plane refits.',
          key: 'interval_0',
          label: 'Initial refit interval',
          maximum: 10000.0,
          minimum: 1.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 1.5,
          description: 'Multiplier applied to the refit interval after each refit.',
          key: 'interval_factor',
          label: 'Refit interval factor',
          maximum: 10.0,
          minimum: 1.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: true,
          description: 'Derive the distance threshold (~3 sigma) and the minimum cluster size from measured cloud noise. Explicit values still win per parameter.',
          key: 'adaptive',
          label: 'Adaptive thresholds',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'boolean',
        },
        {
          default: 42,
          description: 'Deterministic seed for the noise estimator (STANDARDS §6).',
          key: 'noise_seed',
          label: 'Noise-estimator seed',
          maximum: null,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: null,
          description: 'Filter expression restricting which points are segmented. Empty means the whole cloud.',
          key: 'filter',
          label: 'Point filter',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'string',
        },
      ],
      ready: false,
      runnable: true,
      stage: 'planes',
      summary: 'detect planar surfaces',
    },
    {
      blockers: [
        'missing_stage_input: stage \'rooms\' requires cloud \'cloud\' which is not present in the project',
        'missing_stage_input: stage \'rooms\' requires cloud \'planes\' which is not present in the project',
        'missing_stage_input: stage \'rooms\' requires cloud \'plane_centroids\' which is not present in the project',
        'missing_stage_input: stage \'rooms\' requires cloud \'plane_normals\' which is not present in the project',
      ],
      cancellable: true,
      command: 'rux create rooms',
      issues: [
        {
          artifact: 'cloud',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds',
          message: 'stage \'rooms\' requires cloud \'cloud\' which is not present in the project',
          severity: 'error',
        },
        {
          artifact: 'planes',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
            'rux create planes',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds\n    rux create planes',
          message: 'stage \'rooms\' requires cloud \'planes\' which is not present in the project',
          severity: 'error',
        },
        {
          artifact: 'plane_centroids',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
            'rux create planes',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds\n    rux create planes',
          message: 'stage \'rooms\' requires cloud \'plane_centroids\' which is not present in the project',
          severity: 'error',
        },
        {
          artifact: 'plane_normals',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
            'rux create planes',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds\n    rux create planes',
          message: 'stage \'rooms\' requires cloud \'plane_normals\' which is not present in the project',
          severity: 'error',
        },
      ],
      log_name: 'segment_rooms',
      outputs: [
        'rooms',
      ],
      parameters: [
        {
          default: 0.5,
          description: 'Spatial discretisation used to build the plane graph.',
          key: 'grid_size',
          label: 'Grid size [m]',
          maximum: 10.0,
          minimum: 0.01,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 1.0,
          description: 'Higher values produce more, smaller rooms.',
          key: 'resolution',
          label: 'Leiden resolution',
          maximum: 10.0,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 0.01,
          description: 'Randomness of the refinement phase; lower is more deterministic.',
          key: 'beta',
          label: 'Leiden beta',
          maximum: 1.0,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 100,
          description: 'Finite bound on the Leiden iteration count.',
          key: 'max_iter',
          label: 'Maximum iterations',
          maximum: 1000.0,
          minimum: -1.0,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: 5,
          description: 'Neighbours polled per point when propagating room labels off the sampled subset.',
          key: 'propagate_k',
          label: 'Propagation neighbours',
          maximum: null,
          minimum: 1.0,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: 0.5,
          description: 'Points with no room label inside this radius stay unlabeled.',
          key: 'propagate_max_radius',
          label: 'Propagation radius [m]',
          maximum: 10.0,
          minimum: 0.0,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: null,
          description: 'Filter expression restricting which points are partitioned. Empty means the whole cloud.',
          key: 'filter',
          label: 'Point filter',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'string',
        },
      ],
      ready: false,
      runnable: true,
      stage: 'rooms',
      summary: 'partition the plane graph into rooms',
    },
    {
      blockers: [
        'missing_stage_input: stage \'instances\' requires cloud \'cloud\' which is not present in the project',
        'missing_stage_input: stage \'instances\' requires one of the clouds \'labels\' or \'planes\', none of which is present in the project',
      ],
      cancellable: true,
      command: 'rux create instances',
      issues: [
        {
          artifact: 'cloud',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds',
          message: 'stage \'instances\' requires cloud \'cloud\' which is not present in the project',
          severity: 'error',
        },
        {
          artifact: 'labels',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
            'rux create annotate -n <model>',
            'rux create project',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds\n    rux create annotate -n <model>\n    rux create project',
          message: 'stage \'instances\' requires one of the clouds \'labels\' or \'planes\', none of which is present in the project',
          severity: 'error',
        },
      ],
      log_name: 'segment_instances',
      outputs: [
        'instances',
      ],
      parameters: [
        {
          default: 'labels',
          description: 'Name of the semantic-label cloud to split into instances.',
          key: 'semantic_cloud',
          label: 'Semantic cloud',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'string',
        },
        {
          default: 'instances',
          description: 'Name of the instance-label cloud to write.',
          key: 'output_cloud',
          label: 'Output cloud',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'string',
        },
        {
          default: 0.5,
          description: 'Euclidean distance threshold separating two instances.',
          key: 'cluster_tolerance',
          label: 'Cluster tolerance [m]',
          maximum: 5.0,
          minimum: 0.01,
          presence_sensitive: false,
          type: 'number',
        },
        {
          default: 50,
          description: 'Smallest accepted instance, in points.',
          key: 'min_cluster_size',
          label: 'Minimum instance size',
          maximum: 100000.0,
          minimum: 1.0,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: 1000000,
          description: 'Largest accepted instance, in points.',
          key: 'max_cluster_size',
          label: 'Maximum instance size',
          maximum: 10000000.0,
          minimum: 10.0,
          presence_sensitive: false,
          type: 'integer',
        },
        {
          default: null,
          description: 'Restrict clustering to these semantic labels. Empty means every label above 0.',
          key: 'labels',
          label: 'Semantic labels',
          maximum: null,
          minimum: null,
          presence_sensitive: false,
          type: 'integer_list',
        },
      ],
      ready: false,
      runnable: true,
      stage: 'instances',
      summary: 'split semantic labels into spatial instances',
    },
    {
      blockers: [
        'missing_stage_input: stage \'mesh\' requires cloud \'cloud\' which is not present in the project',
        'missing_stage_input: stage \'mesh\' requires cloud \'normals\' which is not present in the project',
        'missing_stage_input: stage \'mesh\' requires cloud \'rooms\' which is not present in the project',
        'missing_stage_input: stage \'mesh\' requires cloud \'planes\' which is not present in the project',
        'missing_stage_input: stage \'mesh\' requires cloud \'plane_centroids\' which is not present in the project',
        'missing_stage_input: stage \'mesh\' requires cloud \'plane_normals\' which is not present in the project',
      ],
      cancellable: false,
      command: 'rux create mesh',
      issues: [
        {
          artifact: 'cloud',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds',
          message: 'stage \'mesh\' requires cloud \'cloud\' which is not present in the project',
          severity: 'error',
        },
        {
          artifact: 'normals',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds',
          message: 'stage \'mesh\' requires cloud \'normals\' which is not present in the project',
          severity: 'error',
        },
        {
          artifact: 'rooms',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
            'rux create planes',
            'rux create rooms',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds\n    rux create planes\n    rux create rooms',
          message: 'stage \'mesh\' requires cloud \'rooms\' which is not present in the project',
          severity: 'error',
        },
        {
          artifact: 'planes',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
            'rux create planes',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds\n    rux create planes',
          message: 'stage \'mesh\' requires cloud \'planes\' which is not present in the project',
          severity: 'error',
        },
        {
          artifact: 'plane_centroids',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
            'rux create planes',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds\n    rux create planes',
          message: 'stage \'mesh\' requires cloud \'plane_centroids\' which is not present in the project',
          severity: 'error',
        },
        {
          artifact: 'plane_normals',
          check: 'missing_stage_input',
          commands: [
            'rux import rtabmap <scan.db>',
            'rux create clouds',
            'rux create planes',
          ],
          hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds\n    rux create planes',
          message: 'stage \'mesh\' requires cloud \'plane_normals\' which is not present in the project',
          severity: 'error',
        },
      ],
      log_name: '',
      outputs: [
        'mesh',
      ],
      parameters: [],
      ready: false,
      runnable: false,
      stage: 'mesh',
      summary: 'solidify the cell complex into a room-partitioned mesh',
    },
  ],
};

/** `GET /api/v1/stages/planes/validation` on that same empty project. */
export const STAGE_VALIDATION_BLOCKED: StageInfo = {
  blockers: [
    'missing_stage_input: stage \'planes\' requires cloud \'cloud\' which is not present in the project',
    'missing_stage_input: stage \'planes\' requires cloud \'normals\' which is not present in the project',
  ],
  cancellable: true,
  command: 'rux create planes',
  issues: [
    {
      artifact: 'cloud',
      check: 'missing_stage_input',
      commands: [
        'rux import rtabmap <scan.db>',
        'rux create clouds',
      ],
      hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds',
      message: 'stage \'planes\' requires cloud \'cloud\' which is not present in the project',
      severity: 'error',
    },
    {
      artifact: 'normals',
      check: 'missing_stage_input',
      commands: [
        'rux import rtabmap <scan.db>',
        'rux create clouds',
      ],
      hint: 'Run the following commands in order:\n    rux import rtabmap <scan.db>\n    rux create clouds',
      message: 'stage \'planes\' requires cloud \'normals\' which is not present in the project',
      severity: 'error',
    },
  ],
  log_name: 'segment_planes',
  outputs: [
    'planes',
    'plane_centroids',
    'plane_normals',
  ],
  parameters: [
    {
      default: 25.0,
      description: 'Maximum normal deviation for a point to join a plane.',
      key: 'angle_threshold',
      label: 'Angle threshold [deg]',
      maximum: 365.0,
      minimum: 0.0,
      presence_sensitive: false,
      type: 'number',
    },
    {
      default: 0.07,
      description: 'Maximum point-to-plane distance. Sending this key at all pins the threshold and switches off adaptive derivation for it.',
      key: 'plane_dist_threshold',
      label: 'Distance threshold [m]',
      maximum: 1.0,
      minimum: 0.0,
      presence_sensitive: true,
      type: 'number',
    },
    {
      default: 1000,
      description: 'Smallest accepted plane, in points. Sending this key at all pins it and switches off adaptive derivation for it.',
      key: 'min_inliers',
      label: 'Minimum cluster size',
      maximum: 1000000.0,
      minimum: 3.0,
      presence_sensitive: true,
      type: 'integer',
    },
    {
      default: 0.5,
      description: 'Neighbourhood radius used while growing a plane.',
      key: 'radius',
      label: 'Region-growing radius [m]',
      maximum: 5.0,
      minimum: 0.0,
      presence_sensitive: false,
      type: 'number',
    },
    {
      default: 16.0,
      description: 'Initial interval between plane refits.',
      key: 'interval_0',
      label: 'Initial refit interval',
      maximum: 10000.0,
      minimum: 1.0,
      presence_sensitive: false,
      type: 'number',
    },
    {
      default: 1.5,
      description: 'Multiplier applied to the refit interval after each refit.',
      key: 'interval_factor',
      label: 'Refit interval factor',
      maximum: 10.0,
      minimum: 1.0,
      presence_sensitive: false,
      type: 'number',
    },
    {
      default: true,
      description: 'Derive the distance threshold (~3 sigma) and the minimum cluster size from measured cloud noise. Explicit values still win per parameter.',
      key: 'adaptive',
      label: 'Adaptive thresholds',
      maximum: null,
      minimum: null,
      presence_sensitive: false,
      type: 'boolean',
    },
    {
      default: 42,
      description: 'Deterministic seed for the noise estimator (STANDARDS §6).',
      key: 'noise_seed',
      label: 'Noise-estimator seed',
      maximum: null,
      minimum: 0.0,
      presence_sensitive: false,
      type: 'integer',
    },
    {
      default: null,
      description: 'Filter expression restricting which points are segmented. Empty means the whole cloud.',
      key: 'filter',
      label: 'Point filter',
      maximum: null,
      minimum: null,
      presence_sensitive: false,
      type: 'string',
    },
  ],
  ready: false,
  runnable: true,
  stage: 'planes',
  summary: 'detect planar surfaces',
};

/**
 * `GET /api/v1/pipeline-log`, after clouds -> planes -> rooms were run
 * through `POST /jobs`. Every entry therefore carries a `job_id` marker.
 */
export const PIPELINE_LOG: { entries: PipelineLogEntry[] } = {
  entries: [
    {
      error_msg: '',
      finished_at: '2026-09-09 07:26:33',
      id: 3,
      parameters: '{"job_id":"63b63407-f32f-4084-b838-0fb8c391e947","resolution":1.5}',
      stage: 'segment_rooms',
      started_at: '2026-09-09 07:26:33',
      status: 'success',
    },
    {
      error_msg: '',
      finished_at: '2026-09-09 07:26:31',
      id: 2,
      parameters: '{"job_id":"dabee4d9-e013-4eb9-b63d-041e6a38dfa4"}',
      stage: 'segment_planes',
      started_at: '2026-09-09 07:26:31',
      status: 'success',
    },
    {
      error_msg: '',
      finished_at: '2026-09-09 07:26:30',
      id: 1,
      parameters: '{"job_id":"a69f2b2c-9185-4a28-acd4-699adb2d0dab","resolution":0.05}',
      stage: 'cloud_reconstruction',
      started_at: '2026-09-09 07:26:30',
      status: 'success',
    },
  ],
};

/** `GET /api/v1/jobs`, after the recorded `planes` run finished. */
/**
 * The job id embedded in `PIPELINE_LOG`'s newest entry.
 *
 * This is the join `pipeline_log` -> job (#274 S6): a run started through
 * `POST /jobs` writes its id into the stored parameters, which is how durable
 * history is matched back to a job after the server that ran it has forgotten
 * about it.
 */
export const PIPELINE_LOG_JOB_ID = '63b63407-f32f-4084-b838-0fb8c391e947';

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

// ---------------------------------------------------------------------------
// Phase 4 (#265): the editor surfaces.
//
// Recorded on 2026-09-09 from a `rux gui` serving a copy of
// `tests/fixtures/scans/office_corridor.rux` driven clouds -> planes -> rooms,
// with building components and material passports written directly through
// `ProjectDB` (the pipeline route to them, `create instances` -> `create
// windows`, needs exported SAM/YOLO engines this machine does not have) and a
// label legend seeded with the class names an annotate run would produce.
// Everything below is the body the server actually returned.
// ---------------------------------------------------------------------------

/** `GET /frames` — unfiltered. This scan has no segmentation images. */
export const FRAMES: FrameList = {
  height: 960,
  ids: [1995, 1996, 1997, 1998, 1999, 2000, 2001, 2002, 2003, 2004],
  segmented_count: 0,
  total_count: 10,
  width: 720,
};

/** `GET /frames/1997`. */
export const FRAME: FrameInfo = {
  has_confidence: true,
  has_depth: true,
  has_segmentation: false,
  id: 1997,
  intrinsics: {
    cx: 360.99725341796875,
    cy: 476.7929382324219,
    fx: 799.1746826171875,
    fy: 799.1746826171875,
    height: 960,
    width: 720,
  },
  pose: [
    -0.8896197080612183, -0.05107032507658005, 0.4534837007522583, 22.86699676513672,
    0.4499998390674591, 0.05242680013179779, 0.8914709091186523, 8.574508666992188,
    -0.06938230991363525, 0.9973161816596985, -0.023634541779756546, 0.33636292815208435,
    0.0, 0.0, 0.0, 1.0,
  ],
  timestamp: 1773126993.1998198,
};

/** `GET /components` — the `area` and `source_instance_guid` fields are #265 Phase 4. */
export const COMPONENTS: ComponentInfo[] = [
  {
    area: 1.4400000000000002,
    confidence: 0.94,
    guid: 'cmp-0001',
    name: 'Window-01',
    parent_id: -1,
    source_instance_guid: 'inst-a1b2c3d4',
    type: 'window',
    vertex_count: 4,
  },
  {
    area: 1.44,
    confidence: 0.88,
    guid: 'cmp-0002',
    name: 'Window-02',
    parent_id: -1,
    source_instance_guid: 'inst-e5f6a7b8',
    type: 'window',
    vertex_count: 4,
  },
  {
    area: 0.75,
    confidence: 0.71,
    guid: 'cmp-0003',
    name: 'Window-03',
    parent_id: -1,
    source_instance_guid: 'inst-c9d0e1f2',
    type: 'window',
    vertex_count: 4,
  },
  {
    area: 1.947499999999998,
    confidence: 0.82,
    guid: 'cmp-0004',
    name: 'Door-01',
    parent_id: -1,
    source_instance_guid: 'inst-33445566',
    type: 'door',
    vertex_count: 4,
  },
  // Manually created: confidence -1, and no provenance link at all.
  {
    area: 1.9475000000000033,
    confidence: -1.0,
    guid: 'cmp-0005',
    name: 'Door-02',
    parent_id: -1,
    type: 'door',
    vertex_count: 4,
  },
  {
    area: 21.840000000000003,
    confidence: -1.0,
    guid: 'cmp-0006',
    name: 'Wall-North',
    parent_id: -1,
    type: 'wall',
    vertex_count: 4,
  },
];

/** `GET /clouds/labels/labels`. */
export const LABEL_LEGEND: LabelLegend = {
  labels: {
    '1': 'wall',
    '2': 'floor',
    '3': 'ceiling',
    '4': 'door',
    '5': 'window',
    '6': 'chair',
    '7': 'table',
    '8': 'monitor',
    '9': 'potted plant',
    '10': 'sink',
  },
};
