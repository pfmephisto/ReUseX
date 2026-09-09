// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { StageCard } from 'reusex-gui';

const noop = () => {};

const resolution = {
  key: 'resolution',
  type: 'number' as const,
  label: 'Voxel size',
  description: 'Grid resolution in metres for the fused cloud.',
  default: 0.05,
  minimum: 0.001,
  maximum: 1,
  presence_sensitive: false,
};

const base = {
  stage: 'planes',
  summary: 'detect planar surfaces',
  command: 'rux create planes',
  runnable: true,
  cancellable: true,
  ready: true,
  outputs: ['planes', 'plane_centroids', 'plane_normals'],
  parameters: [resolution],
  blocked: null,
  warnings: [],
  job: null,
  state: 'idle' as const,
  progress: null,
  queuedCount: 0,
  canRun: true,
  canCancel: false,
  outcome: null,
};

/** Ready to run: summary, outputs, parameter form behind the disclosure. */
export const Ready = () => <StageCard card={base} onRun={noop} onCancel={noop} />;

/** Mid-run with live progress and two more submissions queued behind it. */
export const Running = () => (
  <StageCard
    card={{
      ...base,
      stage: 'clouds',
      summary: 'back-project depth frames into a fused cloud',
      command: 'rux create clouds',
      outputs: ['cloud', 'normals'],
      state: 'running',
      canRun: false,
      canCancel: true,
      queuedCount: 2,
      job: {
        id: '8d57f67b',
        project: 'office.rux',
        stage: 'clouds',
        status: 'running',
        submitted_at: '2026-09-09T07:37:54Z',
        started_at: '2026-09-09T07:37:55Z',
        progress: {
          stage: 'cloud_reconstruction',
          stage_label: 'Assembling cloud',
          current: 7,
          total: 10,
        },
      },
      progress: {
        stage: 'cloud_reconstruction',
        stage_label: 'Assembling cloud',
        current: 7,
        total: 10,
      },
    }}
    onRun={noop}
    onCancel={noop}
  />
);

/** Blocked: the sentence names the missing inputs and the stage to run first. */
export const Blocked = () => (
  <StageCard
    card={{
      ...base,
      stage: 'mesh',
      summary: 'solidify the cell complex into a room-partitioned mesh',
      command: 'rux create mesh',
      outputs: ['mesh'],
      ready: false,
      canRun: false,
      blocked: 'Inputs missing: planes, rooms — run `rux create planes` first.',
    }}
    onRun={noop}
    onCancel={noop}
  />
);

/** Failed run: sticky error plus the honest outcome line. */
export const Failed = () => (
  <StageCard
    card={{
      ...base,
      stage: 'rooms',
      summary: 'partition the plane graph into rooms',
      command: 'rux create rooms',
      outputs: ['rooms'],
      state: 'failed',
      outcome: 'Failed after 12 s — no planes labelled, run `rux create planes` first.',
      job: {
        id: 'c3144ecd',
        project: 'office.rux',
        stage: 'rooms',
        status: 'failed',
        error: 'segment_rooms: plane graph is empty (0 planes)',
        submitted_at: '2026-09-09T07:31:02Z',
        started_at: '2026-09-09T07:31:03Z',
        finished_at: '2026-09-09T07:31:15Z',
      },
    }}
    onRun={noop}
    onCancel={noop}
  />
);
