// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { PipelineLogList } from 'reusex-gui';

/** A finished stage sequence plus one still running, mixing GUI and CLI runs. */
export const Default = () => (
  <PipelineLogList
    entries={[
      {
        id: 45,
        stage: 'mesh_generation',
        status: 'running',
        started_at: '2026-09-09 07:40:12',
        parameters: '{"job_id":"c9204ab7"}',
      },
      {
        id: 43,
        stage: 'segment_rooms',
        status: 'success',
        started_at: '2026-09-08 14:06:05',
        finished_at: '2026-09-08 14:06:19',
        parameters: '{"job_id":"5e77b310"}',
      },
      {
        id: 42,
        stage: 'segment_planes',
        status: 'success',
        started_at: '2026-09-08 14:05:10',
        finished_at: '2026-09-08 14:06:02',
        parameters: '{"job_id":"a1c9e02f"}',
      },
      {
        id: 41,
        stage: 'cloud_reconstruction',
        status: 'success',
        started_at: '2026-09-08 14:02:03',
        finished_at: '2026-09-08 14:04:51',
      },
    ]}
  />
);

/** A clean end-to-end pipeline, all run from the CLI — no "via GUI" badges. */
export const CleanRun = () => (
  <PipelineLogList
    entries={[
      {
        id: 44,
        stage: 'mesh_generation',
        status: 'success',
        started_at: '2026-09-08 14:07:00',
        finished_at: '2026-09-08 14:11:47',
      },
      {
        id: 43,
        stage: 'segment_rooms',
        status: 'success',
        started_at: '2026-09-08 14:06:05',
        finished_at: '2026-09-08 14:06:19',
      },
      {
        id: 42,
        stage: 'segment_planes',
        status: 'success',
        started_at: '2026-09-08 14:05:10',
        finished_at: '2026-09-08 14:06:02',
      },
      {
        id: 41,
        stage: 'cloud_reconstruction',
        status: 'success',
        started_at: '2026-09-08 14:02:03',
        finished_at: '2026-09-08 14:04:51',
      },
    ]}
  />
);

/** `segment_rooms` failed on an empty plane graph — the error line stays visible. */
export const WithFailure = () => (
  <PipelineLogList
    entries={[
      {
        id: 32,
        stage: 'segment_rooms',
        status: 'failed',
        started_at: '2026-09-08 09:16:00',
        finished_at: '2026-09-08 09:16:12',
        error_msg: 'segment_rooms: plane graph is empty (0 planes)',
        parameters: '{"job_id":"c3144ecd"}',
      },
      {
        id: 31,
        stage: 'segment_planes',
        status: 'success',
        started_at: '2026-09-08 09:15:00',
        finished_at: '2026-09-08 09:15:48',
      },
      {
        id: 30,
        stage: 'cloud_reconstruction',
        status: 'success',
        started_at: '2026-09-08 09:12:00',
        finished_at: '2026-09-08 09:14:31',
        parameters: '{"job_id":"3fa1220d"}',
      },
    ]}
  />
);

/** No stage has touched this project yet. */
export const Empty = () => <PipelineLogList entries={[]} />;
