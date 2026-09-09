// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { PipelineLogEntry } from '../api/types';
import {
  chosenParameters,
  epochMs,
  filterByStage,
  formatDuration,
  historyRows,
  jobIdOf,
  stagesInHistory,
} from '../pipeline/history';
import { PIPELINE_LOG, PIPELINE_LOG_JOB_ID, RECORDED_JOB_ID } from './fixtures';

function entry(overrides: Partial<PipelineLogEntry>): PipelineLogEntry {
  return {
    id: 1,
    stage: 'segment_planes',
    status: 'success',
    started_at: '2026-09-08 14:33:06',
    finished_at: '2026-09-08 14:33:07',
    parameters: '{}',
    error_msg: '',
    ...overrides,
  };
}

describe('epochMs', () => {
  it('reads a zoneless sqlite stamp as UTC, not as local time', () => {
    // `datetime('now')` writes UTC without a marker. Parsing it as local time
    // would shift every duration by the viewer's offset.
    expect(epochMs('2026-09-08 14:33:06')).toBe(Date.parse('2026-09-08T14:33:06Z'));
  });

  it('accepts the ISO form of the same stamp', () => {
    expect(epochMs('2026-09-08T14:33:06')).toBe(epochMs('2026-09-08 14:33:06'));
  });

  it('is null for an unreadable stamp rather than NaN', () => {
    expect(epochMs('')).toBeNull();
    expect(epochMs('whenever')).toBeNull();
  });
});

describe('formatDuration', () => {
  it('is null while the entry is still running', () => {
    expect(formatDuration(entry({ finished_at: '', status: 'running' }))).toBeNull();
  });

  it('renders seconds, minutes and hours', () => {
    expect(
      formatDuration(entry({ started_at: '2026-09-08 14:33:06', finished_at: '2026-09-08 14:33:51' })),
    ).toBe('45s');
    expect(
      formatDuration(entry({ started_at: '2026-09-08 14:00:00', finished_at: '2026-09-08 14:03:20' })),
    ).toBe('3m 20s');
    expect(
      formatDuration(entry({ started_at: '2026-09-08 12:00:00', finished_at: '2026-09-08 14:30:00' })),
    ).toBe('2h 30m');
  });

  it('refuses to invent a negative duration', () => {
    expect(
      formatDuration(entry({ started_at: '2026-09-08 14:33:06', finished_at: '2026-09-08 14:00:00' })),
    ).toBeNull();
  });
});

describe('jobIdOf', () => {
  it('finds the marker a GUI-started run leaves behind', () => {
    expect(jobIdOf(`{"job_id":"${RECORDED_JOB_ID}"}`)).toBe(RECORDED_JOB_ID);
  });

  it('is null for a CLI run, which leaves no marker', () => {
    expect(jobIdOf('{"angle_threshold":25}')).toBeNull();
    expect(jobIdOf('')).toBeNull();
    expect(jobIdOf(undefined)).toBeNull();
  });

  it('survives a malformed parameters column', () => {
    // The column is opaque text written by whatever ran the stage. One bad row
    // must not take out the whole history panel.
    expect(jobIdOf('not json')).toBeNull();
    expect(jobIdOf('[1,2,3]')).toBeNull();
    expect(jobIdOf('{"job_id":42}')).toBeNull();
    expect(jobIdOf('{"job_id":""}')).toBeNull();
  });
});

describe('chosenParameters', () => {
  it('drops the injected job marker, which the user did not choose', () => {
    expect(chosenParameters(`{"job_id":"${RECORDED_JOB_ID}","beta":0.2}`)).toEqual({
      beta: 0.2,
    });
  });

  it('is empty for an unparseable or non-object blob', () => {
    expect(chosenParameters('nonsense')).toEqual({});
    expect(chosenParameters('[1]')).toEqual({});
    expect(chosenParameters(undefined)).toEqual({});
  });
});

describe('historyRows', () => {
  it('orders newest first regardless of the order received', () => {
    const rows = historyRows([
      entry({ id: 1, started_at: '2026-09-08 10:00:00' }),
      entry({ id: 3, started_at: '2026-09-08 12:00:00' }),
      entry({ id: 2, started_at: '2026-09-08 11:00:00' }),
    ]);
    expect(rows.map((row) => row.entry.id)).toEqual([3, 2, 1]);
  });

  it('breaks a same-second tie by id, newest first', () => {
    const rows = historyRows([
      entry({ id: 7, started_at: '2026-09-08 10:00:00' }),
      entry({ id: 9, started_at: '2026-09-08 10:00:00' }),
    ]);
    expect(rows.map((row) => row.entry.id)).toEqual([9, 7]);
  });

  it('does not mutate the array it was given', () => {
    const entries = [
      entry({ id: 1, started_at: '2026-09-08 10:00:00' }),
      entry({ id: 2, started_at: '2026-09-08 12:00:00' }),
    ];
    historyRows(entries);
    expect(entries.map((e) => e.id)).toEqual([1, 2]);
  });

  it('precomputes duration, job id and chosen parameters', () => {
    const rows = historyRows(PIPELINE_LOG.entries);
    expect(rows.length).toBe(PIPELINE_LOG.entries.length);
    for (const row of rows) {
      expect(row.parameters).not.toHaveProperty('job_id');
      if (row.entry.finished_at) expect(row.duration).not.toBeNull();
    }
    // Every recorded run was started through POST /jobs, so each carries the
    // marker that joins it back to a job.
    expect(rows.every((row) => row.jobId !== null)).toBe(true);
    expect(rows[0].jobId).toBe(PIPELINE_LOG_JOB_ID);
  });
});

describe('filterByStage / stagesInHistory', () => {
  it('lists each stage once, in first-seen order', () => {
    const rows = historyRows([
      entry({ id: 3, stage: 'segment_planes', started_at: '2026-09-08 12:00:00' }),
      entry({ id: 2, stage: 'cloud_reconstruction', started_at: '2026-09-08 11:00:00' }),
      entry({ id: 1, stage: 'segment_planes', started_at: '2026-09-08 10:00:00' }),
    ]);
    expect(stagesInHistory(rows)).toEqual(['segment_planes', 'cloud_reconstruction']);
  });

  it('returns everything when no stage is selected', () => {
    const rows = historyRows(PIPELINE_LOG.entries);
    expect(filterByStage(rows, null)).toBe(rows);
  });

  it('keeps only the selected stage', () => {
    const rows = historyRows([
      entry({ id: 2, stage: 'segment_planes' }),
      entry({ id: 1, stage: 'cloud_reconstruction' }),
    ]);
    expect(filterByStage(rows, 'segment_planes').map((row) => row.entry.id)).toEqual([2]);
  });
});
