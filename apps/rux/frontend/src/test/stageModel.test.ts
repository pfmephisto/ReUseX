// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { Job, StageInfo } from '../api/types';
import {
  blockedReason,
  buildStageCards,
  latestJobFor,
  outcomeOf,
  queuedCountFor,
  selectJobFor,
} from '../pipeline/stageModel';
import { JOBS, STAGES, STAGES_EMPTY_PROJECT } from './fixtures';

const recordedJob = JOBS.jobs[0];

function jobWith(overrides: Partial<Job>): Job {
  return { ...recordedJob, ...overrides };
}

function stageNamed(name: string, source: { stages: StageInfo[] } = STAGES): StageInfo {
  const found = source.stages.find((entry) => entry.stage === name);
  if (!found) throw new Error(`no recorded stage '${name}'`);
  return found;
}

describe('blockedReason', () => {
  it('is null for a stage whose contract is satisfied', () => {
    expect(blockedReason(stageNamed('planes'))).toBeNull();
  });

  it('names the missing inputs and the command that produces them', () => {
    // Recorded against an empty project: `planes` needs a cloud, and the
    // derived hint (#295) points at the stage that would produce one.
    const reason = blockedReason(stageNamed('planes', STAGES_EMPTY_PROJECT));
    expect(reason).not.toBeNull();
    expect(reason).toContain('Inputs missing:');
    expect(reason).toContain('cloud');
    expect(reason).toMatch(/run .+ first\.$/);
    // The sentence must carry the structured commands, not the terminal-
    // formatted `hint`, which is multi-line prose.
    expect(reason).not.toContain('Run the following commands in order');
    expect(reason).not.toContain('\n');
  });

  it('keeps multi-step guidance in pipeline order', () => {
    const stage: StageInfo = {
      ...stageNamed('planes'),
      ready: false,
      issues: [
        {
          check: 'missing_stage_input',
          message: 'a',
          severity: 'error',
          hint: 'ignored',
          artifact: 'cloud',
          commands: ['rux import rtabmap <scan.db>', 'rux create clouds'],
        },
      ],
    };
    expect(blockedReason(stage)).toBe(
      'Inputs missing: cloud — run rux import rtabmap <scan.db>, then rux create clouds first.',
    );
  });

  it('does not repeat a command shared by several missing inputs', () => {
    const stage: StageInfo = {
      ...stageNamed('planes'),
      ready: false,
      blockers: ['missing_stage_input: a', 'missing_stage_input: b'],
      issues: [
        {
          check: 'missing_stage_input',
          message: 'a',
          severity: 'error',
          hint: "Run 'rux create clouds' to produce 'cloud'",
          artifact: 'cloud',
          commands: ['rux create clouds'],
        },
        {
          check: 'missing_stage_input',
          message: 'b',
          severity: 'error',
          hint: "Run 'rux create clouds' to produce 'normals'",
          artifact: 'normals',
          commands: ['rux create clouds'],
        },
      ],
    };
    expect(blockedReason(stage)).toBe(
      'Inputs missing: cloud, normals — run rux create clouds first.',
    );
  });

  it('falls back to the server message when there is no structured detail', () => {
    const stage: StageInfo = {
      ...stageNamed('planes'),
      ready: false,
      issues: [
        {
          check: 'stage_input_size_mismatch',
          message: 'something is off',
          severity: 'error',
          hint: '',
          artifact: '',
          commands: [],
        },
      ],
    };
    // A blocked card with no reason at all would be worse than prose.
    expect(blockedReason(stage)).toBe('something is off');
  });

  it('ignores warnings, which do not block a run', () => {
    const stage: StageInfo = {
      ...stageNamed('planes'),
      issues: [
        {
          check: 'orphaned_passport',
          message: 'a passport is unused',
          severity: 'warning',
          hint: '',
          artifact: '',
          commands: [],
        },
      ],
    };
    expect(blockedReason(stage)).toBeNull();
  });
});

describe('outcomeOf', () => {
  it('says nothing while the job is still live', () => {
    expect(outcomeOf(jobWith({ status: 'running' }))).toBeNull();
    expect(outcomeOf(jobWith({ status: 'queued' }))).toBeNull();
    expect(outcomeOf(null)).toBeNull();
  });

  it('reports a stage that finished despite a cancel as finished', () => {
    // PR #274 S4: `clouds` does not poll the cancel token, so a cancel it
    // could not honour comes back as `succeeded` with `cancel_requested`
    // still set. Calling that "Cancelled" would tell the user their output
    // was not written when it was — and they would re-run a long stage for
    // nothing.
    const outcome = outcomeOf(
      jobWith({ status: 'succeeded', cancel_requested: true, error: '' }),
    );
    expect(outcome).toBe(
      'Cancel requested, but the stage finished first — its output was written.',
    );
  });

  it('reports a genuinely cancelled stage as cancelled', () => {
    expect(
      outcomeOf(
        jobWith({
          status: 'cancelled',
          cancel_requested: true,
          error: 'plane segmentation cancelled',
        }),
      ),
    ).toBe('Cancelled — plane segmentation cancelled');
  });

  it('distinguishes a failure after a cancel request from a plain failure', () => {
    expect(
      outcomeOf(jobWith({ status: 'failed', cancel_requested: true, error: 'boom' })),
    ).toBe('Failed after a cancel request — boom');
    expect(
      outcomeOf(jobWith({ status: 'failed', cancel_requested: false, error: 'boom' })),
    ).toBe('Failed — boom');
  });

  it('reports an ordinary success plainly', () => {
    expect(
      outcomeOf(jobWith({ status: 'succeeded', cancel_requested: false, error: '' })),
    ).toBe('Finished.');
  });
});

describe('latestJobFor', () => {
  it('is null when the stage has never been run', () => {
    expect(latestJobFor('rooms', [])).toBeNull();
  });

  it('picks the newest submission, not the array order', () => {
    const older = jobWith({ id: 'a', stage: 'planes', submitted_at: '2026-09-08T10:00:00Z' });
    const newer = jobWith({ id: 'b', stage: 'planes', submitted_at: '2026-09-08T11:00:00Z' });
    expect(latestJobFor('planes', [newer, older])?.id).toBe('b');
    expect(latestJobFor('planes', [older, newer])?.id).toBe('b');
  });

  it('breaks a tie deterministically rather than by array order', () => {
    const first = jobWith({ id: 'a', stage: 'planes', submitted_at: '2026-09-08T10:00:00Z' });
    const second = jobWith({ id: 'b', stage: 'planes', submitted_at: '2026-09-08T10:00:00Z' });
    expect(latestJobFor('planes', [first, second])?.id).toBe(
      latestJobFor('planes', [second, first])?.id,
    );
  });

  it('ignores jobs belonging to another stage', () => {
    expect(latestJobFor('rooms', [jobWith({ stage: 'planes' })])).toBeNull();
  });
});

describe('selectJobFor', () => {
  it('prefers the running job over a newer queued one', () => {
    // Submitting three runs leaves two queued behind the executing one, and
    // the newest of the three is queued. A card showing that would say
    // "Waiting for the worker…" while the worker was busy on this very stage.
    const running = jobWith({
      id: 'a',
      stage: 'planes',
      status: 'running',
      submitted_at: '2026-09-09T10:00:00Z',
    });
    const queued = jobWith({
      id: 'b',
      stage: 'planes',
      status: 'queued',
      submitted_at: '2026-09-09T10:00:05Z',
    });
    expect(selectJobFor('planes', [running, queued])?.id).toBe('a');
  });

  it('prefers a queued job over an older finished one', () => {
    const done = jobWith({
      id: 'a',
      stage: 'planes',
      status: 'succeeded',
      submitted_at: '2026-09-09T10:00:00Z',
    });
    const queued = jobWith({
      id: 'b',
      stage: 'planes',
      status: 'queued',
      submitted_at: '2026-09-09T10:00:05Z',
    });
    expect(selectJobFor('planes', [done, queued])?.id).toBe('b');
  });

  it('falls back to the newest finished job', () => {
    const older = jobWith({
      id: 'a',
      stage: 'planes',
      status: 'succeeded',
      submitted_at: '2026-09-09T10:00:00Z',
    });
    const newer = jobWith({
      id: 'b',
      stage: 'planes',
      status: 'failed',
      submitted_at: '2026-09-09T11:00:00Z',
    });
    expect(selectJobFor('planes', [older, newer])?.id).toBe('b');
  });
});

describe('queuedCountFor', () => {
  it('counts the runs waiting behind the one on the card', () => {
    const jobs = [
      jobWith({ id: 'a', stage: 'planes', status: 'running' }),
      jobWith({ id: 'b', stage: 'planes', status: 'queued' }),
      jobWith({ id: 'c', stage: 'planes', status: 'queued' }),
      jobWith({ id: 'd', stage: 'rooms', status: 'queued' }),
    ];
    const shown = selectJobFor('planes', jobs);
    expect(shown?.id).toBe('a');
    expect(queuedCountFor('planes', jobs, shown)).toBe(2);
  });

  it('does not count the job it is showing', () => {
    const jobs = [jobWith({ id: 'b', stage: 'planes', status: 'queued' })];
    const shown = selectJobFor('planes', jobs);
    expect(queuedCountFor('planes', jobs, shown)).toBe(0);
  });
});

describe('buildStageCards', () => {
  it('produces one card per catalogue entry, in order', () => {
    const cards = buildStageCards(STAGES.stages, []);
    expect(cards.map((card) => card.stage)).toEqual(STAGES.stages.map((s) => s.stage));
  });

  it('starts idle with no progress and no outcome', () => {
    const card = buildStageCards([stageNamed('planes')], [])[0];
    expect(card.state).toBe('idle');
    expect(card.job).toBeNull();
    expect(card.progress).toBeNull();
    expect(card.outcome).toBeNull();
    expect(card.canCancel).toBe(false);
    expect(card.queuedCount).toBe(0);
  });

  it('offers Run only for a runnable, ready, idle stage', () => {
    const ready = buildStageCards([stageNamed('planes')], [])[0];
    expect(ready.canRun).toBe(true);

    const blocked = buildStageCards([stageNamed('planes', STAGES_EMPTY_PROJECT)], [])[0];
    expect(blocked.ready).toBe(false);
    expect(blocked.canRun).toBe(false);

    // `mesh` has no runner. The card is still listed so its readiness is
    // visible, but a Run button that always 400s would be worse than none.
    const noRunner = buildStageCards([stageNamed('mesh')], [])[0];
    expect(noRunner.runnable).toBe(false);
    expect(noRunner.canRun).toBe(false);
    expect(noRunner.parameters).toEqual([]);
  });

  it('shows progress only while the job is actually running', () => {
    const progress = { stage: 'region_growing', stage_label: 'Region Growing', current: 3, total: 10 };

    const running = buildStageCards(
      [stageNamed('planes')],
      [jobWith({ stage: 'planes', status: 'running', progress })],
    )[0];
    expect(running.state).toBe('running');
    expect(running.progress).toEqual(progress);
    expect(running.canRun).toBe(false);
    expect(running.canCancel).toBe(true);

    // A finished job keeps its last progress payload; rendering it would leave
    // a bar frozen at 3/10 next to the word "Succeeded".
    const done = buildStageCards(
      [stageNamed('planes')],
      [jobWith({ stage: 'planes', status: 'succeeded', progress })],
    )[0];
    expect(done.progress).toBeNull();
    expect(done.canCancel).toBe(false);
    expect(done.canRun).toBe(true);
  });

  it('offers Cancel for a queued job of a stage that cannot stop mid-run', () => {
    // `clouds` is not cancellable, but a job still sitting in the queue can be
    // dropped before it starts — hiding the button would be a lie.
    const card = buildStageCards(
      [stageNamed('clouds')],
      [jobWith({ stage: 'clouds', status: 'queued' })],
    )[0];
    expect(card.cancellable).toBe(false);
    expect(card.canCancel).toBe(true);
  });

  it('separates warnings from blockers', () => {
    const stage: StageInfo = {
      ...stageNamed('planes'),
      issues: [
        {
          check: 'orphaned_passport',
          message: 'a passport is unused',
          severity: 'warning',
          hint: '',
          artifact: '',
          commands: [],
        },
      ],
    };
    const card = buildStageCards([stage], [])[0];
    expect(card.warnings).toEqual(['a passport is unused']);
    expect(card.blocked).toBeNull();
    expect(card.canRun).toBe(true);
  });
});
