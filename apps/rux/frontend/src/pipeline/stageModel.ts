// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Job, JobProgress, StageInfo, ValidationIssue } from '../api/types';
import { isTerminal } from '../api/types';

/** What a stage card is doing right now. `idle` = never run this session. */
export type StageRunState = 'idle' | 'queued' | 'running' | 'succeeded' | 'failed' | 'cancelled';

export interface StageCard {
  stage: string;
  summary: string;
  command: string;
  runnable: boolean;
  cancellable: boolean;
  ready: boolean;
  outputs: string[];
  parameters: StageInfo['parameters'];
  /** One sentence naming the missing inputs and what to run. Null when ready. */
  blocked: string | null;
  /** Warnings that do not block the run. */
  warnings: string[];
  /** The most recent job for this stage, or null. */
  job: Job | null;
  state: StageRunState;
  progress: JobProgress | null;
  /** Runs of this stage waiting behind the one shown. */
  queuedCount: number;
  canRun: boolean;
  canCancel: boolean;
  /** What actually happened, once the job is terminal. Null while it runs. */
  outcome: string | null;
}

function errorIssues(issues: ValidationIssue[]): ValidationIssue[] {
  return issues.filter((issue) => issue.severity === 'error');
}

/**
 * "Inputs missing: cloud, normals — run `rux create clouds` first."
 *
 * Built from the issues' `artifact` and `commands` fields, never from their
 * prose. `commands` is the resolution the server derived by walking the stage
 * contract backwards to the first prerequisite that IS satisfied (#295), so
 * this sentence names the stage the user should actually run next — not merely
 * the stage that nominally produces the missing artifact, which might itself
 * be blocked. The sibling `hint` field says the same thing formatted for a
 * terminal, multi-line and all; splicing that into a sentence would read as
 * "run Run the following commands in order: …".
 */
export function blockedReason(stage: StageInfo): string | null {
  const errors = errorIssues(stage.issues ?? []);
  if (errors.length === 0) return null;

  const artifacts = unique(errors.map((issue) => issue.artifact).filter(Boolean));
  // Order matters and must be preserved: these are pipeline steps to run in
  // sequence, so deduplication keeps first occurrence rather than sorting.
  const commands = unique(errors.flatMap((issue) => issue.commands ?? []).filter(Boolean));

  // No structured detail at all — fall back to whatever the server said rather
  // than inventing a reason or, worse, showing a blocked card with no reason.
  if (artifacts.length === 0 && commands.length === 0)
    return errors.map((issue) => issue.message).join('; ');

  const subject =
    artifacts.length > 0 ? `Inputs missing: ${artifacts.join(', ')}` : 'Inputs not satisfied';
  if (commands.length === 0) return `${subject}.`;
  return `${subject} — run ${commands.join(', then ')} first.`;
}

function unique(values: string[]): string[] {
  return values.filter((value, index) => values.indexOf(value) === index);
}

/**
 * What actually happened to a finished job.
 *
 * The interesting case is the honest-cancel one (PR #274 S4): a cancel the
 * stage could not honour — because it does not poll the token, or because it
 * finished first — is reported by the server as `succeeded` with
 * `cancel_requested` still true. The card must say so. Rendering that as
 * "Cancelled" would tell the user their output was not written when it was,
 * and they would re-run a twenty-minute stage for nothing.
 */
export function outcomeOf(job: Job | null): string | null {
  if (!job || !isTerminal(job.status)) return null;

  if (job.status === 'cancelled') return job.error ? `Cancelled — ${job.error}` : 'Cancelled.';

  if (job.status === 'failed')
    return job.cancel_requested
      ? `Failed after a cancel request${job.error ? ` — ${job.error}` : '.'}`
      : `Failed${job.error ? ` — ${job.error}` : '.'}`;

  if (job.cancel_requested)
    return 'Cancel requested, but the stage finished first — its output was written.';

  return 'Finished.';
}

/** The newest job for @p stage, by submission time then id. */
export function latestJobFor(stage: string, jobs: Job[]): Job | null {
  const mine = jobs.filter((job) => job.stage === stage);
  if (mine.length === 0) return null;
  return mine.reduce((best, job) => (isNewer(job, best) ? job : best));
}

/**
 * The job a stage card should be *about*.
 *
 * Not simply the newest: submitting three runs of one stage leaves two of them
 * queued behind the one actually executing, and the newest of those three is a
 * queued one. A card that showed it would say "Waiting for the worker…" while
 * the worker was visibly busy on that very stage, and hide the only progress
 * bar the user cares about. So a running job wins, then a queued one, then the
 * newest finished one.
 */
export function selectJobFor(stage: string, jobs: Job[]): Job | null {
  const mine = jobs.filter((job) => job.stage === stage);
  if (mine.length === 0) return null;

  const newestOf = (candidates: Job[]) =>
    candidates.length === 0
      ? null
      : candidates.reduce((best, job) => (isNewer(job, best) ? job : best));

  return (
    newestOf(mine.filter((job) => job.status === 'running')) ??
    newestOf(mine.filter((job) => job.status === 'queued')) ??
    newestOf(mine)
  );
}

/** How many jobs of @p stage are waiting behind the one on the card. */
export function queuedCountFor(stage: string, jobs: Job[], shown: Job | null): number {
  return jobs.filter(
    (job) => job.stage === stage && job.status === 'queued' && job.id !== shown?.id,
  ).length;
}

function isNewer(candidate: Job, incumbent: Job): boolean {
  if (candidate.submitted_at !== incumbent.submitted_at)
    return candidate.submitted_at > incumbent.submitted_at;
  // Same timestamp: fall back to id so the choice is at least stable across
  // renders rather than depending on array order.
  return candidate.id > incumbent.id;
}

function runStateOf(job: Job | null): StageRunState {
  if (!job) return 'idle';
  return job.status;
}

/**
 * Turn the catalogue plus live job state into one card model per stage.
 *
 * Everything the card renders is decided here, in a pure function, so the
 * awkward combinations — blocked but currently running, cancellable but
 * already finished, runnable with no runner — are testable without a DOM.
 */
export function buildStageCards(stages: StageInfo[], jobs: Job[]): StageCard[] {
  return stages.map((stage) => {
    const job = selectJobFor(stage.stage, jobs);
    const state = runStateOf(job);
    const busy = state === 'queued' || state === 'running';
    const blocked = blockedReason(stage);

    return {
      stage: stage.stage,
      summary: stage.summary ?? '',
      command: stage.command ?? '',
      runnable: stage.runnable,
      cancellable: stage.cancellable,
      ready: stage.ready,
      outputs: stage.outputs ?? [],
      parameters: stage.parameters ?? [],
      blocked,
      warnings: (stage.issues ?? [])
        .filter((issue) => issue.severity === 'warning')
        .map((issue) => issue.message),
      job,
      state,
      progress: state === 'running' ? (job?.progress ?? null) : null,
      queuedCount: queuedCountFor(stage.stage, jobs, job),
      // Blocked stages stay clickable-looking nowhere: the server would refuse
      // the submission anyway, and a button that produces a 400 is worse than
      // one that explains itself.
      canRun: stage.runnable && stage.ready && !busy,
      // A cancel request is accepted for any live job. Whether it takes effect
      // mid-run is `cancellable`; that difference is surfaced as wording on the
      // button, not by hiding it — a queued job of a non-cancellable stage can
      // still be dropped before it starts.
      canCancel: busy && job !== null,
      outcome: outcomeOf(job),
    };
  });
}
