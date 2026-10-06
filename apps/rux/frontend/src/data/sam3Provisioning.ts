// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The managed SAM3 model's provisioning flow as data (spec B1).
 *
 * The GUI never names a model: the server downloads the ONNX bundle and builds
 * device-specific TensorRT engines on first use. A segment request that
 * arrives before that is done answers 503 — and so does one that merely found
 * the project database busy. This module is the single place that tells the
 * two apart (by asking `GET /models/sam3/status` after a 503), polls while the
 * model is prepared, and retries the pending run automatically. The
 * Segmentering view, the label queue and the panorama panel all go through
 * `runWithProvisioning`, so they behave identically.
 *
 * Pure apart from the injected `run`/`status`/`sleep`; vitest drives it with a
 * scripted server.
 */

import type { Sam3ModelState, Sam3ModelStatus } from '../api/types';

export const FIRST_RUN_COPY = 'Første kørsel henter og bygger modellen (kan tage flere minutter).';

const COUNT_WORDS = ['', 'én', 'to', 'tre', 'fire'];

/**
 * An install whose engines predate the current engine recipe: the next run
 * rebuilds just those engines once (no download), not the whole model.
 */
export function updateCopy(engines: number): string {
  const n = COUNT_WORDS[engines] ?? String(engines);
  const what = engines === 1 ? `${n} motor` : `${n} motorer`;
  return `Engangsopdatering: næste kørsel bygger ${what} om, så bokse og punkter når modellen (ingen download, et par minutter).`;
}
export const CONFLICT_COPY =
  'Et pipeline-job kører og holder skrivelåsen. Vent til det er færdigt, og prøv igen.';
export const BUSY_COPY = 'Projektdatabasen er optaget af en anden skrivning. Prøv igen om lidt.';

/** Poll interval while the model is downloaded or built. */
export const POLL_MS = 2000;
/** Delay before re-sending after a busy-database 503. */
export const BUSY_DELAY_MS = 1000;
export const MAX_BUSY_RETRIES = 3;

const PREPARING: ReadonlySet<Sam3ModelState> = new Set(['absent', 'not_built', 'downloading', 'building']);

export type Sam3Phase = 'unknown' | 'ready' | 'first-run' | 'preparing' | 'error';

/** What the status chip shows. */
export interface Sam3View {
  phase: Sam3Phase;
  /** Short chip text. */
  label: string;
  /** Longer line under the chip, or null. */
  message: string | null;
  /** 0..1 for a progress bar, or null for none. */
  progress: number | null;
}

const LABELS: Record<Sam3ModelState, string> = {
  ready: 'Model klar',
  absent: 'Model ikke hentet',
  not_built: 'Model ikke bygget',
  downloading: 'Henter model',
  building: 'Bygger model',
  error: 'Modelfejl',
};

export function sam3View(status: Sam3ModelStatus | null | undefined): Sam3View {
  if (!status) return { phase: 'unknown', label: 'Model: ukendt status', message: null, progress: null };
  const label = LABELS[status.state] ?? status.state;
  switch (status.state) {
    case 'ready':
      return { phase: 'ready', label, message: null, progress: null };
    case 'not_built': {
      const update = status.update_engines ?? [];
      if (update.length > 0)
        return { phase: 'first-run', label: 'Model skal opdateres', message: updateCopy(update.length), progress: null };
      return { phase: 'first-run', label, message: FIRST_RUN_COPY, progress: null };
    }
    case 'absent':
      return { phase: 'first-run', label, message: FIRST_RUN_COPY, progress: null };
    case 'downloading':
    case 'building': {
      const p = Number.isFinite(status.progress) ? Math.min(1, Math.max(0, status.progress)) : 0;
      const fallback = status.state === 'downloading' ? 'Henter SAM3-modellen…' : 'Bygger TensorRT-motorer…';
      return { phase: 'preparing', label, message: status.message || fallback, progress: p };
    }
    case 'error':
      return { phase: 'error', label, message: status.message || 'Modellen kunne ikke klargøres.', progress: null };
    default:
      return { phase: 'unknown', label, message: status.message || null, progress: null };
  }
}

/** An HTTP failure as the client reports it (`ApiRequestError` fits). */
interface HttpFailure {
  status: number;
  message: string;
}

function isHttpFailure(e: unknown): e is HttpFailure {
  return (
    typeof e === 'object' &&
    e !== null &&
    typeof (e as HttpFailure).status === 'number' &&
    typeof (e as HttpFailure).message === 'string'
  );
}

export type SegmentErrorAction =
  | { kind: 'await-model' }
  | { kind: 'retry-busy' }
  | { kind: 'fail'; message: string };

/**
 * What to do after a failed segment request.
 *
 * `status` is the model status fetched *after* a 503 (null when it could not
 * be read, which is handled like "ready": the 503 then can only be the busy
 * database). `busyRetries` is how many busy retries were already spent.
 */
export function decideAfterError(
  err: HttpFailure,
  status: Sam3ModelStatus | null,
  busyRetries: number,
  maxBusyRetries = MAX_BUSY_RETRIES,
): SegmentErrorAction {
  if (err.status === 409) return { kind: 'fail', message: CONFLICT_COPY };
  if (err.status === 503) {
    if (status && PREPARING.has(status.state)) return { kind: 'await-model' };
    if (status?.state === 'error') return { kind: 'fail', message: sam3View(status).message ?? err.message };
    if (busyRetries < maxBusyRetries) return { kind: 'retry-busy' };
    // A readable "ready" status pins the 503 on the busy database; with no
    // readable status the server's own reason is the honest message.
    return { kind: 'fail', message: status ? BUSY_COPY : err.message };
  }
  // 500 "SAM3 model preparation failed: …" and everything else: the server's
  // own message is the most specific thing to show.
  return { kind: 'fail', message: err.message };
}

/**
 * Whether a label-queue item's failure stops the whole queue: only causes that
 * would fail every remaining frame the same way — a write lock held by a job
 * (409), a 503 that outlived the busy retries, or a model that could not be
 * prepared (500 "SAM3 model preparation failed: …"). A per-frame 500 (a DB
 * read error, an inference exception) only fails its item.
 */
export function stopsQueue(cause: unknown): boolean {
  if (!isHttpFailure(cause)) return false;
  if (cause.status === 409 || cause.status === 503) return true;
  return cause.status === 500 && cause.message.includes('SAM3 model preparation failed');
}

/** Whether the status chip should keep polling with no run pending: a download or build is in flight. */
export function shouldPollStatus(status: Sam3ModelStatus | null | undefined): boolean {
  return status?.state === 'downloading' || status?.state === 'building';
}

/** A run that failed for a reason the user should read. `cause` is the HTTP error, if any. */
export class SegmentRunError extends Error {
  readonly cause?: unknown;
  constructor(message: string, cause?: unknown) {
    super(message);
    this.name = 'SegmentRunError';
    this.cause = cause;
  }
}

/** The caller went away (unmount, frame change) while waiting for the model. */
export class SegmentCancelled extends Error {
  constructor() {
    super('cancelled');
    this.name = 'SegmentCancelled';
  }
}

export interface ProvisioningDeps {
  /** `GET /models/sam3/status`; resolve null when it cannot be read. */
  status: () => Promise<Sam3ModelStatus | null>;
  sleep: (ms: number) => Promise<void>;
  /** Every status seen, for the chip. */
  onStatus?: (status: Sam3ModelStatus) => void;
  isCancelled?: () => boolean;
  pollMs?: number;
  busyDelayMs?: number;
  maxBusyRetries?: number;
}

/**
 * Run `run`, waiting out model preparation and busy-database 503s.
 *
 * Resolves with the first successful result. Rejects with `SegmentRunError`
 * (message ready to show) for a refusal, `SegmentCancelled` when cancelled,
 * or the original error when it is not an HTTP failure.
 */
export async function runWithProvisioning<T>(run: () => Promise<T>, deps: ProvisioningDeps): Promise<T> {
  const pollMs = deps.pollMs ?? POLL_MS;
  const busyDelayMs = deps.busyDelayMs ?? BUSY_DELAY_MS;
  const maxBusy = deps.maxBusyRetries ?? MAX_BUSY_RETRIES;
  const cancelled = () => deps.isCancelled?.() ?? false;
  const readStatus = async () => {
    const s = await deps.status().catch(() => null);
    if (s) deps.onStatus?.(s);
    return s;
  };

  let busyRetries = 0;
  for (;;) {
    if (cancelled()) throw new SegmentCancelled();
    try {
      return await run();
    } catch (err) {
      if (!isHttpFailure(err)) throw err;
      const status = err.status === 503 ? await readStatus() : null;
      const action = decideAfterError(err, status, busyRetries, maxBusy);
      if (action.kind === 'fail') throw new SegmentRunError(action.message, err);
      if (action.kind === 'retry-busy') {
        busyRetries += 1;
        await deps.sleep(busyDelayMs);
        continue;
      }
      // await-model: poll until ready (then re-send) or error (then stop).
      for (;;) {
        await deps.sleep(pollMs);
        if (cancelled()) throw new SegmentCancelled();
        const s = await readStatus();
        if (s?.state === 'ready') break;
        if (s?.state === 'error') throw new SegmentRunError(sam3View(s).message ?? 'SAM3-fejl', err);
      }
    }
  }
}
