// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { Sam3ModelState, Sam3ModelStatus } from '../api/types';
import {
  BUSY_COPY,
  CONFLICT_COPY,
  FIRST_RUN_COPY,
  SegmentCancelled,
  SegmentRunError,
  decideAfterError,
  runWithProvisioning,
  sam3View,
} from '../data/sam3Provisioning';

const st = (state: Sam3ModelState, progress = 0, message = ''): Sam3ModelStatus => ({
  state,
  progress,
  message,
  use_cuda: true,
});

const httpError = (status: number, message = 'x') => Object.assign(new Error(message), { status });

describe('sam3View', () => {
  it('maps each state to a chip, message and progress', () => {
    expect(sam3View(null).phase).toBe('unknown');
    expect(sam3View(st('ready'))).toMatchObject({ phase: 'ready', message: null, progress: null });
    expect(sam3View(st('absent'))).toMatchObject({ phase: 'first-run', message: FIRST_RUN_COPY });
    expect(sam3View(st('not_built'))).toMatchObject({ phase: 'first-run', message: FIRST_RUN_COPY });
    expect(sam3View(st('downloading', 0.4, 'onnx 40%'))).toMatchObject({
      phase: 'preparing',
      progress: 0.4,
      message: 'onnx 40%',
    });
    expect(sam3View(st('building', 2, ''))).toMatchObject({ phase: 'preparing', progress: 1 });
    expect(sam3View(st('error', 0, 'disk full'))).toMatchObject({ phase: 'error', message: 'disk full' });
  });
});

describe('decideAfterError', () => {
  it('waits for the model on a 503 while it is being prepared', () => {
    for (const s of ['absent', 'not_built', 'downloading', 'building'] as const) {
      expect(decideAfterError(httpError(503), st(s), 0)).toEqual({ kind: 'await-model' });
    }
  });

  it('treats a 503 with a ready (or unknown) model as a busy database, at most 3 times', () => {
    expect(decideAfterError(httpError(503), st('ready'), 0)).toEqual({ kind: 'retry-busy' });
    expect(decideAfterError(httpError(503), null, 2)).toEqual({ kind: 'retry-busy' });
    expect(decideAfterError(httpError(503), st('ready'), 3)).toEqual({ kind: 'fail', message: BUSY_COPY });
  });

  it('surfaces a model error, a failed preparation and a 409 with their own copy', () => {
    expect(decideAfterError(httpError(503), st('error', 0, 'boom'), 0)).toEqual({ kind: 'fail', message: 'boom' });
    const prep = 'SAM3 model preparation failed: no space left';
    expect(decideAfterError(httpError(500, prep), null, 0)).toEqual({ kind: 'fail', message: prep });
    expect(decideAfterError(httpError(409, 'job'), null, 0)).toEqual({ kind: 'fail', message: CONFLICT_COPY });
    expect(decideAfterError(httpError(400, 'bad prompt'), null, 0)).toEqual({ kind: 'fail', message: 'bad prompt' });
  });
});

/** A scripted server: `run` answers from `runs`, `status` from `statuses`. */
function script(runs: Array<() => unknown>, statuses: Array<Sam3ModelStatus | null>) {
  const log: string[] = [];
  let r = 0;
  let s = 0;
  const run = async () => {
    log.push('run');
    const step = runs[Math.min(r++, runs.length - 1)];
    return step();
  };
  const deps = {
    status: async () => {
      log.push('status');
      return statuses[Math.min(s++, statuses.length - 1)];
    },
    sleep: async (ms: number) => {
      log.push(`sleep ${ms}`);
    },
    onStatus: (x: Sam3ModelStatus) => log.push(`on ${x.state}`),
  };
  return { log, run, deps };
}

const throws503 = () => {
  throw httpError(503, 'SAM3 model is being prepared');
};

describe('runWithProvisioning', () => {
  it('returns the first result when the model is ready', async () => {
    const { run, deps, log } = script([() => 'ok'], []);
    await expect(runWithProvisioning(run, deps)).resolves.toBe('ok');
    expect(log).toEqual(['run']);
  });

  it('polls every 2 s while the model is prepared, then retries the run automatically', async () => {
    const { run, deps, log } = script(
      [throws503, () => 'mask'],
      [st('absent'), st('downloading', 0.5), st('building', 0.1), st('ready')],
    );
    await expect(runWithProvisioning(run, deps)).resolves.toBe('mask');
    expect(log).toEqual([
      'run',
      'status',
      'on absent',
      'sleep 2000',
      'status',
      'on downloading',
      'sleep 2000',
      'status',
      'on building',
      'sleep 2000',
      'status',
      'on ready',
      'run',
    ]);
  });

  it('stops with the server message when preparation ends in error', async () => {
    const { run, deps } = script([throws503], [st('building'), st('error', 0, 'trtexec failed')]);
    const err = await runWithProvisioning(run, deps).catch((e: unknown) => e);
    expect(err).toBeInstanceOf(SegmentRunError);
    expect((err as Error).message).toBe('trtexec failed');
  });

  it('retries a busy-database 503 after 1 s, three times, then gives up', async () => {
    const { run, deps, log } = script([throws503], [st('ready')]);
    const err = await runWithProvisioning(run, deps).catch((e: unknown) => e);
    expect((err as Error).message).toBe(BUSY_COPY);
    expect(log.filter((l) => l === 'run')).toHaveLength(4);
    expect(log.filter((l) => l === 'sleep 1000')).toHaveLength(3);
  });

  it('does not retry a 409 or a 500', async () => {
    const a = script([() => { throw httpError(409); }], []);
    await expect(runWithProvisioning(a.run, a.deps)).rejects.toThrow(CONFLICT_COPY);
    expect(a.log).toEqual(['run']);
    const b = script([() => { throw httpError(500, 'SAM3 model preparation failed: x'); }], []);
    await expect(runWithProvisioning(b.run, b.deps)).rejects.toThrow('SAM3 model preparation failed: x');
  });

  it('rethrows non-HTTP errors untouched', async () => {
    const boom = new TypeError('network');
    const { run, deps } = script([() => { throw boom; }], []);
    await expect(runWithProvisioning(run, deps)).rejects.toBe(boom);
  });

  it('stops polling when cancelled', async () => {
    let cancelled = false;
    const { run, deps } = script([throws503], [st('downloading')]);
    const sleep = async () => {
      cancelled = true;
    };
    const err = await runWithProvisioning(run, { ...deps, sleep, isCancelled: () => cancelled }).catch(
      (e: unknown) => e,
    );
    expect(err).toBeInstanceOf(SegmentCancelled);
  });
});
