// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { createSerialQueue } from '../app/serialQueue';

function deferred() {
  let resolve!: () => void;
  const promise = new Promise<void>((r) => {
    resolve = r;
  });
  return { promise, resolve };
}

describe('serial queue', () => {
  it('runs tasks one at a time, in the order they were enqueued', async () => {
    const q = createSerialQueue();
    const log: string[] = [];
    const gate = deferred();
    const first = q.enqueue(async () => {
      log.push('a:start');
      await gate.promise;
      log.push('a:end');
    });
    const second = q.enqueue(async () => {
      log.push('b');
    });
    await Promise.resolve();
    await Promise.resolve();
    expect(log).toEqual(['a:start']);
    gate.resolve();
    await first;
    await second;
    expect(log).toEqual(['a:start', 'a:end', 'b']);
  });

  it('keeps going after a failed task and reports the failure to its caller', async () => {
    const q = createSerialQueue();
    const log: string[] = [];
    const bad = q.enqueue(async () => {
      throw new Error('boom');
    });
    const good = q.enqueue(async () => {
      log.push('after');
    });
    await expect(bad).rejects.toThrow('boom');
    await good;
    expect(log).toEqual(['after']);
  });
});
