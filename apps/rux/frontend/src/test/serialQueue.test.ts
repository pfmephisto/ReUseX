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
  it('idle settles only after every task enqueued before it, failing ones included', async () => {
    const q = createSerialQueue();
    const log: string[] = [];
    const gate = deferred();
    void q.enqueue(async () => {
      await gate.promise;
      log.push('a');
    });
    void q
      .enqueue(async () => {
        log.push('b');
        throw new Error('boom');
      })
      .catch(() => log.push('b:caught'));
    const idle = q.idle().then(() => log.push('idle'));
    const cGate = deferred();
    void q.enqueue(async () => {
      await cGate.promise; // released only after idle settled: idle must not wait for c
      log.push('c');
    });
    await Promise.resolve();
    expect(log).toEqual([]);
    gate.resolve();
    await idle;
    expect(log.slice(0, 3)).toEqual(['a', 'b', 'b:caught']);
    expect(log.indexOf('idle')).toBeGreaterThan(log.indexOf('b'));
    expect(log).not.toContain('c');
    cGate.resolve();
    await expect(q.idle()).resolves.toBeUndefined();
    expect(log).toContain('c');
  });

  it('idle on an empty queue resolves at once', async () => {
    await expect(createSerialQueue().idle()).resolves.toBeUndefined();
  });
});
