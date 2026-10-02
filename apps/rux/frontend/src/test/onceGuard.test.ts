// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { createOnceGuard } from '../app/onceGuard';

function deferred() {
  let resolve!: () => void;
  let reject!: (cause: unknown) => void;
  const promise = new Promise<void>((res, rej) => {
    resolve = res;
    reject = rej;
  });
  return { promise, resolve, reject };
}

describe('createOnceGuard (F9)', () => {
  it('sends once for a double call while the first is pending', () => {
    const guard = createOnceGuard();
    const d = deferred();
    let sent = 0;
    const send = () => {
      sent += 1;
      return d.promise;
    };
    expect(guard.run(send)).toBe(true);
    expect(guard.run(send)).toBe(false);
    expect(sent).toBe(1);
    expect(guard.pending).toBe(true);
  });

  it('reopens at once when the run returns nothing (nothing was sent)', () => {
    const guard = createOnceGuard();
    let calls = 0;
    expect(guard.run(() => void (calls += 1))).toBe(true);
    expect(guard.pending).toBe(false);
    expect(guard.run(() => void (calls += 1))).toBe(true);
    expect(calls).toBe(2);
  });

  it('reopens and rethrows when the run throws', () => {
    const guard = createOnceGuard();
    const boom = new Error('boom');
    expect(() =>
      guard.run(() => {
        throw boom;
      }),
    ).toThrow(boom);
    expect(guard.pending).toBe(false);
    expect(guard.run(() => undefined)).toBe(true);
  });

  it('reopens after the pending run settles, resolved or rejected', async () => {
    const guard = createOnceGuard();
    const ok = deferred();
    guard.run(() => ok.promise);
    ok.resolve();
    await ok.promise;
    await Promise.resolve();
    expect(guard.pending).toBe(false);

    const bad = deferred();
    guard.run(() => bad.promise);
    expect(guard.pending).toBe(true);
    bad.reject(new Error('nope'));
    await bad.promise.catch(() => {});
    await Promise.resolve();
    expect(guard.pending).toBe(false);
    expect(guard.run(() => undefined)).toBe(true);
  });
});
