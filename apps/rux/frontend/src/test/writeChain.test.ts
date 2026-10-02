// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { beforeEach, describe, expect, it } from 'vitest';

import { appWriteChain, chainFor } from '../app/writeChain';

describe('write chain', () => {
  // `appWriteChain` is a module singleton: start every test on a drained chain
  // so no test depends on what an earlier one left queued.
  beforeEach(async () => {
    await appWriteChain.idle();
  });

  it("chainFor('app') is the app-wide chain; chainFor('page') a fresh, separate one", () => {
    expect(chainFor('app')).toBe(appWriteChain);
    expect(chainFor()).toBe(appWriteChain);
    const page = chainFor('page');
    expect(page).not.toBe(appWriteChain);
    expect(chainFor('page')).not.toBe(page);
  });

  it("a 'page' chain's pending write does not hold up the app chain's idle", async () => {
    const own = chainFor('page');
    let release!: () => void;
    const held = own.enqueue(() => new Promise<void>((resolve) => (release = resolve)));
    await expect(appWriteChain.idle()).resolves.toBeUndefined();
    release();
    await held;
  });

  it('a first load started after a write reads only once that write has landed', async () => {
    const log: string[] = [];
    let release!: () => void;
    void appWriteChain.enqueue(async () => {
      await new Promise<void>((resolve) => (release = resolve));
      log.push('write');
    });
    const load = appWriteChain.idle().then(() => log.push('read'));
    await Promise.resolve();
    expect(log).toEqual([]);
    release();
    await load;
    expect(log).toEqual(['write', 'read']);
  });
});
