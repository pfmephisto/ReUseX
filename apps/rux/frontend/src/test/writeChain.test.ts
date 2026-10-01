// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { appWriteChain, chainFor } from '../app/writeChain';

describe('write chain', () => {
  it('joins the app-wide chain by default and with scope app', () => {
    expect(chainFor()).toBe(appWriteChain);
    expect(chainFor('app')).toBe(appWriteChain);
  });

  it("gives scope 'page' a chain of its own, which the app chain's idle ignores", async () => {
    const own = chainFor('page');
    expect(own).not.toBe(appWriteChain);
    expect(chainFor('page')).not.toBe(own);
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
