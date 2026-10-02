// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it, vi } from 'vitest';

import { ApiRequestError } from '../api/client';
import { REFRESH_FAILED, cellErrorMessage, refreshAfterSave } from '../kortlaegning/writeOutcome';

describe('cellErrorMessage', () => {
  it('a 400 names the key by its catalogue label, never the raw server text', () => {
    const cause = new ApiRequestError(400, "'lex:294ZkUyYn6TfdwCsbo2X63' must be a whole number, got '1995.5'", '/x');
    expect(cellErrorMessage(cause, 'Year of installation')).toBe('Ugyldig værdi for Year of installation — ikke gemt');
  });

  it('anything else keeps the usual save copy', () => {
    expect(cellErrorMessage(new ApiRequestError(409, 'job', '/x'), 'Mængde')).toBe(
      'Kunne ikke gemme — et pipeline-job kører. Prøv igen om lidt.',
    );
    expect(cellErrorMessage(new Error('netværk'), 'Mængde')).toBe('Kunne ikke gemme: netværk');
  });
});

describe('refreshAfterSave', () => {
  it('a re-read that works reports nothing', async () => {
    const onFailed = vi.fn();
    expect(await refreshAfterSave(async () => {}, onFailed)).toBe(true);
    expect(onFailed).not.toHaveBeenCalled();
  });

  it('a re-read that fails says the view could not refresh, and never throws (the save stands)', async () => {
    const onFailed = vi.fn();
    await expect(
      refreshAfterSave(async () => {
        throw new Error('offline');
      }, onFailed),
    ).resolves.toBe(false);
    expect(onFailed).toHaveBeenCalledWith(REFRESH_FAILED);
    expect(REFRESH_FAILED).toBe('Visningen kunne ikke opdateres — genindlæs siden');
  });
});
