// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { leaveCommit, type DraftValidate } from '../app/textDraft';

const year: DraftValidate<number> = (d) => (/^\d+$/.test(d.trim()) ? { value: Number(d.trim()) } : null);

describe('leaveCommit: a field unmounted while it still has focus', () => {
  it('commits a dirty draft (a field commit is never dropped)', () => {
    expect(leaveCommit({ focused: true, reverted: false, draft: 'Velux', current: '' })).toEqual({
      send: true,
      value: 'Velux',
    });
    expect(leaveCommit({ focused: true, reverted: false, draft: '1995', current: '' }, { validate: year })).toEqual({
      send: true,
      value: 1995,
    });
  });

  it('sends nothing when the blur already ran, the field was reverted, or nothing changed', () => {
    expect(leaveCommit({ focused: false, reverted: false, draft: 'Velux', current: '' }).send).toBe(false);
    expect(leaveCommit({ focused: true, reverted: true, draft: 'Velux', current: '' }).send).toBe(false);
    expect(leaveCommit({ focused: true, reverted: false, draft: ' x ', current: 'x' }).send).toBe(false);
  });

  it('sends nothing for an invalid draft or an emptied required field', () => {
    expect(leaveCommit({ focused: true, reverted: false, draft: 'abc', current: '' }, { validate: year }).send).toBe(false);
    expect(leaveCommit({ focused: true, reverted: false, draft: '', current: 'x' }, true).send).toBe(false);
  });
});
