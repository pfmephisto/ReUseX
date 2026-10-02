// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { draftResets } from '../app/textDraft';

describe('draftResets: snapping a field back after a refused commit', () => {
  it('resets an unfocused field when the signal moved', () => {
    expect(draftResets(0, 1, false)).toBe(true);
  });

  it('never resets a field the user is typing in', () => {
    expect(draftResets(0, 1, true)).toBe(false);
  });

  it('does nothing while the signal has not moved', () => {
    expect(draftResets(2, 2, false)).toBe(false);
    expect(draftResets(2, 2, true)).toBe(false);
  });
});
