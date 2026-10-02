// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { disarms } from '../app/armedConfirm';

describe('disarms: when an armed two-click confirm lets go', () => {
  it('disarms on Escape, not on other keys', () => {
    expect(disarms({ kind: 'key', key: 'Escape' })).toBe(true);
    expect(disarms({ kind: 'key', key: 'Enter' })).toBe(false);
    expect(disarms({ kind: 'key', key: 'Tab' })).toBe(false);
  });

  it('disarms on a press outside the armed button, not on it', () => {
    expect(disarms({ kind: 'pointerdown', inside: false })).toBe(true);
    expect(disarms({ kind: 'pointerdown', inside: true })).toBe(false);
  });

  it('disarms when the page turns busy, not when it settles', () => {
    expect(disarms({ kind: 'busy', busy: true })).toBe(true);
    expect(disarms({ kind: 'busy', busy: false })).toBe(false);
  });

  it('disarms on blur', () => {
    expect(disarms({ kind: 'blur' })).toBe(true);
  });
});
