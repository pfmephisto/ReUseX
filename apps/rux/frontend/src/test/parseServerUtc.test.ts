// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { parseServerUtc } from '../api/types';

describe('parseServerUtc', () => {
  it('parses a sqlite-style zone-less timestamp as UTC', () => {
    const d = parseServerUtc('2026-08-09 10:05:00');
    expect(d).not.toBeNull();
    expect(d!.toISOString()).toBe('2026-08-09T10:05:00.000Z');
  });

  it('returns null for a garbage string', () => {
    expect(parseServerUtc('not a timestamp')).toBeNull();
    expect(parseServerUtc('')).toBeNull();
  });

  it('never reads the string as local time, unlike `new Date(s)`', () => {
    // A regression guard for the bug this helper exists to avoid: passing
    // the zone-less string straight to `new Date` parses it in the host's
    // local timezone, not UTC.
    const d = parseServerUtc('2026-01-01 00:00:00');
    expect(d!.getTime()).toBe(Date.UTC(2026, 0, 1, 0, 0, 0));
  });
});
