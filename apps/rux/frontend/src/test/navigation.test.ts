// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ALL_CASES_PATH, NAV_ENTRIES, badgeText, entriesIn } from '../app/navigation';

describe('navigation model', () => {
  it('lists the case workflow in the prototype order', () => {
    expect(entriesIn('sag').map((e) => e.label)).toEqual([
      'Overblik',
      'Kortlægning',
      'Miljø & prøver',
      'Rapport',
      'Indberetning',
    ]);
  });

  it('keeps every existing technical route reachable under Værktøjer', () => {
    const tools = entriesIn('tools').map((e) => e.to);
    for (const path of [
      '/viewport',
      '/graph-view',
      '/pipeline',
      '/pipeline/log',
      '/frames',
      '/geometry',
      '/instances',
      '/materials',
      '/labels',
      '/export',
    ]) {
      expect(tools).toContain(path);
    }
  });

  it('uses unique, extensionless paths', () => {
    const paths = NAV_ENTRIES.map((e) => e.to);
    expect(new Set(paths).size).toBe(paths.length);
    for (const p of paths) expect(p, p).not.toMatch(/\./);
  });

  it('matches /pipeline exactly so /pipeline/log does not light it up', () => {
    expect(NAV_ENTRIES.find((e) => e.to === '/pipeline')?.end).toBe(true);
  });

  it('gives pending entries a reason a user can read', () => {
    for (const e of NAV_ENTRIES.filter((x) => x.pending !== undefined)) {
      expect(e.pending!.length, e.label).toBeGreaterThan(10);
    }
  });

  it('attaches the review-queue and sample badges to their entries', () => {
    expect(NAV_ENTRIES.find((e) => e.to === '/kortlaegning')?.badge).toBe('reviewQueue');
    expect(NAV_ENTRIES.find((e) => e.to === '/miljoe')?.badge).toBe('pendingSamples');
  });

  it('points "Alle sager" at the case list', () => {
    expect(ALL_CASES_PATH).toBe('/sager');
  });

  it('hides a badge for no count and zero, caps it at 99+', () => {
    expect(badgeText(undefined)).toBeNull();
    expect(badgeText(0)).toBeNull();
    expect(badgeText(7)).toBe('7');
    expect(badgeText(120)).toBe('99+');
  });
});
