// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  ALL_CASES_PATH,
  DRAWER_QUERY,
  NAV_ENTRIES,
  badgeText,
  drawerClickCloses,
  drawerKeyAction,
  displayProjectName,
  entriesIn,
} from '../app/navigation';
import * as navigation from '../app/navigation';
import type { Health, ProjectSummary } from '../api/types';

describe('navigation model', () => {
  it('lists the case workflow in the prototype order', () => {
    expect(entriesIn('sag').map((e) => e.label)).toEqual([
      'Overblik',
      'Kortlægning',
      'Miljø & prøver',
      'Rapport',
      'Indberetning',
      'On-site',
    ]);
  });

  it('lists On-site last in the case workflow', () => {
    const sag = NAV_ENTRIES.filter((e) => e.group === 'sag');
    expect(sag.at(-1)).toEqual({ to: '/on-site', label: 'On-site', group: 'sag' });
  });

  it('keeps every existing technical route reachable under Værktøjer', () => {
    const tools = entriesIn('tools').map((e) => e.to);
    for (const path of [
      '/projektdata',
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

  it('keeps the old project inventory reachable as the first tool', () => {
    const tools = entriesIn('tools');
    expect(tools[0]).toEqual({ to: '/projektdata', label: 'Projektdata', group: 'tools' });
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

  it('makes Kortlægning a live destination', () => {
    const entry = NAV_ENTRIES.find((e) => e.to === '/kortlaegning');
    expect(entry).toBeDefined();
    expect(entry?.pending).toBeUndefined();
  });

  it('makes Miljø & prøver a live destination', () => {
    const entry = NAV_ENTRIES.find((e) => e.to === '/miljoe');
    expect(entry).toBeDefined();
    expect(entry?.pending).toBeUndefined();
  });

  it('makes Rapport a live destination', () => {
    const entry = NAV_ENTRIES.find((e) => e.to === '/rapport');
    expect(entry).toBeDefined();
    expect(entry?.pending).toBeUndefined();
  });

  it('makes Indberetning a live destination', () => {
    const entry = NAV_ENTRIES.find((e) => e.to === '/indberetning');
    expect(entry).toBeDefined();
    expect(entry?.pending).toBeUndefined();
  });

  it('points "Alle sager" at the case list', () => {
    expect(ALL_CASES_PATH).toBe('/sager');
  });

  it('makes "Alle sager" a live link: nothing is pending any more', () => {
    expect('ALL_CASES_PENDING' in navigation).toBe(false);
  });

  it('hides a badge for no count and zero, caps it at 99+', () => {
    expect(badgeText(undefined)).toBeNull();
    expect(badgeText(0)).toBeNull();
    expect(badgeText(7)).toBe('7');
    expect(badgeText(120)).toBe('99+');
  });
});

describe('displayProjectName', () => {
  const health = { project: { name: 'scan.rux', open: true } } as Health;
  it('prefers the first project record name', () => {
    const summary = { projects: [{ id: 'a', name: 'Måløv Byvej 229' }] } as ProjectSummary;
    expect(displayProjectName(summary, health)).toBe('Måløv Byvej 229');
  });
  it('falls back to the file name when the record has no name', () => {
    const summary = { projects: [{ id: 'a', name: '' }] } as ProjectSummary;
    expect(displayProjectName(summary, health)).toBe('scan.rux');
  });
  it('is undefined while nothing has loaded', () => {
    expect(displayProjectName(undefined, undefined)).toBeUndefined();
  });
});

describe('the sidebar drawer below 900px (R5)', () => {
  it('matches the CSS breakpoint, in rem like the existing 45rem query', () => {
    expect(DRAWER_QUERY).toBe('(max-width: 56.25rem)');
  });

  it('closes on Esc anywhere in the shell while open', () => {
    expect(drawerKeyAction({ key: 'Escape', kind: 'control', open: true })).toBe('close');
    expect(drawerKeyAction({ key: 'Escape', kind: 'other', open: true })).toBe('close');
  });

  it('leaves Esc in a text field to the field (R10: revert, not close)', () => {
    expect(drawerKeyAction({ key: 'Escape', kind: 'text', open: true })).toBeNull();
  });

  it('ignores Esc while closed and other keys while open', () => {
    expect(drawerKeyAction({ key: 'Escape', kind: 'other', open: false })).toBeNull();
    expect(drawerKeyAction({ key: 'Enter', kind: 'control', open: true })).toBeNull();
    expect(drawerKeyAction({ key: 'Tab', kind: 'other', open: true })).toBeNull();
  });

  it('closes on any link click inside it, the current page included', () => {
    expect(drawerClickCloses(['SPAN', 'A', 'NAV', 'ASIDE'])).toBe(true);
    expect(drawerClickCloses(['A', 'DIV', 'ASIDE'])).toBe(true);
    expect(drawerClickCloses(['a'])).toBe(true);
  });

  it('stays open for a click on a pending entry or the drawer itself', () => {
    expect(drawerClickCloses(['SPAN', 'SPAN', 'NAV', 'ASIDE'])).toBe(false);
    expect(drawerClickCloses(['DIV', 'ASIDE'])).toBe(false);
    expect(drawerClickCloses([])).toBe(false);
  });
});
