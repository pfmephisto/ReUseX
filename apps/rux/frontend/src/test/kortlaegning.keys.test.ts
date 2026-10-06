// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { dialogAction, EVIDENCE_LAST_KEY, EVIDENCE_TABS, tableAction } from '../kortlaegning/keys';

const k = (key: string, extra: Partial<{ metaKey: boolean; ctrlKey: boolean; altKey: boolean; inField: boolean; isControl: boolean }> = {}) => ({
  key,
  inField: false,
  ...extra,
});

describe('table keys', () => {
  it('navigates and acts', () => {
    expect(tableAction(k('ArrowDown'))).toEqual({ type: 'move', delta: 1 });
    expect(tableAction(k('k'))).toEqual({ type: 'move', delta: -1 });
    expect(tableAction(k('ArrowRight'))).toEqual({ type: 'expand' });
    expect(tableAction(k('ArrowLeft'))).toEqual({ type: 'collapse' });
    expect(tableAction(k('Enter'))).toEqual({ type: 'open' });
    expect(tableAction(k('G'))).toEqual({ type: 'approve' });
    expect(tableAction(k('a'))).toEqual({ type: 'reject' });
    expect(tableAction(k('v'))).toEqual({ type: 'star' });
    expect(tableAction(k('4'))).toEqual({ type: 'evidence', tab: 'punktsky' });
  });

  it('stays out of the way while typing, except Escape', () => {
    expect(tableAction(k('g', { inField: true }))).toBeNull();
    expect(tableAction(k('ArrowDown', { inField: true }))).toBeNull();
    expect(tableAction(k('Escape', { inField: true }))).toEqual({ type: 'blur' });
  });

  it('leaves Enter and Space to a focused button, but keeps the other keys', () => {
    expect(tableAction(k('Enter', { isControl: true }))).toBeNull();
    expect(tableAction(k(' ', { isControl: true }))).toBeNull();
    expect(tableAction(k('ArrowDown', { isControl: true }))).toEqual({ type: 'move', delta: 1 });
    expect(tableAction(k('g', { isControl: true }))).toEqual({ type: 'approve' });
  });

  it('ignores modified keys so browser shortcuts keep working', () => {
    expect(tableAction(k('a', { ctrlKey: true }))).toBeNull();
    expect(tableAction(k('1', { metaKey: true }))).toBeNull();
  });
});

describe('dialog keys', () => {
  it('closes, approves-and-advances and pages even from a field', () => {
    expect(dialogAction(k('Escape', { inField: true }))).toEqual({ type: 'close' });
    expect(dialogAction(k('Enter', { ctrlKey: true, inField: true }))).toEqual({ type: 'approveNext' });
    expect(dialogAction(k('Enter', { metaKey: true }))).toEqual({ type: 'approveNext' });
    expect(dialogAction(k('PageDown', { inField: true }))).toEqual({ type: 'move', delta: 1 });
    expect(dialogAction(k('PageUp'))).toEqual({ type: 'move', delta: -1 });
  });

  it('uses letter shortcuts only outside fields', () => {
    expect(dialogAction(k('g'))).toEqual({ type: 'approve' });
    expect(dialogAction(k('g', { inField: true }))).toBeNull();
    expect(dialogAction(k('2'))).toEqual({ type: 'evidence', tab: 'pano' });
    expect(dialogAction(k('3'))).toEqual({ type: 'evidence', tab: 'foto' });
    expect(dialogAction(k('Enter'))).toBeNull();
  });

  it('orders the evidence tabs as the 1–5 keys: Plan · 360° · Foto · Punktsky · Rum', () => {
    expect([...EVIDENCE_TABS]).toEqual(['plan', 'pano', 'foto', 'punktsky', 'rum']);
    expect(EVIDENCE_LAST_KEY).toBe('5');
    expect(tableAction(k('1'))).toEqual({ type: 'evidence', tab: 'plan' });
    expect(tableAction(k('5'))).toEqual({ type: 'evidence', tab: 'rum' });
    expect(tableAction(k('6'))).toBeNull();
    expect(dialogAction(k('5', { inField: true }))).toBeNull();
  });
});
