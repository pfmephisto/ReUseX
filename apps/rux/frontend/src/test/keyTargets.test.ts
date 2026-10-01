// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { isControl, isField, targetKind } from '../app/keyTargets';

const t = (tagName: string, type?: string) => ({ tagName, type }) as unknown as EventTarget;

describe('key targets', () => {
  it('classifies typing, choosing and activating targets', () => {
    expect(targetKind({ tagName: 'INPUT', type: 'search' })).toBe('text');
    expect(targetKind({ tagName: 'INPUT' })).toBe('text');
    expect(targetKind({ tagName: 'TEXTAREA' })).toBe('text');
    expect(targetKind({ tagName: 'input', type: 'checkbox' })).toBe('choice');
    expect(targetKind({ tagName: 'SELECT' })).toBe('choice');
    expect(targetKind({ tagName: 'BUTTON' })).toBe('control');
    expect(targetKind({ tagName: 'A' })).toBe('control');
    expect(targetKind({ tagName: 'INPUT', type: 'submit' })).toBe('control');
    expect(targetKind({ tagName: 'DIV' })).toBe('other');
    expect(targetKind(null)).toBe('other');
  });

  it('keeps the Kortlægning meaning of isField / isControl', () => {
    expect(isField(t('INPUT', 'text'))).toBe(true);
    expect(isField(t('SELECT'))).toBe(true);
    expect(isField(t('TEXTAREA'))).toBe(true);
    expect(isField(t('BUTTON'))).toBe(false);
    expect(isControl(t('BUTTON'))).toBe(true);
    expect(isControl(t('A'))).toBe(true);
    expect(isControl(t('DIV'))).toBe(false);
    expect(isField(null)).toBe(false);
  });
});
