// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { emptyRowText, TABS, typeRowClick } from '../components/kortlaegning/SurveyTable';

describe('typeRowClick', () => {
  it('selects an unselected row on the first click', () => {
    expect(typeRowClick(false, 1)).toBe('select');
  });

  it('folds the selected row on a single click', () => {
    expect(typeRowClick(true, 1)).toBe('toggle');
  });

  it('does nothing on a double-click\'s second click, selected or not', () => {
    expect(typeRowClick(true, 2)).toBe('ignore');
    expect(typeRowClick(false, 2)).toBe('ignore');
  });
});

describe('tabs (spec A3)', () => {
  it('has a fourth tab Afvist after Alle', () => {
    expect(TABS.map((t) => t.label)).toEqual(['Til gennemsyn', 'Godkendt', 'Alle', 'Afvist']);
    expect(TABS[3].id).toBe('rejected');
  });

  it('explains an empty Afvist tab instead of blaming the filters', () => {
    expect(emptyRowText('rejected', 0)).toBe('Ingen afviste typer.');
    expect(emptyRowText('rejected', 2)).toBe('Ingen rækker matcher filtrene.');
    expect(emptyRowText('queue', 0)).toBe('Ingen rækker matcher filtrene.');
  });
});
