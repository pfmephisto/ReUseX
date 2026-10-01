// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { TREATMENTS } from '../api/types';
import {
  circToken,
  confidencePercent,
  ENV_LABEL,
  ENV_TONE,
  formatNumber,
  formatQuantity,
  formatTonnes,
  parseDanishNumber,
  STAGE_LABEL,
  TREATMENT_LABEL,
} from '../kortlaegning/vocab';

describe('kortlægning vocabulary', () => {
  it('labels every treatment in Danish and maps it to its circ token', () => {
    expect(TREATMENTS.map((t) => TREATMENT_LABEL[t])).toEqual([
      'Bevaring',
      'Genbrug',
      'Genanvendelse',
      'Nyttiggørelse',
      'Bortskaffelse',
    ]);
    expect(circToken('nyttiggoerelse')).toBe('var(--circ-nyttiggoerelse)');
  });

  it('labels and tones the environment statuses', () => {
    expect(ENV_LABEL.afventer).toBe('Afventer prøve');
    expect(ENV_LABEL.ren_proevesvar).toBe('Ren (prøvesvar)');
    expect(ENV_TONE).toEqual({
      ren_screening: 'good',
      afventer: 'wait',
      forurenet: 'crit',
      ren_proevesvar: 'good',
    });
    expect(STAGE_LABEL.sendt).toBe('Sendt til lab');
  });

  it('formats numbers the Danish way', () => {
    expect(formatNumber(1240)).toBe('1.240');
    expect(formatNumber(3.1)).toBe('3,1');
    expect(formatNumber(2.25, 2)).toBe('2,25');
    expect(formatQuantity(1240, 'm²')).toBe('1.240 m²');
    expect(formatTonnes(58)).toBe('58 t');
    expect(formatTonnes(null)).toBe('');
  });

  it('turns confidence into a whole percentage', () => {
    expect(confidencePercent(0.82)).toBe(82);
    expect(confidencePercent(null)).toBeNull();
  });

  it('parses what a Danish user types', () => {
    expect(parseDanishNumber('1.240')).toBe(1240);
    expect(parseDanishNumber('1.240,5')).toBe(1240.5);
    expect(parseDanishNumber('22,5')).toBe(22.5);
    expect(parseDanishNumber(' 18 ')).toBe(18);
    expect(parseDanishNumber('')).toBeNull();
    expect(parseDanishNumber('abc')).toBeNull();
    expect(parseDanishNumber('-3')).toBeNull(); // quantities are never negative
  });
});
