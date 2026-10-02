// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import {
  cardDate,
  cardSubline,
  caseStats,
  caseStatus,
  NO_AUTH_WARNING,
  OPEN_ANOTHER_COMMAND,
  phoneCommand,
} from '../sager/model';
import { surveyFractions, surveySummary } from './surveyFixtures';

describe('case status (R3)', () => {
  it('is Kladde with no survey types at all', () => {
    const empty = surveySummary({ counts: { queue: 0, approved: 0, rejected: 0, all: 0 } });
    expect(caseStatus(empty, surveyFractions())).toEqual({ label: 'Kladde', tone: 'wait' });
  });

  it('is not Kladde when only rejected types exist', () => {
    const rejected = surveySummary({ counts: { queue: 0, approved: 0, rejected: 2, all: 0 } });
    const f = surveyFractions({ fractions: [], blocking: [], blocking_types: 0, ready: false });
    expect(caseStatus(rejected, f)).toEqual({ label: 'Gennemgået', tone: 'good' });
  });

  it('is Gennemgang while anything is queued or blocks', () => {
    expect(caseStatus(surveySummary(), surveyFractions())).toEqual({ label: 'Gennemgang', tone: 'accent' });
    const reviewed = surveySummary({ counts: { queue: 0, approved: 11, rejected: 0, all: 11 } });
    expect(caseStatus(reviewed, surveyFractions({ blocking_types: 2 })).label).toBe('Gennemgang');
  });

  it('never claims done while the fractions are unknown', () => {
    const reviewed = surveySummary({ counts: { queue: 0, approved: 11, rejected: 0, all: 11 } });
    expect(caseStatus(reviewed, undefined).label).toBe('Gennemgang');
    expect(caseStatus(reviewed, null).label).toBe('Gennemgang');
  });

  it('is ready when Indberetning is', () => {
    const reviewed = surveySummary({ counts: { queue: 0, approved: 11, rejected: 0, all: 11 } });
    const ready = surveyFractions({ blocking: [], blocking_types: 0, ready: true });
    expect(caseStatus(reviewed, ready)).toEqual({ label: 'Klar til indberetning', tone: 'good' });
  });
});

describe('card stats', () => {
  it('reads the summary: components, reuse share, queue', () => {
    expect(caseStats(surveySummary())).toEqual([
      { key: 'types', value: '11', label: 'komponenter' },
      { key: 'reuse', value: '54 %', label: 'bevaring/genbrug' },
      { key: 'queue', value: '7', label: 'til gennemsyn' },
    ]);
  });

  it('shows a dash for an unknown reuse share and nothing for an empty survey', () => {
    expect(caseStats(surveySummary({ reuse_share: null }))?.[1].value).toBe('—');
    expect(caseStats(surveySummary({ counts: { queue: 0, approved: 0, rejected: 0, all: 0 } }))).toBeNull();
  });

  it('is not null when every type is rejected (F7): the stats must not contradict Gennemgået', () => {
    const rejected = surveySummary({ counts: { queue: 0, approved: 0, rejected: 2, all: 0 } });
    expect(caseStats(rejected)).toEqual([
      { key: 'types', value: '0', label: 'komponenter' },
      { key: 'reuse', value: '54 %', label: 'bevaring/genbrug' },
      { key: 'queue', value: '0', label: 'til gennemsyn' },
    ]);
  });
});

describe('card text', () => {
  it('joins address and organisation, else says none is registered', () => {
    const p = { id: 'p', name: 'Måløv Byvej 229' };
    expect(cardSubline({ ...p, building_address: 'Måløv Byvej 229, 2760 Måløv', survey_organisation: 'Link Arkitektur' })).toBe(
      'Måløv Byvej 229, 2760 Måløv · udarbejdet af Link Arkitektur',
    );
    expect(cardSubline({ ...p, building_address: '  ' })).toBe('Ingen adresse registreret');
    expect(cardSubline(undefined)).toBe('Ingen adresse registreret');
  });

  it('dates the card by the registration date, in Danish form', () => {
    expect(cardDate({ id: 'p', name: 'x', survey_date: '2026-08-09' })).toBe('Registreret 09.08.2026');
    expect(cardDate({ id: 'p', name: 'x' })).toBe('—');
  });
});

describe('commands (R1, R10)', () => {
  it('says how to open another case and how to reach this one from a phone', () => {
    expect(OPEN_ANOTHER_COMMAND).toBe('rux -p <fil>.rux gui');
    expect(phoneCommand('maaloev.rux', '8426')).toBe(
      'rux -p maaloev.rux gui --bind <din-ip> --allow-origin http://<din-ip>:8426',
    );
    expect(phoneCommand('maaloev.rux', '')).toBe(
      'rux -p maaloev.rux gui --bind <din-ip> --allow-origin http://<din-ip>:8420',
    );
    expect(NO_AUTH_WARNING).toMatch(/ingen adgangskontrol/);
  });
});
