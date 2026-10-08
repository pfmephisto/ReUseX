// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import type { CaseSummary } from '../api/types';
import {
  cardDate,
  cardFileLine,
  cardSubline,
  cardTitle,
  caseStats,
  caseStatus,
  caseWriteErrorText,
  SERVE_DIRECTORY_COMMAND,
  sortCases,
  uploadPercent,
} from '../sager/model';
import * as sager from '../sager/model';
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

describe('commands (R1)', () => {
  it('says how to serve more than one case', () => {
    expect(SERVE_DIRECTORY_COMMAND).toBe('ruxd --local <mappe>');
    expect('OPEN_ANOTHER_COMMAND' in sager).toBe(false);
  });

  it('sager no longer exports the phone command (On-site moved to the mobile app)', () => {
    expect('phoneCommand' in sager).toBe(false);
    expect('NO_AUTH_WARNING' in sager).toBe(false);
  });
});

function summary(over: Partial<CaseSummary> = {}): CaseSummary {
  return {
    id: 'kontor',
    name: 'Kontor',
    file_name: 'kontor.rux',
    created_at: '2026-10-08T10:00:00Z',
    archived: false,
    size_bytes: 1_234_567,
    deletable: true,
    open: false,
    ...over,
  };
}

describe('case list cards (S2)', () => {
  it('titles a card by the building record, else the case name', () => {
    expect(cardTitle(summary(), { id: 'p', name: '  Rådhuset ' })).toBe('Rådhuset');
    expect(cardTitle(summary(), { id: 'p', name: ' ' })).toBe('Kontor');
    expect(cardTitle(summary(), undefined)).toBe('Kontor');
  });

  it('names the file and its size, never a path', () => {
    expect(cardFileLine(summary())).toBe('kontor.rux · 1,2 MB');
  });

  it('lists active cases first, each group by Danish name order', () => {
    const sorted = sortCases([
      summary({ id: 'z', name: 'Ærø', archived: false }),
      summary({ id: 'a', name: 'Arkiv', archived: true }),
      summary({ id: 'b', name: 'Bygning', archived: false }),
    ]);
    expect(sorted.map((c) => c.id)).toEqual(['b', 'z', 'a']);
  });

  it('shows upload progress as a whole percent', () => {
    expect(uploadPercent(0, 0)).toBe('0 %');
    expect(uploadPercent(1, 3)).toBe('33 %');
    expect(uploadPercent(5, 5)).toBe('100 %');
  });

  it('explains a failed create or upload in Danish', () => {
    const err = (status: number) => new ApiRequestError(status, 'x', '/api/v1/uploads');
    expect(caseWriteErrorText(err(413), 'upload')).toMatch(/større/);
    expect(caseWriteErrorText(err(422), 'upload')).toMatch(/ikke et ReUseX-projekt/);
    expect(caseWriteErrorText(err(429), 'upload')).toMatch(/for mange uploads/);
    expect(caseWriteErrorText(err(409), 'create')).toMatch(/kan ikke oprette/);
    expect(caseWriteErrorText(err(409), 'upload')).toMatch(/kan ikke modtage/);
    expect(caseWriteErrorText(new Error('net'), 'upload')).toMatch(/Upload mislykkedes/);
    expect(caseWriteErrorText(new DOMException('a', 'AbortError'), 'upload')).toBe('Upload afbrudt.');
  });
});
