// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import {
  draftNotice,
  formatBytesDa,
  generatedToast,
  generateErrorMessage,
  LIST_REFRESH_FAILED,
  parseServerTime,
  REPORT_FOOTNOTE,
  reportHeroSub,
  UNKNOWN_STATUS,
  versionDate,
  versionDateTime,
  versionStatus,
  versionTitle,
} from '../rapport/model';
import { reportVersion, surveyFractions, surveySummary } from './surveyFixtures';

describe('server time', () => {
  it('reads sqlite datetime as UTC, and ISO as given', () => {
    expect(parseServerTime('2026-08-09 10:05:00')?.toISOString()).toBe('2026-08-09T10:05:00.000Z');
    expect(parseServerTime('2026-08-09T10:05:00Z')?.toISOString()).toBe('2026-08-09T10:05:00.000Z');
    expect(parseServerTime('igår')).toBeNull();
  });

  it('formats the version date like the prototype', () => {
    expect(versionDate('2026-08-09 10:05:00')).toBe('09.08.2026');
    expect(versionDate('igår')).toBe('igår');
  });
});

describe('version rows', () => {
  it('sizes in Danish units', () => {
    expect(formatBytesDa(15)).toBe('15 B');
    expect(formatBytesDa(86016)).toBe('84 KB');
    expect(formatBytesDa(2516582)).toBe('2,4 MB');
  });

  it('titles a version with its ordinal', () => {
    expect(versionTitle(reportVersion({ version: 3 }))).toBe('Ressourcekortlægning — v3');
    expect(versionTitle(reportVersion({ label: '', version: 1 }))).toBe('Ressourcekortlægning — v1');
  });

  it('marks complete, draft, and unknown', () => {
    expect(versionStatus(reportVersion({ blocking_types: 0 }))).toEqual({
      tone: 'good',
      text: 'Komplet',
      title: 'Alle typer var godkendt og afklaret, da versionen blev genereret.',
    });
    expect(versionStatus(reportVersion({ blocking_types: 7 }))).toEqual({
      tone: 'wait',
      text: 'Udkast',
      title: '7 typer var ikke godkendt eller afventede prøvesvar.',
    });
    expect(versionStatus(reportVersion({ blocking_types: 1 }))?.title).toBe(
      '1 type var ikke godkendt eller afventede prøvesvar.',
    );
    expect(versionStatus(reportVersion({ blocking_types: null }))).toBeNull();
  });
});

describe('hero and notices', () => {
  it('summarises the case like the prototype', () => {
    expect(reportHeroSub(surveySummary())).toBe('11 komponenter · 54 % bevaring/genbrug · 1 forurenet · 2 prøver afventer');
    expect(reportHeroSub(surveySummary({ reuse_share: null, pending_samples: 1 }))).toBe(
      '11 komponenter · — bevaring/genbrug · 1 forurenet · 1 prøve afventer',
    );
  });

  it('warns that a new version will be a draft while types block', () => {
    expect(draftNotice(surveyFractions())).toBe(
      '7 typer er ikke godkendt eller afventer prøvesvar — en ny version bliver et udkast, og de indgår ikke i mængderne.',
    );
    expect(draftNotice(surveyFractions({ ready: true, blocking: [], blocking_types: 0 }))).toBeNull();
  });

  it('confirms a generation and says why one failed', () => {
    expect(generatedToast(reportVersion({ version: 3, blocking_types: 0 }))).toBe('Ressourcekortlægning — v3 genereret');
    expect(generatedToast(reportVersion({ version: 1, blocking_types: 7 }))).toBe('Ressourcekortlægning — v1 genereret (udkast)');
    expect(generateErrorMessage(new ApiRequestError(409, 'job', '/r'))).toBe(
      'Kunne ikke generere — et pipeline-job kører. Prøv igen om lidt.',
    );
    expect(generateErrorMessage(new ApiRequestError(503, 'busy', '/r'))).toBe(
      'Kunne ikke generere — projektet skrives til lige nu. Prøv igen om lidt.',
    );
    expect(generateErrorMessage(new ApiRequestError(500, 'PDF generation failed: typst not found', '/r'))).toBe(
      'Kunne ikke generere rapporten: PDF generation failed: typst not found',
    );
  });

  it('states the report rules without the MRK signature', () => {
    expect(REPORT_FOOTNOTE).toBe(
      'Kun godkendte mængder indgår i rapportens kortlægningsafsnit. Versioner er uforanderlige — en ny generering giver en ny version med tidsstempel. Inventarlisten er en aktuel eksport og gemmes ikke som version.',
    );
  });
});

describe('beyond the brief', () => {
  it('gives the full local time for a tooltip, in the zone asked for', () => {
    expect(versionDateTime('2026-08-09 10:05:00', 'Europe/Copenhagen')).toBe('09.08.2026 kl. 12.05');
    expect(versionDateTime('2026-01-15 23:30:00', 'Europe/Copenhagen')).toBe('16.01.2026 kl. 00.30');
    expect(versionDateTime('igår')).toBe('igår');
  });

  it('reads an ISO time only with an explicit zone', () => {
    expect(parseServerTime('2026-08-09T12:05:00+02:00')?.toISOString()).toBe('2026-08-09T10:05:00.000Z');
    expect(parseServerTime('2026-08-09T10:05:00')).toBeNull();
    expect(parseServerTime('2026-08-09 10:05')?.toISOString()).toBe('2026-08-09T10:05:00.000Z');
  });

  it('words a version with no recorded status neutrally, never as Komplet', () => {
    expect(UNKNOWN_STATUS.text).toBe('Ukendt status');
    expect(UNKNOWN_STATUS.text).not.toMatch(/Komplet|Udkast/);
  });

  it('keeps a stale list apart from a failed generation', () => {
    expect(LIST_REFRESH_FAILED).toBe('Rapporten blev genereret, men listen kunne ikke opdateres.');
    expect(LIST_REFRESH_FAILED).not.toMatch(/Kunne ikke generere/);
  });
});
