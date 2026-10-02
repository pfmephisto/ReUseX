// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import {
  defaultExportTemplateId,
  draftNotice,
  formatBytesDa,
  generatedToast,
  generateErrorMessage,
  HERO_SCOPE,
  LIST_REFRESH_FAILED,
  NO_TEMPLATE,
  parseTemplateChoice,
  REPORT_FOOTNOTE,
  reportHeroSub,
  ressourcetabelHint,
  UNKNOWN_STATUS,
  validChoice,
  versionDate,
  versionDateTime,
  versionStatus,
  versionTitle,
} from '../rapport/model';
import { reportVersion, surveyFractions, surveySummary } from './surveyFixtures';

describe('server time', () => {
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
      title: '7 typer var ikke godkendt, afventede prøvesvar eller manglede tons.',
    });
    expect(versionStatus(reportVersion({ blocking_types: 1 }))?.title).toBe(
      '1 type var ikke godkendt, afventede prøvesvar eller manglede tons.',
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
    // The hero is the whole survey, not the PDF's approved subset, and says so.
    expect(HERO_SCOPE).toBe('Hele kortlægningen (inkl. ikke-godkendte)');
  });

  it('warns that a new version will be a draft while types block', () => {
    expect(draftNotice(surveyFractions())).toBe(
      '7 typer afventer gennemsyn eller prøvesvar, eller mangler tons — en ny version bliver et udkast, og de indgår ikke i mængderne.',
    );
    expect(draftNotice(surveyFractions({ blocking_types: 1 }))).toBe(
      '1 type afventer gennemsyn eller prøvesvar, eller mangler tons — en ny version bliver et udkast, og de indgår ikke i mængderne.',
    );
    expect(draftNotice(surveyFractions({ ready: true, blocking: [], blocking_types: 0 }))).toBeNull();
    // An empty survey is not ready, but nothing blocks: no draft warning (a known edge).
    expect(draftNotice(surveyFractions({ ready: false, fractions: [], blocking: [], blocking_types: 0 }))).toBeNull();
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

  it('words a version with no recorded status neutrally, never as Komplet', () => {
    expect(UNKNOWN_STATUS.text).toBe('Ukendt status');
    expect(UNKNOWN_STATUS.text).not.toMatch(/Komplet|Udkast/);
  });

  it('keeps a stale list apart from a failed generation', () => {
    expect(LIST_REFRESH_FAILED).toBe('Rapporten blev genereret, men listen kunne ikke opdateres.');
    expect(LIST_REFRESH_FAILED).not.toMatch(/Kunne ikke generere/);
  });
});

describe('template choices (R10)', () => {
  const T = [
    { id: 3, seed: 'materialepas' },
    { id: 5, seed: null },
    { id: 8, seed: 'screening' },
  ];

  it('parses the select value', () => {
    expect(parseTemplateChoice(NO_TEMPLATE)).toBeNull();
    expect(parseTemplateChoice('8')).toBe(8);
    expect(parseTemplateChoice('x')).toBeNull();
  });

  it('drops a choice whose template is gone', () => {
    expect(validChoice(T, 5)).toBe(5);
    expect(validChoice(T, 99)).toBeNull();
    expect(validChoice(T, null)).toBeNull();
  });

  it('defaults the export to the screening seed, then the first template', () => {
    expect(defaultExportTemplateId(T)).toBe(8);
    expect(defaultExportTemplateId([{ id: 5, seed: null }])).toBe(5);
    expect(defaultExportTemplateId([])).toBeNull();
  });

  it('says what the Ressourcetabel will hold', () => {
    expect(ressourcetabelHint(null)).toBe('Rapporten genereres uden ressourcetabel.');
    expect(ressourcetabelHint({ name: 'Hurtig genbrugsscreening', resolved_keys: ['a', 'b'] })).toBe(
      'Ressourcetabel med 2 felter fra "Hurtig genbrugsscreening".',
    );
    expect(ressourcetabelHint({ name: 'Tom', resolved_keys: [] })).toBe(
      'Skabelonen "Tom" har ingen felter — tabellen bliver tom.',
    );
  });
});
