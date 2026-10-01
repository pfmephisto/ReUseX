// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { ProjectInfo, ProjectSummary } from '../api/types';
import {
  caseName,
  CIRC_EMPTY_TEXT,
  circularityAriaLabel,
  circularitySegments,
  heroSubline,
  INVALID_YEAR_TOAST,
  kpis,
  metaPatch,
  percentText,
  quickLinks,
  versionsText,
  wholePercents,
  yearCommit,
} from '../overblik/model';
import { reportVersion, surveySummary } from './surveyFixtures';

function projectSummary(projects: ProjectInfo[], path = 'maaloev.rux'): ProjectSummary {
  return {
    path,
    schema_version: 23,
    projects,
    clouds: [],
    meshes: [],
    sensor_frames: { total_count: 0, segmented_count: 0 },
    panoramic_images: { total_count: 0, matched_count: 0 },
    components: { total_count: 0, count_by_type: {} },
    materials: [],
  };
}

describe('whole percents', () => {
  it('reproduces the prototype legend on the demo tonnes', () => {
    expect(wholePercents([640, 76, 576.8, 1.6, 40.4])).toEqual([48, 6, 43, 0, 3]);
  });

  it('always sums to 100 and is all zero for no tonnes', () => {
    expect(wholePercents([1, 1, 1]).reduce((a, b) => a + b, 0)).toBe(100);
    expect(wholePercents([2, 1])).toEqual([67, 33]);
    expect(wholePercents([0, 0])).toEqual([0, 0]);
  });
});

describe('whole percents, edge cases (R4)', () => {
  const sum = (xs: number[]) => xs.reduce((a, b) => a + b, 0);

  it('reproduces 48/6/43/0/3 through the segments on the demo seed', () => {
    const segs = circularitySegments(surveySummary().circularity);
    expect(segs.map((s) => [s.label, s.percent])).toEqual([
      ['Bevaring', 48],
      ['Genbrug', 6],
      ['Genanvendelse', 43],
      ['Nyttiggørelse', 0],
      ['Bortskaffelse', 3],
    ]);
    expect(sum(segs.map((s) => s.percent))).toBe(100);
  });

  it('never yields NaN when everything is zero', () => {
    const out = wholePercents([0, 0, 0, 0, 0]);
    expect(out).toEqual([0, 0, 0, 0, 0]);
    expect(out.some((x) => Number.isNaN(x))).toBe(false);
    expect(wholePercents([])).toEqual([]);
  });

  it('gives a single class all of it', () => {
    expect(wholePercents([12.5])).toEqual([100]);
    expect(wholePercents([0, 7, 0])).toEqual([0, 100, 0]);
  });

  it('breaks ties toward the earlier (higher) waste-hierarchy step', () => {
    expect(wholePercents([1, 1, 1])).toEqual([34, 33, 33]);
    expect(wholePercents([1, 1, 1, 1, 1, 1, 1])).toEqual([15, 15, 14, 14, 14, 14, 14]);
    expect(wholePercents([1, 1])).toEqual([50, 50]);
  });

  it('sums to 100 on awkward splits', () => {
    for (const xs of [[1, 2, 3], [0.1, 0.2, 99.7], [1e-9, 1, 1], [3, 3, 3, 1]]) {
      expect(sum(wholePercents(xs))).toBe(100);
    }
  });
});

describe('circularity bar wording', () => {
  it('names every step and its percent', () => {
    const segs = circularitySegments({ bevaring: 0, genbrug: 10, genanvendelse: 30, nyttiggoerelse: 0, bortskaffelse: 0 });
    expect(circularityAriaLabel(segs)).toBe(
      'Fordeling af materialemængde på affaldshierarkiet: Genbrug 25 %, Genanvendelse 75 %',
    );
  });

  it('says there is nothing yet when no step has tonnes', () => {
    expect(CIRC_EMPTY_TEXT).toBe('Ingen mængder registreret endnu');
    expect(circularityAriaLabel([])).toBe(
      'Fordeling af materialemængde på affaldshierarkiet: ingen mængder registreret endnu',
    );
  });
});

describe('circularity segments', () => {
  it('keeps waste-hierarchy order and only steps with tonnes', () => {
    const segs = circularitySegments({ bevaring: 0, genbrug: 10, genanvendelse: 30, nyttiggoerelse: 0, bortskaffelse: 0 });
    expect(segs.map((s) => [s.treatment, s.label, s.tonnes, s.percent])).toEqual([
      ['genbrug', 'Genbrug', 10, 25],
      ['genanvendelse', 'Genanvendelse', 30, 75],
    ]);
  });

  it('is empty when nothing has tonnes', () => {
    expect(circularitySegments({ bevaring: 0, genbrug: 0, genanvendelse: 0, nyttiggoerelse: 0, bortskaffelse: 0 })).toEqual([]);
  });
});

describe('KPIs', () => {
  it('reads the prototype row from the demo summary', () => {
    expect(kpis(surveySummary())).toEqual([
      { key: 'types', value: '11', label: 'Komponenter' },
      { key: 'classified', value: '—', label: 'Klassificeret', hint: 'Kræver instansskyen' },
      { key: 'reuse', value: '54', unit: '%', label: 'Bevaring / genbrug' },
      { key: 'queue', value: '7', label: 'Til gennemsyn', ink: 'warn' },
      { key: 'samples', value: '2', label: 'Prøver afventer', ink: 'crit' },
    ]);
  });

  it('drops the action ink at zero and shows the classified share', () => {
    const k = kpis(surveySummary({ counts: { queue: 0, approved: 11, rejected: 0, all: 11 }, pending_samples: 0, classified_share: 0.724 }));
    expect(k.find((x) => x.key === 'queue')?.ink).toBeUndefined();
    expect(k.find((x) => x.key === 'samples')?.ink).toBeUndefined();
    expect(k.find((x) => x.key === 'classified')).toEqual({ key: 'classified', value: '72', unit: '%', label: 'Klassificeret' });
  });

  it('formats a missing share as a dash', () => {
    expect(percentText(null)).toBe('—');
    expect(percentText(0.536)).toBe('54');
  });
});

describe('quick links', () => {
  it('says what waits on each screen', () => {
    expect(quickLinks(surveySummary(), [reportVersion(), reportVersion({ id: 2, version: 2 }), reportVersion({ id: 3, version: 3 })])).toEqual([
      { to: '/kortlaegning', title: 'Kortlægning', sub: '7 til gennemsyn · 11 typer' },
      { to: '/miljoe', title: 'Miljø & prøver', sub: '2 prøver afventer svar' },
      { to: '/rapport', title: 'Rapport', sub: '3 versioner' },
      { to: '/indberetning', title: 'Indberetning', sub: '4 af 11 typer godkendt' },
    ]);
  });

  it('counts versions in Danish, and says when they are loading or failed', () => {
    expect(versionsText([])).toBe('Ingen versioner endnu');
    expect(versionsText([reportVersion()])).toBe('1 version');
    expect(versionsText(undefined)).toBe('Henter versioner…');
    expect(versionsText(null)).toBe('Versioner kunne ikke hentes');
    expect(quickLinks(surveySummary({ pending_samples: 1 }), [])[1].sub).toBe('1 prøve afventer svar');
  });
});

describe('case hero', () => {
  const record: ProjectInfo = {
    id: 'p1',
    name: 'Måløv Byvej 229',
    building_address: 'Måløv Byvej 229, 2760 Måløv',
    year_of_construction: 1978,
    survey_date: '2026-08-09',
    survey_organisation: 'Link Arkitektur',
  };

  it('names the case from the record, else the file', () => {
    expect(caseName(projectSummary([record]))).toBe('Måløv Byvej 229');
    expect(caseName(projectSummary([]), { ...record, name: '  ' })).toBe('maaloev');
    expect(caseName(projectSummary([record]), { ...record, name: 'Ny' })).toBe('Ny');
  });

  it('joins only the fields that are set', () => {
    expect(heroSubline(record)).toBe(
      'Måløv Byvej 229, 2760 Måløv · opført 1978 · registreret 2026-08-09 · udarbejdet af Link Arkitektur',
    );
    expect(heroSubline({ id: 'p', name: 'x', year_of_construction: 0 })).toBe('');
    expect(heroSubline(undefined)).toBe('');
  });
});

describe('metadata commits', () => {
  it('clears an emptied optional field with null, never the name', () => {
    expect(metaPatch('building_address', '')).toEqual({ building_address: null });
    expect(metaPatch('notes', 'Tag udskiftet 2004')).toEqual({ notes: 'Tag udskiftet 2004' });
    expect(metaPatch('name', 'Måløv')).toEqual({ name: 'Måløv' });
  });

  it('sends a year only when it changed and is a year', () => {
    expect(yearCommit('1978', 1978)).toEqual({ send: false, invalid: false });
    expect(yearCommit(' 1979 ', 1978)).toEqual({ send: true, value: 1979 });
    expect(yearCommit('', 1978)).toEqual({ send: true, value: null });
    expect(yearCommit('', 0)).toEqual({ send: false, invalid: false });
    expect(yearCommit('', undefined)).toEqual({ send: false, invalid: false });
    expect(yearCommit('0', 1978)).toEqual({ send: true, value: null });
    expect(yearCommit('nittenhalvfjerds', 1978)).toEqual({ send: false, invalid: true });
    expect(yearCommit('19780', undefined)).toEqual({ send: false, invalid: true });
    expect(yearCommit('978', 1978)).toEqual({ send: false, invalid: true });
    expect(yearCommit('197', undefined)).toEqual({ send: false, invalid: true });
    expect(yearCommit('20240', 1978)).toEqual({ send: false, invalid: true });
    expect(yearCommit('1978.5', 1978)).toEqual({ send: false, invalid: true });
    expect(yearCommit('0000', 1978)).toEqual({ send: true, value: null });
    expect(INVALID_YEAR_TOAST).toBe('Byggeår skal være et årstal, fx 1978.');
  });
});
