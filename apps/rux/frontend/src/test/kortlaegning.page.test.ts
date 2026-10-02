// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import type { Sample, SurveySummary, SurveySyncReport, SurveyType } from '../api/types';
import { saveErrorMessage } from '../app/saveError';
import {
  approvedMessage,
  blockedMessage,
  cappedList,
  coverageParts,
  syncMessage,
} from '../routes/KortlaegningPage';

function type(overrides: Partial<SurveyType> = {}): SurveyType {
  return {
    id: 1,
    name: 'Vinduespartier, aluminium',
    eak_code: '17.04.02',
    eak_name: 'Aluminium',
    bim7aa_code: '312 Udv. vinduer',
    unit: 'stk',
    treatment: 'genbrug',
    review_status: 'queue',
    confidence: 0.82,
    mass_t: 3.1,
    note: '',
    starred: false,
    semantic_class: 1,
    environment_status: 'afventer',
    sample_ids: [],
    quantity: 38,
    parts: [],
    created_at: '',
    updated_at: '',
    ...overrides,
  };
}

function sample(overrides: Partial<Sample> = {}): Sample {
  return {
    id: 1,
    code: 'P-01',
    title: 'PCB i fugemasse',
    what: '',
    stage: 'sendt',
    result: null,
    type_ids: [1],
    part_code: null,
    created_at: '',
    updated_at: '',
    ...overrides,
  };
}

function summary(overrides: Partial<SurveySummary> = {}): SurveySummary {
  return {
    counts: { queue: 0, approved: 0, rejected: 0, all: 0 },
    circularity: { bevaring: 0, genbrug: 0, genanvendelse: 0, nyttiggoerelse: 0, bortskaffelse: 0 },
    total_mass_t: 0,
    reuse_share: null,
    pending_samples: 0,
    contaminated_types: 0,
    unlabeled_points: null,
    classified_share: null,
    rooms_without_parts: [],
    ...overrides,
  };
}

describe('blockedMessage', () => {
  it('names the pending linked samples like the gate note does', () => {
    const samples = [
      sample(),
      sample({ id: 2, code: 'P-02', stage: 'svar' }),
      sample({ id: 3, code: 'P-03', title: 'Asbest i lim' }),
    ];
    expect(blockedMessage(type({ sample_ids: [1, 2, 3] }), samples)).toBe(
      'Kan ikke godkendes — afventer prøvesvar (P-01 · PCB i fugemasse, P-03 · Asbest i lim)',
    );
  });

  it('drops the parenthesis when no linked sample is pending', () => {
    const answered = sample({ stage: 'svar', result: 'ren' });
    expect(blockedMessage(type({ sample_ids: [1] }), [answered])).toBe('Kan ikke godkendes — afventer prøvesvar');
  });

  it('drops the parenthesis when no sample is linked', () => {
    expect(blockedMessage(type(), [sample()])).toBe('Kan ikke godkendes — afventer prøvesvar');
  });
});

describe('saveErrorMessage', () => {
  it('explains a running pipeline job (409) in Danish', () => {
    expect(saveErrorMessage(new ApiRequestError(409, 'job running', '/survey/types/1'))).toBe(
      'Kunne ikke gemme — et pipeline-job kører. Prøv igen om lidt.',
    );
  });

  it('explains a server that is not ready (503) in Danish', () => {
    expect(saveErrorMessage(new ApiRequestError(503, 'database locked', '/survey/types/1'))).toBe(
      'Kunne ikke gemme — serveren er ikke klar.',
    );
  });

  it('shows the message for any other failure', () => {
    expect(saveErrorMessage(new ApiRequestError(500, 'boom', '/survey/types/1'))).toBe('Kunne ikke gemme: boom');
    expect(saveErrorMessage(new TypeError('Failed to fetch'))).toBe('Kunne ikke gemme: Failed to fetch');
  });
});

describe('approvedMessage', () => {
  it('counts what is left in the queue', () => {
    const types = [type({ id: 1, review_status: 'approved' }), type({ id: 2 }), type({ id: 3 })];
    expect(approvedMessage('Betonsøjler, bærende', types)).toBe(
      '✓ Betonsøjler, bærende godkendt · 2 tilbage i køen',
    );
  });
});

describe('coverageParts', () => {
  it('is empty when nothing is uncovered', () => {
    expect(coverageParts(summary({ unlabeled_points: 0 }))).toEqual([]);
  });

  const surveyed = { queue: 3, approved: 0, rejected: 0, all: 3 };

  it('lists unlabeled points and rooms without parts', () => {
    expect(
      coverageParts(
        summary({ counts: surveyed, unlabeled_points: 2400, rooms_without_parts: ['Kælder', 'Tagrum'] }),
      ),
    ).toEqual(['2.400 punkter uklassificeret', '2 rum uden registrerede bygningsdele (Kælder, Tagrum)']);
  });

  it('omits the clause whose figure is absent', () => {
    expect(coverageParts(summary({ counts: surveyed, rooms_without_parts: ['Kælder'] }))).toEqual([
      '1 rum uden registrerede bygningsdele (Kælder)',
    ]);
  });

  it('caps the room list at five names', () => {
    const rooms = Array.from({ length: 454 }, (_, i) => `Rum ${i + 1}`);
    expect(coverageParts(summary({ counts: surveyed, rooms_without_parts: rooms }))).toEqual([
      '454 rum uden registrerede bygningsdele (Rum 1, Rum 2, Rum 3, Rum 4, Rum 5 … og 449 flere)',
    ]);
  });

  it('leaves rooms out before the first sync, when every room lacks parts', () => {
    expect(
      coverageParts(summary({ unlabeled_points: 140698, rooms_without_parts: ['Rum 1', 'Rum 2'] })),
    ).toEqual(['140.698 punkter uklassificeret']);
  });
});

describe('cappedList', () => {
  it('joins short lists whole', () => {
    expect(cappedList(['A', 'B', 'C', 'D', 'E'])).toBe('A, B, C, D, E');
  });

  it('names the first five and counts the rest', () => {
    expect(cappedList(['A', 'B', 'C', 'D', 'E', 'F'])).toBe('A, B, C, D, E … og 1 flere');
    expect(cappedList(['A', 'B', 'C'], 2)).toBe('A, B … og 1 flere');
  });
});

describe('syncMessage', () => {
  function report(overrides: Partial<SurveySyncReport> = {}): SurveySyncReport {
    return {
      types_created: 0,
      parts_created: 0,
      parts_existing: 0,
      instances_seen: 0,
      instances_backfilled: 0,
      rooms_assigned: true,
      parts_orphaned: 0,
      orphaned_codes: [],
      ...overrides,
    };
  }

  it('counts the parts and types it created', () => {
    expect(syncMessage(report({ types_created: 10, parts_created: 155, instances_seen: 155 }))).toBe(
      '155 bygningsdele oprettet i 10 nye typer',
    );
    expect(syncMessage(report({ types_created: 1, parts_created: 1, instances_seen: 1 }))).toBe(
      '1 bygningsdel oprettet i 1 ny type',
    );
  });

  it('reports parts added to existing types, not "no instances"', () => {
    expect(syncMessage(report({ parts_created: 4, parts_existing: 151, instances_seen: 155 }))).toBe(
      '4 bygningsdele tilføjet til eksisterende typer',
    );
  });

  it('says every instance is already surveyed when nothing is new', () => {
    expect(syncMessage(report({ parts_existing: 155, instances_seen: 155 }))).toBe(
      'Ingen nye bygningsdele — alle 155 instanser er allerede kortlagt',
    );
  });

  it('says there are no instances only when there are none', () => {
    expect(syncMessage(report())).toBe('Ingen instanser at kortlægge — opret instanser først');
  });

  it('puts orphaned parts first', () => {
    expect(syncMessage(report({ parts_orphaned: 2, parts_created: 3, instances_seen: 3 }))).toBe(
      '2 del(e) peger på instanser der ikke findes længere',
    );
  });
});
