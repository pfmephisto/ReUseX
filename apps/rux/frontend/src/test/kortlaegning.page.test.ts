// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { ApiRequestError } from '../api/client';
import type { Sample, SurveySummary, SurveyType } from '../api/types';
import { saveErrorMessage } from '../app/saveError';
import { approvedMessage, blockedMessage, coverageParts } from '../routes/KortlaegningPage';

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
    unlabeled_points: null,
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

  it('lists unlabeled points and rooms without parts', () => {
    expect(
      coverageParts(summary({ unlabeled_points: 2400, rooms_without_parts: ['Kælder', 'Tagrum'] })),
    ).toEqual(['2.400 punkter uklassificeret', '2 rum uden registrerede bygningsdele (Kælder, Tagrum)']);
  });

  it('omits the clause whose figure is absent', () => {
    expect(coverageParts(summary({ rooms_without_parts: ['Kælder'] }))).toEqual([
      '1 rum uden registrerede bygningsdele (Kælder)',
    ]);
  });
});
