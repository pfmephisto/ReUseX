// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { Sample, SurveyPart, SurveyType } from '../api/types';
import {
  approveBlocked,
  gateNoteText,
  linkedSamples,
  panelTitle,
  quantityCommitValue,
  sampleLineText,
} from '../components/kortlaegning/DetailPanel';

function part(overrides: Partial<SurveyPart> = {}): SurveyPart {
  return {
    code: 'RX-008',
    type_id: 1,
    cloud: 'instances',
    instance_id: 7,
    room_id: 1,
    room_name: 'Office Zone',
    quantity: 26,
    starred: false,
    note: '',
    material_guid: null,
    instance_guid: 'guid-1',
    orphaned: false,
    ...overrides,
  };
}

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
    environment_status: 'ren_screening',
    sample_ids: [],
    quantity: 38,
    parts: [part()],
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

describe('panelTitle', () => {
  it('is just the type name with no part selected', () => {
    expect(panelTitle(type(), null)).toBe('Vinduespartier, aluminium');
  });

  it('prefixes the part code with no space before the ·', () => {
    expect(panelTitle(type(), part())).toBe('RX-008 · Vinduespartier, aluminium');
  });
});

describe('linkedSamples', () => {
  it('returns the samples named by sample_ids, in that order', () => {
    const s1 = sample({ id: 1, code: 'P-01' });
    const s2 = sample({ id: 2, code: 'P-02' });
    const t = type({ sample_ids: [2, 1] });
    expect(linkedSamples(t, [s1, s2]).map((s) => s.code)).toEqual(['P-02', 'P-01']);
  });

  it('drops ids that have no matching sample', () => {
    const t = type({ sample_ids: [99] });
    expect(linkedSamples(t, [sample()])).toEqual([]);
  });

  it('is empty when sample_ids is empty', () => {
    expect(linkedSamples(type({ sample_ids: [] }), [sample()])).toEqual([]);
  });
});

describe('sampleLineText', () => {
  it('falls back to the screening-only message with no linked sample', () => {
    expect(sampleLineText(type({ sample_ids: [] }), [])).toBe(
      'Ingen prøve koblet — miljøstatus fra screening: ren.',
    );
  });

  it('names the sample, its title and stage when one is linked, no result yet', () => {
    const t = type({ sample_ids: [1] });
    const line = sampleLineText(t, [sample({ stage: 'sendt', result: null })]);
    expect(line).toBe('Miljøstatus styres af P-01 · PCB i fugemasse — Sendt til lab');
  });

  it('appends the result once the sample has one', () => {
    const t = type({ sample_ids: [1] });
    const line = sampleLineText(t, [sample({ stage: 'svar', result: 'ren' })]);
    expect(line).toBe('Miljøstatus styres af P-01 · PCB i fugemasse — Svar modtaget · ren');
  });

  it('joins multiple linked samples', () => {
    const t = type({ sample_ids: [1, 2] });
    const samples = [
      sample({ id: 1, code: 'P-01', stage: 'sendt' }),
      sample({ id: 2, code: 'P-02', title: 'Asbest i loft', stage: 'planlagt' }),
    ];
    expect(sampleLineText(t, samples)).toBe(
      'Miljøstatus styres af P-01 · PCB i fugemasse — Sendt til lab, P-02 · Asbest i loft — Planlagt',
    );
  });
});

describe('approveBlocked', () => {
  it('blocks only on afventer', () => {
    expect(approveBlocked(type({ environment_status: 'afventer' }))).toBe(true);
    expect(approveBlocked(type({ environment_status: 'ren_screening' }))).toBe(false);
    expect(approveBlocked(type({ environment_status: 'ren_proevesvar' }))).toBe(false);
    expect(approveBlocked(type({ environment_status: 'forurenet' }))).toBe(false);
  });
});

describe('gateNoteText', () => {
  it('is null when not blocked', () => {
    expect(gateNoteText(type({ environment_status: 'ren_screening' }), [])).toBeNull();
  });

  it('names the pending sample as code · title when blocked', () => {
    const t = type({ environment_status: 'afventer', sample_ids: [1] });
    expect(gateNoteText(t, [sample({ code: 'P-01', title: 'PCB i fugemasse', stage: 'sendt' })])).toBe(
      'Kan ikke godkendes endnu — afventer prøvesvar (P-01 · PCB i fugemasse).',
    );
  });

  it('joins multiple pending samples as code · title, comma-separated', () => {
    const t = type({ environment_status: 'afventer', sample_ids: [1, 2] });
    const samples = [
      sample({ id: 1, code: 'P-01', title: 'PCB i fugemasse', stage: 'sendt' }),
      sample({ id: 2, code: 'P-02', title: 'Asbest i loft', stage: 'planlagt' }),
    ];
    expect(gateNoteText(t, samples)).toBe(
      'Kan ikke godkendes endnu — afventer prøvesvar (P-01 · PCB i fugemasse, P-02 · Asbest i loft).',
    );
  });

  it('excludes a linked sample that already has svar, naming only the pending one', () => {
    const t = type({ environment_status: 'afventer', sample_ids: [1, 2] });
    const samples = [
      sample({ id: 1, code: 'P-01', title: 'PCB i fugemasse', stage: 'svar', result: 'ren' }),
      sample({ id: 2, code: 'P-02', title: 'Asbest i loft', stage: 'sendt' }),
    ];
    expect(gateNoteText(t, samples)).toBe(
      'Kan ikke godkendes endnu — afventer prøvesvar (P-02 · Asbest i loft).',
    );
  });

  it('drops the parenthetical when no linked sample resolves as pending', () => {
    const t = type({ environment_status: 'afventer', sample_ids: [] });
    expect(gateNoteText(t, [])).toBe('Kan ikke godkendes endnu — afventer prøvesvar.');
  });
});

describe('quantityCommitValue', () => {
  it('returns the parsed value when it differs from current', () => {
    expect(quantityCommitValue('40', 38)).toBe(40);
  });

  it('parses Danish grouping and decimal comma', () => {
    expect(quantityCommitValue('1.240,5', 1000)).toBe(1240.5);
  });

  it('returns null when the text does not parse', () => {
    expect(quantityCommitValue('abc', 38)).toBeNull();
    expect(quantityCommitValue('', 38)).toBeNull();
  });

  it('returns null when the parsed value equals current (no-op commit)', () => {
    expect(quantityCommitValue('38', 38)).toBeNull();
  });

  it('treats 0 as a valid, distinct value', () => {
    expect(quantityCommitValue('0', 38)).toBe(0);
    expect(quantityCommitValue('0', 0)).toBeNull();
  });
});
