// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/** Builders for survey types and samples in the Miljø & prøver tests. */

import type { Sample, SurveyType } from '../api/types';

export function surveyType(over: Partial<SurveyType> = {}): SurveyType {
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
    semantic_class: -2,
    environment_status: 'ren_screening',
    sample_ids: [],
    quantity: 38,
    parts: [],
    created_at: '',
    updated_at: '',
    ...over,
  };
}

export function sample(over: Partial<Sample> = {}): Sample {
  return {
    id: 1,
    code: 'P-01',
    title: 'PCB i fugemasse',
    what: 'Fugemasse omkring vinduespartier',
    stage: 'sendt',
    result: null,
    type_ids: [],
    created_at: '',
    updated_at: '',
    ...over,
  };
}
