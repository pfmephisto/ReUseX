// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/** Builders for survey types, samples, summaries, fractions and report versions in the Phase 4–5 tests. */

import type {
  ReportPdfVersion,
  Sample,
  SurveyBlockingType,
  SurveyFraction,
  SurveyFractions,
  SurveySummary,
  SurveyType,
} from '../api/types';

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

/** The demo seed's summary (prototype v2 numbers). */
export function surveySummary(over: Partial<SurveySummary> = {}): SurveySummary {
  return {
    counts: { queue: 7, approved: 4, rejected: 0, all: 11 },
    circularity: { bevaring: 640, genbrug: 76, genanvendelse: 576.8, nyttiggoerelse: 1.6, bortskaffelse: 40.4 },
    total_mass_t: 1334.8,
    reuse_share: 716 / 1334.8,
    pending_samples: 2,
    contaminated_types: 1,
    unlabeled_points: null,
    classified_share: null,
    rooms_without_parts: [],
    ...over,
  };
}

export function fraction(over: Partial<SurveyFraction> = {}): SurveyFraction {
  return { eak_code: '17.01.01', name: 'Beton', treatment: 'genanvendelse', mass_t: 190, contaminated: false, ...over };
}

export function blockingType(over: Partial<SurveyBlockingType> = {}): SurveyBlockingType {
  return {
    type_id: 2,
    name: 'Betonsøjler, bærende',
    eak_code: '17.01.01',
    treatment: 'genbrug',
    mass_t: 58,
    reason: 'review',
    ...over,
  };
}

/** The demo seed's fractions: three ready rows, 199,2 t, seven blocking types. */
export function surveyFractions(over: Partial<SurveyFractions> = {}): SurveyFractions {
  const blocking = [
    blockingType(),
    blockingType({ type_id: 3, name: 'Betondæk, etagedæk', treatment: 'genanvendelse', mass_t: 380 }),
    blockingType({ type_id: 5, name: 'Stålspær, tagkonstruktion', eak_code: '17.04.05', mass_t: 14 }),
    blockingType({ type_id: 6, name: 'Vinduespartier, aluminium', eak_code: '17.04.02', mass_t: 3.1, reason: 'sample' }),
    blockingType({ type_id: 8, name: 'Gulvbelægning, linoleum', eak_code: '17.09.04', treatment: 'nyttiggoerelse', mass_t: 1.6, reason: 'sample' }),
    blockingType({ type_id: 9, name: 'Indvendige døre, træ', eak_code: '17.02.01', mass_t: 0.9 }),
    blockingType({ type_id: 11, name: 'Indvendige murvægge, malet', eak_code: '17.01.02', treatment: 'bortskaffelse', mass_t: 38 }),
  ];
  return {
    fractions: [
      fraction(),
      fraction({ eak_code: '17.04.05', name: 'Jern og stål', mass_t: 6.8 }),
      fraction({ eak_code: '17.06.04', name: 'Isoleringsmateriale', treatment: 'bortskaffelse', mass_t: 2.4 }),
    ],
    total_t: 199.2,
    blocking_types: blocking.length,
    blocking,
    ready: false,
    ...over,
  };
}

export function reportVersion(over: Partial<ReportPdfVersion> = {}): ReportPdfVersion {
  return {
    id: 1,
    created_at: '2026-08-09 10:05:00',
    label: 'Ressourcekortlægning',
    size_bytes: 2516582,
    version: 1,
    blocking_types: 0,
    ...over,
  };
}
