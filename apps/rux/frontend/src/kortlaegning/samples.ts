// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A survey type's linked samples, as pure functions: which samples a type
 * names, the sample line under the EAK/behandling fields (as data and as
 * text), and the still-pending ones the gate note and the refused-approval
 * toast name. Shared by DetailPanel, EditDialog and SampleLine; DetailPanel
 * re-exports them so its callers keep their import.
 */

import type { Sample, SurveyType } from '../api/types';
import { RESULT_LABEL, STAGE_LABEL } from './vocab';

/** The sample line for a type with no linked sample. */
export const SAMPLE_LINE_NONE = 'Ingen prøve koblet — miljøstatus fra screening: ren.';

/** What precedes the linked samples on the sample line (a space follows it). */
export const SAMPLE_LINE_LINKED_PREFIX = 'Miljøstatus styres af';

/** The samples a survey type's `sample_ids` names, in that order. */
export function linkedSamples(type: SurveyType, samples: Sample[]): Sample[] {
  const byId = new Map(samples.map((s) => [s.id, s]));
  return type.sample_ids.map((id) => byId.get(id)).filter((s): s is Sample => s !== undefined);
}

/** The sample line as data: each linked sample with its text, or the type to register one for. */
export type SampleLineModel =
  | { kind: 'none'; typeId: number }
  | { kind: 'linked'; items: { id: number; text: string }[] };

export function sampleLineModel(type: SurveyType, samples: Sample[]): SampleLineModel {
  const linked = linkedSamples(type, samples);
  if (linked.length === 0) return { kind: 'none', typeId: type.id };
  return {
    kind: 'linked',
    items: linked.map((s) => {
      const line = `${s.code} · ${s.title} — ${STAGE_LABEL[s.stage]}`;
      return { id: s.id, text: s.result ? `${line} · ${RESULT_LABEL[s.result]}` : line };
    }),
  };
}

/**
 * The sample line as plain text: which sample(s) drive the type's
 * miljøstatus, or the screening-only fallback when none are linked.
 */
export function sampleLineText(type: SurveyType, samples: Sample[]): string {
  const m = sampleLineModel(type, samples);
  return m.kind === 'none'
    ? SAMPLE_LINE_NONE
    : `${SAMPLE_LINE_LINKED_PREFIX} ${m.items.map((i) => i.text).join(', ')}`;
}

/**
 * The linked samples still awaiting an answer (`stage !== 'svar'`), as
 * `code · title` joined by ', ' — or '' when none are. The gate note and the
 * page's refused-approval toast both name them this way.
 */
export function pendingSampleList(type: SurveyType, samples: Sample[]): string {
  return linkedSamples(type, samples)
    .filter((s) => s.stage !== 'svar')
    .map((s) => `${s.code} · ${s.title}`)
    .join(', ');
}
