// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Sager as data: the one card `ruxd --local` can show (R1) and the command that
 * opens another case. Every figure is read off a server response; this module
 * only words it.
 */

import type { ProjectInfo, SurveyFractions, SurveySummary } from '../api/types';
import type { Tone } from '../kortlaegning/vocab';
import { danishDate, percentText } from '../overblik/model';

export interface CaseStatus {
  label: string;
  tone: Tone;
}

/**
 * A survey with nothing in it: no queued/approved/rejected types at all.
 * Shared by `caseStatus` (Kladde) and `caseStats` (F7) so the two never
 * contradict each other — a survey whose types are all rejected (`all`
 * excludes `rejected`, so it reads 0) is not "no survey", it is done.
 */
function hasNoSurveyTypes(s: SurveySummary): boolean {
  return s.counts.all === 0 && s.counts.rejected === 0;
}

/**
 * The card's status pill (R3), a wording of server numbers. `f` is
 * `GET /survey/fractions`: `undefined` while loading, `null` when it failed.
 * Without it nothing can be called done, so it reads Gennemgang. Agrees with
 * Overblik/Indberetning's readiness rule (`f.ready`): a project with no
 * fraction rows is never "Klar til indberetning", since `ready` itself
 * requires at least one fraction row and no blockers — this function never
 * re-derives that, only reads it.
 */
export function caseStatus(s: SurveySummary, f: SurveyFractions | null | undefined): CaseStatus {
  if (hasNoSurveyTypes(s)) return { label: 'Kladde', tone: 'wait' };
  if (s.counts.queue > 0 || !f || f.blocking_types > 0) return { label: 'Gennemgang', tone: 'accent' };
  if (f.ready) return { label: 'Klar til indberetning', tone: 'good' };
  return { label: 'Gennemgået', tone: 'good' };
}

export interface CaseStat {
  key: string;
  value: string;
  label: string;
}

/** What the stats line says for a project with no survey types. */
export const NO_SURVEY_TEXT = 'Ingen kortlægning endnu';

/**
 * The card's stats line (R2), or null when the project has no survey yet
 * (F7: the same "no types at all" test `caseStatus` uses for Kladde, so a
 * survey whose types are all rejected still shows its stats rather than
 * contradicting a Gennemgået pill with "Ingen kortlægning endnu").
 */
export function caseStats(s: SurveySummary): CaseStat[] | null {
  if (hasNoSurveyTypes(s)) return null;
  return [
    { key: 'types', value: String(s.counts.all), label: 'komponenter' },
    { key: 'reuse', value: s.reuse_share === null ? '—' : `${percentText(s.reuse_share)} %`, label: 'bevaring/genbrug' },
    { key: 'queue', value: String(s.counts.queue), label: 'til gennemsyn' },
  ];
}

/** Address and organisation, whichever the record has (no bygherre is stored). */
export function cardSubline(p: ProjectInfo | undefined): string {
  const parts: string[] = [];
  const address = p?.building_address?.trim();
  if (address) parts.push(address);
  const org = p?.survey_organisation?.trim();
  if (org) parts.push(`udarbejdet af ${org}`);
  return parts.length > 0 ? parts.join(' · ') : 'Ingen adresse registreret';
}

/** The foot's date: the registration date in place of the prototype's Frist (not stored). */
export function cardDate(p: ProjectInfo | undefined): string {
  const date = p?.survey_date?.trim();
  return date ? `Registreret ${danishDate(date)}` : '—';
}

/** How to open another case: `ruxd --local` serves the project it was started with. */
export const OPEN_ANOTHER_COMMAND = 'ruxd --local <fil>.rux';
