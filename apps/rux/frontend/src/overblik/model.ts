// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Overblik as data: the circularity percents, the KPI row, the quick links,
 * the case hero and the metadata editor's commits. Every figure is read off a
 * server response; this module only rounds and words it.
 */

import type { RuxApiClient } from '../api/client';
import type { ProjectInfo, ProjectSummary, ReportPdfVersion, SurveySummary, Treatment } from '../api/types';
import { TREATMENTS } from '../api/types';
import { INDBERETNING_PATH, KORTLAEGNING_PATH, MILJOE_PATH, RAPPORT_PATH } from '../app/links';
import { TREATMENT_LABEL } from '../kortlaegning/vocab';

export interface CircSegment {
  treatment: Treatment;
  label: string;
  tonnes: number;
  /** Whole percent; a bar's percents always sum to 100. */
  percent: number;
}

/**
 * Whole percents that sum to exactly 100 (largest remainder). Rounding each
 * share on its own can print 99 % or 101 % in total; ties go to the earlier
 * (higher waste-hierarchy) step.
 */
export function wholePercents(values: readonly number[]): number[] {
  const total = values.reduce((s, v) => s + v, 0);
  if (total <= 0) return values.map(() => 0);
  const exact = values.map((v) => (100 * v) / total);
  const out = exact.map((e) => Math.floor(e));
  let left = 100 - out.reduce((s, v) => s + v, 0);
  const byRemainder = exact.map((e, i) => ({ i, r: e - out[i] })).sort((a, b) => b.r - a.r || a.i - b.i);
  for (const { i } of byRemainder) {
    if (left <= 0) break;
    out[i] += 1;
    left -= 1;
  }
  return out;
}

/** The bar's segments in waste-hierarchy order — only steps that have tonnes. */
export function circularitySegments(circularity: Record<Treatment, number>): CircSegment[] {
  const tonnes = TREATMENTS.map((t) => Math.max(0, circularity[t] ?? 0));
  const percents = wholePercents(tonnes);
  return TREATMENTS.map((t, i) => ({
    treatment: t,
    label: TREATMENT_LABEL[t],
    tonnes: tonnes[i],
    percent: percents[i],
  })).filter((s) => s.tonnes > 0);
}

/** What the bar and its legend say when no step has tonnes yet. */
export const CIRC_EMPTY_TEXT = 'Ingen mængder registreret endnu';

/**
 * The bar's accessible name: every step in words with its whole percent, so
 * the colours are never the only carrier of the split.
 */
export function circularityAriaLabel(segments: readonly CircSegment[]): string {
  const prefix = 'Fordeling af materialemængde på affaldshierarkiet';
  if (segments.length === 0) return `${prefix}: ${CIRC_EMPTY_TEXT.toLowerCase()}`;
  return `${prefix}: ${segments.map((s) => `${s.label} ${s.percent} %`).join(', ')}`;
}

export interface Kpi {
  key: string;
  value: string;
  unit?: string;
  label: string;
  /** Ink for a figure that asks for action (prototype: queue warn, samples crit). */
  ink?: 'warn' | 'crit';
  hint?: string;
}

export function percentText(share: number | null): string {
  return share === null ? '—' : String(Math.round(share * 100));
}

/**
 * The five KPI tiles (R4). "Scanningsdækning" in the prototype is not
 * measured by anything in the project; "Klassificeret" is: the share of the
 * instance cloud's points that carry an instance label.
 */
export function kpis(s: SurveySummary): Kpi[] {
  const classified: Kpi =
    s.classified_share === null
      ? { key: 'classified', value: '—', label: 'Klassificeret', hint: 'Kræver instansskyen' }
      : { key: 'classified', value: percentText(s.classified_share), unit: '%', label: 'Klassificeret' };
  const reuse: Kpi =
    s.reuse_share === null
      ? { key: 'reuse', value: '—', label: 'Bevaring / genbrug' }
      : { key: 'reuse', value: percentText(s.reuse_share), unit: '%', label: 'Bevaring / genbrug' };
  const queue: Kpi = { key: 'queue', value: String(s.counts.queue), label: 'Til gennemsyn' };
  if (s.counts.queue > 0) queue.ink = 'warn';
  const samples: Kpi = { key: 'samples', value: String(s.pending_samples), label: 'Prøver afventer' };
  if (s.pending_samples > 0) samples.ink = 'crit';
  return [{ key: 'types', value: String(s.counts.all), label: 'Komponenter' }, classified, reuse, queue, samples];
}

export interface QuickLink {
  to: string;
  title: string;
  sub: string;
}

/** `undefined`: still loading; `null`: the list failed to load. */
export function versionsText(v: readonly ReportPdfVersion[] | null | undefined): string {
  if (v === undefined) return 'Henter versioner…';
  if (v === null) return 'Versioner kunne ikke hentes';
  if (v.length === 0) return 'Ingen versioner endnu';
  return v.length === 1 ? '1 version' : `${v.length} versioner`;
}

export function quickLinks(s: SurveySummary, versions: readonly ReportPdfVersion[] | null | undefined): QuickLink[] {
  return [
    { to: KORTLAEGNING_PATH, title: 'Kortlægning', sub: `${s.counts.queue} til gennemsyn · ${s.counts.all} typer` },
    {
      to: MILJOE_PATH,
      title: 'Miljø & prøver',
      sub: s.pending_samples === 1 ? '1 prøve afventer svar' : `${s.pending_samples} prøver afventer svar`,
    },
    { to: RAPPORT_PATH, title: 'Rapport', sub: versionsText(versions) },
    { to: INDBERETNING_PATH, title: 'Indberetning', sub: `${s.counts.approved} af ${s.counts.all} typer godkendt` },
  ];
}

/** The record's name (the edited one when given), else the `.rux` file stem. */
export function caseName(summary: ProjectSummary, project?: ProjectInfo): string {
  const name = (project ?? summary.projects[0])?.name?.trim();
  return name ? name : summary.path.replace(/\.rux$/i, '');
}

/** The hero's sub line: only the metadata the record has (R5). */
export function heroSubline(p: ProjectInfo | undefined): string {
  if (!p) return '';
  const parts: string[] = [];
  const address = p.building_address?.trim();
  if (address) parts.push(address);
  if (p.year_of_construction && p.year_of_construction > 0) parts.push(`opført ${p.year_of_construction}`);
  const date = p.survey_date?.trim();
  if (date) parts.push(`registreret ${date}`);
  const org = p.survey_organisation?.trim();
  if (org) parts.push(`udarbejdet af ${org}`);
  return parts.join(' · ');
}

export type MetaField = 'name' | 'building_address' | 'survey_date' | 'survey_organisation' | 'notes';

export interface MetaFieldSpec {
  key: MetaField;
  label: string;
  required?: boolean;
  multiline?: boolean;
}

/** The editor's text fields, in order; the year sits between address and date. */
export const META_FIELDS: readonly MetaFieldSpec[] = [
  { key: 'name', label: 'Navn', required: true },
  { key: 'building_address', label: 'Adresse' },
  { key: 'survey_date', label: 'Registreringsdato' },
  { key: 'survey_organisation', label: 'Udarbejdet af' },
  { key: 'notes', label: 'Noter', multiline: true },
];

export type ProjectPatch = Parameters<RuxApiClient['patchProject']>[1];

/** One field's sparse PATCH. An emptied optional field is cleared with null. */
export function metaPatch(key: MetaField, value: string): ProjectPatch {
  if (key === 'name') return { name: value };
  return { [key]: value === '' ? null : value };
}

export type YearCommit = { send: true; value: number | null } | { send: false; invalid: boolean };

export const INVALID_YEAR_TOAST = 'Byggeår skal være et årstal, fx 1978.';

/**
 * What the year field sends on blur. Unchanged → nothing (an untouched blur
 * never commits); empty or 0 → clear (null); exactly four digits → that year
 * (`0000` clears like 0); anything else, including 3 or 5 digits → nothing,
 * `invalid` so the caller reverts and says why.
 */
export function yearCommit(draft: string, current: number | undefined): YearCommit {
  const cur = current && current > 0 ? current : null;
  const t = draft.trim();
  if (t !== '' && t !== '0' && !/^\d{4}$/.test(t)) return { send: false, invalid: true };
  const n = t === '' ? 0 : Number(t);
  const value = n > 0 ? n : null;
  return value === cur ? { send: false, invalid: false } : { send: true, value };
}
