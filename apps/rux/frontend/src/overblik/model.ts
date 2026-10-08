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
import { draftCommit, type DraftCommit } from '../app/textDraft';
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

/**
 * Indberetning's sub-line. The server's blocking count wins: approved types
 * can still block (a pending sample, no tonnes), so "4 af 11 typer godkendt"
 * would read as further along than Indberetning says. `blockingTypes` is
 * `GET /survey/fractions`' `blocking_types`; while that is loading
 * (`undefined`) or failed (`null`), and when it is 0, the approved count.
 */
export function indberetningText(s: SurveySummary, blockingTypes: number | null | undefined): string {
  if (blockingTypes) return blockingTypes === 1 ? '1 type blokerer' : `${blockingTypes} typer blokerer`;
  return `${s.counts.approved} af ${s.counts.all} typer godkendt`;
}

export function quickLinks(
  s: SurveySummary,
  versions: readonly ReportPdfVersion[] | null | undefined,
  blockingTypes?: number | null,
): QuickLink[] {
  return [
    { to: KORTLAEGNING_PATH, title: 'Kortlægning', sub: `${s.counts.queue} til gennemsyn · ${s.counts.all} typer` },
    {
      to: MILJOE_PATH,
      title: 'Miljø & prøver',
      sub: s.pending_samples === 1 ? '1 prøve afventer svar' : `${s.pending_samples} prøver afventer svar`,
    },
    { to: RAPPORT_PATH, title: 'Rapport', sub: versionsText(versions) },
    { to: INDBERETNING_PATH, title: 'Indberetning', sub: indberetningText(s, blockingTypes) },
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
  if (date) parts.push(`registreret ${danishDate(date)}`);
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

export type YearCommit = DraftCommit<number | null>;

export const INVALID_YEAR_TOAST = 'Byggeår skal være et årstal, fx 1978.';

/** Said when the required case name was emptied and snapped back. */
export const EMPTY_NAME_TOAST = 'Sagsnavnet kan ikke være tomt.';

/** The earliest year of construction the field accepts. */
export const MIN_YEAR = 1000;

/** The latest year accepted: next year, so a building finishing soon can be entered. */
export function maxYear(now: Date = new Date()): number {
  return now.getFullYear() + 1;
}

/** What the year field shows for a stored year: '' for not set (0 or absent). */
export function yearText(current: number | undefined): string {
  return current && current > 0 ? String(current) : '';
}

/**
 * The year field's validator. Empty, `0` or `0000` → clear (null); exactly
 * four digits within MIN_YEAR…`max` → that year; anything else — 3 or 5
 * digits, a decimal, an implausible year like `0005` or one after next year —
 * is invalid (null) and is never sent.
 */
export function parseYear(draft: string, max: number = maxYear()): { value: number | null } | null {
  const t = draft.trim();
  if (t === '' || t === '0' || t === '0000') return { value: null };
  if (!/^\d{4}$/.test(t)) return null;
  const n = Number(t);
  return n >= MIN_YEAR && n <= max ? { value: n } : null;
}

/**
 * What the year field sends on blur (`draftCommit` over `parseYear`).
 * Unchanged → nothing (an untouched blur never commits); a clear or a
 * plausible year → that; anything invalid → nothing, `invalid` so the caller
 * reverts and says why.
 */
export function yearCommit(draft: string, current: number | undefined, max: number = maxYear()): YearCommit {
  return draftCommit(draft, yearText(current), (d) => parseYear(d, max));
}

/**
 * A stored date in Danish form (F24): `2026-08-09` → `09.08.2026`. A value
 * that is not an ISO date (a time suffix is ignored) is shown as stored,
 * trimmed, rather than guessed at.
 */
export function danishDate(stored: string): string {
  const t = stored.trim();
  const m = /^(\d{4})-(\d{2})-(\d{2})(?:[T ].*)?$/.exec(t);
  if (!m) return t;
  const [y, mo, d] = [Number(m[1]), Number(m[2]), Number(m[3])];
  // Round-trip through Date.UTC: an impossible date (2026-13-45, 2026-02-30)
  // rolls over, so it no longer matches and is shown as stored.
  const date = new Date(Date.UTC(y, mo - 1, d));
  const real = date.getUTCFullYear() === y && date.getUTCMonth() === mo - 1 && date.getUTCDate() === d;
  return real ? `${m[3]}.${m[2]}.${m[1]}` : t;
}

/** Where a new record id's randomness comes from (injectable for tests). */
export interface IdSource {
  /** `crypto.randomUUID`, which only exists in a secure context. */
  randomUUID?: () => string;
  now: () => number;
  random: () => number;
}

/** The browser's sources. `crypto.randomUUID` is absent over plain http on a LAN address (`ruxd --local --bind`). */
export function browserIdSource(): IdSource {
  const c = globalThis.crypto;
  return {
    randomUUID: typeof c?.randomUUID === 'function' ? () => c.randomUUID() : undefined,
    now: Date.now,
    random: Math.random,
  };
}

/**
 * An id for a project record that does not exist yet (PATCH upserts it). A
 * UUID when the context allows one, else a time-plus-random id that is
 * unique enough for the one record a `.rux` holds.
 */
export function newRecordId(src: IdSource = browserIdSource()): string {
  if (src.randomUUID) return src.randomUUID();
  const rand = src.random().toString(36).slice(2, 12).padEnd(10, '0');
  return `p-${src.now().toString(36)}-${rand}`;
}
