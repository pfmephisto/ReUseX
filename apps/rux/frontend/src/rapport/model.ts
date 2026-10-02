// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Rapport as data: how a stored version reads (date, size, title, complete or
 * draft), the hero line, the draft warning and the toasts. `blocking_types`
 * and `version` are the server's (schema v23); this only words them.
 */

import { ApiRequestError } from '../api/client';
import { parseServerUtc } from '../api/types';
import type { ReportPdfVersion, SurveyFractions, SurveySummary, Template } from '../api/types';
import { errorMessage } from '../app/saveError';
import type { Tone } from '../kortlaegning/vocab';
import { pickTemplate } from '../kortlaegning/templatePick';
import { percentText } from '../overblik/model';

/** "09.08.2026" in local time; the raw string when it does not parse. */
export function versionDate(s: string): string {
  const d = parseServerUtc(s);
  return d ? d.toLocaleDateString('da-DK', { day: '2-digit', month: '2-digit', year: 'numeric' }) : s;
}

/** "09.08.2026 kl. 12.05" in local time (or `timeZone`); the raw string when it does not parse. */
export function versionDateTime(s: string, timeZone?: string): string {
  const d = parseServerUtc(s);
  if (!d) return s;
  const date = d.toLocaleDateString('da-DK', { day: '2-digit', month: '2-digit', year: 'numeric', timeZone });
  const time = d.toLocaleTimeString('da-DK', { hour: '2-digit', minute: '2-digit', timeZone });
  return `${date} kl. ${time}`;
}

export function formatBytesDa(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  const kb = bytes / 1024;
  if (kb < 1024) return `${Math.round(kb).toLocaleString('da-DK')} KB`;
  return `${(kb / 1024).toLocaleString('da-DK', { minimumFractionDigits: 1, maximumFractionDigits: 1 })} MB`;
}

export function versionTitle(v: ReportPdfVersion): string {
  return `${v.label.trim() || 'Ressourcekortlægning'} — v${v.version}`;
}

export interface VersionStatus {
  tone: Tone;
  text: string;
  title: string;
}

/** Complete / draft at generation time (R7); null for a pre-v23 version. */
export function versionStatus(v: ReportPdfVersion): VersionStatus | null {
  if (v.blocking_types === null) return null;
  if (v.blocking_types === 0) {
    return { tone: 'good', text: 'Komplet', title: 'Alle typer var godkendt og afklaret, da versionen blev genereret.' };
  }
  const n = v.blocking_types;
  return {
    tone: 'wait',
    text: 'Udkast',
    title: `${n === 1 ? '1 type' : `${n} typer`} var ikke godkendt, afventede prøvesvar eller manglede tons.`,
  };
}

/**
 * What a version with `blocking_types: null` shows instead of a pill: it was
 * generated before the status was recorded, so it is neither complete nor a
 * draft as far as anyone can tell.
 */
export const UNKNOWN_STATUS = {
  text: 'Ukendt status',
  title: 'Versionen er ældre end statusregistreringen — det vides ikke, om den var komplet.',
} as const;

/**
 * The hero line: "11 komponenter · 54 % bevaring/genbrug · 1 forurenet · 2
 * prøver afventer". Its figures are the whole survey (every non-rejected
 * type), not the approved subset the PDF reports — see `HERO_SCOPE`.
 */
export function reportHeroSub(s: SurveySummary): string {
  const reuse = s.reuse_share === null ? '—' : `${percentText(s.reuse_share)} %`;
  return [
    `${s.counts.all} komponenter`,
    `${reuse} bevaring/genbrug`,
    `${s.contaminated_types} forurenet`,
    s.pending_samples === 1 ? '1 prøve afventer' : `${s.pending_samples} prøver afventer`,
  ].join(' · ');
}

/** The hero's scope caption: its figures are not the PDF's approved-only ones. */
export const HERO_SCOPE = 'Hele kortlægningen (inkl. ikke-godkendte)';

/**
 * Null when nothing blocks. An empty survey is not `ready` (nothing to
 * report) but blocks nothing either, so it gets no draft warning; a version
 * generated then is marked Komplet (a known edge, see the spec).
 */
export function draftNotice(f: SurveyFractions): string | null {
  if (f.ready || f.blocking_types === 0) return null;
  const n = f.blocking_types;
  return `${n === 1 ? '1 type' : `${n} typer`} afventer gennemsyn eller prøvesvar, eller mangler tons — en ny version bliver et udkast, og de indgår ikke i mængderne.`;
}

export function generatedToast(v: ReportPdfVersion): string {
  return `${versionTitle(v)} genereret${v.blocking_types ? ' (udkast)' : ''}`;
}

export function generateErrorMessage(cause: unknown): string {
  if (cause instanceof ApiRequestError) {
    if (cause.status === 409) return 'Kunne ikke generere — et pipeline-job kører. Prøv igen om lidt.';
    if (cause.status === 503) return 'Kunne ikke generere — projektet skrives til lige nu. Prøv igen om lidt.';
  }
  return `Kunne ikke generere rapporten: ${errorMessage(cause)}`;
}

/** The generation succeeded; only the re-read of the list after it failed (F22). */
export const LIST_REFRESH_FAILED = 'Rapporten blev genereret, men listen kunne ikke opdateres.';

/** The prototype's footnote without the MRK signature, which nothing models (R7). */
export const REPORT_FOOTNOTE =
  'Kun godkendte mængder indgår i rapportens kortlægningsafsnit. Versioner er uforanderlige — en ny generering giver en ny version med tidsstempel. Inventarlisten er en aktuel eksport og gemmes ikke som version.';

/** The select value for "Ingen" (no Ressourcetabel). */
export const NO_TEMPLATE = '';

export function parseTemplateChoice(value: string): number | null {
  return /^\d+$/.test(value) && Number(value) > 0 ? Number(value) : null;
}

/** The choice if its template still exists, else null (R10). */
export function validChoice(templates: readonly Pick<Template, 'id'>[], id: number | null): number | null {
  return id !== null && templates.some((t) => t.id === id) ? id : null;
}

/**
 * Data-eksport's default: the screening seed, else the first template (R10).
 * Delegates to Kortlægning's `pickTemplate` (spec §6.1) so both pickers share
 * one rule; only `id`/`seed` are read, so a narrower fixture is accepted.
 * `seed` is typed as plain `string | null` rather than `Template['seed']`
 * (`TemplateSeed | null`) because an inline test fixture's string-literal
 * properties widen to `string`, and the comparison inside `pickTemplate`
 * never needs the narrower type.
 */
export function defaultExportTemplateId(
  templates: readonly (Pick<Template, 'id'> & { seed: string | null })[],
): number | null {
  return pickTemplate(templates as readonly Template[], null)?.id ?? null;
}

export function ressourcetabelHint(t: Pick<Template, 'name' | 'resolved_keys'> | null): string {
  if (!t) return 'Rapporten genereres uden ressourcetabel.';
  const n = t.resolved_keys.length;
  if (n === 0) return `Skabelonen "${t.name}" har ingen felter — tabellen bliver tom.`;
  return `Ressourcetabel med ${n === 1 ? '1 felt' : `${n} felter`} fra "${t.name}".`;
}

/**
 * What a settled CSV-option write does to the page's optimistic template
 * copy. Only the newest write per template decides (its ticket is still the
 * latest): its success applies the server's template, its failure re-reads
 * (or reverts). An older write — success or failure — is ignored, because a
 * newer edit is already showing and was built on top of it; that edit's own
 * outcome settles the final state.
 */
export type CsvWriteOutcome = 'apply' | 'reconcile' | 'ignore';

export function csvWriteOutcome(isLatest: boolean, ok: boolean): CsvWriteOutcome {
  if (!isLatest) return 'ignore';
  return ok ? 'apply' : 'reconcile';
}
